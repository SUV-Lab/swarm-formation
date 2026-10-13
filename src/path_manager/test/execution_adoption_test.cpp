// Actual planner calculations and their execution compatibility boundary.
// Numerical/evaluation regressions live in the existing mission/chain tests;
// this test checks that a candidate does not become execution state merely
// because it was calculated, and that delayed/replayed adoption is refused.
#include "path_manager/mission_planner.h"
#include "path_manager/path_manager.h"
#include "path_manager/ros_planning_backend.h"
#include "path_manager/ros_terrain_grid.h"

#include <iostream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace pm = path_manager;
namespace {
// Compile-time contract: the execution adapter offers const views and explicit
// adoption/update methods; old raw slots and concrete owners cannot escape.
template <typename T, typename = void>
struct HasPublicRawTrajectorySlot : std::false_type {};
template <typename T>
struct HasPublicRawTrajectorySlot<T,
    std::void_t<decltype(std::declval<T &>().traj_)>> : std::true_type {};
template <typename T, typename = void>
struct HasPublicExecutionOwner : std::false_type {};
template <typename T>
struct HasPublicExecutionOwner<T,
    std::void_t<decltype(std::declval<T &>().execution_)>> : std::true_type {};
template <typename T, typename = void>
struct HasPublicEnvironmentOwner : std::false_type {};
template <typename T>
struct HasPublicEnvironmentOwner<T,
    std::void_t<decltype(std::declval<T &>().environment_)>> : std::true_type {};
template <typename T, typename = void>
struct HasExecutionManager : std::false_type {};
template <typename T>
struct HasExecutionManager<T,
    std::void_t<decltype(std::declval<T &>().executionManager())>> : std::true_type {};

static_assert(!HasPublicRawTrajectorySlot<pm::PathManager>::value,
              "the legacy raw trajectory slot must not be public");
static_assert(!HasPublicExecutionOwner<pm::PathManager>::value,
              "the execution owner must remain private");
static_assert(!HasPublicEnvironmentOwner<pm::PathManager>::value,
              "the environment owner must remain private");
static_assert(!HasExecutionManager<pm::MissionPlanner>::value,
              "the generic planner API must not expose the ROS execution adapter");
static_assert(std::is_same_v<decltype(std::declval<pm::PathManager &>().executionTrajectory()),
                             const ego_planner::LocalTrajData &>);
static_assert(std::is_same_v<decltype(std::declval<pm::PathManager &>().referenceTrajectory()),
                             const ego_planner::GlobalTrajData &>);
static_assert(std::is_same_v<decltype(std::declval<pm::PathManager &>().swarmTrajectories()),
                             const ego_planner::SwarmTrajData &>);
static_assert(std::is_same_v<decltype(std::declval<pm::PathManager &>().riskZones()),
                             const std::vector<pm::RiskZone> &>);
static_assert(std::is_const_v<mmp_vehicle_dynamics::VehicleModel::Ptr::element_type>);

int failures = 0;
void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

pm::TerrainGrid flatTerrain(float height = 2.0f)
{
  pm::TerrainGrid terrain;
  terrain.rows = 180;
  terrain.cols = 150;
  terrain.resolution = 2.0;
  terrain.center_x = 180.0;
  terrain.center_y = 150.0;
  terrain.length_x = 360.0;
  terrain.length_y = 300.0;
  terrain.elevation.assign(180 * 150, height);
  return terrain;
}

pm::MissionPlanRequest mission()
{
  pm::MissionPlanRequest request;
  request.head.src = pm::StartStateSource::STATED_VECTOR;
  request.head.pos_u = Eigen::Vector3d(30.0, 150.0, 6.0);
  request.head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
  request.head.acc_u.setZero();
  request.waypoints = {Eigen::Vector3d(330.0, 150.0, 4.0)};
  request.tail = ego_planner::TailBoundary::pinned(
      request.head.vel_u, Eigen::Vector3d::Zero());
  return request;
}

std::vector<double> coefficients(const poly_traj::Trajectory &trajectory)
{
  std::vector<double> result;
  for (const auto &piece : trajectory) {
    result.push_back(piece.getDuration());
    const auto &matrix = piece.getCoeffMat();
    result.insert(result.end(), matrix.data(), matrix.data() + matrix.size());
  }
  return result;
}

struct ExecutionSnapshot {
  std::vector<double> trajectory, reference;
  uint64_t id, counter;
  int drone_id;
  double duration, start, reference_duration, reference_start;
  double reference_target, reference_previous_target;
  Eigen::Vector3d start_position;
};

ExecutionSnapshot snapshot(const pm::PathManager &manager)
{
  const auto &local = manager.executionTrajectory();
  const auto &reference = manager.referenceTrajectory();
  return {coefficients(local.traj), coefficients(reference.traj),
          local.traj_id,
          manager.sealPlanningComputation({}).previous_identity_counter,
          local.drone_id, local.duration, local.start_time, reference.duration,
          reference.global_start_time, reference.glb_t_of_lc_tgt,
          reference.last_glb_t_of_lc_tgt, local.start_pos};
}

bool same(const ExecutionSnapshot &a, const ExecutionSnapshot &b)
{
  return a.trajectory == b.trajectory && a.reference == b.reference &&
         a.id == b.id && a.counter == b.counter && a.drone_id == b.drone_id &&
         a.duration == b.duration && a.start == b.start &&
         a.reference_duration == b.reference_duration &&
         a.reference_start == b.reference_start &&
         a.reference_target == b.reference_target &&
         a.reference_previous_target == b.reference_previous_target &&
         a.start_position == b.start_position;
}

std::unique_ptr<pm::RosPlanningBackend> backend(
    const char *config, const std::string &name, bool chain, bool route)
{
  pm::RosPlanningOptions runtime;
  runtime.node_name = name;
  runtime.node_options.arguments({"--ros-args", "--params-file", config});
  runtime.node_options.parameter_overrides({
      rclcpp::Parameter("drone_id", 7),
      rclcpp::Parameter("chain/enable", chain),
      rclcpp::Parameter("chain/segments", 3),
      rclcpp::Parameter("chain/phase/enable", false),
      rclcpp::Parameter("chain/author_from_route", route),
      rclcpp::Parameter("chain/parallel", route),
      rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
  auto result = pm::makeRosPlanningBackend(std::move(runtime));
  pm::PlanningEnvironment environment;
  environment.terrain = flatTerrain();
  environment.risk_zones = std::vector<pm::RiskZone>{};
  result->initialize({}, std::move(environment));
  return result;
}

void exercise(const char *config, const std::string &mode, bool chain, bool route)
{
  auto planner = backend(config, "adoption_" + mode, chain, route);
  auto manager = planner->executionManager();
  auto candidate = planner->compute(mission());
  check(candidate.hasTrajectory(), mode + " calculates an owned trajectory: " +
                                      candidate.status.detail);
  if (!candidate.hasTrajectory()) return;
  const uint64_t expected_count = chain && !route ? 5 : 1;
  check(candidate.identity_advances == expected_count,
        mode + " retains historical successful intermediate identity count");
  check(manager->executionTrajectory().traj_id == 0 &&
            candidate.previous_identity_counter == 0 &&
            manager->executionTrajectory().duration == 0.0 &&
            manager->executionTrajectory().start_time == 0.0,
        mode + " initial calculation does not allocate an execution identity");
  const auto final = coefficients(candidate.product->trajectory);
  const auto reference = coefficients(candidate.product->reference_trajectory);
  const auto first = manager->applyPlanningComputation(candidate);
  check(first.applied && first.trajectory_id == expected_count &&
            first.drone_id == 7,
        mode + " explicit adoption assigns the historical final identity");
  check(coefficients(manager->executionTrajectory().traj) == final &&
            coefficients(manager->referenceTrajectory().traj) == reference &&
            manager->executionTrajectory().start_time ==
                candidate.product->start_time &&
            manager->referenceTrajectory().global_start_time ==
                candidate.product->reference_start_time,
        mode + " adoption preserves owned coefficients and original clock points");

  const auto active = snapshot(*manager);
  check(!manager->applyPlanningComputation(candidate).applied &&
            same(active, snapshot(*manager)),
        mode + " replayed adoption is refused without modifying active execution");

  const auto early = planner->compute({});
  check(!early.hasTrajectory() && !early.product &&
            early.status.reason == pm::PlanReason::INITIAL_STATE_UNSPECIFIED &&
            early.identity_advances == 0 && !early.invalidate_execution &&
            same(active, snapshot(*manager)),
        mode + " early refusal retains the active trajectory and consumes no identity");
  check(manager->applyPlanningComputation(early).applied &&
            same(active, snapshot(*manager)),
        mode + " applying an early refusal preserves historical execution fields");

  auto next = planner->compute(mission());
  check(next.hasTrajectory() && same(active, snapshot(*manager)),
        mode + " second calculation leaves every active execution field unchanged");
  if (!next.hasTrajectory()) return;
  check(coefficients(next.product->trajectory) == final &&
            coefficients(next.product->reference_trajectory) == reference,
        mode + " repeated computation preserves the complete numerical product");

  if (!chain) {
    // Even an early-refused newer request supersedes the association between
    // an older candidate and the manager's retained calculation diagnostics.
    const auto newer_refusal = planner->compute({});
    check(!newer_refusal.hasTrajectory() &&
              !manager->applyPlanningComputation(next).applied &&
              same(active, snapshot(*manager)),
          "newer calculation makes an earlier candidate stale without adopting either");
    next = planner->compute(mission());
    check(next.hasTrajectory() && same(active, snapshot(*manager)),
          "a fresh calculation restores a usable adoption context without storage mutation");
    if (!next.hasTrajectory()) return;
  }

  auto other = backend(config, "foreign_" + mode, chain, route);
  auto foreign_manager = other->executionManager();
  check(!foreign_manager->applyPlanningComputation(next).applied &&
            foreign_manager->executionTrajectory().traj_id == 0,
        mode + " another manager cannot adopt this computation");

  check(planner->setTerrain(flatTerrain(2.1f)), mode + " environment update accepted");
  const auto updated = snapshot(*manager);
  check(!manager->applyPlanningComputation(next).applied &&
            same(updated, snapshot(*manager)),
        mode + " environment-stale adoption is refused without undoing live changes");

  // No numerical solve is needed to check that binding changes invalidate
  // this manager's outstanding receipt context, including refusal receipts.
  const auto before_model_change = planner->compute({});
  planner->setVehicleModel(nullptr);
  check(!manager->applyPlanningComputation(before_model_change).applied &&
            same(updated, snapshot(*manager)),
        mode + " model rebinding rejects an outstanding receipt without withdrawing execution");
  const auto before_search_change = planner->compute({});
  planner->setRouteSearchBackend(nullptr);
  check(!manager->applyPlanningComputation(before_search_change).applied &&
            same(updated, snapshot(*manager)),
        mode + " search rebinding rejects an outstanding receipt without withdrawing execution");

  // Exercise the execution compatibility disposition independently of its
  // evaluator producer. Existing gate tests cover actual unsafe calculations.
  // A solved-but-refused candidate historically consumed an identity and then
  // cleared duration/start_time; it was never a public successful result.
  pm::PlanningComputation refused;
  refused.recordProduct(*next.product);
  refused.status = pm::PlanResult::failedBecause(
      pm::PlanReason::STITCHED_FLIGHT_UNSAFE, "test: whole-flight refusal");
  refused.invalidate_execution = true;
  refused = manager->sealPlanningComputation(std::move(refused));
  check(!refused.hasTrajectory(), mode + " rejected intermediate is not a flyable product");
  const auto refusal_receipt = manager->applyPlanningComputation(refused);
  check(refusal_receipt.applied && refusal_receipt.trajectory_id ==
            updated.counter + 1 && manager->executionTrajectory().duration == 0.0 &&
            manager->executionTrajectory().start_time == 0.0,
        mode + " postsolve refusal consumes its old identity and invalidates execution");
  const auto withdrawn = snapshot(*manager);
  check(!manager->applyPlanningComputation(next).applied &&
            same(withdrawn, snapshot(*manager)),
        mode + " stale candidate cannot resurrect the withdrawn flight");

  const auto public_failure = planner->plan({});
  check(!public_failure.hasTrajectory() &&
            public_failure.trajectory.getPieceNum() == 0 &&
            public_failure.reference_trajectory.getPieceNum() == 0 &&
            public_failure.trajectory_id == 0 && !public_failure.evaluation.evaluated &&
            public_failure.phases.empty() && same(withdrawn, snapshot(*manager)),
        mode + " failed compatibility plan returns no old or rejected trajectory");
  check(coefficients(candidate.product->trajectory) == final &&
            coefficients(candidate.product->reference_trajectory) == reference,
        mode + " owned result survives later computations, adoption and withdrawal");
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: execution_adoption_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  exercise(argv[1], "direct", false, false);
  exercise(argv[1], "route", true, true);
  exercise(argv[1], "baseline", true, false);
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures
            << " failure(s)\n";
  return failures ? 1 : 0;
}
