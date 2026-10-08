// External installed-target consumer: no source include paths/private hooks.
#include <path_manager/mission_planner.h>
#include "path_manager/ros_planning_backend.h"
#include <path_manager/path_manager.h>
#include <path_optimizer/optimizer_request.h>
#include <mmp_vehicle_dynamics/vehicle_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace md = mmp_vehicle_dynamics;
namespace {
int failures = 0;
void check(bool ok, const std::string &message) {
  std::cout << (ok ? "[OK] " : "[FAIL] ") << message << std::endl;
  if (!ok) ++failures;
}
struct Observations {
  std::atomic<int> limits{0}, evaluate{0}, penalty{0}, domain{0}, utilization{0};
  std::atomic<int> within{0}, velocity{0}, handoff{0}, geometry{0};
  std::atomic<bool> physical_scale_seen{false};
  mutable std::mutex mutex;
  std::set<std::thread::id> penalty_threads;
  size_t threadCount() const {
    std::lock_guard<std::mutex> lock(mutex);
    return penalty_threads.size();
  }
};
// A deliberately different, thread-safe model identity. Gradients and physical
// acceptance delegate to the stable fixed-wing model; its audit metric is a
// unique marker so hidden whole-flight fallback to the default cannot pass.
class CountingModel final : public md::VehicleModel {
 public:
  CountingModel(md::Parameters parameters, std::shared_ptr<Observations> stats,
                double marker, bool valid = true)
      : delegate_(parameters), stats_(std::move(stats)), marker_(marker), valid_(valid) {}
  bool valid() const override { return valid_ && delegate_.valid(); }
  md::VehicleLimits limits() const override {
    ++stats_->limits; return delegate_.limits();
  }
  md::Evaluation evaluate(const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                          const Eigen::Vector3d &a) const override {
    ++stats_->evaluate; return delegate_.evaluate(p, v, a);
  }
  md::CostGradient penalty(const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                           const Eigen::Vector3d &a, double weight) const override {
    ++stats_->penalty;
    if (p.x() > 1000.0 && v.norm() > 100.0) stats_->physical_scale_seen = true;
    { std::lock_guard<std::mutex> lock(stats_->mutex);
      stats_->penalty_threads.insert(std::this_thread::get_id()); }
    return delegate_.penalty(p, v, a, weight);
  }
  md::EnvelopeDomain domain(const md::Evaluation &e) const override {
    ++stats_->domain; return delegate_.domain(e);
  }
  double utilization(const md::Evaluation &, md::EnvelopeLimit *limit) const override {
    ++stats_->utilization;
    if (limit) *limit = md::EnvelopeLimit::None;
    return marker_;
  }
  bool withinEnvelope(const md::Evaluation &e) const override {
    ++stats_->within; return delegate_.withinEnvelope(e);
  }
  md::HandoffVerdict velocityVerdict(const Eigen::Vector3d &v, double floor,
                                    double maximum) const override {
    ++stats_->velocity; return delegate_.velocityVerdict(v, floor, maximum);
  }
  md::HandoffVerdict handoffVerdict(const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                                   const Eigen::Vector3d &a, double floor,
                                   double maximum) const override {
    ++stats_->handoff; return delegate_.handoffVerdict(p, v, a, floor, maximum);
  }
  md::PlanningGeometry planningGeometry(double cruise, double initial,
                                        double margin) const override {
    ++stats_->geometry; return delegate_.planningGeometry(cruise, initial, margin);
  }
  // No fixedWingTransitionParameters capability: the general cruise path
  // must not need the fixed-wing transition law.
 private:
  const md::FixedWingModel delegate_;
  const std::shared_ptr<Observations> stats_;
  const double marker_;
  const bool valid_;
};

grid_map_msgs::msg::GridMap::SharedPtr terrain() {
  auto map = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr int rows = 180, cols = 150;
  map->info.resolution = 2.0; map->info.length_x = 360.0; map->info.length_y = 300.0;
  map->info.pose.position.x = 180.0; map->info.pose.position.y = 150.0;
  map->info.pose.orientation.w = 1.0; map->layers = {"elevation"}; map->data.resize(1);
  auto &layer = map->data.front(); layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index"; layer.layout.dim[0].size = cols;
  layer.layout.dim[0].stride = cols * rows; layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = rows; layer.layout.dim[1].stride = rows;
  layer.data.assign(rows * cols, 2.0f); return map;
}
path_manager::MissionPlanRequest mission() {
  path_manager::MissionPlanRequest req;
  req.head.src = path_manager::StartStateSource::STATED_VECTOR;
  req.head.pos_u = Eigen::Vector3d(30.0, 150.0, 4.0);
  req.head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
  req.start_altitude = path_manager::StartAltitude::COMMAND_AGL;
  req.waypoints = {Eigen::Vector3d(330.0, 150.0, 4.0)};
  req.tail = ego_planner::TailBoundary::pinned(req.head.vel_u, Eigen::Vector3d::Zero());
  return req;
}
std::vector<double> coefficients(const poly_traj::Trajectory &trajectory) {
  std::vector<double> result;
  for (const auto &piece : trajectory) {
    result.push_back(piece.getDuration()); const auto &m = piece.getCoeffMat();
    result.insert(result.end(), m.data(), m.data() + m.size());
  }
  return result;
}
rclcpp::Node::SharedPtr makeNode(const char *config, const std::string &name) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config});
  options.parameter_overrides({rclcpp::Parameter("chain/enable", true),
      rclcpp::Parameter("chain/segments", 3),
      rclcpp::Parameter("chain/phase/enable", false),
      rclcpp::Parameter("chain/author_from_route", true),
      rclcpp::Parameter("chain/parallel", true),
      rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
  return rclcpp::Node::make_shared(name, options);
}

template<class Target>
void rejectInvalid(Target &target, const md::VehicleModel::Ptr &invalid,
                   const std::string &where) {
  bool refused = false;
  try { target.setVehicleModel(invalid); }
  catch (const std::invalid_argument &) { refused = true; }
  check(refused, where + " rejects an invalid model before replacing the current binding");
}
void exerciseReset(const char *config) {
  auto configured_node = [&](const std::string &name, bool enabled, bool invalid_mass) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", config});
    options.parameter_overrides({
        rclcpp::Parameter("optimization/dynamics_enable", enabled),
        rclcpp::Parameter("optimization/dynamics_mass_kg", invalid_mass ? -1.0 : 1300.0),
        rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
    auto node = rclcpp::Node::make_shared(name, options);
    node->declare_parameter("enable_debug_logs", false);
    return node;
  };
  auto model = std::make_shared<CountingModel>(md::Parameters{}, std::make_shared<Observations>(), 0.2718281828459045);
  auto expect_reset_refusal = [&](ego_planner::PolyTrajOptimizer &optimizer,
                                  const std::string &description) {
    const auto identity = optimizer.vehicleModel();
    const bool enabled = optimizer.dynamicsEnabled();
    bool refused = false;
    try { optimizer.setVehicleModel(nullptr); }
    catch (const std::invalid_argument &) { refused = true; }
    check(refused, description + " rejects an invalid dormant ROS default on reset");
    check(optimizer.vehicleModel() == identity && optimizer.dynamicsEnabled() == enabled,
          description + " reset refusal preserves both custom identity and enable state");
  };
  {
    auto node = configured_node("vehicle_reset_invalid_default", true, true);
    ego_planner::PolyTrajOptimizer optimizer;
    optimizer.setParam(node);
    check(!optimizer.dynamicsEnabled() && !optimizer.vehicleModel()->valid(),
          "invalid ROS defaults initially disable dynamics while retaining the requested setting");
    optimizer.setVehicleModel(model);
    check(optimizer.dynamicsEnabled() && optimizer.vehicleModel() == model,
          "valid custom injection restores the originally requested dynamics enable after invalid defaults");
    expect_reset_refusal(optimizer, "recovered model");
  }
  {
    auto node = configured_node("vehicle_reset_custom_authority", true, true);
    ego_planner::PolyTrajOptimizer optimizer;
    optimizer.setVehicleModel(model);
    optimizer.setParam(node);
    check(optimizer.dynamicsEnabled() && optimizer.vehicleModel() == model,
          "custom model configured before invalid ROS physical settings remains authoritative and enabled");
    expect_reset_refusal(optimizer, "preconfigured custom model");
  }
  {
    auto node = configured_node("vehicle_reset_explicit_disable", false, false);
    ego_planner::PolyTrajOptimizer optimizer;
    optimizer.setVehicleModel(model);
    optimizer.setParam(node);
    check(!optimizer.dynamicsEnabled() && optimizer.vehicleModel() == model,
          "explicit ROS dynamics disable is honored with a previously injected valid model");
    optimizer.setVehicleModel(model);
    check(!optimizer.dynamicsEnabled() && optimizer.vehicleModel() == model,
          "valid custom reinjection cannot override an explicit ROS dynamics disable");
    optimizer.setVehicleModel(nullptr);
    check(!optimizer.dynamicsEnabled() && optimizer.vehicleModel()->valid() &&
              optimizer.vehicleModel()->fixedWingTransitionParameters(),
          "reset to valid ROS defaults preserves the explicit dynamics disable");
  }
}

void exercise(const char *config) {
  constexpr double marker_a = 0.2718281828459045, marker_b = 0.3141592653589793;
  path_manager::MissionPlanResult retained;
  ego_planner::OptimizerSolveResult retained_solve;
  std::vector<double> flight_coefficients, solver_coefficients;
  std::weak_ptr<const md::VehicleModel> weak_a, weak_b, weak_final;
  std::shared_ptr<Observations> stats_a = std::make_shared<Observations>();
  {
    auto node = makeNode(config, "vehicle_model_external");
    auto backend = path_manager::makeRosPlanningBackend(node); auto *ros = backend.get();
    path_manager::MissionPlanner planner({}, {}, std::move(backend));
    auto manager = ros->executionManager(); // Explicit ROS adapter inspection only.
    manager->initOptimizer(); // The facade deliberately initializes lazily.
    const auto defaults = manager->vehicleModel();
    check(defaults && defaults->valid() && defaults->fixedWingTransitionParameters(),
          "installed facade constructs a valid default fixed-wing model");
    if (!defaults || !defaults->fixedWingTransitionParameters()) return;
    const md::Parameters parameters = *defaults->fixedWingTransitionParameters();
    auto mock = std::make_shared<CountingModel>(parameters, stats_a, marker_a);
    const auto *identity_a = mock.get(); weak_a = mock;
    planner.setVehicleModel(mock); mock.reset();
    check(!weak_a.expired() && manager->vehicleModel().get() == identity_a,
          "facade and manager retain the exact model after caller ownership is released");
    check(manager->vehicleModel()->fixedWingTransitionParameters() == nullptr,
          "generic model does not inherit a fixed-wing transition capability");
    auto invalid = std::make_shared<CountingModel>(parameters, std::make_shared<Observations>(), 0.0, false);
    rejectInvalid(planner, invalid, "facade");
    check(manager->vehicleModel().get() == identity_a, "facade refusal preserves the prior identity");
    rejectInvalid(*manager, invalid, "manager");
    check(manager->vehicleModel().get() == identity_a, "manager refusal preserves the prior identity");
    manager->initOptimizer(true);
    check(manager->vehicleModel().get() == identity_a, "forced optimizer setup preserves the injected model identity");
    ros->setTerrainData(terrain()); planner.setRiskZones({});

    auto request = mission();
    auto absolute_head = request.head; absolute_head.pos_u.z() += 2.0;
    const int limits_before = stats_a->limits;
    const int penalties_before = stats_a->penalty;
    const auto route = manager->planCommittedRoute(absolute_head, request.waypoints);
    check(route.has_value(), "real route search succeeds with the injected model");
    check(stats_a->limits > limits_before && stats_a->penalty == penalties_before,
          "front-end route preparation queries injected model limits without running optimizer cost");

    auto worker = manager->makeConfiguredOptimizer();
    check(worker->vehicleModel().get() == identity_a, "new independent optimizer shares the exact manager model");
    rejectInvalid(*worker, invalid, "optimizer");
    check(worker->vehicleModel().get() == identity_a, "optimizer refusal preserves the prior identity");
    ego_planner::OptimizerSolveRequest solve;
    solve.path = {Eigen::Vector3d(30.0, 150.0, 6.0), Eigen::Vector3d(110.0, 150.0, 6.0)};
    solve.waypoints = {solve.path.back()}; solve.cap_ref = {6.0, 6.0}; solve.edge_leg = {0};
    solve.head_pos = solve.path.front(); solve.head_vel = Eigen::Vector3d(2.0, 0.0, 0.0);
    solve.tail = ego_planner::TailBoundary::pinned(solve.head_vel, Eigen::Vector3d::Zero());
    solve.band = {5.5, 6.4, 1000.0}; solve.max_vel = 2.0;
    retained_solve = worker->solve(solve);
    check(retained_solve.hasTrajectory(), "configured independent worker solves with the custom model");
    check(stats_a->penalty > penalties_before && stats_a->evaluate > 0 && stats_a->domain > 0 &&
              stats_a->utilization > 0 && stats_a->physical_scale_seen,
          "solver calls the custom gradient and audit in physical SI coordinates");
    check(retained_solve.hasTrajectory() && retained_solve.envelope_peak == marker_a,
          "owned optimizer result carries the injected model's audit metric");
    check(solve.path.size() == 2 && solve.cap_ref.size() == 2,
          "model injection preserves the owning solve request contract");
    solver_coefficients = coefficients(retained_solve.trajectory);

    const int geometry_before = stats_a->geometry;
    retained = planner.plan(request);
    check(retained.hasTrajectory(), "public facade plans a complete mission with a generic model: " + retained.status.detail);
    check(retained.evaluation.evaluated && retained.phases.size() == 3,
          "custom model participates in the real parallel three-span plan");
    check(retained.evaluation.evaluated && retained.evaluation.util_peak == marker_a,
          "whole-flight final evaluation uses the same injected model's metric");
    std::cout << "[OBSERVED] geometry_delta=" << (stats_a->geometry - geometry_before)
              << " velocity_calls=" << stats_a->velocity << " handoff_calls=" << stats_a->handoff << std::endl;
    check(stats_a->geometry > geometry_before && stats_a->velocity > 0,
          "chain authoring and velocity admission query the same custom model");
    const int handoff_before = stats_a->handoff;
    const auto pva_problem = manager->pvaEnvelopeProblem(
        absolute_head.pos_u, absolute_head.vel_u, absolute_head.acc_u);
    check(pva_problem.empty() && stats_a->handoff > handoff_before,
          "an explicit valid PVA admission calls the same custom model's handoff verdict");
    check(stats_a->threadCount() > 1, "parallel segment optimizers call the shared model from multiple threads");
    flight_coefficients = coefficients(retained.trajectory);

    auto transition_request = request;
    transition_request.head.vel_u = Eigen::Vector3d(0.8, 0.0, 0.0);
    transition_request.head.acc_prescribed = false;
    const auto unsupported = planner.plan(transition_request);
    check(!unsupported.hasTrajectory() && unsupported.trajectory.getPieceNum() == 0 &&
              unsupported.reference_trajectory.getPieceNum() == 0 && !unsupported.evaluation.evaluated &&
              unsupported.status.detail.find("does not support the fixed-wing 3DOF transition generator") != std::string::npos,
          "a generic model refuses an unsupported transition without partial flight products");

    const int old_penalties = stats_a->penalty;
    auto stats_b = std::make_shared<Observations>();
    auto replacement = std::make_shared<CountingModel>(parameters, stats_b, marker_b);
    const auto *identity_b = replacement.get(); weak_b = replacement;
    planner.setVehicleModel(replacement); replacement.reset();
    check(manager->vehicleModel().get() == identity_b && worker->vehicleModel().get() == identity_a,
          "serial replacement updates the manager while a completed pre-existing worker retains its model");
    check(!weak_a.expired(), "pre-existing worker owns its old model after manager replacement");
    worker.reset();
    check(weak_a.expired(), "old model is released after its last completed worker is destroyed");
    {
      auto next_worker = manager->makeConfiguredOptimizer();
      check(next_worker->vehicleModel().get() == identity_b, "workers created after replacement receive the new identity");
    }
    const auto replaced_result = planner.plan(request);
    check(replaced_result.hasTrajectory() && replaced_result.evaluation.evaluated &&
              replaced_result.evaluation.util_peak == marker_b,
          "next mission uses the replacement model throughout final evaluation");
    check(stats_a->penalty == old_penalties && stats_b->penalty > 0,
          "new solves stop calling the old model and use the replacement cost");
    planner.setVehicleModel(nullptr);
    check(weak_b.expired() && manager->vehicleModel() &&
              manager->vehicleModel()->fixedWingTransitionParameters(),
          "null restores the configured fixed-wing model and releases the replacement");
    const auto restored = planner.plan(request);
    check(restored.hasTrajectory(), "default model remains usable after reset");
    auto final_model = std::make_shared<CountingModel>(parameters, std::make_shared<Observations>(), marker_a);
    weak_final = final_model; planner.setVehicleModel(final_model); final_model.reset();
    check(!weak_final.expired(), "current model remains owned until planner destruction");
    check(coefficients(retained.trajectory) == flight_coefficients &&
              coefficients(retained_solve.trajectory) == solver_coefficients,
          "later models and solves do not rewrite retained trajectory storage");
    manager.reset();
  }
  check(weak_final.expired(), "planner destruction releases its current model without a cycle");
  check(retained.hasTrajectory() && retained_solve.hasTrajectory() &&
            coefficients(retained.trajectory) == flight_coefficients &&
            coefficients(retained_solve.trajectory) == solver_coefficients,
        "mission and optimizer outputs remain evaluable after model, planner and node destruction");
  if (retained.hasTrajectory()) check(retained.trajectory.getPos(0.5 * retained.trajectory.getTotalDuration()).allFinite(),
                                    "retained mission evaluates without a live vehicle model");
}
} // namespace
int main(int argc, char **argv) {
  if (argc != 2) { std::cerr << "usage: vehicle_model_consumer <optimizer_params.yaml>\n"; return 2; }
  rclcpp::init(argc, argv);
  try { exerciseReset(argv[1]); exercise(argv[1]); }
  catch (const std::exception &e) { check(false, std::string("unexpected exception: ") + e.what()); }
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
