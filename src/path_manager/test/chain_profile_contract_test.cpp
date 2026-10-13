// Profile semantics must be identical before worker creation, in real workers
// and after a refused request. Drive the installed public mission boundary.
#include "path_manager/mission_planner.h"
#include "path_manager/ros_planning_backend.h"
#include "path_manager/path_manager.h"

#include <iostream>
#include <string>
#include <vector>

namespace pm = path_manager;
namespace {
int failures = 0;
void check(bool condition, const std::string &label)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << label << '\n';
  if (!condition) ++failures;
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
void exercise(const char *config)
{
  pm::RosPlanningOptions options;
  options.node_name = "chain_profile_contract_test";
  options.node_options.arguments({"--ros-args", "--params-file", config});
  pm::PlanSettings settings;
  settings.attach_visualization = false;
  settings.parameters["chain/enable"] = true;
  settings.parameters["chain/segments"] = int64_t(0);
  settings.parameters["chain/auto_pieces_per_segment"] = int64_t(35);
  settings.parameters["chain/author_from_route"] = true;
  settings.parameters["chain/parallel"] = false;
  settings.parameters["chain/phase/enable"] = false;
  settings.parameters["chain/terminal/enable"] = false;
  settings.parameters["optimization/risk_parallel_threads"] = int64_t(1);
  pm::TerrainGrid terrain;
  terrain.rows = 180; terrain.cols = 150; terrain.resolution = 2.0;
  terrain.length_x = 360.0; terrain.length_y = 300.0;
  terrain.center_x = 180.0; terrain.center_y = 150.0;
  terrain.elevation.assign(180 * 150, 2.0f);
  pm::PlanningEnvironment environment;
  environment.terrain = std::move(terrain);
  environment.risk_zones = std::vector<pm::RiskZone>{};
  auto backend = pm::makeRosPlanningBackend(std::move(options));
  auto *adapter = backend.get();
  pm::MissionPlanner planner(std::move(settings), std::move(environment),
                             std::move(backend));
  const auto node = adapter->node();
  const auto manager = adapter->executionManager();
  const auto set = [&](const std::string &name, auto value) {
    if (!node->has_parameter(name)) node->declare_parameter(name, value);
    const auto result = node->set_parameter(rclcpp::Parameter(name, value));
    check(result.successful, "test setup accepts " + name);
  };
  const auto profile = [&](const std::string &name,
                           const std::vector<std::string> &values) {
    set(name, values);
  };
  const auto clear = [&] {
    for (int i = 1; i <= 4; ++i)
      profile("chain/seg" + std::to_string(i) + "/params", {});
    for (const char *phase : {"departure", "cruise", "arrival"})
      profile(std::string("chain/phase/") + phase + "/params", {});
  };
  clear();
  pm::MissionPlanRequest request;
  request.head.src = pm::StartStateSource::STATED_VECTOR;
  request.head.pos_u = Eigen::Vector3d(30.0, 150.0, 4.0);
  request.head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
  request.start_altitude = pm::StartAltitude::COMMAND_AGL;
  request.waypoints = {Eigen::Vector3d(330.0, 150.0, 4.0)};
  request.tail = ego_planner::TailBoundary::pinned(
      request.head.vel_u, Eigen::Vector3d::Zero());
  const auto ordinary = planner.plan(request);
  check(ordinary.hasTrajectory(), "ordinary short mission succeeds");
  if (!ordinary.hasTrajectory()) return;
  const auto ordinary_values = coefficients(ordinary.trajectory);
  const auto ordinary_reference = coefficients(ordinary.reference_trajectory);
  const auto same = [&](const pm::MissionPlanResult &result, const std::string &label) {
    check(result.hasTrajectory() && coefficients(result.trajectory) == ordinary_values &&
              coefficients(result.reference_trajectory) == ordinary_reference,
          label);
  };
  profile("chain/seg2/params", {"optimization/max_vel=0.5", "optimization/max_vel=2.0"});
  same(planner.plan(request), "early fallback honors the last duplicate value exactly");
  profile("chain/seg2/params", {"optimization/max_vel=nan", "optimization/max_vel=2.0"});
  same(planner.plan(request), "superseded malformed value is not an active hard requirement");
  clear();
  profile("chain/phase/departure/params", {"optimization/max_vel=0.5"});
  profile("chain/phase/cruise/params", {"optimization/max_vel=inf"});
  profile("chain/phase/arrival/params", {"optimization/max_vel=nan"});
  same(planner.plan(request), "disabled phase profiles cannot change or refuse fallback");
  clear();
  set("chain/phase/enable", true);
  profile("chain/phase/arrival/params", {"optimization/max_vel=0.5"});
  profile("chain/seg2/params", {"optimization/max_vel=2.0"});
  same(planner.plan(request), "segment hard maximum overrides the phase value before fallback min");
  profile("chain/phase/arrival/params", {"optimization/max_vel=nan"});
  same(planner.plan(request), "valid segment requirement supersedes malformed phase requirement");
  profile("chain/phase/arrival/params", {"optimization/max_vel=2.0"});
  profile("chain/seg2/params", {"optimization/max_vel=nan"});
  const auto invalid_segment = planner.plan(request);
  check(!invalid_segment.hasTrajectory() &&
            invalid_segment.status.detail.find("invalid hard profile seg2") != std::string::npos,
        "malformed segment requirement cannot be replaced by its valid phase value");
  profile("chain/phase/arrival/params", {"optimization/max_vel=0.5", "optimization/max_vel=2.0"});
  profile("chain/seg2/params", {});
  same(planner.plan(request), "phase list duplicates also use their final value");
  profile("chain/phase/cruise/params", {"optimization/max_vel=nan"});
  same(planner.plan(request), "two-segment fallback does not consume an unused cruise profile");
  clear();
  set("chain/phase/enable", false);
  profile("chain/seg2/params", {"optimization/max_vel=1.8"});
  const auto capped = planner.plan(request);
  check(capped.hasTrajectory() && capped.status.detail.find("hard limits preserved") != std::string::npos &&
            capped.trajectory.getVel(capped.trajectory.getTotalDuration() * 0.5).norm() < 1.9,
        "a real effective hard ceiling still reaches the fallback optimizer");

  // Exercise the actual four-to-three merge after either serial or parallel
  // workers. Equivalent effective worker limits must produce the same merge,
  // whether a phase default was superseded or a mission default was implicit.
  for (const bool parallel : {false, true}) {
    const std::string mode = parallel ? "parallel merge" : "serial merge";
    clear();
    set("chain/segments", 4);
    set("chain/phase/enable", true);
    set("chain/parallel", parallel);
    const auto shared_max_vel = node->get_parameter("optimization/max_vel").as_double();
    const auto shared_threads = node->get_parameter("optimization/risk_parallel_threads").as_int();
    const auto merge = [&] {
      set("chain/jitter/fail_segment", 2);
      return planner.plan(request);
    };
    const auto same_merge = [&](const pm::MissionPlanResult &result,
                                const pm::MissionPlanResult &reference,
                                const std::string &label) {
      check(result.hasTrajectory() && reference.hasTrajectory() &&
                result.phases.size() == 3 && reference.phases.size() == 3 &&
                coefficients(result.trajectory) == coefficients(reference.trajectory) &&
                coefficients(result.reference_trajectory) == coefficients(reference.reference_trajectory),
            mode + " " + label);
    };

    profile("chain/phase/cruise/params", {"optimization/max_vel=2.0"});
    profile("chain/seg2/params", {"optimization/max_vel=2.0"});
    profile("chain/seg3/params", {"optimization/max_vel=2.0"});
    const auto merge_reference = merge();
    check(merge_reference.hasTrajectory() && merge_reference.phases.size() == 3,
          mode + " rescues one failed cruise worker with a real four-to-three span merge");
    profile("chain/phase/cruise/params", {"optimization/max_vel=0.5"});
    same_merge(merge(), merge_reference,
               "keeps explicit segment ceilings over a valid lower-precedence phase");
    profile("chain/phase/cruise/params",
            {"optimization/max_vel=0.5", "optimization/max_vel=nan"});
    same_merge(merge(), merge_reference,
               "keeps effective segment ceilings over an invalid lower-precedence phase");

    // The untouched neighbor inherits optimization/max_vel=2.0. Omitting
    // its override cannot let the other worker's 4.0 ceiling relax that
    // portion of the merged trajectory.
    profile("chain/phase/cruise/params", {});
    profile("chain/seg2/params", {"optimization/max_vel=4.0"});
    profile("chain/seg3/params", {"optimization/max_vel=2.0"});
    const auto explicit_default = merge();
    check(explicit_default.hasTrajectory() && explicit_default.phases.size() == 3,
          mode + " merges distinct explicit worker ceilings");
    profile("chain/seg3/params", {});
    same_merge(merge(), explicit_default,
               "preserves an inherited mission ceiling exactly like its explicit value");

    profile("chain/phase/cruise/params", {"optimization/max_vel=2.0"});
    profile("chain/seg2/params", {"optimization/max_vel=2.0"});
    profile("chain/seg3/params", {"optimization/max_vel=2.0"});
    same_merge(merge(), merge_reference,
               "recovers the original complete trajectories after different retry profiles");
    check(node->get_parameter("optimization/max_vel").as_double() == shared_max_vel &&
              node->get_parameter("optimization/risk_parallel_threads").as_int() == shared_threads,
          mode + " leaves shared optimizer defaults unchanged after failed workers and retries");
  }
  set("chain/phase/enable", false);

  for (const std::string mode : {"early", "baseline", "route", "parallel"}) {
    clear();
    set("chain/segments", mode == "early" ? 0 : 3);
    set("chain/author_from_route", mode != "baseline");
    set("chain/parallel", mode == "parallel");
    const auto before = planner.plan(request);
    check(before.hasTrajectory(), mode + " normal reference succeeds");
    if (!before.hasTrajectory()) continue;
    const auto expected = coefficients(before.trajectory);
    const auto expected_reference = coefficients(before.reference_trajectory);
    const auto execution_id = manager->executionTrajectory().traj_id;
    for (const std::string value : {"nan", "inf", "-inf", "0", "-1", "1.8x", ""}) {
      // An invalid last occurrence must not resurrect its earlier valid value.
      profile("chain/seg2/params", {"optimization/max_vel=2.0", "optimization/max_vel=" + value});
      const auto refused = planner.plan(request);
      check(!refused.hasTrajectory() && refused.trajectory.getPieceNum() == 0 &&
                refused.reference_trajectory.getPieceNum() == 0 && refused.trajectory_id == 0 &&
                refused.status.detail.find("invalid hard profile") != std::string::npos,
            mode + " refuses malformed hard maximum " + value + " with an owned empty result");
      check(manager->executionTrajectory().traj_id == execution_id &&
                manager->executionTrajectory().duration > 0.0 &&
                coefficients(manager->executionTrajectory().traj) == expected,
            mode + " malformed profile preserves existing execution");
    }
    profile("chain/seg2/params", {});
    const auto recovered = planner.plan(request);
    check(recovered.hasTrajectory() && coefficients(recovered.trajectory) == expected &&
              coefficients(recovered.reference_trajectory) == expected_reference,
          mode + " normal request after refusals preserves all coefficients and times");
  }
}
}  // namespace
int main(int argc, char **argv)
{
  if (argc < 2) return 2;
  rclcpp::init(argc, argv);
  try { exercise(argv[1]); }
  catch (const std::exception &error) {
    std::cerr << "unexpected exception: " << error.what() << '\n';
    ++failures;
  }
  rclcpp::shutdown();
  std::cout << (failures == 0 ? "PASS" : "FAIL") << ": " << failures << " failed checks\n";
  return failures == 0 ? 0 : 1;
}
