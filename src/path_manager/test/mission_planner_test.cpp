// The public whole-mission API, without an FSM or direct access to planning
// internals. Synthetic inputs keep this independent of maps and ROS peers.
#include "path_manager/mission_planner.h"
#include "path_manager/ros_planning_backend.h"
#include "path_manager/ros_terrain_grid.h"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

grid_map_msgs::msg::GridMap::SharedPtr flatTerrain(float height = 2.0f)
{
  auto map = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr int rows = 180, cols = 150;
  map->info.resolution = 2.0;
  map->info.length_x = 360.0;
  map->info.length_y = 300.0;
  map->info.pose.position.x = 180.0;
  map->info.pose.position.y = 150.0;
  map->info.pose.orientation.w = 1.0;
  map->layers = {"elevation"};
  map->data.resize(1);
  auto &layer = map->data.front();
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index";
  layer.layout.dim[0].size = cols;
  layer.layout.dim[0].stride = cols * rows;
  layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = rows;
  layer.layout.dim[1].stride = rows;
  layer.data.assign(rows * cols, height);
  return map;
}

path_manager::MissionPlanRequest mission()
{
  path_manager::MissionPlanRequest request;
  request.head.src = path_manager::StartStateSource::STATED_VECTOR;
  request.head.pos_u = Eigen::Vector3d(30.0, 150.0, 4.0);
  request.head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
  request.head.acc_u.setZero();
  request.start_altitude = path_manager::StartAltitude::COMMAND_AGL;
  request.waypoints = {Eigen::Vector3d(330.0, 150.0, 4.0)};
  request.tail = ego_planner::TailBoundary::pinned(
      request.head.vel_u, Eigen::Vector3d::Zero());
  return request;
}

std::vector<double> coefficients(const poly_traj::Trajectory &trajectory)
{
  std::vector<double> values;
  for (const auto &piece : trajectory) {
    values.push_back(piece.getDuration());
    const auto &matrix = piece.getCoeffMat();
    values.insert(values.end(), matrix.data(), matrix.data() + matrix.size());
  }
  return values;
}

void checkRefusal(const path_manager::MissionPlanResult &result,
                  const std::string &description)
{
  check(result.status.outcome == path_manager::PlanOutcome::FAILED &&
            !result.hasTrajectory() && !result.status.detail.empty(),
        description + " returns a failed result with a reason");
  check(result.trajectory.getPieceNum() == 0 &&
            result.reference_trajectory.getPieceNum() == 0 &&
            result.phases.empty() && !result.evaluation.evaluated &&
            result.trajectory_id == 0,
        description + " does not return the previous flight or evaluation");
}

void exercise(const char *config_path, bool chained)
{
  const std::string mode = chained ? "chain" : "direct";
  path_manager::MissionPlanResult retained;
  std::vector<double> final_coefficients, reference_coefficients;
  Eigen::Vector3d midpoint = Eigen::Vector3d::Zero();
  double midpoint_time = 0.0;
  size_t phase_count = 0;
  double envelope_peak = 0.0;
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", config_path});
    options.parameter_overrides({
        rclcpp::Parameter("drone_id", 7),
        rclcpp::Parameter("chain/enable", chained),
        rclcpp::Parameter("chain/segments", 3),
        rclcpp::Parameter("chain/phase/enable", false),
        rclcpp::Parameter("chain/author_from_route", true),
        rclcpp::Parameter("chain/parallel", true),
        rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
    path_manager::RosPlanningOptions runtime;
    runtime.node_name = "mission_api_" + mode;
    runtime.node_options = options;
    path_manager::PlanningEnvironment environment;
    environment.terrain = std::move(*path_manager::decodeTerrainGrid(flatTerrain()).terrain);
    path_manager::MissionPlanner planner({}, std::move(environment),
        path_manager::makeRosPlanningBackend(std::move(runtime)));
    check(planner.setRiskZones({}), mode + " accepts an empty risk-zone set");

    auto request = mission();
    retained = planner.plan(request);
    check(retained.hasTrajectory(), mode + " plans through one public call: " +
                                       retained.status.detail);
    if (!retained.hasTrajectory()) return;
    check(retained.trajectory.getPieceNum() > 0 &&
              retained.reference_trajectory.getPieceNum() > 0,
          mode + " returns final and reference trajectories");
    check(retained.drone_id == 7 && retained.trajectory_id > 0 &&
              retained.start_time > 0.0 && retained.reference_start_time > 0.0 &&
              retained.environment_generation > 0,
          mode + " returns the existing identity, times and environment revision");
    const Eigen::Vector3d start(30.0, 150.0, 6.0);
    const Eigen::Vector3d goal(330.0, 150.0, 6.0);
    check((retained.trajectory.getJuncPos(0) - start).norm() < 1e-7 &&
              (retained.trajectory.getJuncPos(
                   retained.trajectory.getPieceNum()) - goal).norm() < 1e-7,
          mode + " resolves start and waypoint AGL against the supplied terrain");
    check((retained.trajectory.getJuncVel(0) - request.head.vel_u).norm() < 1e-7 &&
              retained.trajectory.getJuncAcc(0).norm() < 1e-7 &&
              (retained.trajectory.getJuncVel(
                   retained.trajectory.getPieceNum()) - request.tail.vel).norm() < 1e-7,
          mode + " preserves the supplied initial PVA and final velocity");
    check(request.head.pos_u.z() == 4.0 && request.waypoints.back().z() == 4.0,
          mode + " leaves the caller's mission input unchanged");
    if (chained) {
      check(retained.evaluation.evaluated && retained.phases.size() == 3,
            "chain returns the actual three-span whole-flight evaluation");
      if (!retained.phases.empty()) {
        check(std::abs(retained.phases.back().t_end -
                       retained.trajectory.getTotalDuration()) < 1e-8,
              "phase spans cover the returned flight");
      }
    } else {
      check(!retained.evaluation.evaluated && retained.phases.empty(),
            "direct result does not claim an unavailable whole-flight audit");
    }
    final_coefficients = coefficients(retained.trajectory);
    reference_coefficients = coefficients(retained.reference_trajectory);
    midpoint_time = 0.5 * retained.trajectory.getTotalDuration();
    midpoint = retained.trajectory.getPos(midpoint_time);
    phase_count = retained.phases.size();
    envelope_peak = retained.evaluation.util_peak;

    auto unspecified = request;
    unspecified.head.src = path_manager::StartStateSource::UNSPECIFIED;
    const auto no_head = planner.plan(unspecified);
    checkRefusal(no_head, mode + " unspecified initial state");
    check(no_head.status.reason == path_manager::PlanReason::INITIAL_STATE_UNSPECIFIED,
          mode + " preserves the unspecified-state reason");

    auto unsupported = request;
    unsupported.head.vel_u = Eigen::Vector3d(4.0, 0.0, 0.0);
    const auto bad_head = planner.plan(unsupported);
    checkRefusal(bad_head, mode + " unsupported 400 m/s initial state");
    check(bad_head.status.reason == path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
          mode + " preserves the unsupported-state reason");

    auto invalid_tail = request;
    invalid_tail.tail = ego_planner::TailBoundary::pinned(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    checkRefusal(planner.plan(invalid_tail), mode + " invalid final velocity");

    // An already resolved or trajectory-derived start must not receive the
    // terrain offset again. Refusals above must also leave the API reusable.
    auto absolute = request;
    absolute.start_altitude = path_manager::StartAltitude::ABSOLUTE;
    absolute.head.pos_u = start;
    const auto again = planner.plan(absolute);
    check(again.hasTrajectory() && again.trajectory_id > retained.trajectory_id,
          mode + " successfully retries after refusals with a newer identity");
    if (again.hasTrajectory()) {
      check((again.trajectory.getJuncPos(0) - start).norm() < 1e-7,
            mode + " keeps an absolute head absolute");
    }

    check(planner.setTerrain(std::move(*path_manager::decodeTerrainGrid(flatTerrain(3.0f)).terrain)),
          mode + " accepts owned terrain refresh");
    check(coefficients(retained.trajectory) == final_coefficients &&
              coefficients(retained.reference_trajectory) == reference_coefficients &&
              retained.phases.size() == phase_count &&
              retained.evaluation.util_peak == envelope_peak,
          mode + " later plans and environment changes cannot rewrite a retained result");
  }
  check(retained.hasTrajectory() &&
            coefficients(retained.trajectory) == final_coefficients &&
            coefficients(retained.reference_trajectory) == reference_coefficients &&
            (retained.trajectory.getPos(midpoint_time) - midpoint).norm() == 0.0 &&
            retained.phases.size() == phase_count &&
            retained.evaluation.util_peak == envelope_peak,
        mode + " result remains usable after planner and node destruction");
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: mission_planner_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  exercise(argv[1], false);
  exercise(argv[1], true);
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures
            << " failed check(s)\n";
  return failures ? 1 : 0;
}
