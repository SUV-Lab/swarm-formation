// Run with the existing optimizer YAML. No FSM, executor, mission topic or
// direct planner/optimizer sequencing is needed to obtain a whole flight.
#include "path_manager/mission_planner.h"
#include "path_manager/ros_planning_backend.h"

#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <iostream>
#include <utility>

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: mission_planner_example <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  int exit_code = 0;
  {
    // Select the retained ROS implementation explicitly; the caller creates
    // no Node. The generic settings/environment/request API has no ROS types.
    path_manager::RosPlanningOptions runtime;
    runtime.node_name = "mission_planner_example";
    runtime.node_options.arguments({"--ros-args", "--params-file", argv[1]});
    path_manager::PlanSettings settings;  // sparse typed overrides, if needed

    // A 36 km x 30 km flat terrain at 200 m ASL. Existing frame units are
    // retained: 1 u = 100 m for the supplied production configuration.
    path_manager::TerrainGrid terrain;
    terrain.rows = 180;
    terrain.cols = 150;
    terrain.resolution = 2.0;
    terrain.length_x = 360.0;
    terrain.length_y = 300.0;
    terrain.center_x = 180.0;
    terrain.center_y = 150.0;
    terrain.elevation.assign(150 * 180, 2.0f);
    path_manager::PlanningEnvironment environment;
    environment.terrain = std::move(terrain);
    // Missing risk_zones/dynamic_obstacles fields preserve configured state.
    path_manager::MissionPlanner planner(
        std::move(settings), std::move(environment),
        path_manager::makeRosPlanningBackend(std::move(runtime)));

    path_manager::MissionPlanRequest mission;
    mission.head.src = path_manager::StartStateSource::STATED_VECTOR;
    mission.head.pos_u = Eigen::Vector3d(30.0, 150.0, 4.0);
    mission.head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
    mission.start_altitude = path_manager::StartAltitude::COMMAND_AGL;
    mission.waypoints = {Eigen::Vector3d(330.0, 150.0, 4.0)};
    mission.tail = ego_planner::TailBoundary::pinned(
        mission.head.vel_u, Eigen::Vector3d::Zero());

    const path_manager::MissionPlanResult result = planner.plan(mission);
    if (!result.hasTrajectory()) {
      std::cerr << "Planning failed: " << result.status.detail << '\n';
      exit_code = 1;
    } else {
      const auto &flight = result.trajectory;
      std::cout << std::fixed << std::setprecision(6)
                << "trajectory_id=" << result.trajectory_id
                << " pieces=" << flight.getPieceNum()
                << " duration_s=" << flight.getTotalDuration()
                << " phases=" << result.phases.size()
                << " whole_flight_evaluated=" << result.evaluation.evaluated
                << '\n';
      for (double fraction : {0.0, 0.5, 1.0}) {
        const double t = fraction * flight.getTotalDuration();
        std::cout << "t=" << t << " pos_u=" << flight.getPos(t).transpose()
                  << " vel_u_s=" << flight.getVel(t).transpose()
                  << " acc_u_s2=" << flight.getAcc(t).transpose() << '\n';
      }
      if (result.status.outcome == path_manager::PlanOutcome::DEGRADED)
        std::cout << "Degraded: " << result.status.detail << '\n';
    }
  }
  rclcpp::shutdown();
  return exit_code;
}
