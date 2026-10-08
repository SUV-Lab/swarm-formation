// Temporary external consumer. The only mode override disables the chain;
// default mode uses the installed YAML unchanged, including auto-N fallback.
#include <path_manager/mission_planner.h>
#include <path_manager/ros_planning_backend.h>
#include <path_manager/ros_terrain_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

void jsonString(std::ostream &out, const std::string &value)
{
  out << '"';
  for (unsigned char ch : value) {
    if (ch == '"' || ch == '\\') out << '\\' << ch;
    else if (ch < 0x20) {
      const char hex[] = "0123456789abcdef";
      out << "\\u00" << hex[ch >> 4] << hex[ch & 0x0f];
    } else out << ch;
  }
  out << '"';
}
void emitVector(std::ostream &out, const Eigen::Vector3d &v)
{
  out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
void trajectory(std::ostream &out, const poly_traj::Trajectory &traj)
{
  out << "{\"piece_count\":" << traj.getPieceNum()
      << ",\"duration\":" << traj.getTotalDuration() << ",\"pieces\":[";
  bool first = true;
  for (const auto &piece : traj) {
    if (!first) out << ',';
    first = false;
    out << "{\"duration\":" << piece.getDuration() << ",\"coefficients\":[";
    const auto &c = piece.getCoeffMat();
    for (int row = 0; row < c.rows(); ++row) {
      if (row) out << ',';
      out << '[';
      for (int col = 0; col < c.cols(); ++col) {
        if (col) out << ',';
        out << c(row, col);
      }
      out << ']';
    }
    out << "]}";
  }
  out << "],\"samples\":[";
  first = true;
  if (traj.getPieceNum() > 0) for (double fraction : {0.0, 0.5, 1.0}) {
    if (!first) out << ',';
    first = false;
    const double t = fraction * traj.getTotalDuration();
    out << "{\"t\":" << t << ",\"position\":";
    emitVector(out, traj.getPos(t));
    out << ",\"velocity\":";
    emitVector(out, traj.getVel(t));
    out << ",\"acceleration\":";
    emitVector(out, traj.getAcc(t));
    out << '}';
  }
  out << "]}";
}

int main(int argc, char **argv)
{
  if (argc != 4 || (std::string(argv[2]) != "direct" && std::string(argv[2]) != "default")) {
    std::cerr << "usage: exact_mission_consumer <optimizer_params.yaml> <direct|default> <output.json>\n";
    return 2;
  }
  const std::string mode(argv[2]);
  rclcpp::init(argc, argv);
  int rc = 0;
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", argv[1]});
    if (mode == "direct")
      options.parameter_overrides({rclcpp::Parameter("chain/enable", false)});
    auto node = rclcpp::Node::make_shared("exact_mission_" + mode, options);
    path_manager::MissionPlanner planner({}, {}, path_manager::makeRosPlanningBackend(node));
    auto terrain = std::make_shared<grid_map_msgs::msg::GridMap>();
    terrain->info.resolution = 2.0;
    terrain->info.length_x = 360.0;
    terrain->info.length_y = 300.0;
    terrain->info.pose.position.x = 180.0;
    terrain->info.pose.position.y = 150.0;
    terrain->info.pose.orientation.w = 1.0;
    terrain->layers = {"elevation"};
    terrain->data.resize(1);
    auto &elevation = terrain->data.front();
    elevation.layout.dim.resize(2);
    elevation.layout.dim[0].label = "column_index";
    elevation.layout.dim[0].size = 150;
    elevation.layout.dim[0].stride = 150 * 180;
    elevation.layout.dim[1].label = "row_index";
    elevation.layout.dim[1].size = 180;
    elevation.layout.dim[1].stride = 180;
    elevation.data.assign(150 * 180, 2.0f);
    auto decoded = path_manager::decodeTerrainGrid(terrain);
    if (!decoded.terrain || !planner.setTerrain(std::move(*decoded.terrain))) {
      std::cerr << "Cannot pass owned terrain input\n"; return 4;
    }
    path_manager::MissionPlanRequest mission;
    mission.head.src = path_manager::StartStateSource::STATED_VECTOR;
    mission.head.pos_u = Eigen::Vector3d(30.0, 150.0, 4.0);
    mission.head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
    mission.start_altitude = path_manager::StartAltitude::COMMAND_AGL;
    mission.waypoints = {Eigen::Vector3d(330.0, 150.0, 4.0)};
    mission.tail = ego_planner::TailBoundary::pinned(
        mission.head.vel_u, Eigen::Vector3d::Zero());
    const auto result = planner.plan(mission);
    std::ofstream out(argv[3]);
    if (!out) {
      std::cerr << "Cannot open output: " << argv[3] << '\n';
      rc = 3;
    } else {
      out << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10) << std::boolalpha;
      out << "{\"schema\":1,\"mode\":";
      jsonString(out, mode);
      out << ",\"input\":{\"start\":";
      emitVector(out, mission.head.pos_u);
      out << ",\"velocity\":";
      emitVector(out, mission.head.vel_u);
      out << ",\"goal\":";
      emitVector(out, mission.waypoints.front());
      out << ",\"terrain_resolution\":2,\"terrain_height\":2,\"terrain_shape\":[180,150],\"start_is_agl\":true,\"tail_prescribed\":true}"
          << ",\"status\":{\"outcome\":" << static_cast<int>(result.status.outcome)
          << ",\"reason\":" << static_cast<int>(result.status.reason) << ",\"detail\":";
      jsonString(out, result.status.detail);
      out << "},\"final\":";
      trajectory(out, result.trajectory);
      out << ",\"reference\":";
      trajectory(out, result.reference_trajectory);
      out << ",\"phases\":[";
      for (size_t i = 0; i < result.phases.size(); ++i) {
        if (i) out << ',';
        out << "{\"t_end\":" << result.phases[i].t_end
            << ",\"kind\":" << static_cast<int>(result.phases[i].kind)
            << ",\"name\":";
        jsonString(out, result.phases[i].name);
        out << '}';
      }
      const auto &v = result.evaluation;
      out << "],\"evaluation\":{\"evaluated\":" << v.evaluated
          << ",\"clean\":" << v.clean << ",\"underground\":" << v.underground
          << ",\"no_cruise\":" << v.no_cruise << ",\"viol_pct\":" << v.viol_pct
          << ",\"util_peak\":" << v.util_peak << ",\"envelope_bad\":" << v.envelope_bad
          << ",\"zone_hard_n\":" << v.zone_hard_n
          << ",\"zone_standoff_n\":" << v.zone_standoff_n
          << ",\"zone_soft_n\":" << v.zone_soft_n
          << ",\"zone_junction_n\":" << v.zone_junction_n
          << ",\"zone_junction_hard_n\":" << v.zone_junction_hard_n
          << ",\"policy_measurable\":" << v.policy_measurable
          << "},\"identity\":{\"trajectory_id\":" << result.trajectory_id
          << ",\"drone_id\":" << result.drone_id
          << ",\"environment_generation\":" << result.environment_generation
          << "},\"times\":{\"start\":" << result.start_time
          << ",\"reference_start\":" << result.reference_start_time << "}}\n";
      std::cout << "CAPTURE " << mode << " has_trajectory=" << result.hasTrajectory()
                << " pieces=" << result.trajectory.getPieceNum()
                << " reference_pieces=" << result.reference_trajectory.getPieceNum()
                << " evaluated=" << result.evaluation.evaluated << '\n';
      rc = result.hasTrajectory() ? 0 : 1;
    }
  }
  rclcpp::shutdown();
  return rc;
}
