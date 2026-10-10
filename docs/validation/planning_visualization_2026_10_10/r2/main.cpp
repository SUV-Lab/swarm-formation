#include <path_manager/mission_planner.h>
#include <path_manager/ros_planning_backend.h>
#include <path_manager/ros_terrain_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <mmp_mission_msgs/srv/load_map.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include "recorded_request.h"
#include "value_settings.h"

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


int main(int argc, char** argv) {
  if (argc!=5) { std::cerr << "usage: r2_consumer optimizer.yaml hardware.yaml mission.yaml result.json\n"; return 2; }
  rclcpp::init(argc,argv);
  path_manager::MissionPlanResult result;
  path_manager::MissionPlanRequest input;
  grid_map_msgs::msg::GridMap::SharedPtr accepted_map;
  mmp_mission_msgs::srv::LoadMap::Response::SharedPtr load_result;
  bool input_unchanged=false;
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args","--params-file",argv[1],"--params-file",argv[2]});
    auto node=rclcpp::Node::make_shared("r2_external_mission_consumer",options);
    const double unit_m=node->declare_parameter("manager/initial_speed_unit_m",100.0);
    const bool use_value_input = std::getenv("MMP_VALUE_INPUT") != nullptr;
    const bool use_initial_environment = use_value_input &&
        std::string(std::getenv("MMP_VALUE_INPUT")) == "initial_environment";
    path_manager::PlanSettings settings;
    std::unique_ptr<path_manager::RosPlanningBackend> backend;
    if (use_value_input) {
      settings = valueSettings(node);
      path_manager::RosPlanningOptions planning_options;
      planning_options.node_name = "r2_external_value_input_consumer";
      backend = path_manager::makeRosPlanningBackend(std::move(planning_options));
      std::cout << "OWNED_SETTING_COUNT " << settings.parameters.size() << std::endl;
    } else {
      backend = path_manager::makeRosPlanningBackend(node);
    }
    settings.attach_visualization = std::getenv("MMP_VISUALIZATION_OFF") == nullptr;
    const auto *ros_adapter = backend.get();
    std::unique_ptr<path_manager::MissionPlanner> planner;
    if (!use_initial_environment)
      planner = std::make_unique<path_manager::MissionPlanner>(
          std::move(settings), path_manager::PlanningEnvironment{}, std::move(backend));
    grid_map_msgs::msg::GridMap::SharedPtr latest;
    auto subscription=node->create_subscription<grid_map_msgs::msg::GridMap>(
        "/terrain/grid_map",rclcpp::QoS(1).reliable().transient_local(),
        [&](grid_map_msgs::msg::GridMap::SharedPtr map) { latest=std::move(map); });
    auto client=node->create_client<mmp_mission_msgs::srv::LoadMap>("/terrain/load_map");
    if(!client->wait_for_service(std::chrono::seconds(30))) throw std::runtime_error("terrain loader service unavailable");
    auto limit=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    while(!latest && rclcpp::ok() && std::chrono::steady_clock::now()<limit)
      { rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(20)); }
    if(!latest || !latest->layers.empty() || !latest->data.empty()) throw std::runtime_error("isolated terrain loader must start world=none");
    const auto initial_grid = path_manager::decodeTerrainGrid(latest);
    if (initial_grid.update != path_manager::TerrainGridUpdate::CLEAR)
      throw std::runtime_error("world=none does not decode as a pure clear update");
    if (planner) planner->clearTerrain();
    latest.reset();
    auto load=std::make_shared<mmp_mission_msgs::srv::LoadMap::Request>();
    load->world="full_map"; load->mission_path=argv[3];
    load->corridor_halfwidth_u=0.0; load->resolution_m=0.0;
    load->request_token="r2_external_library_verification";
    auto future=client->async_send_request(load);
    if(rclcpp::spin_until_future_complete(node,future,std::chrono::seconds(300))!=rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("terrain load timed out");
    load_result=future.get();
    if(!load_result->success || !load_result->mission_resolved) throw std::runtime_error("terrain load failed: "+load_result->message);
    limit=std::chrono::steady_clock::now()+std::chrono::seconds(60);
    while((!latest || latest->layers.empty()) && rclcpp::ok() && std::chrono::steady_clock::now()<limit)
      { rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(20)); }
    if(!latest || latest->layers.empty()) throw std::runtime_error("resolved terrain GridMap not received");
    accepted_map=latest;
    const auto &info=latest->info;
    const double origin_x=info.pose.position.x-info.length_x/2.0;
    const double origin_y=info.pose.position.y-info.length_y/2.0;
    if(std::abs(info.resolution*100.0-load_result->actual_resolution_m)>1e-8 ||
       std::abs(origin_x-load_result->min_bound_u[0])>1e-8 ||
       std::abs(origin_y-load_result->min_bound_u[1])>1e-8)
      throw std::runtime_error("GridMap does not match typed LoadMap response");
    auto decoded = path_manager::decodeTerrainGrid(latest);
    if (decoded.update != path_manager::TerrainGridUpdate::REPLACE || !decoded.terrain)
      throw std::runtime_error("R2 GridMap does not decode as an owned TerrainGrid");
    if (use_initial_environment) {
      path_manager::PlanningEnvironment environment;
      environment.terrain = std::move(*decoded.terrain);
      planner = std::make_unique<path_manager::MissionPlanner>(
          std::move(settings), std::move(environment), std::move(backend));
      std::cout << "OWNED_TERRAIN_AT_CONSTRUCTION true" << std::endl;
    } else {
      std::string terrain_error;
      if (!planner->setTerrain(std::move(*decoded.terrain), &terrain_error))
        throw std::runtime_error("owned R2 terrain input rejected: " + terrain_error);
    }
    if (use_value_input) {
      if (ros_adapter->node() == node)
        throw std::runtime_error("value-input backend unexpectedly reused the caller's communication node");
      std::cout << "PLANNING_NODE_CREATED_BY_BACKEND true" << std::endl;
    }
    input=recordedRequest(unit_m);
    const auto &m=load_result->mission;
    if(std::abs(m.start.x-input.head.pos_u.x())>1e-10 || std::abs(m.start.y-input.head.pos_u.y())>1e-10 ||
       std::abs(m.start.z/100.0-input.head.pos_u.z())>1e-10 ||
       std::abs(m.goal.x-input.waypoints.back().x())>1e-10 || std::abs(m.goal.y-input.waypoints.back().y())>1e-10 ||
       std::abs(m.goal.z/100.0-input.waypoints.back().z())>1e-10 ||
       m.initial_speed_mps!=200.0 || !m.use_initial_speed)
      throw std::runtime_error("resolved mission differs from recorded R2 command");
    const auto before=input;
    std::cout<<"PLANNING_R2 chain="<<ros_adapter->chainEnabled()<<" segments="<<ros_adapter->configuredSegments()<<std::endl;
    result=planner->plan(input);
    input_unchanged=(input.head.pos_u==before.head.pos_u && input.head.vel_u==before.head.vel_u && input.waypoints==before.waypoints);
  }
  // Evaluate and serialize only after MissionPlanner and its node are destroyed.
  std::ofstream out(argv[4]);
  if(!out) throw std::runtime_error("cannot write result");
  out<<std::scientific<<std::setprecision(std::numeric_limits<double>::max_digits10)<<std::boolalpha;
  out<<"{\"schema\":1,\"source\":\"public MissionPlanner only, actual Python loader, no FSM\",\"input_unchanged\":"<<input_unchanged
     <<",\"serialized_after_planner_destruction\":true,\"status\":{\"outcome\":"<<static_cast<int>(result.status.outcome)<<",\"reason\":"<<static_cast<int>(result.status.reason)<<",\"detail\":";
  jsonString(out,result.status.detail);
  out<<"},\"final\":"; trajectory(out,result.trajectory);
  out<<",\"reference\":"; trajectory(out,result.reference_trajectory);
  out<<",\"phases\":[";
  for(size_t i=0;i<result.phases.size();++i) { if(i)out<<','; out<<"{\"t_end\":"<<result.phases[i].t_end<<",\"kind\":"<<static_cast<int>(result.phases[i].kind)<<",\"name\":";jsonString(out,result.phases[i].name);out<<'}'; }
  const auto &v=result.evaluation;
  out<<"],\"evaluation\":{\"evaluated\":"<<v.evaluated<<",\"clean\":"<<v.clean<<",\"underground\":"<<v.underground<<",\"no_cruise\":"<<v.no_cruise<<",\"viol_pct\":"<<v.viol_pct<<",\"util_peak\":"<<v.util_peak<<",\"envelope_bad\":"<<v.envelope_bad<<",\"zone_hard_n\":"<<v.zone_hard_n<<",\"zone_standoff_n\":"<<v.zone_standoff_n<<",\"zone_soft_n\":"<<v.zone_soft_n<<",\"zone_junction_n\":"<<v.zone_junction_n<<",\"zone_junction_hard_n\":"<<v.zone_junction_hard_n<<",\"policy_measurable\":"<<v.policy_measurable<<"},\"identity\":{\"trajectory_id\":"<<result.trajectory_id<<",\"drone_id\":"<<result.drone_id<<",\"environment_generation\":"<<result.environment_generation<<"},\"terrain\":{\"resolution_u\":"<<accepted_map->info.resolution<<",\"length_x\":"<<accepted_map->info.length_x<<",\"length_y\":"<<accepted_map->info.length_y<<",\"origin_x\":"<<(accepted_map->info.pose.position.x-accepted_map->info.length_x/2.0)<<",\"origin_y\":"<<(accepted_map->info.pose.position.y-accepted_map->info.length_y/2.0)<<",\"mission_sha256\":";
  jsonString(out,load_result->mission.mission_sha256); out<<",\"frame_digest\":"; jsonString(out,load_result->mission.frame_digest);out<<"}}\n";
  std::cout<<"R2_RESULT has_trajectory="<<result.hasTrajectory()<<" pieces="<<result.trajectory.getPieceNum()<<" phases="<<result.phases.size()<<" input_unchanged="<<input_unchanged<<std::endl;
  rclcpp::shutdown();
  return result.hasTrajectory() && input_unchanged && result.evaluation.evaluated && result.phases.size()>1 ? 0:1;
}
