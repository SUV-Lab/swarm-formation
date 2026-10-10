#include "path_manager/ros_planning_visualization.h"

namespace path_manager {
RosPlanningVisualization::RosPlanningVisualization(rclcpp::Node &node,
                                                   bool pipeline_enabled) {
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  front_end_ = node.create_publisher<nav_msgs::msg::Path>("/planning/front_end_path", qos);
  opt_ = node.create_publisher<visualization_msgs::msg::Marker>("/viz/opt_trajectory", qos);
  global_ = node.create_publisher<visualization_msgs::msg::Marker>("/viz/global_trajectory", qos);
  front_tube_ = node.create_publisher<visualization_msgs::msg::Marker>("/viz/front_end_path", qos);
  obstacles_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/viz/dynamic_obstacles", qos);
  risk_field_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/viz/risk_field", qos);
  chain_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/viz/chain_segments", qos);
  if (pipeline_enabled) {
    pipeline_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/debug/pipeline", qos);
    RCLCPP_INFO(node.get_logger(), "[DEBUG-PIPELINE] /debug/pipeline enabled "
                "(shortcut_points + inner_points per plan)");
  }
  trajectory_risk_ = node.create_publisher<visualization_msgs::msg::Marker>("/viz/traj_risk", qos);
  profile_ = node.create_publisher<std_msgs::msg::Float64MultiArray>("/viz/risk_profile", qos);
  heatmap_ = node.create_publisher<grid_map_msgs::msg::GridMap>("/viz/risk_heatmap", qos);
}
void RosPlanningVisualization::frontEndPath(const nav_msgs::msg::Path &m) const { front_end_->publish(m); }
void RosPlanningVisualization::tube(visualization::TubeChannel channel,
                                    const visualization_msgs::msg::Marker &m) const {
  switch (channel) {
    case visualization::TubeChannel::Optimized: opt_->publish(m); break;
    case visualization::TubeChannel::Reference: global_->publish(m); break;
    case visualization::TubeChannel::FrontEnd: front_tube_->publish(m); break;
  }
}
void RosPlanningVisualization::chain(const visualization_msgs::msg::MarkerArray &m) const { chain_->publish(m); }
void RosPlanningVisualization::pipeline(const visualization_msgs::msg::MarkerArray &m) const { if (pipeline_) pipeline_->publish(m); }
void RosPlanningVisualization::trajectoryRisk(const visualization_msgs::msg::Marker &m) const { trajectory_risk_->publish(m); }
void RosPlanningVisualization::riskProfile(const std_msgs::msg::Float64MultiArray &m) const { profile_->publish(m); }
void RosPlanningVisualization::riskHeatmap(const grid_map_msgs::msg::GridMap &m) const { heatmap_->publish(m); }
void RosPlanningVisualization::riskField(const visualization_msgs::msg::MarkerArray &m) const { risk_field_->publish(m); }
void RosPlanningVisualization::dynamicObstacles(const visualization_msgs::msg::MarkerArray &m) const { obstacles_->publish(m); }
void RosPlanningVisualization::clearTrajectory(const visualization_msgs::msg::Marker &m) const {
  opt_->publish(m);
  global_->publish(m);
  trajectory_risk_->publish(m);
  profile_->publish(std_msgs::msg::Float64MultiArray{});
}
}  // namespace path_manager
