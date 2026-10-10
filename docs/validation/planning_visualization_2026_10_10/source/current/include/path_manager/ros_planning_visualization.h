#pragma once

#include <rclcpp/rclcpp.hpp>
#include "path_manager/planning_visualization_data.h"

namespace path_manager {
// Concrete ROS connection only. Publishers keep their original topic/QoS;
// messages are prepared separately and this class never reads planner state.
class RosPlanningVisualization {
 public:
  explicit RosPlanningVisualization(rclcpp::Node &node, bool pipeline_enabled);
  bool pipelineEnabled() const { return static_cast<bool>(pipeline_); }
  void frontEndPath(const nav_msgs::msg::Path &message) const;
  void tube(visualization::TubeChannel channel,
            const visualization_msgs::msg::Marker &message) const;
  void chain(const visualization_msgs::msg::MarkerArray &message) const;
  void pipeline(const visualization_msgs::msg::MarkerArray &message) const;
  void trajectoryRisk(const visualization_msgs::msg::Marker &message) const;
  void riskProfile(const std_msgs::msg::Float64MultiArray &message) const;
  void riskHeatmap(const grid_map_msgs::msg::GridMap &message) const;
  void riskField(const visualization_msgs::msg::MarkerArray &message) const;
  void dynamicObstacles(const visualization_msgs::msg::MarkerArray &message) const;
  // Preserve the existing clear matrix: opt/global/risk, then empty profile.
  void clearTrajectory(const visualization_msgs::msg::Marker &clear) const;
 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr front_end_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr opt_, global_, front_tube_, trajectory_risk_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_, risk_field_, chain_, pipeline_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr profile_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr heatmap_;
};
}  // namespace path_manager
