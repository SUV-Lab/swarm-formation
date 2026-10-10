#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace ego_planner {

// Synchronous read overlay for an optimizer's configuration. Existing ROS
// values/default declarations remain the authority for keys not overridden.
// No node parameter is set and no parameter event/callback is triggered by an
// override. Values are borrowed only while configuring; the optimizer stores
// the resulting numerical settings, not this view or these parameters.
// Callers serialize configuration with live parameter updates. Matching uses
// the complete existing name; the last supplied value wins. Parameter types
// follow rclcpp's typed reads (mismatches throw rather than silently coerce).
// Only keys consumed by the configuration reader have an effect.
class RosParameterView {
 public:
  RosParameterView(const rclcpp::Node::SharedPtr &node,
                   const std::vector<rclcpp::Parameter> &overrides)
      : node_(node), overrides_(overrides) {}

  bool has_parameter(const std::string &name) const {
    return find(name) != nullptr || node_->has_parameter(name);
  }
  template <typename T>
  bool get_parameter(const std::string &name, T &value) const {
    if (const auto *parameter = find(name)) {
      value = parameter->get_value<T>();
      return true;
    }
    return node_->get_parameter(name, value);
  }

 private:
  const rclcpp::Parameter *find(const std::string &name) const {
    for (auto it = overrides_.rbegin(); it != overrides_.rend(); ++it)
      if (it->get_name() == name) return &*it;
    return nullptr;
  }
  rclcpp::Node::SharedPtr node_;
  const std::vector<rclcpp::Parameter> &overrides_;
};

}  // namespace ego_planner
