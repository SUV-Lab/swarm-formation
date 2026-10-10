#pragma once
#include <path_manager/plan_settings.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

// Test adapter only: copy the effective YAML/NodeOptions values, then current
// declared values, into the library's owned settings. No defaults table and no
// ROS objects survive in PlanSettings.
inline path_manager::PlanSetting settingValue(const rclcpp::ParameterValue &value) {
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return value.get<bool>();
    case rclcpp::ParameterType::PARAMETER_INTEGER: return value.get<int64_t>();
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return value.get<double>();
    case rclcpp::ParameterType::PARAMETER_STRING: return value.get<std::string>();
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: return value.get<std::vector<uint8_t>>();
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: return value.get<std::vector<bool>>();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: return value.get<std::vector<int64_t>>();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: return value.get<std::vector<double>>();
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: return value.get<std::vector<std::string>>();
    default: throw std::invalid_argument("cannot copy an unset ROS setting");
  }
}
inline path_manager::PlanSettings valueSettings(const rclcpp::Node::SharedPtr &node) {
  path_manager::PlanSettings settings;
  for (const auto &entry : node->get_node_parameters_interface()->get_parameter_overrides())
    settings.parameters.emplace(entry.first, settingValue(entry.second));
  for (const auto &name : node->list_parameters({}, 1000).names) {
    const auto value = node->get_parameter(name).get_parameter_value();
    if (value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
      settings.parameters.insert_or_assign(name, settingValue(value));
  }
  return settings;
}
