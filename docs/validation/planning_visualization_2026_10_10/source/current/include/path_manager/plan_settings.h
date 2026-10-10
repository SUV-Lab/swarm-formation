#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "mmp_vehicle_dynamics/vehicle_model.hpp"

namespace path_manager {
class RouteSearchBackend;

// Sparse startup overrides. Existing names and exact value types are retained
// during this ROS-backed stage; numerical defaults remain at their consumers.
// These are forwarded once and never become a second live settings authority.
using PlanSetting = std::variant<
    bool, int64_t, double, std::string, std::vector<uint8_t>,
    std::vector<bool>, std::vector<int64_t>, std::vector<double>,
    std::vector<std::string>>;

struct PlanSettings {
  std::map<std::string, PlanSetting> parameters;
  mmp_vehicle_dynamics::VehicleModel::Ptr vehicle_model;
  std::shared_ptr<RouteSearchBackend> route_search_backend;
  // Optional ROS visualization connection. False creates no visualization
  // publishers and skips message preparation; planning, risk masks, obstacle
  // orientation, execution policy and numerical settings remain active.
  // Covers PM's /viz/* display channels, /debug/pipeline and the display route
  // /planning/front_end_path. ROS backend, terrain/SDF and execution transport
  // still operate; this flag does not request a ROS-free backend.
  bool attach_visualization{true};
};
}  // namespace path_manager
