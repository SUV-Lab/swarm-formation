#pragma once
#include "path_manager/builtin_route_search.h"
#include <rclcpp/node.hpp>

namespace path_manager {

// Declare/read the existing names and defaults, preserving YAML overrides.
// This owns manager/front_end, manager/fm2_*, manager/risk_smha_w,
// manager/astar_step_size, manager/astar_bypass_shortcut and built-in search
// cost/policy tuning. It does not leak these choices into the generic query.
BuiltinRouteSearchConfig readRosBuiltinRouteSearchConfig(rclcpp::Node &node);

}  // namespace path_manager
