#include "path_manager/ros_route_search.h"

namespace path_manager {

BuiltinRouteSearchConfig readRosBuiltinRouteSearchConfig(rclcpp::Node &node) {
  const auto read = [&](const char *name, auto fallback) {
    if (!node.has_parameter(name)) node.declare_parameter(name, fallback);
    auto value = fallback;
    node.get_parameter(name, value);
    return value;
  };
  BuiltinRouteSearchConfig config;
  // Preserve the existing exact-string selector: everything except "fm2"
  // selects A*, including the legacy handling of an unknown name.
  config.algorithm = read("manager/front_end", std::string("fm2")) == "fm2"
      ? BuiltinRouteSearchConfig::Algorithm::FM2
      : BuiltinRouteSearchConfig::Algorithm::ASTAR;
  config.fm2.coarse_k = read("manager/fm2_coarse_k", 4);
  config.fm2.max_cells = read("manager/fm2_max_cells", 8000000);
  config.fm2.star = read("manager/fm2_star", true);
  config.fm2.altitude_weight = read("manager/fm2_alt_penalty", 2.0);
  config.fm2.altitude_scale_up = read("manager/fm2_alt_zscale", 10.0);
  config.fm2.altitude_scale_down = read("manager/fm2_alt_zscale_down", 2.0);
  config.astar.smha_weight = read("manager/risk_smha_w", 2.0);
  config.astar.step_size = read("manager/astar_step_size", 1.0);
  config.risk_weight = read("manager/risk_weight", 1.0);
  config.risk_barrier = read("manager/risk_barrier", 100.0);
  config.risk_goal_taper_radius = read("manager/risk_goal_taper_radius", 30.0);
  config.roughness_weight = read("manager/fm2_rough_weight", 0.0);
  config.roughness_slope0 = read("manager/fm2_rough_slope0", 0.20);
  config.lexicographic_zone_avoidance = read("manager/zone_avoid_lexicographic", true);
  config.bypass_shortcut = read("manager/astar_bypass_shortcut", false);
  return config;
}

}  // namespace path_manager
