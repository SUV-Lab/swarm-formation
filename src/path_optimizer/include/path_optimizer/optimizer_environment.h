#ifndef PATH_OPTIMIZER_OPTIMIZER_ENVIRONMENT_H
#define PATH_OPTIMIZER_OPTIMIZER_ENVIRONMENT_H

#include "path_optimizer/route_cost_profile.h"

#include <functional>
#include <vector>

namespace path_planner { namespace sdf { class IDistanceField; } }

namespace ego_planner {

// Risk geometry uses the existing planner frame. Zone order also indexes
// the visibility and shadow callbacks; inactive slots must not be removed.
struct RiskZone {
  Eigen::Vector3d center;
  double reach;            // horizontal semi-axis, frame units
  double peak;             // raw yaml max_risk_level
  double vertical_reach{0.0}; // vertical semi-axis; <=0 falls back to reach
};

// Settings supplied by the planner after the optimizer's ROS parameter
// adapter runs. This is that runtime setup surface, not a second copy of
// every numerical solver parameter. Existing setter normalization is kept.
struct OptimizerRuntimeSettings {
  double obstacle_clearance{0.5};
  bool collision_reject{true};
  bool envelope_reject{true};
  double envelope_violation_max{0.25};
  double envelope_violation_hard_max{0.30};
  int lbfgs_mem_size{64};
  double lbfgs_g_epsilon{0.1};
  int lbfgs_past{3};
  double lbfgs_delta{1.0e-6};
  int lbfgs_max_linesearch{0};
  double lbfgs_f_dec{0.0};
  double lbfgs_s_curv{0.0};
  double ground_height{-1.0};
  double virtual_ceil_height{-1.0};
  double risk_goal_taper_radius{0.0};
};

struct OptimizerRiskEnvironment {
  std::vector<RiskZone> zones;
  std::function<double(size_t, const Eigen::Vector3d &, Eigen::Vector3d *)>
      visibility;
  ShadowCeilingQuery shadow_ceiling;
};

// A live binding, not an immutable environment snapshot. The object owns its
// zone vector and callback objects, while the SDF and anything captured by
// reference/pointer in callbacks remain borrowed. Those providers must
// remain alive while bound and stay stable during each solve,
// including its parallel cost evaluation. Rebinding and solving are serial
// operations. Rebind ordered risk geometry and its providers together when
// the risk set changes. When terrain resolution changes, rebind its queries
// and terrain_cell_u together. Empty fields explicitly remove previous bindings.
struct OptimizerEnvironment {
  const path_planner::sdf::IDistanceField *distance_field{nullptr};
  TerrainHeightQuery terrain_height;
  TerrainHeightGradientQuery terrain_gradient;
  double terrain_cell_u{0.0};
  OptimizerRiskEnvironment risk;
};

}  // namespace ego_planner

#endif  // PATH_OPTIMIZER_OPTIMIZER_ENVIRONMENT_H
