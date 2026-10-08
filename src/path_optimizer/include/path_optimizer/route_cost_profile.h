#ifndef PATH_OPTIMIZER_ROUTE_COST_PROFILE_H
#define PATH_OPTIMIZER_ROUTE_COST_PROFILE_H

#include <Eigen/Core>
#include <cstddef>
#include <functional>
#include <vector>

namespace ego_planner {

using TerrainHeightQuery = std::function<float(double, double)>;
using TerrainHeightGradientQuery =
    std::function<bool(double, double, float *, float *, float *)>;
using ShadowCeilingQuery =
    std::function<double(size_t, const Eigen::Vector3d &)>;

struct RouteCostZone {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double reach{0.0};
};

struct PieceAltitudeCapConfig {
  // Optimizer defaults; its adapter supplies the current configured values.
  double slope{0.10};
  double headroom{0.4};
  double zone_relax{0.0};
  double shadow_margin{0.0};
  double obstacle_clearance{0.5};
  double terrain_cell_u{0.0};
  bool use_risk_zones{false};
};

enum class PieceAltitudeCapStatus {
  SCALAR_FALLBACK,
  ACTIVE,
  REFERENCE_MISMATCH
};

struct PieceAltitudeCapResult {
  PieceAltitudeCapStatus status{PieceAltitudeCapStatus::SCALAR_FALLBACK};
  Eigen::VectorXd ceilings;
  double cap_min{0.0};
  double cap_max{0.0};
  int relaxed_pieces{0};
  int shadow_pieces{0};
  double shadow_lift_max{0.0};
};

// Freeze per-piece ceilings from the ordered absolute route and its paired
// altitude reference. Geometry/settings are expected to be finite, in the
// caller's consistent planner units; no normalization or new clamps occur.
// Empty references select scalar fallback. Other size mismatches, including
// fewer than two route vertices, return REFERENCE_MISMATCH without querying.
// Zone slots must retain the shadow provider's original indexing, including
// inactive zones. Callbacks are borrowed by reference only for this call;
// their backing terrain/zone data must remain stable. No callback is retained.
// A present gradient provider takes precedence even when it returns false;
// height fallback is used only when the gradient provider is absent.
PieceAltitudeCapResult buildPieceAltitudeCaps(
    const std::vector<Eigen::Vector3d> &route,
    const std::vector<double> &cap_ref,
    const std::vector<RouteCostZone> &zones,
    const PieceAltitudeCapConfig &config,
    const TerrainHeightQuery &height = {},
    const TerrainHeightGradientQuery &height_gradient = {},
    const ShadowCeilingQuery &shadow = {});

struct PieceRoughnessConfig {
  double time_relief{0.0};
  double ride_weight{0.0};
  double terrain_cell_u{0.0};
};

struct PieceRoughnessResult {
  bool active{false};
  Eigen::VectorXd roughness;
  Eigen::VectorXd time_factors;
  int rough_pieces{0};
  double factor_min{1.0};
};

// Freeze AGL-faded slope roughness and optional time-price factors. The same
// synchronous borrowing/unit preconditions apply. The caller supplies its
// already configured relief (the optimizer clamps it in setParam, not here).
// Fewer than two vertices, disabled weights or no terrain callback gives an
// inactive empty result. Height-only terrain enables a zero roughness table
// without querying height. Failed gradient queries still count in the mean.
// Results own their arrays and remain valid after all inputs are destroyed.
PieceRoughnessResult buildPieceRoughness(
    const std::vector<Eigen::Vector3d> &route,
    const PieceRoughnessConfig &config,
    const TerrainHeightQuery &height = {},
    const TerrainHeightGradientQuery &height_gradient = {});

}  // namespace ego_planner

#endif  // PATH_OPTIMIZER_ROUTE_COST_PROFILE_H
