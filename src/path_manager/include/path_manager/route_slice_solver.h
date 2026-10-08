#ifndef PATH_MANAGER_ROUTE_SLICE_SOLVER_H
#define PATH_MANAGER_ROUTE_SLICE_SOLVER_H

#include "path_optimizer/poly_traj_utils.hpp"
#include "path_optimizer/tail_boundary.h"

namespace ego_planner { class PolyTrajOptimizer; }

namespace path_manager {

// Per-solve data in the existing planner frame: positions/caps are in u,
// velocity in u/s and acceleration in u/s^2. The configured optimizer owns
// the existing SI conversion. Goal and tail are absolute boundary states,
// not mission AGL values. Owning the route lets the optimizer insert points
// without changing the caller's committed route or a sibling worker's input.
struct RouteSliceRequest {
  std::vector<Eigen::Vector3d> path;
  std::vector<double> cap;
  Eigen::Vector3d head_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d head_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d head_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal_pos{Eigen::Vector3d::Zero()};
  ego_planner::TailBoundary tail;
  bool suppress_crossing_exempt{false};
};

// Snapshot of the PathManager-owned settings used by a slice solve. These
// are supplied separately from the optimizer's own configuration; the chain
// takes one snapshot before starting its independent optimizer workers.
struct RouteSliceConfig {
  // manager/max_vel for seed/arrival construction. A worker may separately
  // override optimization/max_vel in its configured optimizer, as before.
  double max_vel{0.0};
  double alt_cap_headroom{0.0};
  double alt_floor_headroom{0.0};
  double ground_height{-1.0};
  double weight_altitude{0.0};
};

enum class RouteSliceStatus { SUCCESS, ROUTE_TOO_SHORT, OPTIMIZER_FAILED };

struct RouteSliceResult {
  RouteSliceStatus status{RouteSliceStatus::OPTIMIZER_FAILED};
  // Owns every polynomial piece; remains valid after the optimizer and its
  // environment are destroyed or reused. Failed results have no trajectory.
  poly_traj::Trajectory trajectory;
  // The successful solve's existing envelope audit, copied before reuse.
  // Not a whole-mission verdict: stitching/phase checks remain in the chain.
  double envelope_peak{0.0};

  bool hasTrajectory() const { return status == RouteSliceStatus::SUCCESS; }
};

// Computation only: no PathManager state, ROS parameters, clocks or topic
// publication. opt is a configured, exclusive worker instance. Its borrowed
// SDF/terrain/risk providers must remain alive and stable for this call.
// The optimizer's cap-size fallback and all solve/audit policies are retained.
RouteSliceResult solveRouteSlice(ego_planner::PolyTrajOptimizer &opt,
                                 RouteSliceRequest request,
                                 const RouteSliceConfig &config);

}  // namespace path_manager

#endif  // PATH_MANAGER_ROUTE_SLICE_SOLVER_H
