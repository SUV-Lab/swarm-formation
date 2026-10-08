#ifndef PATH_MANAGER_DIRECT_TRAJECTORY_SOLVER_H
#define PATH_MANAGER_DIRECT_TRAJECTORY_SOLVER_H

#include <cstddef>
#include <vector>

#include "path_optimizer/poly_traj_utils.hpp"
#include "path_optimizer/tail_boundary.h"

namespace ego_planner {
class PolyTrajOptimizer;
}

namespace path_manager {

struct DirectAltitudeConfig {
  // Per-front-end data, not a persistent setting: the route's maximum z
  // before denoising. The direct solve restores this ceiling, unlike the
  // slice solver which also considers every supplied cap value.
  double raw_max_z{-1e9};
  double cap_headroom{0.0};
  double floor_headroom{0.0};
  double ground_height{-1.0};
  double weight{0.0};
};

struct DirectAltitudeBand {
  double z_lo{0.0};
  double z_hi{0.0};
  double weight{0.0};
  // Diagnostics let the adapter retain the existing pre-solve ALT logs.
  double path_max_z_denoised{0.0};
  double path_max_z{0.0};
};

DirectAltitudeBand computeDirectAltitudeBand(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &head_pos, const Eigen::Vector3d &goal_pos,
    const DirectAltitudeConfig &config);

struct DirectTrajectoryRequest {
  std::vector<Eigen::Vector3d> path;
  // Already resolved absolute positions in planner units, not command AGL.
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<double> cap_ref;
  std::vector<size_t> edge_leg;
  Eigen::Vector3d head_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d head_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d head_acc{Eigen::Vector3d::Zero()};
  ego_planner::TailBoundary tail;
  DirectAltitudeBand band;
  // Manager seed/arrival speed; the configured optimizer retains its own
  // optimization/max_vel setting independently.
  double max_vel{0.0};
  bool suppress_crossing_exempt{false};
};

enum class DirectTrajectoryStatus {
  SUCCESS,
  ROUTE_TOO_SHORT,
  NO_WAYPOINTS,
  OPTIMIZER_FAILED,
};

struct DirectTrajectoryResult {
  DirectTrajectoryStatus status{DirectTrajectoryStatus::OPTIMIZER_FAILED};
  // Only successful solves expose trajectories; a failed optimizer may have
  // produced a seed internally, which the existing adapter discarded.
  poly_traj::Trajectory seed;
  poly_traj::Trajectory trajectory;
  // Preserve optimizer path edits and piece attribution on every return,
  // including failure. These are diagnostics, not a deliverable trajectory.
  std::vector<Eigen::Vector3d> optimizer_path;
  std::vector<size_t> piece_leg;

  bool hasTrajectory() const { return status == DirectTrajectoryStatus::SUCCESS; }
};

// Borrow one configured optimizer exclusively until this synchronous call
// returns. Its environment providers must remain alive/stable for the call;
// every returned value owns its storage. No manager, clocks, trajectory IDs,
// publication or execution-state changes occur in this adapter. Existing
// optimizer audits, cap fallback and provenance fallback remain authoritative.
DirectTrajectoryResult solveDirectTrajectory(
    ego_planner::PolyTrajOptimizer &optimizer, DirectTrajectoryRequest request);

}  // namespace path_manager

#endif  // PATH_MANAGER_DIRECT_TRAJECTORY_SOLVER_H
