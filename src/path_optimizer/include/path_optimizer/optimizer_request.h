#ifndef PATH_OPTIMIZER_OPTIMIZER_REQUEST_H
#define PATH_OPTIMIZER_OPTIMIZER_REQUEST_H

#include "path_optimizer/poly_traj_utils.hpp"
#include "path_optimizer/tail_boundary.h"

#include <cstddef>
#include <vector>

namespace ego_planner {

struct OptimizerAltitudeBand {
  double z_lo{-1e9};
  double z_hi{-1.0};
  double weight{0.0};
};

// One solve in absolute planner coordinates: position/cap in u, velocity
// in u/s, acceleration in u/s^2. The configured optimizer retains the SI
// conversion and audits. max_vel is the seed/derived-arrival speed, distinct
// from its configured optimization/max_vel and must be finite and positive.
// Geometry, head, band and prescribed tail values must be finite. A paired
// cap must also be finite; mismatched caps and tags retain the legacy fallback
// policy. These admission checks do not impose flight feasibility policy.
struct OptimizerSolveRequest {
  std::vector<Eigen::Vector3d> path;
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<double> cap_ref;
  std::vector<size_t> edge_leg;
  Eigen::Vector3d head_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d head_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d head_acc{Eigen::Vector3d::Zero()};
  TailBoundary tail;
  OptimizerAltitudeBand band;
  double max_vel{0.0};
  bool suppress_crossing_exempt{false};
};

enum class OptimizerSolveStatus {
  SUCCESS,
  ROUTE_TOO_SHORT,
  NO_WAYPOINTS,
  NOT_CONFIGURED,
  MALFORMED_INPUT,
  OPTIMIZER_FAILED
};

struct OptimizerSolveResult {
  OptimizerSolveStatus status{OptimizerSolveStatus::OPTIMIZER_FAILED};
  // Successful output owns every polynomial piece. Failure never exposes a
  // partial seed or old trajectory/envelope from an earlier solve.
  poly_traj::Trajectory seed;
  poly_traj::Trajectory trajectory;
  // Diagnostic geometry/provenance survives backend refusal as well as
  // success; it may contain midpoint, lead-in or committed-z path edits.
  std::vector<Eigen::Vector3d> optimizer_path;
  std::vector<size_t> piece_leg;
  double envelope_peak{0.0};

  bool hasTrajectory() const { return status == OptimizerSolveStatus::SUCCESS; }
};

}  // namespace ego_planner

#endif  // PATH_OPTIMIZER_OPTIMIZER_REQUEST_H
