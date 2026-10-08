#include "path_manager/direct_trajectory_solver.h"

#include <algorithm>
#include <utility>

#include "path_optimizer/poly_traj_optimizer.h"

namespace path_manager {

DirectAltitudeBand computeDirectAltitudeBand(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &head_pos, const Eigen::Vector3d &goal_pos,
    const DirectAltitudeConfig &config) {
  double path_max_z = std::max(head_pos.z(), goal_pos.z());
  double path_min_z = std::min(head_pos.z(), goal_pos.z());
  for (const auto &p : path) {
    path_max_z = std::max(path_max_z, p.z());
    path_min_z = std::min(path_min_z, p.z());
  }
  const double path_max_z_denoised = path_max_z;
  path_max_z = std::max(path_max_z, config.raw_max_z);
  const double z_hi = path_max_z + config.cap_headroom;
  double z_lo = path_min_z - config.floor_headroom;
  if (config.ground_height > -0.5) {
    z_lo = std::max(z_lo, config.ground_height + 0.5 * config.floor_headroom);
  }
  return {z_lo, z_hi, config.weight, path_max_z_denoised, path_max_z};
}

DirectTrajectoryResult solveDirectTrajectory(
    ego_planner::PolyTrajOptimizer &optimizer, DirectTrajectoryRequest request) {
  ego_planner::OptimizerSolveRequest solve_request;
  solve_request.path = std::move(request.path);
  solve_request.waypoints = std::move(request.waypoints);
  solve_request.cap_ref = std::move(request.cap_ref);
  solve_request.edge_leg = std::move(request.edge_leg);
  solve_request.head_pos = request.head_pos;
  solve_request.head_vel = request.head_vel;
  solve_request.head_acc = request.head_acc;
  solve_request.tail = request.tail;
  solve_request.band = {request.band.z_lo, request.band.z_hi, request.band.weight};
  solve_request.max_vel = request.max_vel;
  solve_request.suppress_crossing_exempt = request.suppress_crossing_exempt;
  auto solved = optimizer.solve(std::move(solve_request));

  DirectTrajectoryResult result;
  result.optimizer_path = std::move(solved.optimizer_path);
  result.piece_leg = std::move(solved.piece_leg);
  if (solved.status == ego_planner::OptimizerSolveStatus::ROUTE_TOO_SHORT) {
    result.status = DirectTrajectoryStatus::ROUTE_TOO_SHORT;
    return result;
  }
  if (solved.status == ego_planner::OptimizerSolveStatus::NO_WAYPOINTS) {
    result.status = DirectTrajectoryStatus::NO_WAYPOINTS;
    return result;
  }
  if (!solved.hasTrajectory()) return result;

  result.status = DirectTrajectoryStatus::SUCCESS;
  result.seed = std::move(solved.seed);
  result.trajectory = std::move(solved.trajectory);
  return result;
}

}  // namespace path_manager
