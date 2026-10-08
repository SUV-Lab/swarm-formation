#include "path_manager/route_slice_solver.h"
#include "path_optimizer/poly_traj_optimizer.h"

#include <algorithm>
#include <utility>

namespace path_manager {

RouteSliceResult solveRouteSlice(ego_planner::PolyTrajOptimizer &opt,
                                 RouteSliceRequest request,
                                 const RouteSliceConfig &config)
{
  if (request.path.size() < 2)
    return {RouteSliceStatus::ROUTE_TOO_SHORT, {}, 0.0};

  double path_max_z = std::max(request.head_pos.z(), request.goal_pos.z());
  double path_min_z = std::min(request.head_pos.z(), request.goal_pos.z());
  for (const auto &p : request.path) {
    path_max_z = std::max(path_max_z, p.z());
    path_min_z = std::min(path_min_z, p.z());
  }
  // The slice's cap carries the committed, pre-denoise-restored ceiling.
  // A direct solve receives a separate pre-denoise maximum instead.
  for (const double c : request.cap) path_max_z = std::max(path_max_z, c);
  const double z_hi = path_max_z + config.alt_cap_headroom;
  double z_lo = path_min_z - config.alt_floor_headroom;
  if (config.ground_height > -0.5) {
    z_lo = std::max(z_lo,
                    config.ground_height + 0.5 * config.alt_floor_headroom);
  }
  ego_planner::OptimizerSolveRequest solve_request;
  solve_request.path = std::move(request.path);
  solve_request.waypoints = {request.goal_pos};
  solve_request.cap_ref = std::move(request.cap);
  solve_request.head_pos = request.head_pos;
  solve_request.head_vel = request.head_vel;
  solve_request.head_acc = request.head_acc;
  solve_request.tail = request.tail;
  solve_request.band = {z_lo, z_hi, config.weight_altitude};
  solve_request.max_vel = config.max_vel;
  solve_request.suppress_crossing_exempt = request.suppress_crossing_exempt;
  auto solved = opt.solve(std::move(solve_request));
  if (!solved.hasTrajectory())
    return {RouteSliceStatus::OPTIMIZER_FAILED, {}, 0.0};

  return {RouteSliceStatus::SUCCESS, std::move(solved.trajectory), solved.envelope_peak};
}

}  // namespace path_manager
