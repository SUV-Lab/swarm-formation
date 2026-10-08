#include "path_optimizer/poly_traj_optimizer.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ego_planner {

void PolyTrajOptimizer::setRuntimeSettings(
    const OptimizerRuntimeSettings &settings)
{
  setObstacleClearance(settings.obstacle_clearance);
  setCollisionReject(settings.collision_reject);
  setEnvelopeReject(settings.envelope_reject,
                    settings.envelope_violation_max,
                    settings.envelope_violation_hard_max);
  setRiskGoalTaperRadius(settings.risk_goal_taper_radius);
  setLbfgsParams(settings.lbfgs_mem_size, settings.lbfgs_g_epsilon,
                 settings.lbfgs_past, settings.lbfgs_delta,
                 settings.lbfgs_max_linesearch, settings.lbfgs_f_dec,
                 settings.lbfgs_s_curv);
  setGroundHeight(settings.ground_height);
  setVirtualCeilHeight(settings.virtual_ceil_height);
}

void PolyTrajOptimizer::setEnvironment(OptimizerEnvironment environment)
{
  setSDFManager(environment.distance_field);
  setTerrainHeightmap(std::move(environment.terrain_height),
                      environment.terrain_cell_u);
  setTerrainHeightGrad(std::move(environment.terrain_gradient));
  setRiskEnvironment(std::move(environment.risk));
}

void PolyTrajOptimizer::setRiskEnvironment(
    OptimizerRiskEnvironment environment)
{
  // Keep the original zone reset/precomputation and provider ordering.
  setRiskZones(environment.zones);
  setRiskVisibility(std::move(environment.visibility));
  setRiskShadowCeiling(std::move(environment.shadow_ceiling));
}

OptimizerSolveResult PolyTrajOptimizer::solve(OptimizerSolveRequest request)
{
  OptimizerSolveResult result;
  result.optimizer_path = std::move(request.path);
  if (result.optimizer_path.size() < 2) {
    result.status = OptimizerSolveStatus::ROUTE_TOO_SHORT;
    return result;
  }
  if (request.waypoints.empty()) {
    result.status = OptimizerSolveStatus::NO_WAYPOINTS;
    return result;
  }
  if (!parameters_initialized_) {
    result.status = OptimizerSolveStatus::NOT_CONFIGURED;
    return result;
  }
  const auto finite_point = [](const Eigen::Vector3d &point) {
    return point.allFinite();
  };
  const bool finite_cap = request.cap_ref.size() != result.optimizer_path.size() ||
      std::all_of(request.cap_ref.begin(), request.cap_ref.end(),
                  [](double value) { return std::isfinite(value); });
  if (!request.head_pos.allFinite() || !request.head_vel.allFinite() ||
      !request.head_acc.allFinite() ||
      (request.tail.prescribe_vel && !request.tail.vel.allFinite()) ||
      (request.tail.prescribe_acc && !request.tail.acc.allFinite()) ||
      !std::isfinite(request.max_vel) || !(request.max_vel > 0.0) ||
      !std::isfinite(request.band.z_lo) || !std::isfinite(request.band.z_hi) ||
      !std::isfinite(request.band.weight) || !finite_cap ||
      !std::all_of(result.optimizer_path.begin(), result.optimizer_path.end(),
                   finite_point) ||
      !std::all_of(request.waypoints.begin(), request.waypoints.end(), finite_point)) {
    result.status = OptimizerSolveStatus::MALFORMED_INPUT;
    return result;
  }

  setAltitudeBand(request.band.z_lo, request.band.z_hi, request.band.weight);
  setSuppressCrossingExempt(request.suppress_crossing_exempt);
  poly_traj::Trajectory seed, trajectory;
  if (!optimizeFromPath(result.optimizer_path, request.head_pos,
                        request.head_vel, request.head_acc, request.waypoints,
                        request.max_vel, seed, trajectory, request.cap_ref,
                        request.tail, request.edge_leg, &result.piece_leg))
    return result;

  result.status = OptimizerSolveStatus::SUCCESS;
  result.seed = std::move(seed);
  result.trajectory = std::move(trajectory);
  result.envelope_peak = lastEnvPeak();
  return result;
}

}  // namespace ego_planner
