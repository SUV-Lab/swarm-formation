#ifndef PATH_MANAGER_MISSION_PLAN_H
#define PATH_MANAGER_MISSION_PLAN_H

#include <cstdint>
#include <string>
#include <vector>

#include "path_manager/flight_evaluation.h"
#include "path_manager/planning_result.h"
#include "path_manager/start_state.h"
#include "path_optimizer/poly_traj_utils.hpp"
#include "path_optimizer/tail_boundary.h"

namespace path_manager {

enum class StartAltitude { ABSOLUTE, COMMAND_AGL };

// All vectors use the existing planner frame and units. StartHead carries
// resolved velocity/acceleration and their provenance; waypoint z remains
// mission AGL, as in PathManager. Tail derivatives are in u/s and u/s^2.
// COMMAND_AGL converts only this request's start once against the current
// terrain. With no terrain it retains the altitude as absolute, matching ROS.
struct MissionPlanRequest {
  StartHead head;
  std::vector<Eigen::Vector3d> waypoints;
  ego_planner::TailBoundary tail;
  StartAltitude start_altitude{StartAltitude::ABSOLUTE};
  // Legacy RosPlanningBackend behavior: formation_type is metadata only.
  // Pattern/size reach the primary optimizer after a successful computation,
  // so they may affect a later solve reusing it; they do not configure all
  // independently created segment workers for this request.
  std::string formation_type{"none"};
  std::vector<Eigen::Vector3d> formation_pattern;
  int formation_size{1};
  // Both values must be finite; otherwise the request is refused before
  // either override is applied. Positive values persist on this planner
  // instance; a later finite non-positive value retains its setting.
  double length_per_piece{0.0};
  double obstacle_clearance{0.0};
};

struct MissionPlanResult {
  // In RosPlanningBackend, rejected adoption (including a SUCCESS/DEGRADED
  // computation without a product) returns FAILED with textual detail and
  // no dedicated PlanReason code. Existing computation failures retain their
  // status when their execution effects are accepted.
  PlanResult status;
  // Value-owned coefficients. No data here refers to planner scratch or
  // execution storage; later plans/environment changes cannot rewrite it.
  poly_traj::Trajectory trajectory;
  // Retains the existing global-slot meaning: MINCO seed for a direct solve,
  // optimized baseline for a baseline chain, full flight for a route chain.
  poly_traj::Trajectory reference_trajectory;
  // Direct (chain-disabled) plans keep evaluated=false: they have the
  // optimizer's existing audits, but no separate whole-flight evaluation.
  FlightVerdict evaluation;
  std::vector<PhaseSpan> phases;
  uint64_t trajectory_id{0};
  int drone_id{-1};
  double start_time{0.0};
  double reference_start_time{0.0};
  // Revision observed when packaging this result, not a frozen environment
  // or a promise of validity after an update. The execution adapter retains
  // responsibility for revalidation and withdrawing an active trajectory.
  uint64_t environment_generation{0};

  bool hasTrajectory() const { return status.hasTrajectory(); }
};

}  // namespace path_manager

#endif  // PATH_MANAGER_MISSION_PLAN_H
