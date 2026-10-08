#ifndef PATH_MANAGER_FLIGHT_ASSEMBLY_H
#define PATH_MANAGER_FLIGHT_ASSEMBLY_H

#include <limits>
#include <vector>

#include "path_manager/flight_evaluation.h"
#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {

struct AssembledFlight {
  poly_traj::Trajectory trajectory;
  std::vector<PhaseSpan> phases;
};

// Append the solved runs in input order. Every route span retains CRUISE
// judgment; phase_labels changes only its display name. An empty input yields
// an empty flight. The returned trajectory and spans own their storage.
AssembledFlight stitchRouteFlight(
    const std::vector<poly_traj::Trajectory> &runs, bool phase_labels);

struct PrefixJoinResult {
  // A rejected join retains the unprefixed input flight.
  AssembledFlight flight;
  double position_error{std::numeric_limits<double>::infinity()};
  double velocity_error{std::numeric_limits<double>::infinity()};
  double acceleration_error{std::numeric_limits<double>::infinity()};
  double jerk_jump{std::numeric_limits<double>::infinity()};
  bool joined{false};
};

// Measure the prefix-tail/flight-head seam in planner units and prepend only
// when all three P/V/A errors are <= the supplied tolerance. Jerk is measured
// but does not gate the join. NaN errors/tolerance fail the same comparisons.
// The prefix gets a TRANSITION span; existing span end times shift by its
// duration. Empty trajectories return unjoined with infinite seam values.
// Logging, refusal policy and stored-trajectory invalidation belong to callers.
PrefixJoinResult prependFlightPrefix(AssembledFlight flight,
                                     const poly_traj::Trajectory &prefix,
                                     double junction_pva_tol_u);

}  // namespace path_manager

#endif  // PATH_MANAGER_FLIGHT_ASSEMBLY_H
