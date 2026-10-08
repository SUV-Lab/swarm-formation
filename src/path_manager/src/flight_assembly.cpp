#include "path_manager/flight_assembly.h"

#include <utility>

namespace path_manager {

AssembledFlight stitchRouteFlight(
    const std::vector<poly_traj::Trajectory> &runs, bool phase_labels) {
  AssembledFlight flight;
  if (runs.empty()) return flight;

  flight.trajectory = runs.front();
  for (size_t i = 1; i < runs.size(); ++i)
    flight.trajectory.append(runs[i]);

  double acc_t = 0.0;
  for (size_t i = 0; i < runs.size(); ++i) {
    acc_t += runs[i].getTotalDuration();
    flight.phases.push_back(
        {acc_t, PhaseKind::CRUISE,
         phase_labels
             ? (i == 0 ? std::string("departure")
                       : i + 1 == runs.size() ? std::string("arrival")
                                              : "cruise-" + std::to_string(i))
             : "seg" + std::to_string(i + 1)});
  }
  return flight;
}

PrefixJoinResult prependFlightPrefix(AssembledFlight flight,
                                     const poly_traj::Trajectory &prefix,
                                     double junction_pva_tol_u) {
  PrefixJoinResult joined;
  joined.flight = std::move(flight);
  poly_traj::Trajectory tt = prefix;
  if (tt.getPieceNum() == 0 || joined.flight.trajectory.getPieceNum() == 0)
    return joined;

  const double Tt = tt.getTotalDuration();
  const auto &chained = joined.flight.trajectory;
  joined.position_error = (tt.getPos(Tt) - chained.getPos(0.0)).norm();
  joined.velocity_error = (tt.getVel(Tt) - chained.getVel(0.0)).norm();
  joined.acceleration_error = (tt.getAcc(Tt) - chained.getAcc(0.0)).norm();
  joined.jerk_jump = (tt.getJer(Tt) - chained.getJer(0.0)).norm();
  if (!(joined.position_error <= junction_pva_tol_u &&
        joined.velocity_error <= junction_pva_tol_u &&
        joined.acceleration_error <= junction_pva_tol_u))
    return joined;

  poly_traj::Trajectory full = prefix;
  full.append(chained);
  std::vector<PhaseSpan> full_spans;
  full_spans.push_back({Tt, PhaseKind::TRANSITION, "transition"});
  for (PhaseSpan sp : joined.flight.phases) {
    sp.t_end += Tt;
    full_spans.push_back(sp);
  }
  joined.flight.trajectory = full;
  joined.flight.phases = full_spans;
  joined.joined = true;
  return joined;
}

}  // namespace path_manager
