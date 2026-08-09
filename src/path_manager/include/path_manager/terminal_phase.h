#ifndef TERMINAL_PHASE_H
#define TERMINAL_PHASE_H

#include <functional>
#include <Eigen/Eigen>
#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {

// [STAGE-3] Terminal-phase generator: a trajectory of genuinely DIFFERENT
// character from the MINCO chain. Nothing here is optimized — the geometry
// is PRESCRIBED (the terminal phase's env costs are ~0 by design): a
// constant-speed descending helix entered through a linear curvature ramp
// (0 -> 1/R, so the entry acceleration is exactly zero — matching the
// arrival contract the chain hands over: level, cruise, a = 0) and exited
// the same way (level, straight, a = 0 — a clean state for whatever takes
// over next). The vertical profile is a quintic smoothstep, C2 with zero
// rate and curvature at both ends.
//
// The analytic path is sampled and converted to piecewise QUINTIC HERMITE
// pieces in the same poly_traj::Trajectory representation the chain uses:
// each piece interpolates the exact analytic P/V/A at its two ends, so
// every internal joint and the handoff seam are C2 by construction — the
// chain's seam audit machinery applies unchanged. This answers stage 3's
// question directly: a non-MINCO generator can join the chain C2 and fly
// as one published trajectory.
struct TerminalHelixParams {
  double radius{58.0};     // u. Bank at cruise ~ atan(v^2 / (g R)):
                           // 200 m/s, R 5.8 km -> ~35 deg.
  double turns{1.0};       // full revolutions of the descending helix
  double final_agl{0.15};  // u above terrain at the exit point
  bool right{true};        // turn direction (right = clockwise from above)
  double ramp_frac{0.5};   // curvature ramp length as a fraction of radius
  double piece_dt{4.0};    // s per quintic Hermite piece
};

class TerminalPhase {
public:
  // Builds the helix descent from the handoff state. entry_vel must be
  // level and non-zero (the arrival contract guarantees it); any vertical
  // component is dropped — the helix flies its own C2 vertical profile, and
  // a nonzero entry vz shows up verbatim in the caller's seam audit.
  // terrain_z(x, y, &elev) returns false over water/off-DEM (sea level 0).
  // min_agl_out (optional) receives the lowest sampled AGL along the helix
  // — the caller warns when the PRESCRIBED geometry, which no optimizer
  // and no collision audit protects, cuts into terrain.
  static poly_traj::Trajectory helixDescent(
      const Eigen::Vector3d &entry_pos, const Eigen::Vector3d &entry_vel,
      const TerminalHelixParams &prm,
      const std::function<bool(double, double, double *)> &terrain_z,
      double *min_agl_out = nullptr);
};

}  // namespace path_manager

#endif  // TERMINAL_PHASE_H
