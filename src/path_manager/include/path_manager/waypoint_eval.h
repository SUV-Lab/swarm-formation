#ifndef PATH_MANAGER_WAYPOINT_EVAL_H
#define PATH_MANAGER_WAYPOINT_EVAL_H

// [WPE] Waypoint extraction from a planned trajectory, and measurement of
// how well flying those waypoints reproduces it.
//
// WHY THIS EXISTS. The planner's product is a trajectory, but a vehicle of
// this class accepts a WAYPOINT LIST. So the deliverable has to be encoded
// as waypoints, and the open question is where and how many to place them.
// This module (1) extracts candidate lists, (2) flies them with a follower,
// (3) measures the reproduction — so placement can be chosen on evidence.
//
// WHAT THE NUMBERS MEAN. The follower here is the project-defined neutral
// 3DOF benchmark law (the transition generator's own pursuit/thrust law on
// a point mass). It has no attitude dynamics, no actuator lag, no wind, no
// estimation error. Reproduction figures characterize THIS law on THIS
// model — structure validation, not real-platform physics validation, and a
// LOWER BOUND on what a real vehicle would show. What is meant to transfer
// is structure: how error trends with waypoint count, and which placement
// strategy beats which. That transfer is itself a claim, so the harness
// measures it (the follower-sensitivity variant) instead of asserting it.
//
// The follower is a std::function seam: when a higher-fidelity model is
// adopted, it binds to the same FlyFn and every extraction/metric here is
// reused unchanged.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mmp_vehicle_dynamics/flight_dynamics.hpp"
#include "mmp_vehicle_dynamics/point_mass_eom.hpp"
#include "path_manager/traj_sampling.h"
#include "path_manager/transition_phase.h"
#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {
namespace waypoint_eval {

// The label every report prints. Compile-time constant so a report cannot
// be produced without it.
inline const char *scopeLabel()
{
  return "project-defined neutral 3DOF benchmark follower — waypoint "
         "structure validation, not real-platform physics validation";
}

// ---------------------------------------------------------------------
// (a) What a waypoint IS
// ---------------------------------------------------------------------
// Position + commanded leg speed. NOT an arrival direction: the wire
// contract this stack already has (TrajectoryCommand.waypoints) is a bare
// point, and a per-waypoint tangent would smuggle in trajectory-shape
// information a waypoint-following vehicle cannot consume — measuring
// reproduction against it would flatter the result. Arrival direction is
// emergent from placement, which is exactly what gets measured here.
// Speed is kept because without it the along-track metrics measure nothing
// (the follower would pick its own speed); the speed-ablation variant
// reports what a position-only wire mode costs.
struct Waypoint {
  Eigen::Vector3d pos_m{0.0, 0.0, 0.0};
  double speed_mps{0.0};   // <= 0: follower uses its cruise default
  // Extraction metadata — NOT part of the vehicle command. The follower's
  // law never reads these; only matching/refinement do.
  double src_arc_m{0.0};
  double src_time_s{0.0};
};

// Planner units -> SI. One conversion boundary for the whole module.
struct FrameScale {
  double unit_xy_m{100.0};
  double unit_z_m{100.0};
  Eigen::Vector3d toSi(const Eigen::Vector3d &v) const
  {
    return Eigen::Vector3d(v.x() * unit_xy_m, v.y() * unit_xy_m,
                           v.z() * unit_z_m);
  }
};

// Dense SI polyline of the source trajectory + arc table + curvature.
struct SourcePath {
  std::vector<double> t_s, s_m;
  std::vector<Eigen::Vector3d> pos_m, vel_mps, acc_mps2;
  std::vector<double> kappa;      // 1/m, geometric
  double total_len_m{0.0};
  double total_time_s{0.0};
  bool empty() const { return pos_m.size() < 2; }
};

// samples <= 0 picks ~25 m chord spacing, clamped to [1024, 16384].
SourcePath buildSourcePath(const poly_traj::Trajectory &traj,
                           const FrameScale &fs, int samples = 0);

// ---------------------------------------------------------------------
// (b) Extraction strategies — S1/S2 are pure geometry (no vehicle model)
// ---------------------------------------------------------------------
// Convention: n waypoints INCLUDING the exact trajectory endpoint, all
// strictly after the start (the follower begins AT the start state, so a
// waypoint there is degenerate). Deterministic; no RNG, no clock.
std::vector<Waypoint> extractUniformArc(const SourcePath &src, int n);

// Density weight w = eps_straight + kappa: curvature attracts waypoints,
// eps keeps straight stretches from starving. lambda scales the curvature
// term; lambda = 0 reproduces uniform arc exactly.
std::vector<Waypoint> extractCurvatureAdaptive(const SourcePath &src, int n,
                                               double lambda = 4.0,
                                               double eps_straight = 1e-4);

// ---------------------------------------------------------------------
// (c) The follower seam
// ---------------------------------------------------------------------
enum class FailReason : int {
  kNone = 0,
  kTimeout,
  kNonFinite,
  kUnrepresentable,
  kDiverged,
  kNoWaypoints,
};
const char *failName(FailReason r);

// Plain SI samples — deliberately NOT PointMassState, so a higher-fidelity
// follower (or a replayed log, or an out-of-process simulator) projects
// into this shape without dragging the point-mass model along.
struct RolloutSample {
  double t_s{0.0};
  Eigen::Vector3d pos_m{0.0, 0.0, 0.0};
  Eigen::Vector3d vel_mps{0.0, 0.0, 0.0};
  Eigen::Vector3d acc_mps2{0.0, 0.0, 0.0};
};
struct WaypointArrival {
  int index{-1};
  double t_s{0.0};
  double miss_m{0.0};   // distance to the waypoint when the leg advanced
};
struct RolloutResult {
  bool completed{false};
  FailReason fail{FailReason::kNone};
  std::vector<RolloutSample> samples;
  std::vector<WaypointArrival> arrivals;
  int saturated_steps{0};
  int total_steps{0};
  std::string follower_id{"unnamed"};   // printed in every row: name what flew
};

struct FollowerStart {
  Eigen::Vector3d pos_m{0.0, 0.0, 0.0};
  Eigen::Vector3d vel_mps{0.0, 0.0, 0.0};
};

// The seam. Everything downstream (refinement, metrics, tables) takes this
// and nothing else — binding a different follower changes no other code.
using FlyFn = std::function<RolloutResult(const std::vector<Waypoint> &,
                                          const FollowerStart &)>;

struct FollowerParams {
  mmp_vehicle_dynamics::Parameters dyn;   // the vehicle model: an INPUT
  double dt_s{0.02};
  // Acceptance geometry, derived from the model unless overridden: at this
  // speed class the turn radius is kilometres, so a radius-only trigger
  // either orbits or needs an absurd radius — the along-leg half-plane
  // test is the primary advance and the radius is a shortcut.
  double accept_radius_m{0.0};            // <=0: derived from dyn
  double lead_time_s{4.0};                // carrot ahead on the leg line
  // <=0: derived as a fraction of the model's own bank limit. A follower
  // is not the generator's primitive enumeration — it should use the
  // authority the vehicle has, or it reports tracking failures that are
  // artifacts of an arbitrary constant.
  double bank_level_rad{0.0};
  double gamma_ramp_rad_per_s{0.15};
  double cruise_speed_mps{0.0};           // <=0: mid of the model band
  double timeout_factor{3.0};             // t_max = max(30, factor * T_src)
  double diverge_factor{10.0};            // abort beyond this x accept radius
  double command_lag_s{0.0};              // >0: first-order command lag
                                          // (sensitivity ablation only)
};
double derivedAcceptRadius(const FollowerParams &p);
double derivedBankLevel(const FollowerParams &p);
// Minimum turn radius this follower can hold at speed v — the geometry a
// reference trajectory must respect to be followable at all.
double minTurnRadius(const FollowerParams &p, double v_mps);

// The 3DOF benchmark follower: the transition generator's pursuit + thrust
// law, flown waypoint-to-waypoint. Saturation is COUNTED, not fatal — a
// follower riding a limit is normal tracking (the generator disqualifies on
// it because it is searching, which is a different job).
RolloutResult flyWaypoints3Dof(const std::vector<Waypoint> &wps,
                               const FollowerStart &start,
                               const FollowerParams &prm);

// S3 — error-driven refinement. Follower-in-the-loop by design: it places
// waypoints where THIS follower deviates, so its output is optimal for the
// follower it was given. Keeps the best completed set; a failed rollout's
// error field is never trusted (insertion falls back to the longest leg).
struct RefineParams {
  int max_waypoints{16};
  double xtrack_tol_m{50.0};
  double min_spacing_m{0.0};   // <=0: 2 x accept radius
};
struct RefineRow {
  int count{0};
  bool completed{false};
  double max_xtrack_m{0.0};
  bool measured{false};        // false: no quality claim from this row
};
struct RefineResult {
  std::vector<Waypoint> best;
  bool converged{false};
  bool any_completed{false};
  std::vector<RefineRow> trace;
};
RefineResult refineByError(const SourcePath &src, const FlyFn &fly,
                           const FollowerStart &start,
                           const RefineParams &prm, double accept_radius_m);

// ---------------------------------------------------------------------
// (d) Metrics
// ---------------------------------------------------------------------
struct SafetyHooks {
  std::function<bool(double x_m, double y_m, double *elev_m)> terrain_z;
  std::function<transition_phase::ZoneProbe(const Eigen::Vector3d &pos_m)>
      zone_probe;
  double min_agl_m{0.0};
};

struct ReproductionMetrics {
  // measured=false voids EVERY quality claim below — the printer emits
  // n/a(reason) rather than numbers from an incomplete flight.
  bool measured{false};
  FailReason fail{FailReason::kNone};

  // Two deviation views. Nearest-point is the honest "how far from the
  // planned path did it ever get". Arc-matched punishes shape distortion
  // at the same progress fraction. A gap between them flags a matching
  // artifact instead of hiding one.
  double max_xtrack_m{0.0}, rms_xtrack_m{0.0};
  double max_arcmatch_m{0.0};
  // Anti-cheat: a flight that CUTS the path (e.g. straight through a
  // switchback) can score a small nearest-point deviation while flying a
  // different route. Length ratio catches that; it is a GATE.
  double len_ratio{0.0};          // flown chordal length / source length
  double duration_ratio{0.0};
  double max_time_skew_s{0.0};
  double terminal_pos_err_m{0.0}, terminal_speed_err_mps{0.0};
  double max_wp_miss_m{0.0};
  double env_peak_util{0.0};
  int env_violation_steps{0};
  double sat_frac{0.0};

  // Safety of the FLOWN path. Sentinels distinguish "measured and fine"
  // from "not measured" — a missing hook never reads as a pass.
  bool agl_measured{false};
  double min_agl_m{0.0};
  bool zone_measured{false};
  int zone_hard_contacts{0};

  // Gate verdicts (tri-state at the printer: PASS / FAIL / SKIPPED).
  bool gate_complete{false};
  bool gate_len{false};
  bool gate_terminal{false};
  bool gate_agl_pass{false}, gate_agl_skipped{true};
  bool gate_zone_pass{false}, gate_zone_skipped{true};
  bool allGatesOk() const
  {
    return measured && gate_complete && gate_len && gate_terminal &&
           (gate_agl_skipped || gate_agl_pass) &&
           (gate_zone_skipped || gate_zone_pass);
  }
};

struct EvalParams {
  // A completed flight ends when the LAST waypoint is captured, so its
  // terminal error is bounded below by the follower's acceptance radius —
  // a literal gate here would just measure that radius. Callers derive it
  // (1.5 x accept radius is the harness convention).
  double terminal_pos_gate_m{150.0};
  double len_ratio_lo{0.95}, len_ratio_hi{1.25};
};

ReproductionMetrics evaluateReproduction(
    const SourcePath &src, const RolloutResult &flown,
    const mmp_vehicle_dynamics::Parameters &dyn, const SafetyHooks &hooks,
    const EvalParams &ep = EvalParams{});

}  // namespace waypoint_eval
}  // namespace path_manager

#endif  // PATH_MANAGER_WAYPOINT_EVAL_H
