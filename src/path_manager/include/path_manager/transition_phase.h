#ifndef PATH_MANAGER_TRANSITION_PHASE_H
#define PATH_MANAGER_TRANSITION_PHASE_H

// [S13] Optimization-free transition generator — v1.
//
// 프로젝트 정의 중립 3DOF 벤치마크 모델(공개 운동방정식 + 명시적 가정
// 파라미터) 기반 알고리즘·연결 구조 검증이며, 실제 플랫폼 물리 검증이
// 아니다.
//
// Pure component (TerminalPhase shape): no PathManager include, no node,
// no params, no logging. Terrain, zone policy and the handoff envelope
// enter as injected closures composed by the coordinator; the component
// forward-propagates the neutral point-mass EOM with RK4 over a small
// deterministic control-primitive family and returns the first candidate
// that survives every fail-closed gate. No trajectory optimization, no
// scoring — and explicitly no exposure-based ranking: zoneExposureRaw
// feeds the audit only.

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mmp_vehicle_dynamics/point_mass_eom.hpp"
#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {
namespace transition_phase {

struct TransitionLimits {
  mmp_vehicle_dynamics::Parameters dyn;  // the single assumption set
  double unit_xy_m{100.0};               // planner-unit scale (adapter)
  double unit_z_m{100.0};
  double dt_s{0.02};                     // RK4 step (convergence-tested)
  double t_max_s{60.0};                  // per-candidate horizon cap
  double min_agl_m{15.0};
  // Handoff speed window — COMPUTED by the coordinator from the injected
  // Parameters (speed_min*(1+margin), min(speed_max, handoff cap)), never
  // hardcoded here.
  double end_speed_min_mps{0.0};
  double end_speed_max_mps{0.0};
  // Capture = a TIME-TO-GO window on the entry tangent (t_go in
  // [1.5 s, 0.85 x blend_t_max]) with a lateral corridor: the blend piece
  // then spans t_go seconds of along-track flight, so it never has to
  // teleport (a radius trigger 30 m out at cruise speed left 0.2 s of
  // polynomial time — the quintic answered with six-figure thrust).
  // The blend must absorb this offset within its CL headroom: the
  // t_go capture window is sized so the correction acceleration
  // 5.77*lateral/T^2 stays a small fraction of g (10 m at a 2.3 s blend
  // demanded cl 1.42 > 1.40; at ~5 s it is 0.22 g — and a 6 m corridor
  // was beyond the fixed law's vertical convergence bandwidth entirely).
  double capture_lateral_m{10.0};
  double capture_align_rad{0.10};
  // §8 experimental initial value, not an acceptance-calibrated number.
  double dwell_s{1.0};
  double knot_dt_s{0.5};                 // adapter knot spacing (snapped)
  double blend_t_max_s{6.0};
  // Adapter interior budgets (poly vs dense RK4 states, SI):
  double adapter_pos_tol_m{2.0};
  double adapter_vel_tol_mps{4.0};
  double adapter_acc_tol_mps2{8.0};
};

struct EntryCandidate {
  int route_vertex{-1};        // derived value only; arc is the identity
  double route_start_s{0.0};   // arc on the committed route (planner units)
  Eigen::Vector3d pos_m{0.0, 0.0, 0.0};      // exact route point (SI)
  Eigen::Vector3d tangent{1.0, 0.0, 0.0};    // unit route tangent
};

// Zone judgment through the coordinator's snapshot closure. STALE/INVALID
// aborts the whole generation — it is never "no contact".
enum class ZoneProbe { CLEAR, CONTACT_HARD, STALE_OR_INVALID };

struct TransitionRequest {
  Eigen::Vector3d initial_pos_m{0.0, 0.0, 0.0};
  Eigen::Vector3d initial_vel_mps{0.0, 0.0, 0.0};
  // ZERO = unspecified (model-implied start acc, mismatch reported in
  // the audit). NONZERO = commanded: the returned trajectory leaves with
  // it EXACTLY (a C2 start bridge absorbs the model gap) or generation
  // refuses — a commanded start state is never silently rewritten.
  Eigen::Vector3d initial_acc_mps2{0.0, 0.0, 0.0};
  std::vector<EntryCandidate> entry_candidates;
  TransitionLimits limits;
  // Closures (all SI). Null zone_probe or null pva_problem refuses
  // generation; null terrain_z with min_agl_m > 0 refuses generation
  // (a missing terrain SOURCE is not "sea level" — only a false lookup
  // WITH a source is, matching the terminal-phase water-floor rule).
  std::function<ZoneProbe(const Eigen::Vector3d &pos_m)> zone_probe;
  std::function<double(const Eigen::Vector3d &pos_m)> zone_exposure_raw;
  std::function<bool(double x_m, double y_m, double *elev_m)> terrain_z;
  std::function<std::string(const Eigen::Vector3d &p_m,
                            const Eigen::Vector3d &v_mps,
                            const Eigen::Vector3d &a_mps2)> pva_problem;
};

struct TransitionAudit {
  int candidates_enumerated{0};
  // Per-gate disqualification counters — the feasibility report that lets
  // primitive-family tuning converge without loosening any gate.
  int disq_finiteness{0};
  int disq_preguard{0};       // V/gamma bounds AHEAD of the EOM's silent
                              // guards (kMinSpeedForRates / kMinCosGamma)
  int disq_representable{0};
  int disq_saturated{0};
  int disq_terrain{0};
  int disq_zone{0};
  // Transition-model limits DURING propagation: speed band, dynamic
  // pressure, load factor — the same Parameters the EOM closes over,
  // enforced per step (saturation flags alone see only CL/thrust/bank).
  int disq_limits{0};
  int disq_timeout{0};
  int disq_end_pva{0};
  int disq_adapter{0};
  // zoneExposureRaw statistics — MEASUREMENT ONLY, never selection.
  // risk_* describe the WINNER trajectory; search_risk_* accumulate over
  // the whole enumeration (disqualified candidates included) — mixing
  // them made the winner's exposure unreadable (review find).
  double risk_max{0.0};
  double risk_integral{0.0};
  double search_risk_max{0.0};
  double search_risk_integral{0.0};
  double adapter_max_pos_err_m{0.0};
  double adapter_max_vel_err_mps{0.0};
  double adapter_max_acc_err_mps2{0.0};
  double dwell_achieved_s{0.0};
  // |commanded a0 - model-implied a0| (m/s^2). With the start blend the
  // returned trajectory LEAVES with the commanded acceleration exactly;
  // this records how much the blend had to absorb.
  double start_acc_mismatch_mps2{0.0};
  int winner_primitive_id{-1};   // enumeration index — reproducibility pin
};

struct TransitionResult {
  bool ok{false};
  std::string reason;
  // Reason precedence for the coordinator's PlanReason mapping:
  // adapter-stage failure vs generation failure.
  bool any_candidate_reached_adapter{false};
  poly_traj::Trajectory traj;    // PLANNER UNITS, quintic pieces
  Eigen::Vector3d end_pos_m{0.0, 0.0, 0.0};   // SI end PVA
  Eigen::Vector3d end_vel_mps{0.0, 0.0, 0.0};
  Eigen::Vector3d end_acc_mps2{0.0, 0.0, 0.0};
  double route_start_s{-1.0};
  double duration_s{0.0};
  TransitionAudit audit;
};

// The ONE entry point; pure and deterministic (no wall clock, no RNG, no
// threads, no unordered containers in the decision path).
TransitionResult generate(const TransitionRequest &req);

// Exposed for the harness (convergence/analytic variants): a single RK4
// step of the point-mass EOM under held commands, forces re-evaluated at
// every stage state; representable = AND over stages, saturated = OR.
// The state is NEVER clamped — an excursion must be caught by a gate.
struct StepFlags {
  bool representable{true};
  bool saturated{false};
};
mmp_vehicle_dynamics::PointMassState rk4Step(
    const mmp_vehicle_dynamics::Parameters &dyn,
    const mmp_vehicle_dynamics::PointMassState &s, double dt_s,
    double cl_cmd, double thrust_cmd_n, double bank_cmd_rad,
    StepFlags *flags);

// EOM-implied inertial acceleration at (state, closed inputs) — the knot
// acceleration the adapter uses; also the seam-jerk measurement input.
Eigen::Vector3d pointMassAcceleration(
    const mmp_vehicle_dynamics::Parameters &dyn,
    const mmp_vehicle_dynamics::PointMassState &s,
    const mmp_vehicle_dynamics::PointMassInputs &u);

}  // namespace transition_phase
}  // namespace path_manager

#endif  // PATH_MANAGER_TRANSITION_PHASE_H
