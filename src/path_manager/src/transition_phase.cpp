// [S13] Optimization-free transition generator — v1 implementation.
//
// 프로젝트 정의 중립 3DOF 벤치마크 모델(공개 운동방정식 + 명시적 가정
// 파라미터) 기반 알고리즘·연결 구조 검증이며, 실제 플랫폼 물리 검증이
// 아니다.

#include "path_manager/transition_phase.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <cstdlib>
#include <limits>

#include "mmp_vehicle_dynamics/flight_dynamics.hpp"
#include "path_manager/quintic_hermite.h"

namespace path_manager {
namespace transition_phase {

namespace {

using mmp_vehicle_dynamics::Parameters;
using mmp_vehicle_dynamics::PointMassDerivative;
using mmp_vehicle_dynamics::PointMassInputs;
using mmp_vehicle_dynamics::PointMassState;

// Deterministic control-law constants. These are part of the fixed
// primitive definition (reviewable, regression-pinned via the winner id),
// not tuned controllers. v1-DEFERRED(param-array config): the family is
// compiled in; config knobs stay scalars.
constexpr double kGammaTargetsRad[] = {-0.1745, 0.0, 0.1745, 0.3491};
constexpr double kBankLevelsRad[] = {0.3, 0.6};
constexpr double kGammaRampRadPerS = 0.15;   // enumerated-target approach
constexpr double kGammaGain = 0.6;           // CL law: gamma-tracking [1/s]
constexpr double kPsiGain = 0.5;             // bank proportional band [rad]
constexpr double kSteerTgoS = 30.0;  // LOS capture-steer inside this t_go
constexpr double kSteerBlendS = 5.0; // ramp->LOS reference CROSS-FADE: a
                                     // hard branch switch stepped the CL
                                     // command level->pushover in one dt
                                     // (measured 8.6 m/s^2 record jump at
                                     // the t_go=30 boundary — no polynomial
                                     // fits a step, the zero-slack
                                     // validator rightly refused)
                                     // (600 m of vertical offset at 15 s
                                     // left 69 m unconverged at the window)
constexpr double kTgoMinS = 1.5;     // shortest blend the envelope can fly
constexpr double kLeadS = 4.0;
// Commanded-start ramp: a NONZERO commanded acceleration is honored by
// COMMAND-SPACE interpolation — inverse dynamics gives the command set
// u0 that produces a0 exactly, and the first kStartBlendS seconds ramp
// u0 -> the primitive law. The propagation itself then LEAVES with the
// commanded acceleration, every step passes the ordinary gates, and no
// posthoc polynomial bridge exists to overshoot the thrust ceiling
// (a position-level bridge to the full-throttle record demanded ~3.21 kN
// > 3.2 kN for ANY bridge duration: a start decelerating harder than the
// record must out-accelerate full throttle to catch it).
constexpr double kStartBlendS = 4.0;  // start-seam bridge: the trajectory
                                      // LEAVES with the commanded PVA and
                                      // reaches a model state over this
                                      // window — or the envelope refuses       // pursuit lead: steer onto the entry
                                     // LINE this many seconds ahead, so
                                     // lateral offset decays EARLY (chasing
                                     // the point itself keeps lateral
                                     // proportional to t_go — never inside
                                     // the corridor when the window opens)
constexpr double kSpeedTargetFrac = 1.02;    // hold V_target just above floor
// Interior-margin command discipline: a demand within this fraction of a
// limit is DISQUALIFIED before pointMassForces ever clamps it — accepted
// candidates prove strict interior margins, and the EOM's saturation
// flags become pure implementation-drift tripwires.
constexpr double kCmdInteriorFrac = 1e-3;
// The law's idle command sits 2% of the thrust range ABOVE thrust_min —
// symmetric with the margin-backed top (t_hi = max*(1-margin)). Riding
// the exact floor left the polynomial's fit noise (~2 N) crossing the
// bare lower bound at any knot density: an ACTIVE boundary admits no
// interior-margin proof (review round: zero-slack validation).
constexpr double kIdleMarginFrac = 0.02;
constexpr double kThrustGainNPerMps = 400.0;  // speed-servo slope (continuous)
// Pre-guard bounds AHEAD of the EOM's silent guards: disqualification must
// provably precede the kMinSpeedForRates / kMinCosGamma distortion, so an
// accepted trajectory never flew a silently-guarded derivative.
constexpr double kPreguardSpeedMps =
    2.0 * mmp_vehicle_dynamics::kMinSpeedForRates;
constexpr double kPreguardGammaRad = 1.40;   // |cos| = 0.17 >> kMinCosGamma

PointMassState advance(const PointMassState &s, const PointMassDerivative &d,
                       double h)
{
  PointMassState out;
  out.position_m = s.position_m + h * d.position;
  out.speed_mps = s.speed_mps + h * d.speed;
  out.flight_path_angle_rad = s.flight_path_angle_rad + h * d.flight_path_angle;
  out.heading_rad = s.heading_rad + h * d.heading;
  return out;
}

double wrapPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

bool finiteState(const PointMassState &s)
{
  return s.position_m.allFinite() && std::isfinite(s.speed_mps) &&
         std::isfinite(s.flight_path_angle_rad) &&
         std::isfinite(s.heading_rad);
}

// Command set for one step, synthesized from the step-start state by the
// fixed laws. demand_interior reports whether every raw demand sat
// strictly inside the envelope (kCmdInteriorFrac margin) BEFORE closing.
struct Commands {
  double cl{0.0};
  double thrust_n{0.0};
  double bank_rad{0.0};
  double gamma_cmd_applied{0.0};  // authority-bounded (anti-windup)
  bool demand_interior{true};
};

Commands synthesizeCommands(const Parameters &dyn, const PointMassState &s,
                            const Eigen::Vector3d &aim_m, double gamma_cmd,
                            double bank_level, const TransitionLimits &lim)
{
  const double capture_align_rad = lim.capture_align_rad;
  Commands c;
  // Bank: proportional band toward the aim bearing, zero once aligned.
  const Eigen::Vector3d to_entry = aim_m - s.position_m;
  const double psi_err =
      wrapPi(std::atan2(to_entry.y(), to_entry.x()) - s.heading_rad);
  // Continuous proportional bank (no deadband cutoff: the hard zeroing
  // at capture_align stepped the lateral acceleration whenever the
  // heading error crossed the threshold; the proportional law already
  // vanishes smoothly as the error does).
  (void)capture_align_rad;
  const double bank =
      (psi_err > 0.0 ? 1.0 : -1.0) *
      std::min(bank_level, std::abs(psi_err) / kPsiGain * bank_level);
  const double bank_hi =
      dyn.bank_angle_max_rad * (1.0 - kCmdInteriorFrac);
  if (std::abs(bank) > bank_hi) c.demand_interior = false;
  c.bank_rad = bank;

  // CL: inverse of the gamma-rate equation at the commanded bank — but
  // the tracking gap fed to it is BOUNDED by the model's own gamma-rate
  // authority (80% of each side). Downward authority is the zero-lift
  // rate -g cos(gamma)/V (negative CL is unreachable: cl_lo = max(0,
  // cl_min), a pushover cannot be commanded); upward authority is the CL
  // headroom at the commanded bank. A commanded gap beyond authority is
  // therefore followed at the achievable rate instead of demanding an
  // impossible lift — and if the UPWARD headroom itself is negative the
  // wing cannot hold gamma at all (stall): honest disqualification.
  const double rho =
      mmp_vehicle_dynamics::airDensity(dyn, s.position_m.z());
  const double q = 0.5 * rho * s.speed_mps * s.speed_mps;
  const double qs = std::max(1e-9, q * dyn.wing_area_m2);
  const double cl_lo = std::max(0.0, dyn.lift_coefficient_min);
  const double cl_hi = dyn.lift_coefficient_max * (1.0 - kCmdInteriorFrac);
  const double gcos = dyn.gravity_mps2 * std::cos(s.flight_path_angle_rad);
  const double vk = std::max(1.0, s.speed_mps) * kGammaGain;
  // Down-authority 0.95: a near-zero-lift pushover (CL demand = 5% of
  // level lift, still interior). 0.8 delayed steep-climb recovery ~2 s
  // and drained the energy budget straight into the stall gate.
  const double gap_dn = -0.95 * gcos / vk;
  const double lift_ceiling =
      qs * cl_hi * std::max(0.1, std::cos(c.bank_rad)) / dyn.mass_kg;
  const double gap_up = 0.8 * (lift_ceiling - gcos) / vk;
  if (gap_up <= 0.0) {
    if (std::getenv("TP_DEBUG"))
      std::fprintf(stderr,
                   "[TPDBG-STALL] V=%.2f z=%.1f rho=%.4f q=%.1f qs=%.1f "
                   "clhi=%.4f ceil=%.3f gcos=%.3f gapup=%.5f\n",
                   s.speed_mps, s.position_m.z(), rho, q, qs, cl_hi,
                   lift_ceiling, gcos, gap_up);
    c.demand_interior = false;  // stall: no CL holds gamma at this q
    c.gamma_cmd_applied = s.flight_path_angle_rad;
    return c;
  }
  const double gap = std::min(
      std::max(gamma_cmd - s.flight_path_angle_rad, gap_dn), gap_up);
  c.gamma_cmd_applied = s.flight_path_angle_rad + gap;
  const double lift_needed =
      dyn.mass_kg * (gcos + vk * gap) /
      std::max(0.1, std::cos(c.bank_rad));
  const double cl_demand = lift_needed / qs;
  if (cl_demand < cl_lo || cl_demand > cl_hi) c.demand_interior = false;
  c.cl = std::min(std::max(cl_demand, cl_lo), cl_hi);

  // Thrust: energy management against the ALTITUDE-DEPENDENT stall
  // floor, not just the handoff window. Below max(v_target, 1.1 x stall)
  // the wing cannot buy level lift inside CL limits, so full margin
  // power; above the window ceiling, idle; in between, drag +
  // gravity-compensation hold. min(t_hi, .) on a climb is the LAW
  // choosing maximum available power (a decision), not a silent clamp —
  // an unsustainable climb bleeds speed and the CL/stall gates judge
  // the result.
  const double cd = dyn.zero_lift_drag_coefficient +
                    dyn.induced_drag_factor * c.cl * c.cl;
  const double drag_n = q * dyn.wing_area_m2 * cd;
  const double t_hi = dyn.thrust_max_n * (1.0 - dyn.constraint_margin);
  const double W = dyn.mass_kg * dyn.gravity_mps2;
  const double stall = std::sqrt(
      2.0 * W / std::max(1e-9, rho * dyn.wing_area_m2 * cl_hi));
  const double v_lo = std::max(
      lim.end_speed_min_mps * kSpeedTargetFrac, 1.1 * stall);
  const double v_hi = lim.end_speed_max_mps * 0.98;
  const double t_lo =
      dyn.thrust_min_n +
      kIdleMarginFrac * (dyn.thrust_max_n - dyn.thrust_min_n);
  // CONTINUOUS thrust law: hold (drag + gravity compensation) plus a
  // proportional speed servo toward the [v_lo, v_hi] band, clamped to
  // [t_lo, t_hi]. The earlier hard branch switch (full power below v_lo,
  // idle above v_hi) put ~2-3 m/s^2 acceleration DISCONTINUITIES into
  // the record whenever V rode a band edge (real-terrain smoke: the
  // fit's acceleration error pinned at ~4 m/s^2 at EVERY knot density —
  // no polynomial follows a square wave, and the zero-slack validator
  // rightly refused the curve). A continuous law is fittable; the
  // window gates still judge the outcome.
  const double v_ref =
      std::min(std::max(s.speed_mps, v_lo), v_hi);  // nearest band point
  const double t_hold = std::max(
      t_lo,
      std::min(t_hi, drag_n + W * std::sin(s.flight_path_angle_rad)));
  const double thrust = std::max(
      t_lo, std::min(t_hi, t_hold + kThrustGainNPerMps *
                                        (v_ref - s.speed_mps)));
  if (thrust > dyn.thrust_max_n * (1.0 - kCmdInteriorFrac))
    c.demand_interior = false;
  c.thrust_n = std::min(
      std::max(thrust, dyn.thrust_min_n),
      dyn.thrust_max_n * (1.0 - kCmdInteriorFrac));
  return c;
}


}  // namespace

Eigen::Vector3d pointMassAcceleration(const Parameters &dyn,
                                      const PointMassState &s,
                                      const PointMassInputs &u)
{
  const PointMassDerivative d =
      mmp_vehicle_dynamics::pointMassDerivative(dyn, s, u);
  const double cg = std::cos(s.flight_path_angle_rad);
  const double sg = std::sin(s.flight_path_angle_rad);
  const double cp = std::cos(s.heading_rad);
  const double sp = std::sin(s.heading_rad);
  const Eigen::Vector3d dir(cg * cp, cg * sp, sg);
  const Eigen::Vector3d ddir_dgamma(-sg * cp, -sg * sp, cg);
  const Eigen::Vector3d ddir_dpsi(-cg * sp, cg * cp, 0.0);
  return d.speed * dir +
         s.speed_mps * (d.flight_path_angle * ddir_dgamma +
                        d.heading * ddir_dpsi);
}

// Signed inverse of the point-mass closure at one state: decompose the
// required aero+thrust force m*(a + g*z) in the wind triad (t, n, l) —
// l = (-sin psi, cos psi, 0), the direction a POSITIVE bank accelerates
// toward (the shared evaluateInverseDynamics reports |bank| via acos and
// loses exactly this sign; a left-turn start command would come back as
// a right-bank u0 and the repro check would refuse every lateral start).
void invertPointMassCommands(const Parameters &dyn,
                             const PointMassState &s,
                         const Eigen::Vector3d &a_cmd, double *cl,
                         double *thrust_n, double *bank_rad)
{
  const double cg = std::cos(s.flight_path_angle_rad);
  const double sg = std::sin(s.flight_path_angle_rad);
  const double cp = std::cos(s.heading_rad);
  const double sp = std::sin(s.heading_rad);
  const Eigen::Vector3d t_hat(cg * cp, cg * sp, sg);
  const Eigen::Vector3d n_hat(-sg * cp, -sg * sp, cg);
  const Eigen::Vector3d l_hat(-sp, cp, 0.0);
  const Eigen::Vector3d F =
      dyn.mass_kg *
      (a_cmd + Eigen::Vector3d(0.0, 0.0, dyn.gravity_mps2));
  const double f_t = F.dot(t_hat);
  const double f_n = F.dot(n_hat);
  const double f_l = F.dot(l_hat);
  const double lift = std::hypot(f_n, f_l);
  *bank_rad = lift > 1e-12 ? std::atan2(f_l, f_n) : 0.0;
  const double rho =
      mmp_vehicle_dynamics::airDensity(dyn, s.position_m.z());
  const double qs = std::max(
      1e-9, 0.5 * rho * s.speed_mps * s.speed_mps * dyn.wing_area_m2);
  *cl = lift / qs;
  const double cd = dyn.zero_lift_drag_coefficient +
                    dyn.induced_drag_factor * (*cl) * (*cl);
  *thrust_n = f_t + qs * cd;
}

PointMassState rk4Step(const Parameters &dyn, const PointMassState &s,
                       double dt_s, double cl_cmd, double thrust_cmd_n,
                       double bank_cmd_rad, StepFlags *flags)
{
  StepFlags f;
  // Forces re-evaluated at EVERY stage state (commands held): the frozen-
  // forces shortcut integrates a different system than the one gated.
  // representable = AND over stages, saturated = OR over stages.
  const auto stage = [&](const PointMassState &x) {
    const auto r = mmp_vehicle_dynamics::pointMassForces(
        dyn, x, cl_cmd, thrust_cmd_n, bank_cmd_rad);
    f.representable = f.representable && r.state_representable;
    f.saturated = f.saturated || r.saturated();
    return mmp_vehicle_dynamics::pointMassDerivative(dyn, x, r.inputs);
  };
  const PointMassDerivative k1 = stage(s);
  const PointMassDerivative k2 = stage(advance(s, k1, 0.5 * dt_s));
  const PointMassDerivative k3 = stage(advance(s, k2, 0.5 * dt_s));
  const PointMassDerivative k4 = stage(advance(s, k3, dt_s));
  PointMassDerivative sum;
  sum.position =
      (k1.position + 2.0 * k2.position + 2.0 * k3.position + k4.position) /
      6.0;
  sum.speed = (k1.speed + 2.0 * k2.speed + 2.0 * k3.speed + k4.speed) / 6.0;
  sum.flight_path_angle =
      (k1.flight_path_angle + 2.0 * k2.flight_path_angle +
       2.0 * k3.flight_path_angle + k4.flight_path_angle) /
      6.0;
  sum.heading =
      (k1.heading + 2.0 * k2.heading + 2.0 * k3.heading + k4.heading) / 6.0;
  if (flags) *flags = f;
  // NO state clamping — an excursion must be caught by a gate, never
  // silently absorbed (the follower-node integrator clamps; this one must
  // not, or the gates judge a different trajectory than the one flown).
  return advance(s, sum, dt_s);
}

namespace {

// Dense propagation record for one candidate.
struct Sample {
  PointMassState state;
  Eigen::Vector3d acc;   // EOM-implied inertial acceleration (SI)
};

struct CandidateOutcome {
  bool captured{false};
  bool reached_adapter{false};
  std::vector<Sample> samples;   // at dt_s spacing, index 0 = start
  double dwell_s{0.0};
  double risk_max{0.0};          // search-side exposure (this candidate)
  double risk_integral{0.0};
};

}  // namespace

TrajectoryVerdict validateTransitionTrajectory(
    poly_traj::Trajectory traj, const TransitionRequest &req,
    double *risk_max, double *risk_integral)
{
  const TransitionLimits &lim = req.limits;
  const Parameters &dyn = lim.dyn;
  const double ux = lim.unit_xy_m, uz = lim.unit_z_m;
  const double total_T = traj.getTotalDuration();
  if (!(total_T > 0.0)) return TrajectoryVerdict::FAIL;
  const double W = dyn.mass_kg * dyn.gravity_mps2;
  bool stale = false;
  double rmax = 0.0, rint = 0.0;
  // One sample: every gate at BARE limits (no slack anywhere).
  std::function<bool(double, bool)> sampleOk;
  sampleOk = [&](double t, bool refine) -> bool {
    const Eigen::Vector3d pu = traj.getPos(t);
    const Eigen::Vector3d vu = traj.getVel(t);
    const Eigen::Vector3d au = traj.getAcc(t);
    const Eigen::Vector3d p(pu.x() * ux, pu.y() * ux, pu.z() * uz);
    const Eigen::Vector3d v(vu.x() * ux, vu.y() * ux, vu.z() * uz);
    const Eigen::Vector3d a(au.x() * ux, au.y() * ux, au.z() * uz);
    if (!p.allFinite() || !v.allFinite() || !a.allFinite()) return false;
    const auto refineAround = [&](double tc) {
      for (double tt = std::max(0.0, tc - lim.dt_s);
           tt <= std::min(total_T, tc + lim.dt_s); tt += lim.dt_s / 16.0)
        if (!sampleOk(tt, false)) return false;
      return true;
    };
    if (req.terrain_z) {
      double elev = 0.0;
      if (!req.terrain_z(p.x(), p.y(), &elev)) elev = 0.0;
      const double margin = p.z() - elev - lim.min_agl_m;
      if (margin < 0.0) return false;
      if (refine && margin < 2.0 * lim.adapter_pos_tol_m &&
          !refineAround(t))
        return false;
    }
    switch (req.zone_probe(p)) {
      case ZoneProbe::CLEAR: break;
      case ZoneProbe::CONTACT_HARD: return false;
      case ZoneProbe::STALE_OR_INVALID: stale = true; return false;
    }
    const double V = v.norm();
    if (V > dyn.speed_max_mps + 1e-9 ||
        V < dyn.model_activation_speed_mps - 1e-9)
      return false;
    const double rho_t = mmp_vehicle_dynamics::airDensity(dyn, p.z());
    const double q_t = 0.5 * rho_t * V * V;
    if (q_t > dyn.dynamic_pressure_max_pa) return false;
    // Inverse dynamics EVERYWHERE (minus the cruise flight-path cone),
    // at BARE limits: the polynomial is the executed trajectory, so fit
    // error never widens a physical bound — a candidate whose curve
    // cannot satisfy them is re-expressed on finer knots or refused.
    const auto ev =
        mmp_vehicle_dynamics::evaluateInverseDynamics(dyn, p, v, a);
    const bool bad =
        !ev.valid || ev.load_factor > dyn.load_factor_max + 1e-9 ||
        ev.lift_coefficient > dyn.lift_coefficient_max + 1e-9 ||
        ev.thrust_required_n > dyn.thrust_max_n + 1e-6 ||
        ev.thrust_required_n < dyn.thrust_min_n - 1e-6 ||
        std::abs(ev.bank_angle_rad) > dyn.bank_angle_max_rad + 1e-9;
    if (bad) {
      if (std::getenv("TP_DEBUG"))
        std::fprintf(stderr,
                     "[TPDBG-VAL] t=%.2f/%.2f valid=%d V=%.2f q=%.0f "
                     "cl=%.3f nz=%.2f th=%.0f mu=%.3f\n",
                     t, total_T, ev.valid ? 1 : 0, ev.speed_mps,
                     ev.dynamic_pressure_pa, ev.lift_coefficient,
                     ev.load_factor, ev.thrust_required_n,
                     ev.bank_angle_rad);
      return false;
    }
    // Thin dynamic margins: refine the neighbourhood too.
    if (refine &&
        (ev.load_factor > dyn.load_factor_max - 0.1 ||
         ev.thrust_required_n > 0.98 * dyn.thrust_max_n ||
         q_t > 0.95 * dyn.dynamic_pressure_max_pa) &&
        !refineAround(t))
      return false;
    return true;
  };
  bool ok = true;
  for (double t = 0.0; ok && t < total_T;) {
    ok = sampleOk(t, true);
    const double step = std::min(
        lim.dt_s, 2.0 / std::max(1.0, traj.getVel(t).norm() *
                                          std::max(ux, uz)));
    if (ok && req.zone_exposure_raw) {
      // Exposure integrates with the ACTUAL sample step — the adaptive
      // spacing must not inflate the statistic (audit-only value).
      const Eigen::Vector3d pu = traj.getPos(t);
      const double e = req.zone_exposure_raw(
          Eigen::Vector3d(pu.x() * ux, pu.y() * ux, pu.z() * uz));
      rmax = std::max(rmax, e);
      rint += e * std::min(step, total_T - t);
    }
    t += step;
  }
  if (ok) ok = sampleOk(total_T, false);
  if (stale) return TrajectoryVerdict::STALE;
  if (risk_max) *risk_max = rmax;
  if (risk_integral) *risk_integral = rint;
  return ok ? TrajectoryVerdict::OK : TrajectoryVerdict::FAIL;
}

TransitionResult generate(const TransitionRequest &req)
{
  const bool dbg = std::getenv("TP_TRACE") != nullptr;
  if (dbg) std::fprintf(stderr, "[TP-TRACE] enter generate\n");
  TransitionResult out;
  TransitionAudit &audit = out.audit;
  const TransitionLimits &lim = req.limits;
  const Parameters &dyn = lim.dyn;

  // Preconditions — a missing judgment SOURCE refuses generation outright
  // (fail-closed): null closures cannot be "no problem".
  if (!req.zone_probe || !req.pva_problem) {
    out.reason = "zone/envelope closures missing — generation refused";
    return out;
  }
  if (!req.terrain_z && lim.min_agl_m > 0.0) {
    out.reason =
        "no terrain source while an AGL floor is required — refused";
    return out;
  }
  if (!mmp_vehicle_dynamics::parametersAreValid(dyn)) {
    out.reason = "assumption parameter set invalid — generation refused";
    return out;
  }
  if (req.entry_candidates.empty()) {
    out.reason = "no entry candidates";
    return out;
  }
  if (!(lim.dt_s > 0.0) || !(lim.t_max_s > 0.0) ||
      !(lim.end_speed_min_mps > 0.0) ||
      !(lim.end_speed_max_mps >= lim.end_speed_min_mps)) {
    out.reason = "transition limits malformed — generation refused";
    return out;
  }

  // Initial point-mass state from the commanded PVA.
  const double v0 = req.initial_vel_mps.norm();
  PointMassState s0;
  s0.position_m = req.initial_pos_m;
  s0.speed_mps = v0;
  s0.flight_path_angle_rad =
      v0 > 1e-9 ? std::asin(std::min(
                      1.0, std::max(-1.0, req.initial_vel_mps.z() / v0)))
                : 0.0;
  s0.heading_rad = std::atan2(req.initial_vel_mps.y(),
                              req.initial_vel_mps.x());

  // Entry candidates in ascending arc order (defensive stable sort — the
  // enumeration order is part of the determinism contract).
  std::vector<EntryCandidate> entries = req.entry_candidates;
  std::stable_sort(entries.begin(), entries.end(),
                   [](const EntryCandidate &a, const EntryCandidate &b) {
                     return a.route_start_s < b.route_start_s;
                   });

  const int max_steps =
      static_cast<int>(std::ceil(lim.t_max_s / lim.dt_s));
  const int n_gamma =
      static_cast<int>(sizeof(kGammaTargetsRad) / sizeof(double));
  const int n_bank =
      static_cast<int>(sizeof(kBankLevelsRad) / sizeof(double));

  bool zone_stale_abort = false;
  int primitive_id = -1;

  // Commanded-acc handling: the PLUMBED message bool decides, never the
  // numeric value ("prescribed exactly zero" is representable).
  const bool acc_specified = req.initial_acc_prescribed;
  double u0_cl = 0.0, u0_thrust = 0.0, u0_bank = 0.0;
  double start_acc_repro_err = 0.0;
  if (acc_specified) {
    invertPointMassCommands(dyn, s0, req.initial_acc_mps2, &u0_cl,
                            &u0_thrust, &u0_bank);
    // Accept u0 only if the EOM CLOSED over it reproduces the commanded
    // acceleration — sign errors, clamps and out-of-range demands all
    // surface here as a mismatch, and the mission is refused rather
    // than flown with a rewritten start.
    const auto r0 = mmp_vehicle_dynamics::pointMassForces(
        dyn, s0, u0_cl, u0_thrust, u0_bank);
    start_acc_repro_err =
        (pointMassAcceleration(dyn, s0, r0.inputs) - req.initial_acc_mps2)
            .norm();
    if (start_acc_repro_err > 1e-5 || r0.saturated() ||
        !r0.state_representable) {
      out.reason =
          "commanded start acceleration is not flyable by the model "
          "(inverse commands do not reproduce it inside the envelope)";
      return out;
    }
  }
  if (dbg)
    std::fprintf(stderr,
                 "[TP-TRACE] preconditions ok: %zu entries, dt=%.3f "
                 "tmax=%.1f window=[%.1f,%.1f] agl=%.1f\n",
                 entries.size(), lim.dt_s, lim.t_max_s,
                 lim.end_speed_min_mps, lim.end_speed_max_mps,
                 lim.min_agl_m);

  for (const EntryCandidate &entry : entries) {
    for (int gi = 0; gi < n_gamma && !zone_stale_abort; ++gi) {
      for (int bi = 0; bi < n_bank && !zone_stale_abort; ++bi) {
        ++primitive_id;
        ++audit.candidates_enumerated;
        const double gamma_target = std::min(
            std::max(kGammaTargetsRad[gi],
                     -dyn.flight_path_angle_max_rad *
                         (1.0 - kCmdInteriorFrac)),
            dyn.flight_path_angle_max_rad * (1.0 - kCmdInteriorFrac));
        const double bank_level = kBankLevelsRad[bi];

        if (dbg)
          std::fprintf(stderr, "[TP-TRACE] candidate prim=%d entry_s=%.1f\n",
                       primitive_id, entry.route_start_s);
        CandidateOutcome co;
        PointMassState s = s0;
        double gamma_cmd = s0.flight_path_angle_rad;
        double dwell = 0.0;
        bool disq = false;

        for (int k = 0; k <= max_steps; ++k) {
          // --- state gates on the CURRENT state -----------------------
          if (!finiteState(s)) { ++audit.disq_finiteness; disq = true; break; }
          if (s.speed_mps < kPreguardSpeedMps ||
              std::abs(s.flight_path_angle_rad) > kPreguardGammaRad) {
            ++audit.disq_preguard; disq = true; break;
          }
          if (dbg && k % 250 == 0)
            std::fprintf(
                stderr,
                "[TP-TRACE] prim=%d k=%d V=%.1f gam=%.3f z=%.0f "
                "dwell=%.1f\n",
                primitive_id, k, s.speed_mps, s.flight_path_angle_rad,
                s.position_m.z(), dwell);
          if (req.terrain_z) {
            double elev = 0.0;
            if (dbg && k == 0)
              std::fprintf(stderr, "[TP-TRACE] k0 terrain call\n");
            if (!req.terrain_z(s.position_m.x(), s.position_m.y(), &elev))
              elev = 0.0;   // sea-level floor: lookup false WITH a source
            if (s.position_m.z() - elev < lim.min_agl_m) {
              ++audit.disq_terrain; disq = true; break;
            }
          }
          if (dbg && k == 0)
            std::fprintf(stderr, "[TP-TRACE] k0 zone probe\n");
          switch (req.zone_probe(s.position_m)) {
            case ZoneProbe::CLEAR: break;
            case ZoneProbe::CONTACT_HARD:
              ++audit.disq_zone; disq = true; break;
            case ZoneProbe::STALE_OR_INVALID:
              zone_stale_abort = true; disq = true; break;
          }
          if (disq) break;
          if (req.zone_exposure_raw) {
            const double e = req.zone_exposure_raw(s.position_m);
            co.risk_max = std::max(co.risk_max, e);
            co.risk_integral += e * lim.dt_s;
          }

          // --- termination: overlap-region dwell + capture ------------
          const bool in_window =
              s.speed_mps >= lim.end_speed_min_mps &&
              s.speed_mps <= lim.end_speed_max_mps &&
              std::abs(s.flight_path_angle_rad) <=
                  dyn.flight_path_angle_max_rad;
          dwell = in_window ? dwell + lim.dt_s : 0.0;
          audit.dwell_achieved_s = std::max(audit.dwell_achieved_s, dwell);
          const Eigen::Vector3d to_entry = entry.pos_m - s.position_m;
          const double along = to_entry.dot(entry.tangent);
          const double lateral = (to_entry - along * entry.tangent).norm();
          const double t_go = along / std::max(1.0, s.speed_mps);
          const Eigen::Vector3d vel = mmp_vehicle_dynamics::pointMassVelocity(s);
          if (dbg && k % 250 == 0)
            std::fprintf(stderr,
                         "[TP-TRACE]   along=%.0f lat=%.1f tgo=%.1f\n",
                         along, lateral, t_go);
          if (dwell >= lim.dwell_s && along > 0.0 &&
              t_go >= kTgoMinS && t_go <= 0.85 * lim.blend_t_max_s &&
              lateral <= lim.capture_lateral_m &&
              vel.norm() > 1e-9 &&
              std::acos(std::min(
                  1.0, std::max(-1.0, vel.normalized().dot(entry.tangent)))) <=
                  lim.capture_align_rad) {
            co.captured = true;
            co.dwell_s = dwell;
            if (std::getenv("TP_DEBUG"))
              std::fprintf(stderr,
                           "[TPDBG-CAP] prim=%d k=%d tgo=%.2f lat=%.2f "
                           "V=%.2f gam=%.3f\n",
                           primitive_id, k, t_go, lateral, s.speed_mps,
                           s.flight_path_angle_rad);
            break;
          }
          if (k == max_steps) { ++audit.disq_timeout; disq = true; break; }

          // --- command synthesis + interior-margin discipline ---------
          // Pursuit aim: the point on the entry LINE kLeadS seconds
          // ahead — join the line early, ride it into the window.
          const Eigen::Vector3d aim =
              entry.pos_m -
              entry.tangent *
                  std::max(0.0, along - std::max(1.0, s.speed_mps) * kLeadS);
          const Eigen::Vector3d to_aim = aim - s.position_m;
          // Two gamma references, CROSS-FADED over kSteerBlendS around
          // the t_go = kSteerTgoS boundary (w continuous in state):
          // the enumerated-target ramp far out, the LOS elevation to the
          // aim point near capture.
          const double dg = gamma_target - gamma_cmd;
          const double step = kGammaRampRadPerS * lim.dt_s;
          const double g_ramp =
              gamma_cmd + std::min(std::max(dg, -step), step);
          const double cone =
              dyn.flight_path_angle_max_rad * (1.0 - kCmdInteriorFrac);
          const double los = std::atan2(
              to_aim.z(), std::max(1e-6, to_aim.head<2>().norm()));
          const double g_los = std::min(std::max(los, -cone), cone);
          const double w =
              !(along > 0.0)
                  ? 0.0
                  : std::min(1.0, std::max(0.0, (kSteerTgoS - t_go) /
                                                    kSteerBlendS));
          double g_cmd = (1.0 - w) * g_ramp + w * g_los;
          gamma_cmd = g_cmd;
          Commands cmd =
              synthesizeCommands(dyn, s, aim, g_cmd, bank_level, lim);
          gamma_cmd = cmd.gamma_cmd_applied;  // anti-windup: the ramp
          // continues from what the model could actually follow
          if (acc_specified && k * lim.dt_s < kStartBlendS) {
            // Command-space ramp from the inverse-dynamics start set to
            // the law; the blended commands face the SAME interior
            // discipline — an unflyable commanded acc dies here at k=0.
            const double w = (k * lim.dt_s) / kStartBlendS;
            cmd.cl = (1.0 - w) * u0_cl + w * cmd.cl;
            cmd.thrust_n = (1.0 - w) * u0_thrust + w * cmd.thrust_n;
            cmd.bank_rad = (1.0 - w) * u0_bank + w * cmd.bank_rad;
            const double cl_lo_b = std::max(0.0, dyn.lift_coefficient_min);
            if (cmd.cl < cl_lo_b - 1e-12 ||
                cmd.cl > dyn.lift_coefficient_max * (1.0 - kCmdInteriorFrac) ||
                cmd.thrust_n < dyn.thrust_min_n - 1e-12 ||
                cmd.thrust_n >
                    dyn.thrust_max_n * (1.0 - kCmdInteriorFrac) ||
                std::abs(cmd.bank_rad) >
                    dyn.bank_angle_max_rad * (1.0 - kCmdInteriorFrac))
              cmd.demand_interior = false;
          }
          if (!cmd.demand_interior) {
            if (std::getenv("TP_DEBUG"))
              std::fprintf(stderr,
                           "[TPDBG] prim=%d k=%d t=%.2f V=%.2f gam=%.4f "
                           "gcmd=%.4f cl=%.4f th=%.1f mu=%.3f z=%.1f\n",
                           primitive_id, k, k * lim.dt_s, s.speed_mps,
                           s.flight_path_angle_rad, g_cmd, cmd.cl,
                           cmd.thrust_n, cmd.bank_rad, s.position_m.z());
            ++audit.disq_saturated; disq = true; break;
          }

          // --- propagate (per-stage forces; flags gated) --------------
          StepFlags fl;
          const PointMassState next = rk4Step(
              dyn, s, lim.dt_s, cmd.cl, cmd.thrust_n, cmd.bank_rad, &fl);
          if (!fl.representable) {
            ++audit.disq_representable; disq = true; break;
          }
          if (fl.saturated) {
            // Interior demands still saturated inside pointMassForces —
            // implementation drift; honest disqualification either way.
            ++audit.disq_saturated; disq = true; break;
          }
          const auto closed = mmp_vehicle_dynamics::pointMassForces(
              dyn, s, cmd.cl, cmd.thrust_n, cmd.bank_rad);
          // Transition-model limits the saturation flags cannot see:
          // speed band, dynamic pressure, load factor — the SAME
          // Parameters the EOM closes over, enforced per step. (The
          // gamma range stays the pre-guard cone: the section-8
          // transition cone is not a confirmed number yet.)
          {
            const double rho_s =
                mmp_vehicle_dynamics::airDensity(dyn, s.position_m.z());
            const double q_s = 0.5 * rho_s * s.speed_mps * s.speed_mps;
            const double n_s = std::abs(closed.inputs.lift_n) /
                               (dyn.mass_kg * dyn.gravity_mps2);
            if (s.speed_mps > dyn.speed_max_mps ||
                s.speed_mps < dyn.model_activation_speed_mps ||
                q_s > dyn.dynamic_pressure_max_pa ||
                n_s > dyn.load_factor_max) {
              ++audit.disq_limits; disq = true; break;
            }
          }
          co.samples.push_back({s, pointMassAcceleration(dyn, s, closed.inputs)});
          s = next;
        }
        if (dbg)
          std::fprintf(stderr,
                       "[TP-TRACE] candidate prim=%d done: disq=%d "
                       "captured=%d samples=%zu\n",
                       primitive_id, disq ? 1 : 0, co.captured ? 1 : 0,
                       co.samples.size());
        audit.search_risk_max = std::max(audit.search_risk_max, co.risk_max);
        audit.search_risk_integral += co.risk_integral;
        if (zone_stale_abort) break;
        if (disq || !co.captured) continue;
        // Final captured state joins the record.
        {
          const Commands cmd = synthesizeCommands(
              dyn, s, entry.pos_m, gamma_cmd, bank_level, lim);
          const auto closed = mmp_vehicle_dynamics::pointMassForces(
              dyn, s, cmd.cl, cmd.thrust_n, cmd.bank_rad);
          co.samples.push_back({s, pointMassAcceleration(dyn, s, closed.inputs)});
        }

        // ==== post-capture gates ====================================
        // G4 end-PVA: the handoff state IS the blend target.
        const double v_end = std::min(
            std::max(s.speed_mps, lim.end_speed_min_mps),
            lim.end_speed_max_mps);
        const Eigen::Vector3d end_pos = entry.pos_m;
        const Eigen::Vector3d end_vel = entry.tangent * v_end;
        const Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();
        const std::string pva_problem =
            req.pva_problem(end_pos, end_vel, end_acc);
        if (!pva_problem.empty()) { ++audit.disq_end_pva; continue; }

        co.reached_adapter = true;

        // ==== G5a adapter: polynomialize + interior checks ==========
        out.any_candidate_reached_adapter = true;
        const double ux = lim.unit_xy_m, uz = lim.unit_z_m;
        const auto toU = [&](const Eigen::Vector3d &p) {
          return Eigen::Vector3d(p.x() / ux, p.y() / ux, p.z() / uz);
        };
        const auto toUvel = [&](const Eigen::Vector3d &v) {
          return Eigen::Vector3d(v.x() / ux, v.y() / ux, v.z() / uz);
        };
        if (std::getenv("TP_DEBUG")) {
          // Locate the record's worst acceleration jump — the fit can
          // never beat the record's own discontinuities.
          double worst = 0.0; size_t kw = 0;
          for (size_t k = 0; k + 1 < co.samples.size(); ++k) {
            const double dj =
                (co.samples[k + 1].acc - co.samples[k].acc).norm();
            if (dj > worst) { worst = dj; kw = k; }
          }
          std::fprintf(stderr,
                       "[TPDBG-JUMP] prim=%d worst dAcc=%.3f at k=%zu "
                       "(t=%.2f) acc_k=(%.2f,%.2f,%.2f) acc_k1="
                       "(%.2f,%.2f,%.2f) V=%.1f gam=%.3f\n",
                       primitive_id, worst, kw, kw * lim.dt_s,
                       co.samples[kw].acc.x(), co.samples[kw].acc.y(),
                       co.samples[kw].acc.z(), co.samples[kw + 1].acc.x(),
                       co.samples[kw + 1].acc.y(),
                       co.samples[kw + 1].acc.z(),
                       co.samples[kw].state.speed_mps,
                       co.samples[kw].state.flight_path_angle_rad);
        }
        // The knot ladder: express on the configured knot spacing; if
        // the BARE-limit validator rejects the curve, re-express on
        // halved knots (fit error shrinks ~16x per halving for a
        // quintic) and try again — never widen a physical limit to fit
        // the curve. All levels failing = candidate refused.
        const int last = static_cast<int>(co.samples.size()) - 1;
        const Sample &cap = co.samples.back();
        const Eigen::Vector3d cap_vel =
            mmp_vehicle_dynamics::pointMassVelocity(cap.state);
        // Blend spans the along-track time-to-go at capture, so the
        // quintic's average speed IS the flight speed (no teleport).
        const double blend_T = std::min(
            std::max((end_pos - cap.state.position_m).dot(entry.tangent) /
                         std::max(cap.state.speed_mps, 1.0),
                     kTgoMinS),
            lim.blend_t_max_s);
        const poly_traj::CoefficientMat blend_cm = quinticHermite(
            toU(cap.state.position_m), toUvel(cap_vel), toUvel(cap.acc),
            toU(end_pos), toUvel(end_vel), Eigen::Vector3d::Zero(),
            blend_T);
        poly_traj::Trajectory traj;
        double max_pe = 0.0, max_ve = 0.0, max_ae = 0.0;
        double w_risk_max = 0.0, w_risk_int = 0.0;
        bool accepted = false;
        const int kstep0 = std::max(
            1, static_cast<int>(std::round(lim.knot_dt_s / lim.dt_s)));
        for (int level = 0; level < 3 && !accepted && !zone_stale_abort;
             ++level) {
          const int kstep = std::max(1, kstep0 >> level);
          traj.clear();
          max_pe = max_ve = max_ae = 0.0;
          bool adapter_ok = last >= 1;
          for (int i0 = 0; adapter_ok && i0 < last; i0 += kstep) {
            const int i1 = std::min(last, i0 + kstep);
            const double T = (i1 - i0) * lim.dt_s;
            if (T <= 0.0) break;
            const Sample &a = co.samples[static_cast<size_t>(i0)];
            const Sample &b = co.samples[static_cast<size_t>(i1)];
            const poly_traj::CoefficientMat cm = quinticHermite(
                toU(a.state.position_m),
                toUvel(mmp_vehicle_dynamics::pointMassVelocity(a.state)),
                toUvel(a.acc), toU(b.state.position_m),
                toUvel(mmp_vehicle_dynamics::pointMassVelocity(b.state)),
                toUvel(b.acc), T);
            poly_traj::Piece piece(T, cm);
            // Interior: poly vs the dense RK4 record between the knots.
            for (int k = i0 + 1; k < i1; ++k) {
              const double t = (k - i0) * lim.dt_s;
              const Sample &m = co.samples[static_cast<size_t>(k)];
              const Eigen::Vector3d pp = piece.getPos(t);
              const Eigen::Vector3d pv = piece.getVel(t);
              const Eigen::Vector3d pa = piece.getAcc(t);
              const Eigen::Vector3d pe(
                  pp.x() * ux - m.state.position_m.x(),
                  pp.y() * ux - m.state.position_m.y(),
                  pp.z() * uz - m.state.position_m.z());
              const Eigen::Vector3d vm =
                  mmp_vehicle_dynamics::pointMassVelocity(m.state);
              const Eigen::Vector3d ve(pv.x() * ux - vm.x(),
                                       pv.y() * ux - vm.y(),
                                       pv.z() * uz - vm.z());
              const Eigen::Vector3d ae(pa.x() * ux - m.acc.x(),
                                       pa.y() * ux - m.acc.y(),
                                       pa.z() * uz - m.acc.z());
              max_pe = std::max(max_pe, pe.norm());
              max_ve = std::max(max_ve, ve.norm());
              max_ae = std::max(max_ae, ae.norm());
            }
            traj.emplace_back(T, cm);
          }
          if (std::getenv("TP_DEBUG"))
            std::fprintf(stderr,
                         "[TPDBG-LADDER] prim=%d level=%d kstep=%d "
                         "err p/v/a=%.3g/%.3g/%.3g ok=%d\n",
                         primitive_id, level, kstep, max_pe, max_ve,
                         max_ae, adapter_ok ? 1 : 0);
          if (!adapter_ok || max_pe > lim.adapter_pos_tol_m ||
              max_ve > lim.adapter_vel_tol_mps ||
              max_ae > lim.adapter_acc_tol_mps2)
            continue;   // representation quality gate (finer may help)
          traj.emplace_back(blend_T, blend_cm);
          const TrajectoryVerdict tv = validateTransitionTrajectory(
              traj, req, &w_risk_max, &w_risk_int);
          if (std::getenv("TP_DEBUG"))
            std::fprintf(stderr, "[TPDBG-LADDER] prim=%d level=%d verdict=%d\n",
                         primitive_id, level, static_cast<int>(tv));
          if (tv == TrajectoryVerdict::STALE) {
            zone_stale_abort = true;
            break;
          }
          accepted = tv == TrajectoryVerdict::OK;
        }
        if (zone_stale_abort) break;
        audit.adapter_max_pos_err_m =
            std::max(audit.adapter_max_pos_err_m, max_pe);
        audit.adapter_max_vel_err_mps =
            std::max(audit.adapter_max_vel_err_mps, max_ve);
        audit.adapter_max_acc_err_mps2 =
            std::max(audit.adapter_max_acc_err_mps2, max_ae);
        if (!accepted) { ++audit.disq_adapter; continue; }
        const double total_T = traj.getTotalDuration();

        // ==== winner ================================================
        out.ok = true;
        out.traj = traj;
        out.end_pos_m = end_pos;
        out.end_vel_mps = end_vel;
        out.end_acc_mps2 = end_acc;
        out.route_start_s = entry.route_start_s;
        out.duration_s = total_T;
        audit.winner_primitive_id = primitive_id;
        audit.dwell_achieved_s = co.dwell_s;
        // WINNER exposure statistics come from the final polynomial (the
        // flight that is actually handed off), not the search.
        audit.risk_max = w_risk_max;
        audit.risk_integral = w_risk_int;
        if (acc_specified) {
          audit.start_acc_repro_err_mps2 = start_acc_repro_err;
        } else {
          audit.start_acc_model_mps2 = co.samples.front().acc.norm();
        }
        out.reason.clear();
        return out;
      }
      if (out.ok || zone_stale_abort) break;
    }
    if (out.ok || zone_stale_abort) break;
  }

  if (zone_stale_abort) {
    out.reason =
        "zone policy went stale/invalid during generation — all "
        "candidates disqualified (fail-closed)";
    return out;
  }
  out.reason =
      out.any_candidate_reached_adapter
          ? "every captured candidate failed the polynomial adapter gates"
          : "no candidate survived propagation gates (see audit counters)";
  return out;
}

}  // namespace transition_phase
}  // namespace path_manager
