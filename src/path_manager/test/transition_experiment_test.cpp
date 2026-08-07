// [S13] Transition generator harness — model math, gate fail-closure and
// the deterministic primitive family. Same PASS/FAIL output convention as
// chain_experiment_test.
//
// 프로젝트 정의 중립 3DOF 벤치마크 모델(공개 운동방정식 + 명시적 가정
// 파라미터) 기반 알고리즘·연결 구조 검증이며, 실제 플랫폼 물리 검증이
// 아니다. Consistency between forward propagation and the inverse-dynamics
// evaluator is NAMED consistency — never physics validation.

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "mmp_vehicle_dynamics/flight_dynamics.hpp"
#include "path_manager/quintic_hermite.h"
#include "path_manager/transition_phase.h"

namespace tp = path_manager::transition_phase;
namespace vd = mmp_vehicle_dynamics;

static int failures = 0;
static void expect(bool ok, const std::string &what)
{
  std::cout << (ok ? "[OK]   " : "[FAIL] ") << what << "\n";
  if (!ok) ++failures;
}

// Neutral benchmark parameter set — the bundled defaults; per-variant
// overrides are stated inline with their reason.
static vd::Parameters baseParams() { return vd::Parameters{}; }

static vd::PointMassState makeState(const Eigen::Vector3d &p, double V,
                                    double gamma, double psi)
{
  vd::PointMassState s;
  s.position_m = p;
  s.speed_mps = V;
  s.flight_path_angle_rad = gamma;
  s.heading_rad = psi;
  return s;
}

// Trim commands for exact steady states (level / turn / climb).
struct Trim {
  double cl, thrust_n, bank;
};
static Trim trimFor(const vd::Parameters &p, const vd::PointMassState &s,
                    double bank)
{
  const double rho = vd::airDensity(p, s.position_m.z());
  const double q = 0.5 * rho * s.speed_mps * s.speed_mps;
  const double W = p.mass_kg * p.gravity_mps2;
  // gamma-rate zero: L cos(mu) = m g cos(gamma)
  const double lift = W * std::cos(s.flight_path_angle_rad) / std::cos(bank);
  const double cl = lift / (q * p.wing_area_m2);
  const double cd =
      p.zero_lift_drag_coefficient + p.induced_drag_factor * cl * cl;
  const double drag = q * p.wing_area_m2 * cd;
  // speed-rate zero: T = D + m g sin(gamma)
  const double thrust = drag + W * std::sin(s.flight_path_angle_rad);
  return {cl, thrust, bank};
}

// Default limits for generator variants (SI). The handoff window is
// COMPUTED from the parameter set, mirroring the coordinator rule.
static tp::TransitionLimits makeLimits(const vd::Parameters &dyn)
{
  tp::TransitionLimits lim;
  lim.dyn = dyn;
  lim.end_speed_min_mps = dyn.speed_min_mps * (1.0 + dyn.constraint_margin);
  lim.end_speed_max_mps = dyn.speed_max_mps;
  lim.min_agl_m = 15.0;
  return lim;
}

static tp::TransitionRequest makeRequest(const vd::Parameters &dyn)
{
  tp::TransitionRequest req;
  req.limits = makeLimits(dyn);
  // gamma = 32 deg: OUTSIDE the cruise fpa envelope (30 deg) — the pinned
  // acceptance scenario: recover a steep climbing start to level cruise.
  const double g32 = 32.0 * M_PI / 180.0;
  req.initial_pos_m = Eigen::Vector3d(0.0, 0.0, 800.0);
  // 165 m/s: energy-rich enough to recover (this model's sustained
  // climb angle is ~9 deg at T/W~0.25, so a 32-deg climb is a decaying
  // state; at 140 m/s the same entry is genuinely unrecoverable and the
  // stall gate must refuse it — see the negative control below).
  req.initial_vel_mps =
      165.0 * Eigen::Vector3d(std::cos(g32), 0.0, std::sin(g32));
  req.initial_acc_mps2 = Eigen::Vector3d::Zero();
  // Entry candidates at several altitudes — the coordinator normally
  // offers up to phase/max_candidates; first-fit picks the one the
  // primitive family can actually reach.
  for (int i = 0; i < 3; ++i) {
    tp::EntryCandidate e;
    e.route_vertex = i;
    e.route_start_s = 48.0 + 2.0 * i;
    e.pos_m = Eigen::Vector3d(5000.0, 0.0, 950.0 + 150.0 * i);
    e.tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
    e.cap_z_m = 4000.0;
    req.entry_candidates.push_back(e);
  }
  req.zone_probe = [](const Eigen::Vector3d &) { return tp::ZoneProbe::CLEAR; };
  req.zone_exposure_raw = [](const Eigen::Vector3d &) { return 0.0; };
  req.terrain_z = [](double, double, double *e) { *e = 0.0; return true; };
  req.pva_problem = [](const Eigen::Vector3d &, const Eigen::Vector3d &,
                       const Eigen::Vector3d &) { return std::string(); };
  return req;
}

int main(int argc, char **argv)
{
  std::string variant = argc > 1 ? argv[1] : "all";
  const auto run = [&](const char *name) {
    return variant == "all" || variant == name;
  };

  // ---------------- analytic_level ---------------------------------
  if (run("analytic_level")) {
    // T = D, L = mg, mu = 0 at constant altitude: the derivative is
    // EXACTLY constant (rho fixed), so RK4 must reproduce the straight
    // line to machine precision.
    vd::Parameters p = baseParams();
    auto s = makeState({0, 0, 1000}, 160.0, 0.0, 0.3);
    const Trim tr = trimFor(p, s, 0.0);
    const Eigen::Vector3d v0 = vd::pointMassVelocity(s);
    tp::StepFlags fl;
    double t = 0.0;
    for (int k = 0; k < 250; ++k, t += 0.02)
      s = tp::rk4Step(p, s, 0.02, tr.cl, tr.thrust_n, tr.bank, &fl);
    const Eigen::Vector3d expect_p = Eigen::Vector3d(0, 0, 1000) + v0 * t;
    expect((s.position_m - expect_p).norm() < 1e-6,
           "level trim: straight line to machine precision");
    expect(std::abs(s.speed_mps - 160.0) < 1e-9 &&
               std::abs(s.flight_path_angle_rad) < 1e-9,
           "level trim: V and gamma exactly held");
    expect(!fl.saturated && fl.representable,
           "level trim: no saturation, representable throughout");
  }

  // ---------------- analytic_turn ----------------------------------
  if (run("analytic_turn")) {
    // Steady level turn: psi_dot = g tan(mu) / V, R = V^2/(g tan(mu)).
    vd::Parameters p = baseParams();
    const double mu = 0.5, V = 160.0;
    auto s = makeState({0, 0, 1000}, V, 0.0, 0.0);
    const Trim tr = trimFor(p, s, mu);
    const double T = 10.0;
    for (int k = 0; k < 500; ++k)
      s = tp::rk4Step(p, s, 0.02, tr.cl, tr.thrust_n, tr.bank, nullptr);
    const double psi_dot = p.gravity_mps2 * std::tan(mu) / V;
    expect(std::abs(s.heading_rad - psi_dot * T) < 1e-6,
           "steady turn: heading rate g tan(mu)/V");
    const double R = V * V / (p.gravity_mps2 * std::tan(mu));
    const Eigen::Vector3d center(0.0, R, 1000.0);
    expect(std::abs((s.position_m - center).norm() - R) < 2e-3,
           "steady turn: radius V^2/(g tan mu) (RK4 4th-order error)");
    expect(std::abs(s.speed_mps - V) < 1e-9 &&
               std::abs(s.flight_path_angle_rad) < 1e-9,
           "steady turn: V and gamma exactly held");
  }

  // ---------------- analytic_climb ---------------------------------
  if (run("analytic_climb")) {
    // Fixed-gamma line at constant V: T = D + mg sin(gamma), L = mg
    // cos(gamma). Density held constant (huge scale height) so the trim
    // is exact along the climb.
    vd::Parameters p = baseParams();
    p.density_scale_height_m = 1e12;
    // gamma small enough that the sustained-climb trim stays inside the
    // thrust envelope: D + W sin(gamma) < thrust_max (0.15 rad demanded
    // 3309 N > 3200 — the clamp then bled the "exact" trim, fixture bug).
    const double gamma = 0.08, V = 150.0;
    auto s = makeState({0, 0, 500}, V, gamma, 0.0);
    const Trim tr = trimFor(p, s, 0.0);
    const Eigen::Vector3d v0 = vd::pointMassVelocity(s);
    double t = 0.0;
    for (int k = 0; k < 400; ++k, t += 0.02)
      s = tp::rk4Step(p, s, 0.02, tr.cl, tr.thrust_n, tr.bank, nullptr);
    expect((s.position_m - (Eigen::Vector3d(0, 0, 500) + v0 * t)).norm() <
               1e-4,
           "climb trim: fixed-gamma line (const-density trim)");
    expect(std::abs(s.speed_mps - V) < 1e-6,
           "climb trim: V held (const-density trim)");
  }

  // ---------------- rk4_convergence --------------------------------
  if (run("rk4_convergence")) {
    // Open-loop: fixed commands OFF trim (phugoid-like exchange through
    // the density gradient) so the dynamics are genuinely nonlinear and
    // the end-state error sits far above the double-precision floor
    // (the first fixture used a near-trim arc whose 1e-11 errors were
    // pure float noise and the slope was meaningless). Error vs the
    // dt=1e-4 reference must shrink as O(dt^4).
    vd::Parameters p = baseParams();
    const auto propagate = [&](double dt, bool closed_loop) {
      auto s = makeState({0, 0, 800}, 145.0, 0.30, 0.1);
      const double T = 20.0;
      const int n = static_cast<int>(std::round(T / dt));
      for (int k = 0; k < n; ++k) {
        double cl = 0.95, th = 2200.0, mu = 0.25;
        if (closed_loop) {
          // A representative feedback law (gamma-hold toward 0): commands
          // recomputed each step — the system the generator actually
          // integrates. Its effective order is MEASURED and reported;
          // the dt choice must be justified against this, not the
          // open-loop slope-4 evidence.
          const Trim tr = trimFor(p, s, 0.2);
          cl = tr.cl * (1.0 - 0.4 * s.flight_path_angle_rad);
          th = tr.thrust_n;
          mu = 0.2;
        }
        s = tp::rk4Step(p, s, dt, cl, th, mu, nullptr);
      }
      return s;
    };
    for (int mode = 0; mode < 2; ++mode) {
      const bool closed = mode == 1;
      // dt grid sits ABOVE the double-precision floor: these dynamics
      // are slow (rates ~0.1/s), so at dt=0.02 the global error is
      // ~1e-10 — pure float noise, no measurable slope. The ORDER is
      // measured in the asymptotic regime; the production dt=0.02 then
      // sits far below the smallest measured-error grid point.
      const auto ref = propagate(1e-3, closed);
      std::vector<double> errs;
      for (double dt : {0.4, 0.2, 0.1, 0.05}) {
        const auto s = propagate(dt, closed);
        errs.push_back((s.position_m - ref.position_m).norm() +
                       std::abs(s.speed_mps - ref.speed_mps) * 1.0);
      }
      double slope_min = 1e9, slope_max = -1e9;
      for (size_t i = 0; i + 1 < errs.size(); ++i) {
        const double sl = std::log2(errs[i] / errs[i + 1]);
        slope_min = std::min(slope_min, sl);
        slope_max = std::max(slope_max, sl);
      }
      std::cout << (closed ? "closed" : "open")
                << "-loop convergence: errs";
      for (double e : errs) std::cout << " " << e;
      std::cout << " slopes [" << slope_min << ", " << slope_max << "]\n";
      if (!closed)
        expect(slope_min > 3.2,
               "open-loop RK4 end-state error is O(dt^4)");
      else
        expect(slope_min > 0.9,
               "closed-loop effective order measured and >= 1 (commands "
               "held per step bound the order; dt budget derives from "
               "THIS, the system actually integrated)");
    }
  }

  // ---------------- roundtrip_consistency --------------------------
  if (run("roundtrip_consistency")) {
    // Forward samples -> inverse dynamics: recovered thrust/lift/bank
    // match the closed inputs. NAMED consistency, not physics validation.
    vd::Parameters p = baseParams();
    auto s = makeState({0, 0, 900}, 155.0, 0.05, 0.4);
    double max_th = 0.0, max_lift = 0.0, max_bank = 0.0;
    bool env_agree = true;
    for (int k = 0; k < 200; ++k) {
      const auto r = vd::pointMassForces(p, s, 0.8, 2600.0, 0.35);
      const Eigen::Vector3d v = vd::pointMassVelocity(s);
      const Eigen::Vector3d a = tp::pointMassAcceleration(p, s, r.inputs);
      const auto ev = vd::evaluateInverseDynamics(p, s.position_m, v, a);
      if (!ev.valid) { env_agree = false; break; }
      max_th = std::max(max_th,
                        std::abs(ev.thrust_required_n - r.inputs.thrust_n));
      max_lift = std::max(
          max_lift, std::abs(ev.lift_coefficient * ev.dynamic_pressure_pa *
                                 p.wing_area_m2 -
                             std::abs(r.inputs.lift_n)));
      max_bank =
          std::max(max_bank, std::abs(ev.bank_angle_rad - r.inputs.bank_rad));
      if (!vd::isWithinEnvelope(p, ev)) env_agree = false;
      s = tp::rk4Step(p, s, 0.02, 0.8, 2600.0, 0.35, nullptr);
    }
    std::cout << "roundtrip: max thrust err " << max_th << " N, lift err "
              << max_lift << " N, bank err " << max_bank << " rad\n";
    expect(max_th < 1.0 && max_bank < 1e-6,
           "inverse dynamics recovers the applied thrust and bank");
    expect(max_lift < 5.0, "inverse dynamics recovers the applied lift");
    expect(env_agree,
           "interior trajectory judged within envelope throughout");
  }

  // ---------------- hermite_representation -------------------------
  if (run("hermite_representation")) {
    vd::Parameters p = baseParams();
    // A genuinely maneuvering record: climbing turn.
    std::vector<vd::PointMassState> states;
    std::vector<Eigen::Vector3d> accs;
    auto s = makeState({0, 0, 800}, 150.0, 0.1, 0.0);
    for (int k = 0; k <= 400; ++k) {
      const auto r = vd::pointMassForces(p, s, 0.85, 2700.0, 0.3);
      states.push_back(s);
      accs.push_back(tp::pointMassAcceleration(p, s, r.inputs));
      s = tp::rk4Step(p, s, 0.02, 0.85, 2700.0, 0.3, nullptr);
    }
    double prev_err = 1e18;
    bool endpoint_ok = true, shrinking = true;
    for (double knot_dt : {0.5, 0.25, 0.1}) {
      const int kstep = static_cast<int>(std::round(knot_dt / 0.02));
      double max_err = 0.0;
      for (size_t i0 = 0; i0 + kstep < states.size();
           i0 += static_cast<size_t>(kstep)) {
        const size_t i1 = i0 + static_cast<size_t>(kstep);
        const double T = kstep * 0.02;
        const auto cm = path_manager::quinticHermite(
            states[i0].position_m, vd::pointMassVelocity(states[i0]),
            accs[i0], states[i1].position_m,
            vd::pointMassVelocity(states[i1]), accs[i1], T);
        poly_traj::Piece piece(T, cm);
        if ((piece.getPos(0.0) - states[i0].position_m).norm() > 1e-9 ||
            (piece.getPos(T) - states[i1].position_m).norm() > 1e-7)
          endpoint_ok = false;
        for (int k = 1; k < kstep; ++k)
          max_err = std::max(
              max_err, (piece.getPos(k * 0.02) -
                        states[i0 + static_cast<size_t>(k)].position_m)
                           .norm());
      }
      std::cout << "hermite: knot_dt " << knot_dt << " interior max err "
                << max_err << " m\n";
      if (max_err > prev_err) shrinking = false;
      prev_err = max_err;
    }
    expect(endpoint_ok, "knot PVA matched at machine precision");
    expect(shrinking, "interior error decreases with knot density");
  }

  // ---------------- generator gates --------------------------------
  if (run("gate_saturation")) {
    vd::Parameters p = baseParams();
    p.wing_area_m2 = 0.01;  // q*S starved: every CL demand blows the limit
    auto req = makeRequest(p);
    const auto r = tp::generate(req);
    expect(!r.ok && r.audit.disq_saturated > 0,
           "starved lift authority: disqualified as saturated demand, "
           "never accepted with clamped inputs");
  }

  if (run("gate_preguard")) {
    auto req = makeRequest(baseParams());
    req.initial_vel_mps = Eigen::Vector3d(1.4, 0.0, 0.1);  // V ~ preguard
    const auto r = tp::generate(req);
    expect(!r.ok && r.audit.disq_preguard > 0,
           "near-singular start disqualified BEFORE the EOM's silent "
           "guards can distort a flyable-looking answer");
  }

  if (run("gate_terrain")) {
    auto req = makeRequest(baseParams());
    req.terrain_z = [](double x, double, double *e) {
      *e = x > 1500.0 ? 2000.0 : 0.0;  // a wall across every path
      return true;
    };
    const auto r = tp::generate(req);
    expect(!r.ok && r.audit.disq_terrain > 0,
           "terrain wall: AGL floor disqualifies every candidate");
    auto req2 = makeRequest(baseParams());
    req2.terrain_z = nullptr;  // NO SOURCE at all, floor still required
    const auto r2 = tp::generate(req2);
    expect(!r2.ok && r2.reason.find("terrain source") != std::string::npos,
           "no terrain source while AGL floor required: generation "
           "refused outright");
  }

  if (run("gate_zone")) {
    auto req = makeRequest(baseParams());
    req.zone_probe = [](const Eigen::Vector3d &p) {
      return (p.x() > 2000.0 && p.x() < 2400.0)
                 ? tp::ZoneProbe::CONTACT_HARD
                 : tp::ZoneProbe::CLEAR;
    };
    const auto r = tp::generate(req);
    expect(!r.ok && r.audit.disq_zone > 0,
           "hard-retained zone across the corridor disqualifies");
    auto req2 = makeRequest(baseParams());
    int calls = 0;
    req2.zone_probe = [&calls](const Eigen::Vector3d &) {
      return ++calls > 50 ? tp::ZoneProbe::STALE_OR_INVALID
                          : tp::ZoneProbe::CLEAR;
    };
    const auto r2 = tp::generate(req2);
    expect(!r2.ok && r2.reason.find("stale") != std::string::npos,
           "zone policy going stale mid-generation aborts EVERYTHING "
           "(never read as no-contact)");
  }

  if (run("gate_end_pva")) {
    auto req = makeRequest(baseParams());
    req.pva_problem = [](const Eigen::Vector3d &, const Eigen::Vector3d &,
                         const Eigen::Vector3d &) {
      return std::string("handoff below the margin floor");
    };
    const auto r = tp::generate(req);
    expect(!r.ok && r.audit.disq_end_pva > 0,
           "handoff envelope problem refuses the capture");
    expect(!r.any_candidate_reached_adapter,
           "end-PVA failures are generation failures, not adapter ones");
  }

  // ---------------- feasibility + determinism ----------------------
  if (run("feasibility")) {
    // THE pinned acceptance scenario: gamma = 32 deg (outside the 30 deg
    // cruise cone) at 150 m/s, recovered to a level route entry. The
    // committed primitive family must produce a surviving candidate —
    // this variant is the evidence, and any family edit re-answers it.
    auto req = makeRequest(baseParams());
    const auto r = tp::generate(req);
    std::cout << "feasibility: ok=" << r.ok << " reason='" << r.reason
              << "' winner=" << r.audit.winner_primitive_id
              << " enumerated=" << r.audit.candidates_enumerated
              << " duration=" << r.duration_s << " s\n"
              << "  disq: fin=" << r.audit.disq_finiteness
              << " pre=" << r.audit.disq_preguard
              << " rep=" << r.audit.disq_representable
              << " sat=" << r.audit.disq_saturated
              << " ter=" << r.audit.disq_terrain
              << " zone=" << r.audit.disq_zone
              << " time=" << r.audit.disq_timeout
              << " pva=" << r.audit.disq_end_pva
              << " adapter=" << r.audit.disq_adapter << "\n"
              << "  dwell=" << r.audit.dwell_achieved_s
              << " s, adapter errs p/v/a=" << r.audit.adapter_max_pos_err_m
              << "/" << r.audit.adapter_max_vel_err_mps << "/"
              << r.audit.adapter_max_acc_err_mps2
              << ", start-seam jump=" << r.audit.seam_jerk_start << "\n";
    expect(r.ok, "gamma=32 deg start recovers to a route entry");
    if (r.ok) {
      expect(r.duration_s > 1.0 && r.duration_s < 60.0,
             "transition duration sane");
      const double v_end = r.end_vel_mps.norm();
      expect(v_end >= req.limits.end_speed_min_mps - 1e-9 &&
                 v_end <= req.limits.end_speed_max_mps + 1e-9,
             "handoff speed inside the computed window (never hardcoded)");
      // The polynomial ends EXACTLY at the entry state (planner units).
      const auto &traj = r.traj;
      const double T = traj.getTotalDuration();
      const Eigen::Vector3d pu = const_cast<poly_traj::Trajectory &>(traj).getPos(T);
      const Eigen::Vector3d end_u(r.end_pos_m.x() / req.limits.unit_xy_m,
                                  r.end_pos_m.y() / req.limits.unit_xy_m,
                                  r.end_pos_m.z() / req.limits.unit_z_m);
      expect((pu - end_u).norm() < 1e-9,
             "trajectory tail touches the entry state exactly (blend)");
      expect(r.audit.risk_max == 0.0,
             "exposure statistics present and zero in a zone-free field");
    }
    // Negative control: the SAME 32-deg climb at 140 m/s is below this
    // model's recovery energy budget — the stall/saturation gates must
    // refuse it, never hand back a clamped fiction.
    auto req_lo = makeRequest(baseParams());
    const double g32b = 32.0 * M_PI / 180.0;
    req_lo.initial_vel_mps =
        140.0 * Eigen::Vector3d(std::cos(g32b), 0.0, std::sin(g32b));
    const auto r_lo = tp::generate(req_lo);
    expect(!r_lo.ok && r_lo.audit.disq_saturated > 0,
           "energy-deficient 32-deg climb honestly refused (stall gate)");
  }

  if (run("determinism")) {
    auto req = makeRequest(baseParams());
    const auto a = tp::generate(req);
    const auto b = tp::generate(req);
    expect(a.ok && b.ok, "both runs succeed");
    expect(a.audit.winner_primitive_id == b.audit.winner_primitive_id &&
               a.route_start_s == b.route_start_s &&
               a.end_pos_m == b.end_pos_m && a.end_vel_mps == b.end_vel_mps &&
               a.duration_s == b.duration_s &&
               a.audit.candidates_enumerated == b.audit.candidates_enumerated,
           "byte-identical outcome on identical requests (no clock, no "
           "RNG, no unordered state)");
  }

  if (failures == 0) {
    std::cout << "PASS: 0 failed check(s)\n";
    return 0;
  }
  std::cout << "FAIL: " << failures << " failed check(s)\n";
  return 1;
}
