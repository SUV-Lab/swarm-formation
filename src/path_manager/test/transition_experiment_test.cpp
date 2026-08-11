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
                 ? tp::ZoneProbe::CONTACT_AUTHORED
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
              << ", start acc repro/model="
              << r.audit.start_acc_repro_err_mps2 << "/"
              << r.audit.start_acc_model_mps2 << "\n";
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
      // UNPRESCRIBED (the plumbed message bool is false): the model-
      // implied start acceleration is used and REPORTED. Preserve/refuse
      // for prescribed starts is pinned by the acc_* variants.
      expect(r.audit.start_acc_model_mps2 > 0.0,
             "unprescribed start: model-implied acc measured and "
             "reported, never hidden");
    }
    // Negative control: the SAME 32-deg climb from BELOW the margin
    // floor (125 m/s < speed_min*(1+margin)) is under this model's
    // recovery energy budget — the stall/saturation gates must refuse
    // it, never hand back a clamped fiction. (140 m/s was refused under
    // the point-chase law, but the line-pursuit descent recovers it
    // honestly — the boundary moved DOWN when the law stopped wasting
    // energy, which is exactly what the audit counters are for.)
    auto req_lo = makeRequest(baseParams());
    const double g32b = 32.0 * M_PI / 180.0;
    req_lo.initial_vel_mps =
        125.0 * Eigen::Vector3d(std::cos(g32b), 0.0, std::sin(g32b));
    const auto r_lo = tp::generate(req_lo);
    expect(!r_lo.ok && r_lo.audit.disq_saturated > 0,
           "energy-deficient 32-deg climb honestly refused (stall gate)");
  }

  if (run("acc_preserve_lat")) {
    // SIGNED lateral acceleration inverse: the shared inverse reports
    // |bank| via acos and loses the turn direction. The transition's
    // signed inverse must return sign-correct bank AND reproduce the
    // commanded acceleration through the closed EOM to machine
    // precision, for +lateral, -lateral and pure-longitudinal commands.
    // (Mission-level lateral acceptance is a primitive-family capability
    // question — the audit counters diagnose it; the CONTRACT pinned
    // here is that no command is ever mirrored or silently bent.)
    vd::Parameters p = baseParams();
    const double g32 = 32.0 * M_PI / 180.0;
    const auto s0 = makeState({0.0, 0.0, 800.0}, 165.0, g32, 0.0);
    const Eigen::Vector3d along =
        -4.0 * Eigen::Vector3d(std::cos(g32), 0.0, std::sin(g32));
    for (double lat : {+3.0, -3.0, 0.0}) {
      const Eigen::Vector3d a_cmd = along + Eigen::Vector3d(0.0, lat, 0.0);
      double cl = 0.0, thrust = 0.0, bank = 0.0;
      tp::invertPointMassCommands(p, s0, a_cmd, &cl, &thrust, &bank);
      const auto r = vd::pointMassForces(p, s0, cl, thrust, bank);
      const Eigen::Vector3d a_chk = tp::pointMassAcceleration(p, s0, r.inputs);
      std::cout << "acc_preserve_lat: lat=" << lat << " bank=" << bank
                << " repro err=" << (a_chk - a_cmd).norm() << "\n";
      expect((a_chk - a_cmd).norm() < 1e-9,
             "signed inverse reproduces the commanded acceleration");
      expect(lat == 0.0 ? std::abs(bank) < 1e-9
                        : (lat > 0.0) == (bank > 0.0),
             "bank sign follows the commanded turn direction");
      expect(!r.saturated(), "reproduction used interior commands");
    }
  }

  if (run("classify_acc")) {
    // Classification must not read an UNPRESCRIBED internal acc value:
    // an in-cone climbing velocity (25 deg, inside the 30 deg cone) with
    // the FSM's internal a=0 is a CRUISE-VALID state — but the same PVA
    // with a PRESCRIBED zero is an unflyable hold (thrust ~ 6.6 kN of
    // 3.2 kN) and must classify differently. Pinned at the component
    // contract level here; the coordinator-side pin lives in the chain
    // harness transition variant.
    // (No node here: exercised through generate()'s own gates via the
    // prescribed path — the coordinator classifier pin is integration-
    // level.)
    auto req = makeRequest(baseParams());
    req.initial_acc_prescribed = false;
    req.initial_acc_mps2 = Eigen::Vector3d::Zero();
    const auto r_un = tp::generate(req);
    expect(r_un.ok, "unprescribed internal zero: mission generates");
    req.initial_acc_prescribed = true;
    const auto r_pre = tp::generate(req);
    expect(!r_pre.ok,
           "prescribed zero at the same PVA refused — the bool, never "
           "the value, decides");
  }

  if (run("acc_prescribed_zero")) {
    // Prescribed EXACTLY ZERO at a 32-deg climb: holding a=0 there needs
    // ~7.9 kN of the model's 3.2 kN — preserve is impossible, so the
    // mission must be REFUSED (a numeric-0 sentinel silently switched
    // this to model-derived; the plumbed bool cannot).
    auto req = makeRequest(baseParams());
    req.initial_acc_prescribed = true;
    req.initial_acc_mps2 = Eigen::Vector3d::Zero();
    const auto r = tp::generate(req);
    expect(!r.ok, "prescribed zero acc at a steep climb honestly refused");
  }

  if (run("acc_preserve")) {
    // A commanded, MODEL-FLYABLE nonzero start acceleration (ballistic
    // along-track deceleration at the 32-deg climb: thrust ~ drag, CL
    // ~ 0.86) must be preserved EXACTLY by the command-space ramp.
    auto req = makeRequest(baseParams());
    req.initial_acc_prescribed = true;
    const double g32 = 32.0 * M_PI / 180.0;
    // -4.0 along track: thrust ~ 2.7 kN, CL ~ 0.86 — comfortably inside
    // the envelope, and close enough to the model's own start acc
    // (~ -3.7 tangential) that the bridge's quintic interior overshoot
    // stays under the thrust ceiling (a -5.23 command overshot to
    // 3.22 kN > 3.2 kN and was HONESTLY refused — that boundary lives in
    // gate_startacc's unflyable case).
    req.initial_acc_mps2 =
        -4.0 * Eigen::Vector3d(std::cos(g32), 0.0, std::sin(g32));
    const auto r = tp::generate(req);
    expect(r.ok, "flyable commanded start acceleration accepted");
    if (r.ok) {
      poly_traj::Trajectory t0 = r.traj;
      const Eigen::Vector3d a0_u(
          req.initial_acc_mps2.x() / req.limits.unit_xy_m,
          req.initial_acc_mps2.y() / req.limits.unit_xy_m,
          req.initial_acc_mps2.z() / req.limits.unit_z_m);
      const double a0_err = (t0.getAcc(0.0) - a0_u).norm();
      std::cout << "acc_preserve: |acc(0) - a0| = " << a0_err
                << " u/s^2, repro err = "
                << r.audit.start_acc_repro_err_mps2 << " m/s^2, winner "
                << r.audit.winner_primitive_id << "\n";
      // 1e-7 u/s^2 = 1e-5 m/s^2: the inverse-dynamics command closure
      // reproduces the commanded acc to ~1e-7 m/s^2 (relative 3e-8) —
      // machine-noise scale, no physical meaning at any tighter bound.
      expect(a0_err < 1e-7,
             "trajectory leaves with the COMMANDED acceleration exactly");
      expect(r.audit.start_acc_repro_err_mps2 < 1e-5,
             "signed-inverse commands reproduce the commanded acc");
    }
  }

  if (run("gate_startacc")) {
    // A commanded initial acceleration the model cannot fly (50 g) must
    // be REFUSED — the signed inverse demands commands far outside the
    // envelope and the repro check says no. Never silently replaced.
    auto req = makeRequest(baseParams());
    req.initial_acc_prescribed = true;
    req.initial_acc_mps2 = Eigen::Vector3d(0.0, 0.0, 500.0);
    const auto r = tp::generate(req);
    expect(!r.ok, "unflyable commanded start acceleration refused");
  }

  if (run("validator_negative")) {
    // The validator must catch what dt-grid RK4 agreement cannot: a
    // polynomial whose sampled fit looks fine but whose interior
    // violates a limit. Take a real winner, bump one mid-piece with a
    // t^2(T-t)^2-shaped dip (zero position/velocity at the piece ends,
    // deep interior excursion below the AGL floor over terrain raised
    // for the probe) — the full-span validator must refuse it while the
    // unmodified winner passes the same call.
    auto req = makeRequest(baseParams());
    const auto r = tp::generate(req);
    expect(r.ok, "baseline winner exists");
    if (r.ok) {
      double rm = 0.0, ri = 0.0;
      // ZERO slack: the winner's own polynomial satisfies the BARE
      // physical limits — this is the acceptance the review demanded
      // (fit error re-expresses the curve, it never widens a limit).
      expect(tp::validateTransitionTrajectory(r.traj, req, &rm, &ri) ==
                 tp::TrajectoryVerdict::OK,
             "winner passes the standalone validator at BARE limits");
      const int mid = r.traj.getPieceNum() / 2;
      const double Tm =
          const_cast<poly_traj::Trajectory &>(r.traj)[mid].getDuration();
      // (a) Terrain: z(t) -= c2 t^2 (Tm-t)^2 — endpoints' pos/vel kept,
      // interior dips below the AGL floor between the fit samples.
      {
        poly_traj::Trajectory bad = r.traj;
        poly_traj::CoefficientMat cm = bad[mid].getCoeffMat();
        const double c2 =
            900.0 / req.limits.unit_z_m / std::pow(Tm, 4.0);
        cm(2, 1) -= c2;
        cm(2, 2) += 2.0 * c2 * Tm;
        cm(2, 3) -= c2 * Tm * Tm;
        bad[mid] = poly_traj::Piece(Tm, cm);
        expect(tp::validateTransitionTrajectory(bad, req, &rm, &ri) ==
                   tp::TrajectoryVerdict::FAIL,
               "interior terrain dip REFUSED by the validator");
      }
      // (b) Dynamics: x(t) += c3 t^3 (Tm-t)^2 — position and velocity
      // untouched at BOTH piece ends (left-end acceleration too; the
      // right-end acc does change, which the validator may also see) —
      // the point pinned is that the interior acceleration bump demands
      // thrust/load beyond the model and the curve dies on its interior
      // physics, not merely on a seam.
      {
        poly_traj::Trajectory bad = r.traj;
        poly_traj::CoefficientMat cm = bad[mid].getCoeffMat();
        // t^3(Tm-t)^3 = Tm^3 t^3 - 3 Tm^2 t^4 + 3 Tm t^5 - t^6: the t^6
        // term exceeds the quintic basis, so use t^3 (Tm-t)^2 (zero
        // pos/vel at both ends, acc zero at t=0 only) on an interior
        // piece where the seam acc jump is not judged.
        // Peak interior acceleration ~ c3 * Tm^3 / 8 — size it for
        // ~25 m/s^2 of extra along-track demand (thrust >> ceiling).
        const double c3 =
            200.0 / req.limits.unit_xy_m / std::pow(Tm, 3.0);
        // x(t) += c3 * (Tm^2 t^3 - 2 Tm t^4 + t^5)
        cm(0, 0) += c3;
        cm(0, 1) -= 2.0 * c3 * Tm;
        cm(0, 2) += c3 * Tm * Tm;
        bad[mid] = poly_traj::Piece(Tm, cm);
        expect(tp::validateTransitionTrajectory(bad, req, &rm, &ri) ==
                   tp::TrajectoryVerdict::FAIL,
               "interior thrust/load excursion REFUSED by the validator");
      }
    }
  }

  if (run("gate_overspeed")) {
    // Model ceiling below the start speed: the per-step limit gate (not
    // the saturation flags — speed is not a command) disqualifies.
    vd::Parameters p = baseParams();
    p.speed_max_mps = 150.0;  // start is 165 m/s
    auto req = makeRequest(p);
    const auto r = tp::generate(req);
    expect(!r.ok && r.audit.disq_limits > 0,
           "speed above the model ceiling dies at the limit gate");
  }

  if (run("s8limits")) {
    // [S8] Hard-limit boundaries on the FLOWN polynomial, below/at/above
    // per limit, all through the public validator with synthetic
    // single-piece trajectories whose (V, q, load, CL, thrust) are exact
    // closed forms — no propagation noise. Boundary semantics are
    // inclusive within the validator's stated epsilons, so the AT point
    // is deterministic. Every limit is COMPUTED from the one Parameters
    // set, never a literal.
    vd::Parameters p = baseParams();
    const double ux = 100.0, uz = 100.0, z0 = 800.0;
    const double rho = vd::airDensity(p, z0);
    const double W = p.mass_kg * p.gravity_mps2;
    const auto onePiece = [&](double vx, double az, double ax,
                              double T) {
      // p(t) = p0 + v t + a t^2/2 in UNIT space, level +x flight.
      poly_traj::CoefficientMat cm = poly_traj::CoefficientMat::Zero();
      cm.col(5) = Eigen::Vector3d(0.0, 0.0, z0 / uz);
      cm.col(4) = Eigen::Vector3d(vx / ux, 0.0, 0.0);
      cm.col(3) = Eigen::Vector3d(0.5 * ax / ux, 0.0, 0.5 * az / uz);
      poly_traj::Trajectory t;
      t.emplace_back(T, cm);
      return t;
    };
    tp::TransitionRequest req;   // permissive closures; limits carry dyn
    req.limits.dyn = p;
    req.limits.unit_xy_m = ux;
    req.limits.unit_z_m = uz;
    req.limits.min_agl_m = 5.0;
    req.limits.end_speed_min_mps = p.speed_min_mps;
    req.limits.end_speed_max_mps = p.speed_max_mps;
    req.zone_probe = [](const Eigen::Vector3d &) {
      return tp::ZoneProbe::CLEAR;
    };
    req.terrain_z = [](double, double, double *e) { *e = 0.0; return true; };
    req.pva_problem = [](const Eigen::Vector3d &, const Eigen::Vector3d &,
                         const Eigen::Vector3d &) { return std::string(); };
    double rm = 0.0, ri = 0.0;
    const auto verdict = [&](poly_traj::Trajectory t) {
      return tp::validateTransitionTrajectory(t, req, &rm, &ri);
    };
    const auto OK = tp::TrajectoryVerdict::OK;
    const auto FAIL = tp::TrajectoryVerdict::FAIL;

    // Speed floor (model activation): ballistic piece (zero lift, CL=0)
    // so the CL limit cannot mask the speed gate at low V.
    const double v_act = p.model_activation_speed_mps;
    expect(verdict(onePiece(v_act - 0.01, -p.gravity_mps2, 0.0, 0.2)) ==
               FAIL,
           "V below the activation floor refused");
    expect(verdict(onePiece(v_act, -p.gravity_mps2, 0.0, 0.2)) == OK,
           "V AT the activation floor accepted (inclusive)");
    expect(verdict(onePiece(v_act + 0.01, -p.gravity_mps2, 0.0, 0.2)) ==
               OK,
           "V above the activation floor accepted");

    // Speed ceiling: level flight (CL small, q well under its own cap).
    const double v_max = p.speed_max_mps;
    expect(verdict(onePiece(v_max - 0.01, 0.0, 0.0, 0.5)) == OK,
           "V below the model ceiling accepted");
    expect(verdict(onePiece(v_max, 0.0, 0.0, 0.5)) == OK,
           "V AT the model ceiling accepted (inclusive)");
    expect(verdict(onePiece(v_max + 0.01, 0.0, 0.0, 0.5)) == FAIL,
           "V above the model ceiling refused");

    // Dynamic pressure: the cap moves around the piece's exact q.
    {
      const double v_q = 200.0;
      const double q0 = 0.5 * rho * v_q * v_q;
      vd::Parameters pq = p;
      pq.dynamic_pressure_max_pa = q0 - 1.0;
      req.limits.dyn = pq;
      expect(verdict(onePiece(v_q, 0.0, 0.0, 0.5)) == FAIL,
             "q above the pressure cap refused");
      pq.dynamic_pressure_max_pa = q0;   // gate is strictly greater-than
      req.limits.dyn = pq;
      expect(verdict(onePiece(v_q, 0.0, 0.0, 0.5)) == OK,
             "q AT the pressure cap accepted (inclusive)");
      pq.dynamic_pressure_max_pa = q0 + 1.0;
      req.limits.dyn = pq;
      expect(verdict(onePiece(v_q, 0.0, 0.0, 0.5)) == OK,
             "q below the pressure cap accepted");
      req.limits.dyn = p;
    }

    // Load factor: vertical acceleration a_z gives n = 1 + a_z/g
    // exactly. At n ~ 2.5 a SUSTAINED level pull exceeds either CL (low
    // V) or thrust (induced drag at high V), so each point carries an
    // along-track deceleration that pins the thrust demand at an
    // interior 2 kN — the LOAD limit is then the only quantity crossing
    // its boundary.
    {
      const double v_n = 229.0;
      const double qs_n = 0.5 * rho * v_n * v_n * p.wing_area_m2;
      const auto loadPiece = [&](double az) {
        const double cl = p.mass_kg * (p.gravity_mps2 + az) / qs_n;
        const double cd = p.zero_lift_drag_coefficient +
                          p.induced_drag_factor * cl * cl;
        const double ax = (2000.0 - qs_n * cd) / p.mass_kg;
        return onePiece(v_n, az, ax, 0.2);
      };
      // A curving path rotates the lift frame, so n drifts (+1.1e-3
      // measured over this piece: the ax*sin(gamma) coupling) — an
      // EXACT-equality hold is not constructible for a dynamic
      // quantity. The inclusive side is witnessed at n_max - 5e-3
      // (inside the drift), the exclusive side at n_max + 0.01.
      const double g0 = p.gravity_mps2;
      expect(verdict(loadPiece((p.load_factor_max - 1.02) * g0)) == OK,
             "load below the limit accepted");
      expect(verdict(loadPiece((p.load_factor_max - 1.0 - 5e-3) * g0)) ==
                 OK,
             "load AT the boundary (within drift) accepted");
      expect(verdict(loadPiece((p.load_factor_max - 0.99) * g0)) == FAIL,
             "load above the limit refused");
    }

    // CL via the stall speed: level flight (n = 1) at V around
    // sqrt(2 W / (rho S CLmax)) crosses the CL limit exactly.
    {
      const double v_s = std::sqrt(
          2.0 * W / (rho * p.wing_area_m2 * p.lift_coefficient_max));
      expect(verdict(onePiece(v_s * 1.001, 0.0, 0.0, 0.2)) == OK,
             "CL below the limit (just above stall speed) accepted");
      expect(verdict(onePiece(v_s, 0.0, 0.0, 0.2)) == OK,
             "CL AT the limit accepted (inclusive)");
      expect(verdict(onePiece(v_s * 0.999, 0.0, 0.0, 0.2)) == FAIL,
             "CL above the limit (below stall speed) refused");
    }

    // Transition-policy gamma bound on the flown curve: BALLISTIC
    // pieces (a = -g z, so lift = 0 and thrust = drag — no other limit
    // can mask) with the velocity at gamma below/at/above the policy.
    // gamma DECREASES along a ballistic arc, so the AT point drifts to
    // the inclusive side deterministically.
    {
      const double g_pol = req.limits.max_abs_gamma_rad;
      const auto gammaPiece = [&](double gam) {
        poly_traj::CoefficientMat cm = poly_traj::CoefficientMat::Zero();
        cm.col(5) = Eigen::Vector3d(0.0, 0.0, z0 / uz);
        cm.col(4) =
            Eigen::Vector3d(165.0 * std::cos(gam) / ux, 0.0,
                            165.0 * std::sin(gam) / uz);
        cm.col(3) = Eigen::Vector3d(0.0, 0.0, -0.5 * p.gravity_mps2 / uz);
        poly_traj::Trajectory t;
        t.emplace_back(0.1, cm);
        return t;
      };
      expect(verdict(gammaPiece(g_pol - 0.01)) == OK,
             "gamma below the policy cone accepted");
      expect(verdict(gammaPiece(g_pol)) == OK,
             "gamma AT the policy cone accepted (inclusive; ballistic "
             "drift is downward)");
      expect(verdict(gammaPiece(g_pol + 0.01)) == FAIL,
             "gamma above the policy cone refused");
    }

    // Bank: a level turn's lateral acceleration a_y gives
    // bank = atan(a_y/g) exactly at t=0; thrust is pinned interior with
    // a compensating along-track deceleration (same pattern as load).
    // The frame rotates with the turn, so the inclusive witness sits
    // inside the drift like the load one.
    {
      const double v_b = 229.0;
      const double qs_b = 0.5 * rho * v_b * v_b * p.wing_area_m2;
      const auto bankPiece = [&](double bank) {
        const double ay = p.gravity_mps2 * std::tan(bank);
        const double lift =
            p.mass_kg * std::hypot(p.gravity_mps2, ay);
        const double cl = lift / qs_b;
        const double cd = p.zero_lift_drag_coefficient +
                          p.induced_drag_factor * cl * cl;
        const double ax = (2000.0 - qs_b * cd) / p.mass_kg;
        poly_traj::CoefficientMat cm = poly_traj::CoefficientMat::Zero();
        cm.col(5) = Eigen::Vector3d(0.0, 0.0, z0 / uz);
        cm.col(4) = Eigen::Vector3d(v_b / ux, 0.0, 0.0);
        cm.col(3) =
            Eigen::Vector3d(0.5 * ax / ux, 0.5 * ay / ux, 0.0);
        poly_traj::Trajectory t;
        t.emplace_back(0.1, cm);
        return t;
      };
      expect(verdict(bankPiece(p.bank_angle_max_rad - 0.02)) == OK,
             "bank below the limit accepted");
      expect(verdict(bankPiece(p.bank_angle_max_rad - 5e-3)) == OK,
             "bank AT the boundary (within drift) accepted");
      expect(verdict(bankPiece(p.bank_angle_max_rad + 0.02)) == FAIL,
             "bank above the limit refused");
    }

    // Thrust ceiling/floor: along-track acceleration a_x demands
    // T = D + m a_x at level flight; D is the model's own drag at the
    // piece's exact CL.
    {
      const double v_t = 180.0;
      const double q = 0.5 * rho * v_t * v_t;
      const double qs = q * p.wing_area_m2;
      const double cl = W / qs;   // level lift
      const double cd = p.zero_lift_drag_coefficient +
                        p.induced_drag_factor * cl * cl;
      const double D = qs * cd;
      const double ax_hi = (p.thrust_max_n - D) / p.mass_kg;
      expect(verdict(onePiece(v_t, 0.0, ax_hi - 0.001, 0.2)) == OK,
             "thrust below the ceiling accepted");
      expect(verdict(onePiece(v_t, 0.0, ax_hi, 0.2)) == OK,
             "thrust AT the ceiling accepted (inclusive)");
      expect(verdict(onePiece(v_t, 0.0, ax_hi + 0.001, 0.2)) == FAIL,
             "thrust above the ceiling refused");
      const double ax_lo = (p.thrust_min_n - D) / p.mass_kg;
      expect(verdict(onePiece(v_t, 0.0, ax_lo + 0.001, 0.2)) == OK,
             "thrust above the floor accepted");
      expect(verdict(onePiece(v_t, 0.0, ax_lo, 0.2)) == OK,
             "thrust AT the floor accepted (inclusive)");
      expect(verdict(onePiece(v_t, 0.0, ax_lo - 0.001, 0.2)) == FAIL,
             "thrust below the floor (harder than idle drag) refused");
    }
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
