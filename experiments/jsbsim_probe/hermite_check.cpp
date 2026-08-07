// [CONTRACT-2 EVAL] PVA(t) -> quintic Hermite re-expression check.
//
// The transition pipeline (docs/transition_phase_contract.md §3) hands a
// propagated PVA(t) to a polynomial adapter, which must produce something
// the chain can accept as a prescribed head. This checks the adapter half
// of that claim against a REAL propagated trajectory rather than a synthetic
// one, and measures the four things §6 says must be measured:
//
//   1. knot fidelity      — P/V/A at the sample points the Hermite is built
//                           from (exact by construction; a nonzero number
//                           here means the coefficient convention is wrong)
//   2. C2 across pieces   — the seam property the chain relies on
//   3. INTER-SAMPLE error — where the Hermite actually differs from the
//                           source, i.e. what "approximation" costs
//   4. jerk jumps         — MEASURED, not judged. The bound is spec-pending
//                           (§8), so this prints magnitudes and stops.
//
// The Hermite construction is the same one TerminalPhase uses, kept
// standalone here so the experiment does not drag the planner in.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGPropagate.h>
#include <models/FGAccelerations.h>
#include <models/FGInertial.h>
#include <math/FGLocation.h>

namespace {

constexpr double kFtToM = 0.3048;
constexpr double kDegToRad = M_PI / 180.0;
// Samples before this are discarded from the fidelity statistics: JSBSim
// settles the model for a moment after RunIC.
constexpr double kSettleS = 0.5;

struct PVA {
  double t{0.0};
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
};

// One quintic piece matching exact P/V/A at both ends, in the same
// coefficient convention poly_traj::Piece uses:
//   p(t) = c0 t^5 + c1 t^4 + c2 t^3 + c3 t^2 + c4 t + c5
struct Quintic {
  Eigen::Vector3d c[6];
  double T{0.0};
  Eigen::Vector3d pos(double t) const {
    return ((((c[0] * t + c[1]) * t + c[2]) * t + c[3]) * t + c[4]) * t + c[5];
  }
  Eigen::Vector3d vel(double t) const {
    return (((5.0 * c[0] * t + 4.0 * c[1]) * t + 3.0 * c[2]) * t +
            2.0 * c[3]) * t + c[4];
  }
  Eigen::Vector3d acc(double t) const {
    return ((20.0 * c[0] * t + 12.0 * c[1]) * t + 6.0 * c[2]) * t + 2.0 * c[3];
  }
  Eigen::Vector3d jerk(double t) const {
    return (60.0 * c[0] * t + 24.0 * c[1]) * t + 6.0 * c[2];
  }
};

Quintic hermite(const PVA &s0, const PVA &s1)
{
  const double T = s1.t - s0.t;
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const Eigen::Vector3d A = s1.p - s0.p - s0.v * T - 0.5 * s0.a * T2;
  const Eigen::Vector3d B = s1.v - s0.v - s0.a * T;
  const Eigen::Vector3d C = s1.a - s0.a;
  Quintic q;
  q.T = T;
  q.c[5] = s0.p;
  q.c[4] = s0.v;
  q.c[3] = 0.5 * s0.a;
  q.c[2] = 10.0 * A / T3 - 4.0 * B / T2 + 0.5 * C / T;
  q.c[1] = -15.0 * A / T4 + 7.0 * B / T3 - C / T2;
  q.c[0] = 6.0 * A / T5 - 3.0 * B / T4 + 0.5 * C / T3;
  return q;
}

// Propagate and record dense PVA in ONE fixed frame — the ENU tangent
// plane anchored at the initial point (R0 = Tec2l at t=0, applied to ECEF
// position deltas, ECEF velocity, and the analytic ECEF acceleration).
// See jsbsim_probe.cpp's header for the formula and why the previous
// mixed-frame extraction could never be self-consistent.
std::vector<PVA> propagate(const std::string &root, const std::string &model,
                           double dt, double duration, double elevator,
                           double throttle, double hold_s)
{
  std::vector<PVA> out;
  JSBSim::FGFDMExec fdm;
  fdm.SetRootDir(SGPath(root));
  fdm.SetAircraftPath(SGPath(root + "/aircraft"));
  fdm.SetEnginePath(SGPath(root + "/engine"));
  fdm.SetSystemsPath(SGPath(root + "/systems"));
  fdm.SetDebugLevel(0);
  if (!fdm.LoadModel(model)) return out;
  fdm.Setdt(dt);
  auto ic = fdm.GetIC();
  const double lat0 = 37.5, lon0 = 127.0;
  ic->SetLatitudeDegIC(lat0);
  ic->SetLongitudeDegIC(lon0);
  ic->SetAltitudeASLFtIC(1500.0 / kFtToM);
  ic->SetVtrueFpsIC(190.0 / kFtToM);
  ic->SetFlightPathAngleDegIC(32.0);
  ic->SetPsiDegIC(180.0);
  if (!fdm.RunIC()) return out;

  const JSBSim::FGMatrix33 R0 = fdm.GetPropagate()->GetTec2l();
  const JSBSim::FGLocation &loc0 = fdm.GetPropagate()->GetLocation();
  const JSBSim::FGColumnVector3 r0(loc0(1), loc0(2), loc0(3));
  const JSBSim::FGColumnVector3 omega = fdm.GetInertial()->GetOmegaPlanet();
  const int n = static_cast<int>(duration / dt);
  out.reserve(static_cast<size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const double t = i * dt;
    fdm.SetPropertyValue("fcs/throttle-cmd-norm", throttle);
    fdm.SetPropertyValue("fcs/mixture-cmd-norm", 1.0);
    fdm.SetPropertyValue("fcs/elevator-cmd-norm",
                         (t < hold_s) ? 0.0 : elevator);
    PVA s;
    s.t = t;
    const JSBSim::FGLocation &loc = fdm.GetPropagate()->GetLocation();
    const JSBSim::FGColumnVector3 r_ecef(loc(1), loc(2), loc(3));
    const JSBSim::FGColumnVector3 v_ecef =
        fdm.GetPropagate()->GetECEFVelocity();
    const JSBSim::FGColumnVector3 a_ecef =
        fdm.GetPropagate()->GetTi2ec() *
            fdm.GetAccelerations()->GetUVWidot() -
        2.0 * (omega * v_ecef) - omega * (omega * r_ecef);
    const JSBSim::FGColumnVector3 p_ned = R0 * (r_ecef - r0);
    const JSBSim::FGColumnVector3 v_ned = R0 * v_ecef;
    const JSBSim::FGColumnVector3 a_ned = R0 * a_ecef;
    s.p = Eigen::Vector3d(p_ned(2), p_ned(1), -p_ned(3)) * kFtToM;
    s.v = Eigen::Vector3d(v_ned(2), v_ned(1), -v_ned(3)) * kFtToM;
    s.a = Eigen::Vector3d(a_ned(2), a_ned(1), -a_ned(3)) * kFtToM;
    out.push_back(s);
    if (i == n) break;
    if (!fdm.Run()) break;
  }
  return out;
}

struct Report {
  double knot_p{0.0}, knot_v{0.0}, knot_a{0.0};
  double c0{0.0}, c1{0.0}, c2{0.0};       // seam discontinuities
  double mid_p{0.0}, mid_v{0.0}, mid_a{0.0};        // inter-sample error, max
  double mid_p99{0.0}, mid_v99{0.0}, mid_a99{0.0};  // ... and 99th percentile
  double worst_v{0.0}, worst_v_t{0.0};  // where the velocity error actually is
  double jerk_jump_max{0.0}, jerk_mag_max{0.0};
  int pieces{0};
};

// Build a Hermite chain over `dense` sampled every `stride` steps, then
// measure it against the source it came from.
Report check(const std::vector<PVA> &dense, size_t stride)
{
  Report r;
  std::vector<PVA> knots;
  for (size_t i = 0; i < dense.size(); i += stride) knots.push_back(dense[i]);
  if (knots.back().t < dense.back().t) knots.push_back(dense.back());
  if (knots.size() < 2) return r;

  std::vector<Quintic> pieces;
  for (size_t i = 0; i + 1 < knots.size(); ++i)
    pieces.push_back(hermite(knots[i], knots[i + 1]));
  r.pieces = static_cast<int>(pieces.size());

  // 1. knot fidelity
  for (size_t i = 0; i < pieces.size(); ++i) {
    const Quintic &q = pieces[i];
    r.knot_p = std::max(r.knot_p, (q.pos(0.0) - knots[i].p).norm());
    r.knot_v = std::max(r.knot_v, (q.vel(0.0) - knots[i].v).norm());
    r.knot_a = std::max(r.knot_a, (q.acc(0.0) - knots[i].a).norm());
    r.knot_p = std::max(r.knot_p, (q.pos(q.T) - knots[i + 1].p).norm());
    r.knot_v = std::max(r.knot_v, (q.vel(q.T) - knots[i + 1].v).norm());
    r.knot_a = std::max(r.knot_a, (q.acc(q.T) - knots[i + 1].a).norm());
  }
  // 2. C2 across seams + 4. jerk jump (measured only)
  for (size_t i = 0; i + 1 < pieces.size(); ++i) {
    const Quintic &l = pieces[i], &n = pieces[i + 1];
    r.c0 = std::max(r.c0, (l.pos(l.T) - n.pos(0.0)).norm());
    r.c1 = std::max(r.c1, (l.vel(l.T) - n.vel(0.0)).norm());
    r.c2 = std::max(r.c2, (l.acc(l.T) - n.acc(0.0)).norm());
    r.jerk_jump_max =
        std::max(r.jerk_jump_max, (l.jerk(l.T) - n.jerk(0.0)).norm());
  }
  for (const auto &q : pieces)
    for (int k = 0; k <= 20; ++k)
      r.jerk_mag_max = std::max(r.jerk_mag_max, q.jerk(q.T * k / 20.0).norm());

  // 3. inter-sample error: every dense sample NOT used as a knot.
  // Reported as a 99th percentile as well as a max, because the first
  // instants after RunIC carry a trim transient whose single outlier would
  // otherwise masquerade as a systematic approximation error.
  std::vector<double> ep, ev, ea;
  size_t pi = 0;
  for (const auto &s : dense) {
    while (pi + 1 < pieces.size() && s.t > knots[pi + 1].t) ++pi;
    const double tau = s.t - knots[pi].t;
    if (tau < 0.0 || tau > pieces[pi].T) continue;
    // Skip the post-RunIC trim transient. JSBSim settles the model over the
    // first fraction of a second; those samples say nothing about how well
    // a polynomial represents a flown trajectory, and they dominate a
    // max-norm (they survived even a 99th percentile).
    if (s.t < kSettleS) continue;
    const double e_v = (pieces[pi].vel(tau) - s.v).norm();
    if (e_v > r.worst_v) { r.worst_v = e_v; r.worst_v_t = s.t; }
    ep.push_back((pieces[pi].pos(tau) - s.p).norm());
    ev.push_back(e_v);
    ea.push_back((pieces[pi].acc(tau) - s.a).norm());
  }
  const auto stat = [](std::vector<double> &v, double *mx, double *p99) {
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    *mx = v.back();
    *p99 = v[static_cast<size_t>(0.99 * (v.size() - 1))];
  };
  stat(ep, &r.mid_p, &r.mid_p99);
  stat(ev, &r.mid_v, &r.mid_v99);
  stat(ea, &r.mid_a, &r.mid_a99);
  return r;
}

}  // namespace

int main(int argc, char **argv)
{
  const std::string root = argc > 1 ? argv[1] : "jsbsim";
  const std::string model = argc > 2 ? argv[2] : "global5000";
  const double dur = argc > 3 ? std::atof(argv[3]) : 12.0;

  std::printf("=== PVA -> quintic Hermite re-expression check ===\n");
  std::printf("source: JSBSim %s (bundled EXAMPLE aircraft), %.1f s at "
              "dt=0.01\n\n", model.c_str(), dur);

  const std::vector<PVA> dense =
      propagate(root, model, 0.01, dur, 0.5, 0.4, 1.0);
  if (dense.size() < 10) {
    std::printf("FAIL: propagation produced %zu samples\n", dense.size());
    return 1;
  }
  std::printf("propagated %zu samples\n\n", dense.size());

  // --- Source self-consistency, BEFORE blaming the polynomial ----------
  // A Hermite built on (p, v, a) can only be as coherent as the triple it
  // is given. If d(p)/dt disagrees with the reported v, no knot spacing
  // fixes it and every "inter-sample velocity error" below is really this
  // number. Measured first so the two cannot be confused.
  double dpv = 0.0, dva = 0.0, vmax = 0.0;
  for (size_t i = 1; i + 1 < dense.size(); ++i) {
    const double h = dense[i + 1].t - dense[i - 1].t;
    dpv = std::max(dpv,
                   ((dense[i + 1].p - dense[i - 1].p) / h - dense[i].v).norm());
    dva = std::max(dva,
                   ((dense[i + 1].v - dense[i - 1].v) / h - dense[i].a).norm());
    vmax = std::max(vmax, dense[i].v.norm());
  }
  std::printf("Source self-consistency vs dt — frame-consistent extraction "
              "must CONVERGE\n");
  std::printf("  %8s %14s %16s   (masked = excluding the command-step response t in [0.95, 1.40])\n", "dt[s]", "|dp/dt-v|max", "|dv/dt-a|max");
  for (double cdt : {0.02, 0.01, 0.005, 0.001}) {
    const std::vector<PVA> d2 =
        propagate(root, model, cdt, dur, 0.5, 0.4, 1.0);
    if (d2.size() < 5) continue;
    double e1 = 0.0, e2 = 0.0, e2_t = 0.0, e2_off = 0.0;
    for (size_t i = 1; i + 1 < d2.size(); ++i) {
      if (d2[i].t < kSettleS) continue;
      const double h = d2[i + 1].t - d2[i - 1].t;
      e1 = std::max(e1, ((d2[i + 1].p - d2[i - 1].p) / h - d2[i].v).norm());
      const double e = ((d2[i + 1].v - d2[i - 1].v) / h - d2[i].a).norm();
      if (e > e2) { e2 = e; e2_t = d2[i].t; }
      // Same residual with the command-step RESPONSE masked: the elevator
      // steps at t=1.0 s and the actuator sweeps for a fraction of a second
      // after it. Inside that window `a` is near-discontinuous, which a
      // central difference cannot track at any dt.
      if (d2[i].t < 0.95 || d2[i].t > 1.40) e2_off = std::max(e2_off, e);
    }
    std::printf("  %8.3f %14.6f %16.6f  @t=%.2f   masked: %.6f\n",
                cdt, e1, e2, e2_t, e2_off);
  }
  std::printf("\nSource self-consistency at dt=0.01 (all samples)\n");
  std::printf("  |d(p)/dt - v| max = %.4f m/s   (%.3f%% of |v|max %.1f)\n",
              dpv, 100.0 * dpv / std::max(1e-9, vmax), vmax);
  std::printf("  |d(v)/dt - a| max = %.4f m/s^2\n", dva);
  std::printf("  ^ a floor on everything below: the adapter cannot be more "
              "coherent than its input.\n\n");

  std::printf("Knot spacing sweep — how much does the polynomial cost?\n");
  std::printf("  %7s %7s | %8s %8s %8s | %17s %17s %17s\n", "knot[s]",
              "pieces", "knot dP", "knot dV", "knot dA",
              "mid dP  max/p99", "mid dV  max/p99", "mid dA  max/p99");
  for (size_t stride : {10u, 25u, 50u, 100u, 200u}) {
    const Report r = check(dense, stride);
    if (r.pieces == 0) continue;
    std::printf("  %7.2f %7d | %8.1e %8.1e %8.1e | %8.3f %8.3f | %7.3f %7.3f "
                "| %8.2f %7.2f\n",
                stride * 0.01, r.pieces, r.knot_p, r.knot_v, r.knot_a,
                r.mid_p, r.mid_p99, r.mid_v, r.mid_v99, r.mid_a, r.mid_a99);
  }
  std::printf("  (knot columns: exactness of the construction, metres / m·s^-1"
              " / m·s^-2)\n");
  std::printf("  (mid columns: INTER-SAMPLE error against the propagated "
              "source — the real cost)\n");

  std::printf("  (samples before t=%.1f s excluded: post-RunIC trim "
              "transient)\n", kSettleS);
  {
    const Report rw = check(dense, 10);
    std::printf("  worst velocity error after settling: %.3f m/s at t=%.2f s\n",
                rw.worst_v, rw.worst_v_t);
  }

  std::printf("\nSeam continuity and jerk, at 0.25 s knots\n");
  const Report r = check(dense, 25);
  std::printf("  C0 max %.3e m, C1 max %.3e m/s, C2 max %.3e m/s^2\n",
              r.c0, r.c1, r.c2);
  std::printf("  jerk: peak magnitude %.3f m/s^3, peak seam JUMP %.3f m/s^3\n",
              r.jerk_mag_max, r.jerk_jump_max);
  std::printf("  (jerk is MEASURED only — the bound is spec-pending, "
              "contract §8)\n");
  return 0;
}
