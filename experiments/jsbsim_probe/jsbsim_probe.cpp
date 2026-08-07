// [CONTRACT-2 EVAL] JSBSim integration probe — see README.md for what this
// does and does NOT establish.
//
// Scope, stated precisely because the first version of this file overstated
// it: this measures whether JSBSim can be driven from our C++ and how much
// one propagation costs, and whether a crude search over input profiles can
// reach a PARTIAL entry window (speed + flight-path angle only). It is NOT
// a transition planner and its "success" is NOT a handoff. A real handoff
// additionally needs pvaEnvelopeProblem() to pass, position agreement with
// the global route entry point, velocity aligned with the route tangent,
// load/acceleration limits, sustained dwell, and terrain/zone clearance —
// none of which are checked here.
//

// Fixed mission frame (review find: the first version mixed a FIXED
// approximate plane for position with the INSTANTANEOUS local NED for
// velocity/acceleration — two different frames, so the triple could never
// be self-consistent). Everything below is expressed in ONE frame: the
// ENU tangent plane anchored at the initial point, via
//
//   R0     = ECEF->NED at t=0 (JSBSim's own Tec2l, captured once)
//   p_enu  = ned2enu( R0 * (r_ecef(t) - r_ecef(0)) )
//   v_enu  = ned2enu( R0 * v_ecef(t) )
//   a_enu  = ned2enu( R0 * a_ecef(t) )
//
// a_ecef is computed analytically from JSBSim's inertial acceleration:
//   a_ecef = Ti2ec * UVWidot - 2 w x v_ecef - w x (w x r_ecef)
// (UVWidot is the inertial acceleration in ECI axes; the two omega terms
// are the Coriolis and centrifugal corrections for differentiating in the
// rotating ECEF frame.) This form is CROSS-CHECKED against a central
// difference of v_enu in stage [2] rather than trusted — the previous
// "add the transport term back" formula carried a 1.4% residual and was
// nearly-right rather than right.
//
// The aircraft models are JSBSim's bundled examples, which the project
// documents as approximations built from public data for education and
// entertainment. Nothing here says anything about a real platform.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGPropagate.h>
#include <models/FGAccelerations.h>
#include <models/FGInertial.h>
#include <math/FGLocation.h>

namespace {

constexpr double kFtToM = 0.3048;
constexpr double kDegToRad = M_PI / 180.0;

struct Sample {
  double t;
  double x, y, z;     // local ENU metres (x=east, y=north, z=up)
  double vx, vy, vz;  // m/s
  double ax, ay, az;     // m/s^2, Tb2l * (UVWdot + PQR x UVW)  [see below]
  double bx, by, bz;     // m/s^2, Tb2l * UVWdot  — no transport term
  double dax, day, daz;  // m/s^2, central difference — cross-check only
  double mach, alpha_deg, gamma_deg, speed;
};

// The input profile a transition planner would search over. Deliberately
// crude — three numbers — because the point is to measure the machinery,
// not to propose a guidance law.
struct Profile {
  double elevator{0.0};
  double throttle{1.0};
  double hold_s{2.0};  // seconds at neutral elevator before the command
};

// PARTIAL entry window: the two conditions this probe can actually test.
// The full handoff contract is in docs/transition_phase_contract.md §9-§10.
constexpr double kEntryVMin = 132.0, kEntryVMax = 230.0, kEntryGamma = 30.0;
bool inPartialEntryWindow(double speed, double gamma_deg)
{
  return speed >= kEntryVMin && speed <= kEntryVMax &&
         std::abs(gamma_deg) <= kEntryGamma;
}

struct RunResult {
  std::vector<Sample> s;
  double wall_ms{0.0};
  int steps{0};
};

RunResult propagate(const std::string &root, const std::string &model,
                    double dt, double duration, bool verbose,
                    const Profile &prof)
{
  RunResult out;
  JSBSim::FGFDMExec fdm;
  fdm.SetRootDir(SGPath(root));
  fdm.SetAircraftPath(SGPath(root + "/aircraft"));
  fdm.SetEnginePath(SGPath(root + "/engine"));
  fdm.SetSystemsPath(SGPath(root + "/systems"));
  fdm.SetDebugLevel(0);
  if (!fdm.LoadModel(model)) {
    std::printf("FAIL: LoadModel(%s)\n", model.c_str());
    return out;
  }
  // The example model's own <output> logger spams undefined-property
  // warnings and dumps a CSV into the CWD — physics-irrelevant, off.
  fdm.DisableOutput();
  fdm.Setdt(dt);

  // A transition-shaped initial state, commanded programmatically: 190 m/s
  // at a 32 degree climb — the exact state contract 1 refuses, because it
  // sits outside the cruise model's +/-30 degree cone.
  auto ic = fdm.GetIC();
  const double lat0 = 37.5, lon0 = 127.0;
  ic->SetLatitudeDegIC(lat0);
  ic->SetLongitudeDegIC(lon0);
  ic->SetAltitudeASLFtIC(1500.0 / kFtToM);
  ic->SetVtrueFpsIC(190.0 / kFtToM);
  ic->SetFlightPathAngleDegIC(32.0);
  ic->SetPsiDegIC(180.0);  // heading south, like the r5 mission
  if (!fdm.RunIC()) {
    std::printf("FAIL: RunIC\n");
    return out;
  }

  // Capture the FIXED frame at t=0: JSBSim's own ECEF->NED at the initial
  // location, plus the ECEF origin. Never updated again.
  const JSBSim::FGMatrix33 R0 = fdm.GetPropagate()->GetTec2l();
  const JSBSim::FGLocation &loc0 = fdm.GetPropagate()->GetLocation();
  const JSBSim::FGColumnVector3 r0(loc0(1), loc0(2), loc0(3));
  const JSBSim::FGColumnVector3 omega =
      fdm.GetInertial()->GetOmegaPlanet();
  const auto t0 = std::chrono::steady_clock::now();
  const int n = static_cast<int>(duration / dt);
  out.s.reserve(static_cast<size_t>(n) + 1);

  for (int i = 0; i <= n; ++i) {
    const double t = i * dt;
    fdm.SetPropertyValue("fcs/throttle-cmd-norm", prof.throttle);
    fdm.SetPropertyValue("fcs/mixture-cmd-norm", 1.0);
    fdm.SetPropertyValue("fcs/elevator-cmd-norm",
                         (t < prof.hold_s) ? 0.0 : prof.elevator);
    fdm.SetPropertyValue("fcs/aileron-cmd-norm", 0.0);
    fdm.SetPropertyValue("fcs/rudder-cmd-norm", 0.0);

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
    const JSBSim::FGColumnVector3 a_ned2 = R0 * a_ecef;
    Sample s;
    s.t = t;
    s.x = p_ned(2) * kFtToM;   // NED->ENU: east
    s.y = p_ned(1) * kFtToM;   //           north
    s.z = -p_ned(3) * kFtToM;  //           up
    s.vx = v_ned(2) * kFtToM;
    s.vy = v_ned(1) * kFtToM;
    s.vz = -v_ned(3) * kFtToM;
    s.speed = std::sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);
    s.mach = fdm.GetPropertyValue("velocities/mach");
    s.alpha_deg = fdm.GetPropertyValue("aero/alpha-deg");
    s.gamma_deg = fdm.GetPropertyValue("flight-path/gamma-deg");

    // Acceleration in the SAME fixed frame (analytic, see header comment).
    // The previous transport-term formula is kept as candidate B so stage
    // [2] can show the difference instead of asserting it.
    s.ax = a_ned2(2) * kFtToM;
    s.ay = a_ned2(1) * kFtToM;
    s.az = -a_ned2(3) * kFtToM;
    {
      const JSBSim::FGMatrix33 &Tb2l = fdm.GetPropagate()->GetTb2l();
      const JSBSim::FGColumnVector3 a_b =
          Tb2l * (fdm.GetAccelerations()->GetUVWdot() +
                  fdm.GetPropagate()->GetPQR() * fdm.GetPropagate()->GetUVW());
      s.bx = a_b(2) * kFtToM;
      s.by = a_b(1) * kFtToM;
      s.bz = -a_b(3) * kFtToM;
    }
    s.dax = s.day = s.daz = 0.0;
    out.s.push_back(s);
    if (i == n) break;
    if (!fdm.Run()) {
      std::printf("FAIL: Run() stopped at t=%.2f\n", t);
      break;
    }
    ++out.steps;
  }
  // Central differences kept ONLY to cross-check the transform above.
  for (size_t i = 1; i + 1 < out.s.size(); ++i) {
    const double h = out.s[i + 1].t - out.s[i - 1].t;
    out.s[i].dax = (out.s[i + 1].vx - out.s[i - 1].vx) / h;
    out.s[i].day = (out.s[i + 1].vy - out.s[i - 1].vy) / h;
    out.s[i].daz = (out.s[i + 1].vz - out.s[i - 1].vz) / h;
  }
  out.wall_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();
  if (verbose) {
    std::printf("  %6s %9s %9s %9s %8s %8s %8s %7s %8s\n",
                "t[s]", "x[m]", "y[m]", "z[m]", "|v|", "gamma", "alpha",
                "mach", "|a|");
    const size_t stride = std::max<size_t>(1, out.s.size() / 8);
    for (size_t i = 0; i < out.s.size(); i += stride) {
      const Sample &s = out.s[i];
      std::printf("  %6.2f %9.1f %9.1f %9.1f %8.1f %8.2f %8.2f %7.3f %8.2f\n",
                  s.t, s.x, s.y, s.z, s.speed, s.gamma_deg, s.alpha_deg,
                  s.mach, std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az));
    }
  }
  return out;
}

}  // namespace

int main(int argc, char **argv)
{
  const std::string root = argc > 1 ? argv[1] : "jsbsim";
  const std::string model = argc > 2 ? argv[2] : "global5000";
  const double duration = argc > 3 ? std::atof(argv[3]) : 12.0;

  std::printf("=== JSBSim integration probe (structure + cost only) ===\n");
  std::printf("model=%s (bundled EXAMPLE aircraft) duration=%.1f s\n\n",
              model.c_str(), duration);

  std::printf("[1] commanded transition-shaped IC (190 m/s, gamma 32 deg)\n");
  const RunResult r = propagate(root, model, 0.01, duration, true, Profile{});
  if (r.s.empty()) return 1;
  const Sample &a = r.s.front();
  const Sample &b = r.s.back();
  std::printf("\n  start: |v|=%.1f m/s gamma=%.2f deg  ->  "
              "end: |v|=%.1f m/s gamma=%.2f deg\n",
              a.speed, a.gamma_deg, b.speed, b.gamma_deg);

  // Does JSBSim's own transform agree with differencing its velocity? If
  // these disagree, one of the two readings is wrong and every downstream
  // acceleration claim is suspect.
  double amax = 0.0, dmax_full = 0.0, dmax_bare = 0.0;
  for (size_t i = 1; i + 1 < r.s.size(); ++i) {
    const Sample &s = r.s[i];
    amax = std::max(amax, std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az));
    dmax_full = std::max(dmax_full,
                         std::sqrt((s.ax - s.dax) * (s.ax - s.dax) +
                                   (s.ay - s.day) * (s.ay - s.day) +
                                   (s.az - s.daz) * (s.az - s.daz)));
    dmax_bare = std::max(dmax_bare,
                         std::sqrt((s.bx - s.dax) * (s.bx - s.dax) +
                                   (s.by - s.day) * (s.by - s.day) +
                                   (s.bz - s.daz) * (s.bz - s.daz)));
  }
  std::printf("\n[2] acceleration candidates vs central difference of v "
              "(fixed ENU frame)\n");
  std::printf("  peak |a| = %.3f m/s^2\n", amax);
  std::printf("  A: R0*(Ti2ec*UVWidot - 2w x v - w x (w x r)):  peak gap "
              "%.4f m/s^2 (%.3f%% of peak)\n",
              dmax_full, 100.0 * dmax_full / std::max(1e-9, amax));
  std::printf("  B: Tb2l*(UVWdot + PQR x UVW), moving frame:    peak gap "
              "%.4f m/s^2 (%.3f%% of peak)\n",
              dmax_bare, 100.0 * dmax_bare / std::max(1e-9, amax));

  std::printf("\n[3] cost and step-size sensitivity\n");
  std::printf("  %8s %8s %10s %12s %10s %10s\n", "dt[s]", "steps", "wall[ms]",
              "us/step", "end |v|", "end z[m]");
  for (double dt : {0.001, 0.005, 0.01, 0.02}) {
    const RunResult rr = propagate(root, model, dt, duration, false, Profile{});
    if (rr.s.empty()) continue;
    std::printf("  %8.3f %8d %10.1f %12.1f %10.2f %10.2f\n", dt, rr.steps,
                rr.wall_ms, 1000.0 * rr.wall_ms / std::max(1, rr.steps),
                rr.s.back().speed, rr.s.back().z);
  }
  const RunResult r1 = propagate(root, model, 0.01, duration, false, Profile{});
  const RunResult r2 = propagate(root, model, 0.01, duration, false, Profile{});
  std::printf("\n  determinism across two identical runs: dz=%.3e m, "
              "d|v|=%.3e m/s\n",
              std::abs(r1.s.back().z - r2.s.back().z),
              std::abs(r1.s.back().speed - r2.s.back().speed));

  // --- Partial entry window search ------------------------------------
  // NOT a transition planner and NOT a handoff test. It answers one narrow
  // question: can profiles that put the state inside the speed/climb-angle
  // window be found in a usable amount of compute?
  std::printf("\n[4] grid search for profiles reaching the PARTIAL entry "
              "window (speed + gamma only)\n");
  std::printf("  window: |v| in [%.0f, %.0f] m/s, |gamma| <= %.0f deg, "
              "dwell >= 1.0 s\n", kEntryVMin, kEntryVMax, kEntryGamma);
  std::printf("  NOT checked here: pvaEnvelopeProblem, route entry position, "
              "tangent alignment,\n                    load limits, terrain "
              "and zone clearance\n");
  const double search_dur = 30.0, dwell_need = 1.0;
  int tried = 0, found = 0;
  const auto ts = std::chrono::steady_clock::now();
  // The two instants are DIFFERENT and were conflated in the first version
  // of this probe: entry_t is when the state first satisfies the window,
  // dwell_t is when it has held it long enough to count. The reported PVA
  // belongs to dwell_t.
  double best_entry_t = 1e9, best_dwell_t = 0.0;
  Profile best{};
  Sample best_state{};
  for (double elev = -1.0; elev <= 1.001; elev += 0.25) {
    for (double thr : {0.4, 0.7, 1.0}) {
      Profile p; p.elevator = elev; p.throttle = thr; p.hold_s = 1.0;
      const RunResult rr = propagate(root, model, 0.01, search_dur, false, p);
      ++tried;
      if (rr.s.empty()) continue;
      double dwell = 0.0, entry_t = -1.0;
      for (size_t i = 1; i < rr.s.size(); ++i) {
        if (!inPartialEntryWindow(rr.s[i].speed, rr.s[i].gamma_deg)) {
          dwell = 0.0; entry_t = -1.0;
          continue;
        }
        if (entry_t < 0.0) entry_t = rr.s[i].t;
        dwell += rr.s[i].t - rr.s[i - 1].t;
        if (dwell < dwell_need) continue;
        ++found;
        if (entry_t < best_entry_t) {
          best_entry_t = entry_t;
          best_dwell_t = rr.s[i].t;
          best = p;
          best_state = rr.s[i];
        }
        break;
      }
    }
  }
  const double search_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - ts).count();
  std::printf("  %d profiles propagated (%.0f s each) in %.0f ms — "
              "%d held the window\n", tried, search_dur, search_ms, found);
  if (found > 0) {
    std::printf("  earliest: elevator=%.2f throttle=%.2f | window entered at "
                "t=%.2f s, dwell satisfied at t=%.2f s\n",
                best.elevator, best.throttle, best_entry_t, best_dwell_t);
    std::printf("  state AT t=%.2f s: |v|=%.1f m/s, gamma=%.2f deg, z=%.1f m\n",
                best_dwell_t, best_state.speed, best_state.gamma_deg,
                best_state.z);
    std::printf("  PVA (local ENU, m / m·s^-1 / m·s^-2):\n");
    std::printf("    p=(%.1f, %.1f, %.1f)  v=(%.1f, %.1f, %.1f)  "
                "a=(%.3f, %.3f, %.3f)\n",
                best_state.x, best_state.y, best_state.z, best_state.vx,
                best_state.vy, best_state.vz, best_state.ax, best_state.ay,
                best_state.az);
    std::printf("  ^ a CANDIDATE for a prescribed head — it becomes one only "
                "after the checks listed above.\n");
    // Machine-readable line for the Hermite adapter check.
    std::printf("PVA_CSV,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                best_dwell_t, best_state.x, best_state.y, best_state.z,
                best_state.vx, best_state.vy, best_state.vz, best_state.ax,
                best_state.ay, best_state.az);
  } else {
    std::printf("  => no profile in this 2-D grid held the window.\n");
  }
  return 0;
}
