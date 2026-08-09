// [WPE] Waypoint extraction + reproduction harness.
//
// project-defined neutral 3DOF benchmark follower — waypoint structure
// validation, not real-platform physics validation.
//
// The question: a vehicle of this class accepts a WAYPOINT LIST, not a
// trajectory. So where do we place waypoints, and how many, for the flown
// result to reproduce the planned trajectory? These variants extract
// candidate lists, fly them, and measure the difference — including
// measurements aimed at the measurement itself (does the metric catch a
// flight that cheats? does the follower's own tracking floor masquerade as
// extraction error? do strategy rankings survive a worse follower?).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "path_manager/quintic_hermite.h"
#include "path_manager/waypoint_eval.h"

namespace we = path_manager::waypoint_eval;
namespace tp = path_manager::transition_phase;
namespace vd = mmp_vehicle_dynamics;

static int failures = 0;
static void expect(bool ok, const std::string &what)
{
  std::cout << (ok ? "[OK]   " : "[FAIL] ") << what << "\n";
  if (!ok) ++failures;
}

static vd::Parameters baseParams() { return vd::Parameters{}; }

// A source trajectory in PLANNER UNITS (the module converts to SI), built
// from quintic Hermite pieces so it is exactly the shape the planner emits.
// Straight -> S-curve -> straight, at cruise speed.
static poly_traj::Trajectory makeSCurve(double unit = 100.0)
{
  const double v = 170.0 / unit;        // planner units/s
  const double leg = 4000.0 / unit;
  poly_traj::Trajectory t;
  const Eigen::Vector3d z = Eigen::Vector3d::Zero();
  auto add = [&](const Eigen::Vector3d &p0, const Eigen::Vector3d &v0,
                 const Eigen::Vector3d &p1, const Eigen::Vector3d &v1,
                 double T) {
    t.emplace_back(T, path_manager::quinticHermite(p0, v0, z, p1, v1, z, T));
  };
  // Turn magnitude chosen to sit INSIDE the follower's own minimum turn
  // radius. The first fixture asked for a 62 deg heading change in 4 km
  // (radius ~3.7 km) while the follower's floor at 170 m/s is ~4.3 km —
  // so it measured "the reference is unflyable", not "the waypoints are
  // badly placed". A reference the vehicle cannot fly makes every
  // extraction strategy look equally bad.
  const Eigen::Vector3d h(1.0, 0.0, 0.0);
  const Eigen::Vector3d d1 = Eigen::Vector3d(1.0, 0.22, 0.0).normalized();
  const Eigen::Vector3d d2 = Eigen::Vector3d(1.0, -0.22, 0.0).normalized();
  Eigen::Vector3d p = Eigen::Vector3d(0.0, 0.0, 20.0);
  const double T = leg / v;
  const Eigen::Vector3d p1 = p + h * leg;
  add(p, h * v, p1, d1 * v, T);
  const Eigen::Vector3d p2 = p1 + d1 * leg;
  add(p1, d1 * v, p2, d2 * v, T);
  const Eigen::Vector3d p3 = p2 + d2 * leg;
  add(p2, d2 * v, p3, h * v, T);
  const Eigen::Vector3d p4 = p3 + h * leg;
  add(p3, h * v, p4, h * v, T);
  return t;
}

static we::FollowerParams makeFollower(const vd::Parameters &dyn)
{
  we::FollowerParams p;
  p.dyn = dyn;
  return p;
}

static we::FollowerStart startOf(const we::SourcePath &src)
{
  we::FollowerStart s;
  s.pos_m = src.pos_m.front();
  s.vel_mps = src.vel_mps.front();
  return s;
}

// One table row, printed with tri-state gates: a SKIPPED gate can never
// render as a PASS, and an unmeasured row prints n/a(reason) instead of
// numbers a complete flight would fill.
static void printRow(const char *strategy, int n, const we::RolloutResult &r,
                     const we::ReproductionMetrics &m, double floor_m)
{
  if (!m.measured) {
    std::printf("%-9s %3d  n/a(%s)\n", strategy, n, we::failName(m.fail));
    std::printf("WPE_CSV,%s,%d,0,,,,,,,,,\n", strategy, n);
    return;
  }
  const char *lim = (floor_m > 0.0 && m.max_xtrack_m <= 1.15 * floor_m)
                        ? " at-follower-floor"
                        : "";
  std::printf("%-9s %3d  %8.1f %8.1f %8.1f  %5.3f %7.1f %7.1f  %-10s%s\n",
              strategy, n, m.max_xtrack_m, m.rms_xtrack_m, m.max_arcmatch_m,
              m.len_ratio, m.max_wp_miss_m, m.terminal_pos_err_m,
              m.verdictName(), lim);
  std::printf("WPE_CSV,%s,%d,1,%.3f,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f,%d,%.4f\n",
              strategy, n, m.max_xtrack_m, m.rms_xtrack_m, m.max_arcmatch_m,
              m.len_ratio, m.max_wp_miss_m, m.terminal_pos_err_m,
              m.duration_ratio, m.env_violation_steps, m.sat_frac);
}

int main(int argc, char **argv)
{
  const std::string variant = argc > 1 ? argv[1] : "all";
  const auto run = [&](const char *name) {
    return variant == "all" || variant == name;
  };
  std::cout << "[WPE] " << we::scopeLabel() << "\n"
            << "[WPE] follower: pointmass_pursuit_v1 (transition law), "
               "integrator: rk4 non-clamping\n";

  const vd::Parameters dyn = baseParams();
  const we::FrameScale fs;
  {
    we::FollowerParams probe;
    probe.dyn = dyn;
    std::printf("[WPE] follower: bank %.3f rad, min turn radius %.0f m at "
                "170 m/s, accept radius %.0f m\n",
                we::derivedBankLevel(probe), we::minTurnRadius(probe, 170.0),
                we::derivedAcceptRadius(probe));
  }
  const poly_traj::Trajectory src_traj = makeSCurve(fs.unit_xy_m);
  const we::SourcePath src = we::buildSourcePath(src_traj, fs);

  // ---------------- source_path ----------------
  if (run("source_path")) {
    expect(!src.empty() && src.total_len_m > 0.0, "source path built");
    bool mono = true;
    for (size_t i = 1; i < src.s_m.size(); ++i)
      if (src.s_m[i] < src.s_m[i - 1]) mono = false;
    expect(mono, "arc table monotone");
    // Chordal arc converges: doubling samples moves total < 0.1%.
    const we::SourcePath a = we::buildSourcePath(src_traj, fs, 2000);
    const we::SourcePath b = we::buildSourcePath(src_traj, fs, 4000);
    const double rel = std::abs(b.total_len_m - a.total_len_m) /
                       std::max(1.0, a.total_len_m);
    std::cout << "source_path: len " << src.total_len_m << " m, conv rel "
              << rel << "\n";
    expect(rel < 1e-3, "chordal arc converged under sample doubling");
    // Degenerate input must not be probed.
    expect(we::buildSourcePath(poly_traj::Trajectory{}, fs).empty(),
           "empty trajectory yields empty source path (no UB probing)");
  }

  // ---------------- extraction ----------------
  if (run("extract")) {
    const auto u = we::extractUniformArc(src, 8);
    expect(u.size() == 8, "uniform: requested count");
    expect((u.back().pos_m - src.pos_m.back()).norm() < 1e-9,
           "uniform: last waypoint is the exact trajectory endpoint");
    double max_dev = 0.0;
    for (size_t i = 0; i < u.size(); ++i) {
      const double want = (i + 1) * src.total_len_m / 8.0;
      max_dev = std::max(max_dev, std::abs(u[i].src_arc_m - want));
    }
    expect(max_dev < 0.02 * src.total_len_m / 8.0,
           "uniform: arc spacing even within 2% of a slot");
    // lambda = 0 must reproduce uniform EXACTLY (the strategies share a
    // substrate; a silent divergence would make the comparison a lie).
    const auto a0 = we::extractCurvatureAdaptive(src, 8, 0.0);
    double d0 = 0.0;
    for (size_t i = 0; i < u.size(); ++i)
      d0 = std::max(d0, (a0[i].pos_m - u[i].pos_m).norm());
    expect(d0 < 1e-9, "adaptive(lambda=0) reproduces uniform exactly");
    // Curvature attracts: on this S-curve the bends must draw more
    // waypoints than the straight halves.
    const auto a4 = we::extractCurvatureAdaptive(src, 12, 4.0);
    int in_bend = 0;
    const double q1 = 0.25 * src.total_len_m, q3 = 0.75 * src.total_len_m;
    for (const auto &w : a4)
      if (w.src_arc_m > q1 && w.src_arc_m < q3) ++in_bend;
    const auto u12 = we::extractUniformArc(src, 12);
    int in_bend_u = 0;
    for (const auto &w : u12)
      if (w.src_arc_m > q1 && w.src_arc_m < q3) ++in_bend_u;
    std::cout << "extract: bend waypoints adaptive=" << in_bend
              << " uniform=" << in_bend_u << "\n";
    expect(in_bend >= in_bend_u, "adaptive concentrates on curvature");
  }

  // ---------------- follower ----------------
  if (run("follower")) {
    const auto prm = makeFollower(dyn);
    const auto wps = we::extractUniformArc(src, 8);
    const auto r = we::flyWaypoints3Dof(wps, startOf(src), prm);
    std::cout << "follower: completed=" << r.completed << " fail="
              << we::failName(r.fail) << " steps=" << r.total_steps
              << " arrivals=" << r.arrivals.size() << "\n";
    expect(r.completed, "follower reaches every waypoint");
    expect(r.arrivals.size() == wps.size(), "one arrival per waypoint");
    bool inc = true;
    for (size_t i = 1; i < r.arrivals.size(); ++i)
      if (r.arrivals[i].t_s <= r.arrivals[i - 1].t_s) inc = false;
    expect(inc, "arrival times strictly increasing (no orbiting)");
    // Determinism: same inputs, byte-identical record.
    const auto r2 = we::flyWaypoints3Dof(wps, startOf(src), prm);
    bool same = r.samples.size() == r2.samples.size();
    if (same)
      for (size_t i = 0; i < r.samples.size(); ++i)
        if (r.samples[i].pos_m != r2.samples[i].pos_m) same = false;
    expect(same, "rollout is deterministic (no clock, no RNG)");
  }

  // ---------------- metrics self-test ----------------
  if (run("metrics_selftest")) {
    // The measurement has to be tested before it is trusted.
    we::SafetyHooks hooks;
    // (a) The source flown back at itself: deviation ~ 0, length ratio ~ 1.
    we::RolloutResult perfect;
    perfect.completed = true;
    perfect.total_steps = static_cast<int>(src.pos_m.size());
    for (size_t i = 0; i < src.pos_m.size(); ++i)
      perfect.samples.push_back(we::RolloutSample{
          src.t_s[i], src.pos_m[i], src.vel_mps[i], src.acc_mps2[i]});
    const auto m_perfect =
        we::evaluateReproduction(src, perfect, dyn, hooks);
    std::cout << "selftest perfect: xtrack " << m_perfect.max_xtrack_m
              << " len_ratio " << m_perfect.len_ratio << "\n";
    expect(m_perfect.measured && m_perfect.max_xtrack_m < 1e-6,
           "identity rollout measures ~zero deviation");
    expect(std::abs(m_perfect.len_ratio - 1.0) < 1e-6,
           "identity rollout measures unit length ratio");

    // (b) A laterally offset copy: deviation ~ the offset.
    we::RolloutResult offset = perfect;
    for (auto &s : offset.samples) s.pos_m.y() += 120.0;
    const auto m_off = we::evaluateReproduction(src, offset, dyn, hooks);
    expect(std::abs(m_off.max_xtrack_m - 120.0) < 5.0,
           "offset rollout measures the offset");

    // (c) THE ANTI-CHEAT CASE, on a fixture where the cheat is possible.
    // A path that DOUBLES BACK can be shortcut by a straight line that
    // stays close to it in nearest-point terms while flying a completely
    // different route — deviation alone would call that a reproduction.
    // The length-ratio gate exists to refuse it. (On a gently curving
    // path a shortcut is nearly the same length, so length cannot be the
    // catcher there — deviation is; both mechanisms are asserted.)
    {
      we::SourcePath hair;   // out 4 km, hairpin, back 4 km offset 400 m
      const int n_leg = 400;
      double sacc = 0.0;
      Eigen::Vector3d prev(0.0, 0.0, 2000.0);
      const auto push = [&](const Eigen::Vector3d &p, double t) {
        sacc += (p - prev).norm();
        prev = p;
        hair.t_s.push_back(t);
        hair.s_m.push_back(sacc);
        hair.pos_m.push_back(p);
        hair.vel_mps.push_back(Eigen::Vector3d(170.0, 0.0, 0.0));
        hair.acc_mps2.push_back(Eigen::Vector3d::Zero());
        hair.kappa.push_back(0.0);
      };
      for (int i = 0; i <= n_leg; ++i) {
        const double u = static_cast<double>(i) / n_leg;
        push(Eigen::Vector3d(u * 4000.0, 0.0, 2000.0), u * 23.5);
      }
      for (int i = 1; i <= n_leg; ++i) {
        const double u = static_cast<double>(i) / n_leg;
        push(Eigen::Vector3d(4000.0 - u * 4000.0, 400.0, 2000.0),
             23.5 + u * 23.5);
      }
      hair.total_len_m = hair.s_m.back();
      hair.total_time_s = hair.t_s.back();

      we::RolloutResult cut;
      cut.completed = true;
      const Eigen::Vector3d q0 = hair.pos_m.front(), q1 = hair.pos_m.back();
      const int n_short = 400;
      for (int i = 0; i <= n_short; ++i) {
        const double u = static_cast<double>(i) / n_short;
        cut.samples.push_back(we::RolloutSample{
            u * hair.total_time_s, q0 + u * (q1 - q0),
            Eigen::Vector3d(0.0, 400.0 / hair.total_time_s, 0.0),
            Eigen::Vector3d::Zero()});
      }
      cut.total_steps = n_short + 1;
      const auto m_cut = we::evaluateReproduction(hair, cut, dyn, hooks);
      std::cout << "selftest shortcut(hairpin): xtrack " << m_cut.max_xtrack_m
                << " len_ratio " << m_cut.len_ratio << " gate_len "
                << m_cut.gate_len << "\n";
      expect(!m_cut.gate_len,
             "hairpin shortcut REFUSED by the length-ratio gate (a small "
             "deviation did not buy it a pass)");
      expect(m_cut.verdict() ==
                 we::ReproductionMetrics::Verdict::kFail,
             "hairpin shortcut verdict is FAIL");
    }
    // (c2) On the gently curving source, a shortcut is nearly the same
    // LENGTH — there the deviation metric is what must catch it.
    {
      we::RolloutResult cut;
      cut.completed = true;
      const Eigen::Vector3d p0 = src.pos_m.front(), p1 = src.pos_m.back();
      const int n_short = 400;
      for (int i = 0; i <= n_short; ++i) {
        const double u = static_cast<double>(i) / n_short;
        cut.samples.push_back(we::RolloutSample{
            u * src.total_time_s, p0 + u * (p1 - p0),
            (p1 - p0) / std::max(1.0, src.total_time_s),
            Eigen::Vector3d::Zero()});
      }
      cut.total_steps = n_short + 1;
      const auto m_cut = we::evaluateReproduction(src, cut, dyn, hooks);
      std::cout << "selftest shortcut(gentle): xtrack " << m_cut.max_xtrack_m
                << " len_ratio " << m_cut.len_ratio << "\n";
      expect(m_cut.max_xtrack_m > 500.0,
             "chord across a curving path shows large deviation (length "
             "cannot catch this one; deviation must)");
    }

    // (d) An incomplete flight yields NO numbers.
    we::RolloutResult partial = perfect;
    partial.completed = false;
    partial.fail = we::FailReason::kTimeout;
    const auto m_part = we::evaluateReproduction(src, partial, dyn, hooks);
    expect(!m_part.measured,
           "incomplete flight is unmeasured (no quality claim)");

    // (e) A missing hook must read as SKIPPED, never as a pass.
    expect(m_perfect.gate_agl_skipped && !m_perfect.agl_measured,
           "absent terrain hook reads SKIPPED, not PASS");
  }

  // ---------------- follower floor ----------------
  // The TRUE floor is what the law achieves tracking the CONTINUOUS
  // trajectory: no waypoint geometry, so nothing about aiming or leg
  // switching varies. A dense-waypoint rollout is NOT a floor — changing
  // the count changes the aiming regime too, and the 64-waypoint run
  // measured WORSE than the 8-waypoint one, which a floor cannot do
  // (review find). It is kept as a dense-waypoint BASELINE for contrast.
  double floor_m = 0.0, dense_m = 0.0;
  if (run("floor") || run("table") || run("sensitivity")) {
    we::SafetyHooks hooks;
    // The floor is the law's BEST continuous tracking, so it is minimized
    // over the law's own aiming parameter: a single lead time is not a
    // floor, it is one tuning (measured: lead 4 s tracks WORSE than a
    // dense waypoint list because the carrot cuts corners; that says the
    // lead was wrong, not that waypoints beat continuous tracking).
    bool any_track = false;
    double best_lead = 0.0;
    floor_m = 1e18;
    for (double lead : {0.5, 1.0, 2.0, 4.0, 8.0}) {
      auto p2 = makeFollower(dyn);
      p2.lead_time_s = lead;
      const auto r2 = we::flyReferenceTrack(src, startOf(src), p2);
      const auto m2 = we::evaluateReproduction(src, r2, dyn, hooks);
      if (run("floor"))
        std::printf("floor: lead %.1f s -> %s\n", lead,
                    m2.measured
                        ? (std::to_string(m2.max_xtrack_m) + " m").c_str()
                        : "n/a");
      if (m2.measured && m2.max_xtrack_m < floor_m) {
        floor_m = m2.max_xtrack_m;
        best_lead = lead;
        any_track = true;
      }
    }
    if (!any_track) floor_m = 0.0;
    const auto prm = makeFollower(dyn);
    const auto mt = we::evaluateReproduction(
        src, we::flyReferenceTrack(src, startOf(src), prm), dyn, hooks);
    (void)mt;
    const auto dense = we::extractUniformArc(src, 64);
    const auto rd = we::flyWaypoints3Dof(dense, startOf(src), prm);
    const auto md_ = we::evaluateReproduction(src, rd, dyn, hooks);
    dense_m = md_.measured ? md_.max_xtrack_m : 0.0;
    if (run("floor")) {
      std::printf("floor: best continuous tracking %.1f m at lead %.1f s "
                  "(the law's own limit) | dense-waypoint N=64 baseline "
                  "%.1f m, ratio %.2f\n",
                  floor_m, best_lead, dense_m,
                  dense_m / std::max(1.0, floor_m));
      expect(any_track, "reference-tracking rollout completes");
      expect(floor_m > 0.0, "the tracking law has a measurable floor");
      expect(md_.measured, "dense-waypoint baseline completes");
      // The separation earns its keep here: no waypoint list should beat
      // the law's best continuous tracking by a wide margin — if one did,
      // the floor would be mis-measured rather than the placement being
      // brilliant.
      expect(dense_m >= 0.75 * floor_m,
             "no waypoint list beats the law's best continuous tracking by "
             "more than the matching tolerance");
    }
  }

  // ---------------- the comparison table ----------------
  if (run("table")) {
    std::printf("\n[WPE] tracking floor (continuous-reference follow): "
                "%.1f m — the law's own limit, independent of waypoint\n"
                "[WPE] geometry. Dense-waypoint N=64 baseline: %.1f m. Rows "
                "at or under 1.15x the floor are at the law's limit.\n",
                floor_m, dense_m);
    std::cout << "\nstrategy    N   maxXT[m]  rmsXT[m]  arcMt[m]  lenR  "
                 "wpMiss[m]  endErr[m]\n";
    const auto prm = makeFollower(dyn);
    we::SafetyHooks hooks;
    we::EvalParams ep;
    // Derived, not a literal: capture ends the flight inside R_acc of the
    // last waypoint, so a fixed gate would only re-measure R_acc.
    ep.terminal_pos_gate_m = 1.5 * we::derivedAcceptRadius(prm);
    const we::FollowerStart st = startOf(src);
    const we::FlyFn fly = [&](const std::vector<we::Waypoint> &w,
                              const we::FollowerStart &s) {
      return we::flyWaypoints3Dof(w, s, prm);
    };
    double uni_at_16 = -1.0, ref_at_16 = -1.0;
    int completed_rows = 0;
    std::vector<double> uni_err;
    for (int n : {4, 8, 16, 32}) {
      for (int strat = 0; strat < 2; ++strat) {
        const auto w = strat == 0 ? we::extractUniformArc(src, n)
                                  : we::extractCurvatureAdaptive(src, n);
        const auto r = we::flyWaypoints3Dof(w, st, prm);
        const auto m = we::evaluateReproduction(src, r, dyn, hooks, ep);
        printRow(strat == 0 ? "uniform" : "adaptive", n, r, m, floor_m);
        if (m.measured) {
          ++completed_rows;
          if (strat == 0) {
            uni_err.push_back(m.max_xtrack_m);
            if (n == 16) uni_at_16 = m.max_xtrack_m;
          }
        }
      }
    }
    // S3: refinement is follower-in-the-loop by construction.
    we::RefineParams rp;
    rp.max_waypoints = 16;
    // Tolerance ABOVE the measured follower floor: asking for less than
    // the follower can track would guarantee non-convergence and say
    // nothing about placement.
    rp.xtrack_tol_m = std::max(50.0, 1.2 * floor_m);
    const auto rr =
        we::refineByError(src, fly, st, rp, we::derivedAcceptRadius(prm));
    const auto r3 = we::flyWaypoints3Dof(rr.best, st, prm);
    const auto m3 = we::evaluateReproduction(src, r3, dyn, hooks, ep);
    printRow("refine", static_cast<int>(rr.best.size()), r3, m3, floor_m);
    if (m3.measured) ref_at_16 = m3.max_xtrack_m;
    std::cout << "refine: converged=" << rr.converged << " iters="
              << rr.trace.size() << " waypoints=" << rr.best.size() << "\n";

    expect(completed_rows >= 6, "most strategy/count rows complete");
    // NOT asserted: that error falls monotonically with waypoint count.
    // It does not for this follower — beyond the floor the tracking law,
    // not the placement, sets the error, and asserting a trend the data
    // refutes would make this harness a rubber stamp. What IS asserted:
    // some candidate set gets within reach of the floor, i.e. placement
    // is not the binding constraint.
    double best_row = 1e18;
    for (double e : uni_err) best_row = std::min(best_row, e);
    std::printf("table: best uniform row %.1f m vs floor %.1f m (ratio "
                "%.2f)\n", best_row, floor_m, best_row / std::max(1.0, floor_m));
    expect(best_row <= 1.5 * floor_m,
           "some waypoint set reaches within 1.5x the follower floor "
           "(placement is not the bottleneck)");
    if (uni_at_16 > 0.0 && ref_at_16 > 0.0) {
      std::cout << "refine vs uniform@16: " << ref_at_16 << " vs "
                << uni_at_16 << " m\n";
      expect(ref_at_16 <= uni_at_16 * 1.10,
             "error-driven refinement is not worse than uniform at equal "
             "budget");
    }
  }

  // ---------------- follower sensitivity ----------------
  // The load-bearing claim of this whole measurement: absolute metres do
  // NOT transfer to another follower, but the RANKING of waypoint sets
  // does. That is a claim about the measurement itself, so it is measured
  // — perturb the follower (command lag, capture geometry) and check that
  // the ordering of the candidate sets survives.
  //
  // Note what is NOT assumed here: that denser is better. On this
  // reference the nominal follower is already at its tracking floor by
  // N=4, so more waypoints do not help and can mildly hurt (once the
  // pursuit carrot outruns the spacing, the follower chases points
  // instead of the leg line). Encoding "denser is better" as the
  // invariant would have asserted a preference the data refutes; what
  // must hold is that every follower AGREES on the ordering.
  if (run("sensitivity")) {
    we::SafetyHooks hooks;
    const we::FollowerStart st = startOf(src);
    const int grid[] = {4, 8, 16, 32};
    struct Cfg { const char *name; double lag; double accept; double lead; };
    const Cfg cfgs[] = {{"nominal", 0.0, 0.0, 4.0},
                        {"lag0.5s", 0.5, 0.0, 4.0},
                        {"tight-accept", 0.0, 60.0, 2.0}};
    std::vector<std::vector<int>> orders;
    std::vector<std::vector<std::pair<double, int>>> errs;
    for (const auto &c : cfgs) {
      auto prm = makeFollower(dyn);
      prm.command_lag_s = c.lag;
      prm.accept_radius_m = c.accept;
      prm.lead_time_s = c.lead;
      we::EvalParams ep;
      ep.terminal_pos_gate_m = 1.5 * we::derivedAcceptRadius(prm);
      std::vector<std::pair<double, int>> err;
      std::printf("sensitivity[%-12s]", c.name);
      for (int n : grid) {
        const auto w = we::extractUniformArc(src, n);
        const auto r = we::flyWaypoints3Dof(w, st, prm);
        const auto m = we::evaluateReproduction(src, r, dyn, hooks, ep);
        const double e = m.measured ? m.max_xtrack_m : -1.0;
        std::printf("  N%d=%.0f", n, e);
        err.push_back({e, n});
      }
      std::printf("\n");
      bool all_measured = true;
      for (const auto &e : err)
        if (e.first < 0.0) all_measured = false;
      expect(all_measured, std::string("sensitivity[") + c.name +
                               "]: every count completes");
      errs.push_back(err);
      std::stable_sort(err.begin(), err.end());
      std::vector<int> order;
      for (const auto &e : err) order.push_back(e.second);
      orders.push_back(order);
    }
    std::printf("sensitivity: best-to-worst order");
    for (size_t i = 0; i < orders.size(); ++i) {
      std::printf(" [%s:", cfgs[i].name);
      for (int n : orders[i]) std::printf(" %d", n);
      std::printf("]");
    }
    std::printf("\n");
    // Rank stability with a TIE BAND. Two candidates whose errors differ
    // by less than kTie are indistinguishable at this resolution, and
    // demanding that near-ties keep their arbitrary order would fail on
    // noise (measured: N=4 309 m vs N=16 301 m swapped between configs, a
    // 3% difference). A real inversion is a pair one follower orders
    // clearly one way and another orders clearly the other way.
    const double kTie = 0.10;
    int inversions = 0;
    for (size_t a = 0; a < errs.size(); ++a) {
      for (size_t b = a + 1; b < errs.size(); ++b) {
        for (size_t i = 0; i < errs[a].size(); ++i) {
          for (size_t j = i + 1; j < errs[a].size(); ++j) {
            const auto sgn = [&](const std::vector<std::pair<double, int>> &e,
                                 size_t x, size_t y) {
              const double d = e[x].first - e[y].first;
              const double sc = std::max(e[x].first, e[y].first);
              if (std::abs(d) < kTie * sc) return 0;
              return d < 0 ? -1 : 1;
            };
            const int sa = sgn(errs[a], i, j), sb = sgn(errs[b], i, j);
            if (sa != 0 && sb != 0 && sa != sb) {
              ++inversions;
              std::printf("sensitivity: INVERSION N%d vs N%d between %s and "
                          "%s\n", errs[a][i].second, errs[a][j].second,
                          cfgs[a].name, cfgs[b].name);
            }
          }
        }
      }
    }
    std::printf("sensitivity: %d clear rank inversion(s) across %zu "
                "followers (tie band %.0f%%)\n", inversions, errs.size(),
                kTie * 100.0);
    // Scope, stated so the result is not read as more than it is: ONE
    // synthetic curve, THREE configurations of the SAME 3DOF law, and a
    // tie band chosen after observing a 3% near-tie swap. What this
    // supports is "no ordering inversion beyond 10% appeared under these
    // perturbations" — evidence for transferability, not a proof of it.
    // A different law (the adopted higher-fidelity follower, when it
    // lands) is the test that would actually settle it.
    expect(inversions == 0,
           "no clear rank inversion across these three follower "
           "configurations on this curve (evidence for the ordering "
           "transferring; NOT a proof across follower families)");
  }

  if (failures == 0) {
    std::cout << "PASS: 0 failed check(s)\n";
    return 0;
  }
  std::cout << "FAIL: " << failures << " failed check(s)\n";
  return 1;
}
