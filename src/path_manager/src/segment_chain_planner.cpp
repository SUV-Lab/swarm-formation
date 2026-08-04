#include "path_manager/segment_chain_planner.h"

#include <algorithm>
#include <chrono>
#include <limits>

namespace path_manager {

namespace {

// Dense (t, cumulative arc) table. Baseline and chain have different total
// durations, so the deviation sweep compares them at matched ARC fractions —
// matching normalized time would skew everything after the first seam.
struct ArcTable {
  std::vector<double> t, s;
  double total{0.0};
};

ArcTable buildArcTable(const poly_traj::Trajectory &traj, int samples)
{
  ArcTable a;
  const double T = traj.getTotalDuration();
  a.t.reserve(samples + 1);
  a.s.reserve(samples + 1);
  Eigen::Vector3d prev = traj.getPos(0.0);
  double s = 0.0;
  for (int k = 0; k <= samples; ++k) {
    const double tt = std::min(T, k * T / samples);
    const Eigen::Vector3d p = traj.getPos(tt);
    s += (p - prev).norm();
    prev = p;
    a.t.push_back(tt);
    a.s.push_back(s);
  }
  a.total = s;
  return a;
}

double timeAtArcFrac(const ArcTable &a, double frac)
{
  const double target = frac * a.total;
  const auto it = std::lower_bound(a.s.begin(), a.s.end(), target);
  const size_t i = static_cast<size_t>(std::distance(a.s.begin(), it));
  if (i == 0) return a.t.front();
  if (i >= a.s.size()) return a.t.back();
  const double s0 = a.s[i - 1], s1 = a.s[i];
  const double w = (s1 > s0) ? (target - s0) / (s1 - s0) : 0.0;
  return a.t[i - 1] + w * (a.t[i] - a.t[i - 1]);
}

}  // namespace

SegmentChainPlanner::SegmentChainPlanner(rclcpp::Node::SharedPtr node,
                                         std::shared_ptr<PathManager> path_manager,
                                         swarm_formation::LogManager *log_manager,
                                         int segments, bool inherit_route)
    : node_(node), pm_(path_manager), log_(log_manager),
      segments_(std::max(2, segments)), inherit_route_(inherit_route) {}

bool SegmentChainPlanner::plan(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &start_acc,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               bool start_vel_synthesized)
{
  // Stage-1 scope: one goal. Multi-waypoint missions need a waypoint-to-span
  // assignment that does not exist yet — fall back to the single-shot plan.
  if (waypoints.size() != 1) {
    log_->warnf("[CHAIN] %zu waypoints — stage 1 chains single-goal missions "
                "only, falling back to the single-shot plan", waypoints.size());
    pm_->setStartVelSynthesized(start_vel_synthesized);
    return pm_->planGlobalTraj(start_pos, start_vel, start_acc, waypoints,
                               Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero());
  }

  const auto t_wall = std::chrono::steady_clock::now();

  // === Baseline: the unsplit mission, exactly today's behavior ===
  log_->infof("[CHAIN] baseline plan (unsplit mission; %d chained segments "
              "follow)", segments_);
  pm_->setStartVelSynthesized(start_vel_synthesized);
  if (!pm_->planGlobalTraj(start_pos, start_vel, start_acc, waypoints,
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())) {
    log_->errorf("[CHAIN] baseline plan failed — nothing to chain, nothing "
                 "to fly");
    return false;
  }
  const poly_traj::Trajectory baseline = pm_->traj_.local_traj.traj;
  const double T = baseline.getTotalDuration();
  if (T <= 1e-6 || baseline.getPieceNum() < segments_) {
    // Too short to carve N spans out of. traj_ already holds the baseline.
    log_->warnf("[CHAIN] baseline too small to split (%.3f s, %d pieces) — "
                "flying the baseline", T, baseline.getPieceNum());
    return true;
  }

  // === Junction contracts sampled from the baseline ===
  std::vector<Contract> contracts;
  const double span = T / segments_;
  double t_prev = 0.0;
  for (int i = 1; i < segments_; ++i) {
    Contract c;
    c.t = clearJunctionTime(span * i, t_prev, span, baseline);
    t_prev = c.t;
    c.pos = baseline.getPos(c.t);
    c.vel = baseline.getVel(c.t);
    c.acc = baseline.getAcc(c.t);
    log_->infof("[CHAIN] contract %d @t=%.1f/%.1f s: pos=(%.2f, %.2f, %.2f) "
                "|v|=%.3f u/s vz=%+.3f |a|=%.3f u/s^2",
                i, c.t, T, c.pos.x(), c.pos.y(), c.pos.z(),
                c.vel.norm(), c.vel.z(), c.acc.norm());
    if (nearRiskZone(c.pos)) {
      log_->warnf("[CHAIN] contract %d sits inside a zone's moat+taper reach "
                  "even after nudging — the junction earns exemptions the "
                  "baseline never had; the risk comparison is contaminated "
                  "there", i);
    }
    contracts.push_back(c);
  }

  // Baseline's committed front-end products, sliced per span. Copied out
  // before the segment runs overwrite the manager's last-plan retention.
  std::vector<RouteSlice> slices;
  if (inherit_route_) {
    slices = sliceCommittedRoute(pm_->lastCommittedRoute(),
                                 pm_->lastCommittedCapRef(), contracts);
    if (slices.size() != static_cast<size_t>(segments_)) {
      log_->warnf("[CHAIN] committed route did not slice cleanly (%zu/%d) — "
                  "falling back to per-span front-end search",
                  slices.size(), segments_);
      slices.clear();
    } else {
      for (size_t j = 0; j < slices.size(); ++j) {
        log_->infof("[CHAIN] slice %zu: %zu vertices (cap %zu)",
                    j + 1, slices[j].path.size(), slices[j].cap.size());
      }
    }
  }

  // === Chained segment runs (sequential; each is a full pipeline run) ===
  std::vector<poly_traj::Trajectory> runs;
  for (int i = 0; i < segments_; ++i) {
    const bool last = (i + 1 == segments_);
    const Eigen::Vector3d head_pos = (i == 0) ? start_pos : contracts[i - 1].pos;
    const Eigen::Vector3d head_vel = (i == 0) ? start_vel : contracts[i - 1].vel;
    const Eigen::Vector3d head_acc = (i == 0) ? start_acc : contracts[i - 1].acc;
    // The last span flies to the REAL goal: the original waypoint (AGL
    // semantics) under the arrival contract, exactly as the single-shot
    // plan would end.
    const std::vector<Eigen::Vector3d> goal =
        last ? waypoints : std::vector<Eigen::Vector3d>{contracts[i].pos};
    const Eigen::Vector3d end_vel =
        last ? Eigen::Vector3d::Zero() : contracts[i].vel;
    const Eigen::Vector3d end_acc =
        last ? Eigen::Vector3d::Zero() : contracts[i].acc;

    log_->infof("[CHAIN] segment %d/%d: head (%.2f, %.2f, %.2f) |v|=%.3f -> "
                "%s (%.2f, %.2f, %.2f)",
                i + 1, segments_, head_pos.x(), head_pos.y(), head_pos.z(),
                head_vel.norm(), last ? "goal" : "junction",
                goal.back().x(), goal.back().y(), goal.back().z());
    // Contract heads are trajectory-derived states — never re-aim
    // ([VEL-ALIGN]) and never floor ([STALL-FLOOR]) them: the neighbour's
    // tail pins the same state verbatim.
    pm_->setStartVelSynthesized(i == 0 ? start_vel_synthesized : false);
    const RouteSlice *slice =
        (i < static_cast<int>(slices.size())) ? &slices[i] : nullptr;
    if (!pm_->planGlobalTraj(head_pos, head_vel, head_acc, goal,
                             end_vel, end_acc, /*junction_goal=*/!last,
                             /*junction_head=*/i != 0,
                             slice ? &slice->path : nullptr,
                             slice ? &slice->cap : nullptr)) {
      // Degrade loudly to the baseline: the mission still flies, and the
      // failed experiment is visible in the log, not in the sky.
      log_->warnf("[CHAIN] segment %d/%d FAILED — restoring and flying the "
                  "baseline", i + 1, segments_);
      const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
      pm_->traj_.setGlobalTraj(baseline, now_s);
      pm_->traj_.setLocalTraj(baseline, now_s, pm_->traj_.local_traj.drone_id);
      pm_->publishTrajectoryViz(baseline, baseline);
      return true;
    }
    runs.push_back(pm_->traj_.local_traj.traj);
  }

  // === Stitch and store ===
  poly_traj::Trajectory chained = runs.front();
  for (size_t i = 1; i < runs.size(); ++i) chained.append(runs[i]);

  const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  // GLOBAL slot = the baseline OPTIMIZED trajectory (comparison reference,
  // see the class comment). Must precede setLocalTraj: setGlobalTraj resets
  // the local slot's bookkeeping.
  pm_->traj_.setGlobalTraj(baseline, now_s);
  pm_->traj_.setLocalTraj(chained, now_s, pm_->traj_.local_traj.drone_id);
  pm_->publishTrajectoryViz(chained, baseline);

  logChainReport(baseline, runs, contracts, chained);

  const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_wall).count();
  log_->infof("[CHAIN] chained %d segments: %d pieces, %.1f s flight "
              "(baseline %.1f s), planned in %.1f ms wall",
              segments_, chained.getPieceNum(), chained.getTotalDuration(), T,
              wall_ms);
  return true;
}

std::vector<SegmentChainPlanner::RouteSlice>
SegmentChainPlanner::sliceCommittedRoute(
    const std::vector<Eigen::Vector3d> &route,
    const std::vector<double> &cap,
    const std::vector<Contract> &contracts) const
{
  if (route.size() < 2) return {};
  const bool has_cap = cap.size() == route.size();

  // Forward polyline projection of every contract position. The contract was
  // sampled from the OPTIMIZED baseline, which deviates from the committed
  // route by design — projection finds where to cut; the injected slice
  // endpoint is the contract position itself, so route endpoint == BC.
  struct Cut { size_t seg; double u; Eigen::Vector3d pos; double cap; };
  std::vector<Cut> cuts;
  size_t seg0 = 0;
  for (const auto &c : contracts) {
    double best_d2 = std::numeric_limits<double>::max();
    Cut best{seg0, 0.0, c.pos, 0.0};
    for (size_t i = seg0; i + 1 < route.size(); ++i) {
      const Eigen::Vector3d &a = route[i];
      const Eigen::Vector3d ab = route[i + 1] - a;
      const double len2 = ab.squaredNorm();
      double u = (len2 > 1e-12) ? (c.pos - a).dot(ab) / len2 : 0.0;
      u = std::min(1.0, std::max(0.0, u));
      const double d2 = (a + u * ab - c.pos).squaredNorm();
      if (d2 < best_d2) { best_d2 = d2; best.seg = i; best.u = u; }
    }
    best.cap = has_cap ? std::max(cap[best.seg], cap[best.seg + 1]) : 0.0;
    // A cut landing at or before the previous one means the projection
    // folded back (self-crossing route) — the caller falls back to per-span
    // search rather than planning a backwards slice.
    if (!cuts.empty() && (best.seg < cuts.back().seg ||
                          (best.seg == cuts.back().seg &&
                           best.u <= cuts.back().u))) {
      return {};
    }
    cuts.push_back(best);
    seg0 = best.seg;
  }

  // Virtual cuts at the two mission ends make every slice the same shape.
  std::vector<Cut> all;
  all.push_back({0, 0.0, route.front(), has_cap ? cap.front() : 0.0});
  all.insert(all.end(), cuts.begin(), cuts.end());
  all.push_back({route.size() - 2, 1.0, route.back(),
                 has_cap ? cap.back() : 0.0});

  std::vector<RouteSlice> slices;
  for (size_t j = 0; j + 1 < all.size(); ++j) {
    const Cut &a = all[j], &b = all[j + 1];
    RouteSlice s;
    auto push = [&](const Eigen::Vector3d &p, double cp) {
      if (!s.path.empty() && (p - s.path.back()).norm() < 1e-3) {
        // Coincident with the previous vertex: keep the larger cap so the
        // dedup never tightens the ceiling.
        if (has_cap && !s.cap.empty())
          s.cap.back() = std::max(s.cap.back(), cp);
        return;
      }
      s.path.push_back(p);
      if (has_cap) s.cap.push_back(cp);
    };
    push(a.pos, a.cap);
    for (size_t i = a.seg + 1; i <= b.seg; ++i)
      push(route[i], has_cap ? cap[i] : 0.0);
    push(b.pos, b.cap);
    if (s.path.size() < 2) return {};  // degenerate span
    slices.push_back(std::move(s));
  }
  return slices;
}

double SegmentChainPlanner::clearJunctionTime(
    double t_nominal, double t_prev, double span,
    const poly_traj::Trajectory &traj) const
{
  // Monotonicity by construction: cand >= t_prev + 0.2*span, and since the
  // previous junction accepted at most its nominal + 0.4*span, the next
  // window [nominal + 0.6*span - 0.4*span, ...] is never empty. Junction
  // times can therefore neither coincide nor invert, at ANY segment count —
  // a total-duration window here let ±10% nudges cross once segments > 4.
  const double lo = t_prev + 0.2 * span;
  const auto in_window = [&](double cand) { return cand >= lo; };
  const auto clear = [&](double cand) {
    return !nearRiskZone(traj.getPos(cand));
  };
  if (in_window(t_nominal) && clear(t_nominal)) return t_nominal;
  // Nearest-to-nominal zone-clear candidate wins (the original policy)...
  double calmest = std::max(t_nominal, lo);
  double calmest_a = traj.getAcc(calmest).norm();
  for (double step = 0.08; step <= 0.40 + 1e-9; step += 0.08) {
    for (const double sgn : {+1.0, -1.0}) {
      const double cand = t_nominal + sgn * step * span;
      if (!in_window(cand)) continue;
      if (clear(cand)) {
        log_->infof("[CHAIN] junction @%.1f s nudged to %.1f s (clear of "
                    "zone moat+taper)", t_nominal, cand);
        return cand;
      }
      const double a = traj.getAcc(cand).norm();
      if (a < calmest_a) { calmest = cand; calmest_a = a; }
    }
  }
  // ...but when the window is blanketed by zones (r3-class gauntlets), fall
  // back to the CALMEST baseline state in it, not the nominal time. A
  // junction is a handoff, and a handoff mid-maneuver pins a hard BC in the
  // field's fiercest gradient: observed on r3, the equal-time junction
  // (|a| = 0.111 u/s^2, banking through the zone saddle) trapped the final
  // segment's solve in a condemned basin (bank 55 deg, 41% envelope
  // violations, terrain overlap), while a steady-state junction 30 s
  // earlier (|a| = 0.030) chained cleanly through the same field.
  log_->warnf("[CHAIN] junction @%.1f s: window blanketed by zones — taking "
              "the calmest baseline state @%.1f s (|a|=%.3f u/s^2)",
              t_nominal, calmest, calmest_a);
  return calmest;
}

bool SegmentChainPlanner::nearRiskZone(const Eigen::Vector3d &p) const
{
  const double taper = pm_->riskGoalTaperRadius();
  for (const auto &z : pm_->riskZones()) {
    const double d = (p.head<2>() - z.center.head<2>()).norm();
    if (d < z.reach + taper) return true;
  }
  return false;
}

void SegmentChainPlanner::logChainReport(
    const poly_traj::Trajectory &baseline,
    const std::vector<poly_traj::Trajectory> &runs,
    const std::vector<Contract> &contracts,
    const poly_traj::Trajectory &chained) const
{
  log_->infof("[CHAIN-REPORT] ===== stage-1 split verification =====");

  // Per-run shape: piece counts and durations reveal a span whose sub-plan
  // diverged wildly from its share of the baseline (e.g. a junction that
  // forced a detour the unsplit optimum never took).
  for (size_t i = 0; i < runs.size(); ++i) {
    const ArcTable at = buildArcTable(runs[i], 200);
    log_->infof("[CHAIN-REPORT] run %zu/%zu: %d pieces, %.1f s, %.1f u arc",
                i + 1, runs.size(), runs[i].getPieceNum(),
                runs[i].getTotalDuration(), at.total);
  }

  // Seam continuity. Both sides of every junction were solved against the
  // SAME hard PVA contract, so dP/dV/dA are solver arithmetic, not geometry —
  // anything above 1e-6 means a BC was mangled on the way in (the historical
  // failure mode: [GOAL AGL] re-adding terrain under a junction z). The two
  // tail/head-vs-contract columns say WHICH side broke. Jerk is EXPECTED to
  // jump: the seam is C2 by contract while MINCO's internal joints are C4 —
  // that jump is the intrinsic price of splitting, report it, don't gate it.
  double worst_seam = 0.0;
  for (size_t i = 0; i + 1 < runs.size(); ++i) {
    const auto &a = runs[i];
    const auto &b = runs[i + 1];
    const Eigen::Vector3d dp = a.getJuncPos(a.getPieceNum()) - b.getJuncPos(0);
    const Eigen::Vector3d dv = a.getJuncVel(a.getPieceNum()) - b.getJuncVel(0);
    const Eigen::Vector3d da = a.getJuncAcc(a.getPieceNum()) - b.getJuncAcc(0);
    const Eigen::Vector3d dj =
        a.getJer(a.getTotalDuration()) - b.getJer(0.0);
    const double tail_err =
        (a.getJuncPos(a.getPieceNum()) - contracts[i].pos).norm();
    const double head_err = (b.getJuncPos(0) - contracts[i].pos).norm();
    worst_seam = std::max({worst_seam, dp.norm(), dv.norm(), da.norm()});
    log_->infof("[CHAIN-REPORT] seam %zu: |dP|=%.3e u |dV|=%.3e u/s "
                "|dA|=%.3e u/s^2 | jerk jump |dJ|=%.3f u/s^3 | "
                "tail-vs-contract %.3e, head-vs-contract %.3e",
                i + 1, dp.norm(), dv.norm(), da.norm(), dj.norm(),
                tail_err, head_err);
  }

  // Baseline vs chain, at matched arc fractions (durations differ).
  const ArcTable ab = buildArcTable(baseline, 400);
  const ArcTable ac = buildArcTable(chained, 400);
  double dev_max = 0.0, dev_sum = 0.0, dev_max_s = 0.0;
  constexpr int kSweep = 100;
  for (int k = 0; k <= kSweep; ++k) {
    const double frac = static_cast<double>(k) / kSweep;
    const Eigen::Vector3d pb = baseline.getPos(timeAtArcFrac(ab, frac));
    const Eigen::Vector3d pc = chained.getPos(timeAtArcFrac(ac, frac));
    const double d = (pb - pc).norm();
    dev_sum += d;
    if (d > dev_max) {
      dev_max = d;
      dev_max_s = frac * ab.total;
    }
  }
  log_->infof("[CHAIN-REPORT] baseline %.1f s / %.1f u arc vs chained "
              "%.1f s / %.1f u arc (%+.2f%% time, %+.2f%% arc)",
              baseline.getTotalDuration(), ab.total,
              chained.getTotalDuration(), ac.total,
              100.0 * (chained.getTotalDuration() /
                       std::max(1e-9, baseline.getTotalDuration()) - 1.0),
              100.0 * (ac.total / std::max(1e-9, ab.total) - 1.0));
  log_->infof("[CHAIN-REPORT] deviation vs baseline @matched arc: mean "
              "%.2f u, max %.2f u @s=%.1f u (%.1f km)",
              dev_sum / (kSweep + 1), dev_max, dev_max_s, dev_max_s * 0.1);

  // One-line verdict for the log grep: seams are the stage-1 acceptance
  // criterion; the deviation numbers are context, not a gate.
  if (worst_seam < 1e-6) {
    log_->infof("[CHAIN-REPORT] verdict: PASS — all seams C2-continuous "
                "(worst |d(P,V,A)| = %.3e)", worst_seam);
  } else {
    log_->warnf("[CHAIN-REPORT] verdict: CHECK — seam discontinuity %.3e "
                "above 1e-6: a boundary condition was altered between the "
                "contract and the solve (AGL rewrite? re-aimed head vel?)",
                worst_seam);
  }
}

}  // namespace path_manager
