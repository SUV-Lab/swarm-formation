#include "path_manager/segment_chain_planner.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <thread>

#include "path_manager/terminal_phase.h"

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

  // [CHAIN-PAR] route-parallel mode replaces the whole baseline flow.
  {
    const auto dp = [&](const char *n, bool def) {
      if (!node_->has_parameter(n)) node_->declare_parameter(n, def);
      bool v = def;
      node_->get_parameter(n, v);
      return v;
    };
    const bool author = dp("chain/author_from_route", false);
    const bool par = dp("chain/parallel", false);
    if (author) {
      // Route mode authors contracts and builds every worker's optimizer
      // from the MISSION-WIDE parameter surface — chain/seg<i>/params has
      // no per-segment application point here (and the four
      // PathManager-consumed values could never differ per PARALLEL worker
      // anyway: they live on the shared manager). A dropped requirement
      // must never be silent.
      const std::vector<SegmentOverrides> over = readSegmentOverrides();
      for (int i = 0; i < segments_; ++i) {
        if (!over[static_cast<size_t>(i)].label.empty()) {
          log_->warnf("[CHAIN-PAR] seg%d overrides IGNORED in route mode "
                      "(%s) — use the baseline flow when the segment "
                      "requirement must hold",
                      i + 1, over[static_cast<size_t>(i)].label.c_str());
        }
      }
      return planRouteParallel(start_pos, start_vel, start_acc, waypoints,
                               start_vel_synthesized, par);
    } else if (par) {
      log_->warnf("[CHAIN] chain/parallel needs chain/author_from_route "
                  "(the baseline is inherently sequential) — running the "
                  "sequential baseline flow");
    }
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

  // === [STAGE-2] per-segment requirement overrides (read before the
  // contracts: a slow segment's ceiling must shape its junction speeds) ===
  const std::vector<SegmentOverrides> seg_over = readSegmentOverrides();
  bool any_over = false;
  for (const auto &so : seg_over) any_over |= !so.params.empty();
  // Effective speed ceiling per segment: its optimization/max_vel override,
  // else the mission-wide value. A junction pinned FASTER than a bounding
  // segment's ceiling is a self-contradictory hard BC — the solve brakes
  // through a state its own feasibility term condemns and the envelope
  // audit prices the transient as violations (measured: 34.7% > the 30%
  // hard gate on a 1.8 u/s segment fed 2.0 u/s boundaries).
  double vmax_global = std::numeric_limits<double>::infinity();
  if (node_->has_parameter("optimization/max_vel"))
    vmax_global = node_->get_parameter("optimization/max_vel").as_double();
  std::vector<double> seg_vmax(static_cast<size_t>(segments_), vmax_global);
  for (int i = 0; i < segments_; ++i)
    for (const auto &p : seg_over[static_cast<size_t>(i)].params)
      if (p.get_name() == "optimization/max_vel")
        seg_vmax[static_cast<size_t>(i)] = p.as_double();
  if (seg_vmax.front() < vmax_global - 1e-12 &&
      start_vel.norm() > seg_vmax.front() + 1e-9) {
    log_->warnf("[CHAIN] mission start speed %.3f exceeds segment 1's "
                "ceiling %.3f — the head BC is externally given and stays;"
                " expect envelope pressure at the head",
                start_vel.norm(), seg_vmax.front());
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
    // Junction speed = min ceiling of the two segments it bounds, direction
    // preserved. Both sides pin the SAME rescaled state, so the seam stays
    // exact; the faster neighbour absorbs the deceleration in its interior,
    // where its own ceiling allows it. Only an OVERRIDE-lowered ceiling may
    // rewrite a contract: the mission-wide optimization/max_vel is a SOFT
    // cubic penalty the converged baseline sits ~0.4% above by design, so
    // comparing against it would clip every junction of every mission and
    // silently change the stage-1 (no-override) contracts.
    const double vcap = std::min(seg_vmax[static_cast<size_t>(i - 1)],
                                 seg_vmax[static_cast<size_t>(i)]);
    if (vcap < vmax_global - 1e-12 && c.vel.norm() > vcap + 1e-9) {
      log_->infof("[CHAIN] contract %d speed %.3f -> %.3f u/s "
                  "(slow-segment ceiling, direction kept)",
                  i, c.vel.norm(), vcap);
      c.vel *= vcap / c.vel.norm();
    }
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

  // Union of pristine (mission-wide) values of every overridden parameter.
  std::map<std::string, rclcpp::Parameter> pristine;
  if (any_over) {
    for (const auto &so : seg_over)
      for (const auto &p : so.params)
        if (!pristine.count(p.get_name()))
          pristine.emplace(p.get_name(), node_->get_parameter(p.get_name()));
  }
  // Mission-wide values must survive EVERY exit path (segment failure
  // returns early with the baseline restored; the next mission's baseline
  // must plan pristine). Local classes share the member function's access.
  struct ParamRestore {
    SegmentChainPlanner *self{nullptr};
    const std::map<std::string, rclcpp::Parameter> *vals{nullptr};
    bool done{false};
    // Explicitly invoked BEFORE the final evaluation on every path — the
    // stitched flight must be judged against MISSION-WIDE parameters and
    // the mission-wide dynamics model, not whichever segment's overrides
    // happened to be applied last. The destructor is only the backstop.
    void restore() {
      if (done || !self || !vals || vals->empty()) return;
      done = true;
      try {
        std::vector<rclcpp::Parameter> back;
        for (const auto &kv : *vals) back.push_back(kv.second);
        self->node_->set_parameters(back);
        self->pm_->initOptimizer(/*force_reinit=*/true);
        self->pm_->deliverTrajToOptimizer();
        self->log_->infof("[CHAIN] mission-wide parameters restored "
                          "(%zu overridden)", vals->size());
      } catch (const std::exception &e) {
        self->log_->errorf("[CHAIN] parameter restore FAILED: %s — the next "
                           "plan may run with segment overrides!", e.what());
      }
    }
    ~ParamRestore() { restore(); }
  } restore_guard{any_over ? this : nullptr, &pristine};

  // === Chained segment runs (sequential; each is a full pipeline run) ===
  std::vector<poly_traj::Trajectory> runs;
  for (int i = 0; i < segments_; ++i) {
    const bool last = (i + 1 == segments_);
    if (any_over) {
      // Pristine + this segment's overrides — a segment never inherits a
      // neighbour's values. The forced re-init re-reads the whole optimizer
      // parameter surface (setParam is re-entrant for exactly this).
      std::map<std::string, rclcpp::Parameter> eff = pristine;
      for (const auto &p : seg_over[i].params) eff[p.get_name()] = p;
      std::vector<rclcpp::Parameter> apply;
      apply.reserve(eff.size());
      for (const auto &kv : eff) apply.push_back(kv.second);
      node_->set_parameters(apply);
      pm_->initOptimizer(/*force_reinit=*/true);
      pm_->deliverTrajToOptimizer();
      if (!seg_over[i].label.empty())
        log_->infof("[CHAIN] segment %d/%d requirement overrides: %s",
                    i + 1, segments_, seg_over[i].label.c_str());
    }
    const Eigen::Vector3d head_pos = (i == 0) ? start_pos : contracts[i - 1].pos;
    const Eigen::Vector3d head_vel = (i == 0) ? start_vel : contracts[i - 1].vel;
    const Eigen::Vector3d head_acc = (i == 0) ? start_acc : contracts[i - 1].acc;
    // The last span flies to the REAL goal: the original waypoint (AGL
    // semantics) under the arrival contract, exactly as the single-shot
    // plan would end.
    const std::vector<Eigen::Vector3d> goal =
        last ? waypoints : std::vector<Eigen::Vector3d>{contracts[i].pos};
    Eigen::Vector3d end_vel =
        last ? Eigen::Vector3d::Zero() : contracts[i].vel;
    Eigen::Vector3d end_acc =
        last ? Eigen::Vector3d::Zero() : contracts[i].acc;
    // A slow LAST segment cannot take the default arrival contract — that
    // would pin its tail at the mission-wide cruise speed, above its own
    // ceiling. Prescribe the same level entry (the baseline's own arrival
    // direction) at the segment's ceiling instead.
    if (last && seg_vmax[static_cast<size_t>(i)] < vmax_global - 1e-12) {
      const Eigen::Vector3d base_tail =
          baseline.getJuncVel(baseline.getPieceNum());
      if (base_tail.norm() > seg_vmax[static_cast<size_t>(i)] + 1e-9) {
        Eigen::Vector3d dir = base_tail;
        dir.z() = 0.0;  // level, like the arrival contract (no-op: the
                        // baseline tail is already level by that contract)
        if (dir.norm() > 1e-9) {
          end_vel = dir.normalized() * seg_vmax[static_cast<size_t>(i)];
          end_acc = Eigen::Vector3d::Zero();
          log_->infof("[CHAIN] arrival contract at the segment ceiling: "
                      "|v|=%.3f u/s level along the baseline approach",
                      end_vel.norm());
        }
      }
    }

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
      restore_guard.restore();  // judge the flight under mission-wide params
      logFinalEvaluation(baseline, {baseline.getTotalDuration()},
                         {"baseline"});
      return true;
    }
    runs.push_back(pm_->traj_.local_traj.traj);
  }

  // === Stitch and store ===
  poly_traj::Trajectory chained = runs.front();
  for (size_t i = 1; i < runs.size(); ++i) chained.append(runs[i]);

  // All segments planned — restore mission-wide parameters NOW, so the
  // terminal build and the final evaluation run under the mission's own
  // model, not the last segment's overrides.
  restore_guard.restore();

  // Report on the CHAIN portion (baseline deviation stays apples-to-apples),
  // then append the optional prescribed terminal phase before storage.
  logChainReport(baseline, runs, contracts, chained, seg_over);
  std::vector<double> phase_ends;
  std::vector<std::string> phase_names;
  double acc_t = 0.0;
  for (size_t i = 0; i < runs.size(); ++i) {
    acc_t += runs[i].getTotalDuration();
    phase_ends.push_back(acc_t);
    phase_names.push_back("seg" + std::to_string(i + 1));
  }
  const double t_pre_terminal = chained.getTotalDuration();
  appendTerminalPhase(&chained);
  if (chained.getTotalDuration() > t_pre_terminal + 1e-9) {
    phase_ends.push_back(chained.getTotalDuration());
    phase_names.push_back("terminal");
  }

  const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  // GLOBAL slot = the baseline OPTIMIZED trajectory (comparison reference,
  // see the class comment). Must precede setLocalTraj: setGlobalTraj resets
  // the local slot's bookkeeping.
  pm_->traj_.setGlobalTraj(baseline, now_s);
  pm_->traj_.setLocalTraj(chained, now_s, pm_->traj_.local_traj.drone_id);
  pm_->publishTrajectoryViz(chained, baseline);

  logFinalEvaluation(chained, phase_ends, phase_names);

  const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_wall).count();
  log_->infof("[CHAIN] chained %d segments: %d pieces, %.1f s flight "
              "(baseline %.1f s), planned in %.1f ms wall",
              segments_, chained.getPieceNum(), chained.getTotalDuration(), T,
              wall_ms);
  return true;
}

bool SegmentChainPlanner::authorContractsFromRoute(
    const std::vector<Eigen::Vector3d> &route,
    std::vector<Contract> *contracts) const
{
  const int M = static_cast<int>(route.size());
  if (M < segments_ + 1 || !contracts) return false;
  std::vector<double> s(static_cast<size_t>(M), 0.0);
  for (int i = 1; i < M; ++i)
    s[i] = s[i - 1] + (route[i] - route[i - 1]).norm();
  const double S = s.back();
  if (S < 1e-6) return false;
  const double cruise = pm_->maxVel();

  // [DIFF-BAL] Segment SOLVE cost is driven by what the optimizer must
  // fight — risk-zone stretches dominate the per-iteration cost and
  // terrain-following arms the terrain term — not by arc length: measured
  // on r4, arc-balanced tenths still spread 3 ms .. 2.2 s and the slowest
  // segment IS the parallel wall. Weight each interval by a difficulty
  // density and place junctions at equal cumulative WEIGHT; selection
  // (zone-clear, calm, grade) is unchanged, only the balance space moves.
  // Densities are coarse deliberately (3x zone, up to 3x terrain-following
  // over the [ROUGH] 1.0..2.5 u AGL fade) — balance needs ranking, not
  // prediction. DEFAULT OFF: measured on r4, warping the windows into
  // weight-space compresses them over easy stretches, starves the calm
  // candidate pool and pushed a junction onto a bendier vertex — segment 2
  // envelope-rejected at 28.7% and the whole plan failed. Arc balance at
  // fine N already delivers (N=10: 4.35x, +0.1% flight); revisit together
  // with a junction-shift retry.
  bool diff_balance = false;
  if (!node_->has_parameter("chain/difficulty_balance"))
    node_->declare_parameter("chain/difficulty_balance", false);
  node_->get_parameter("chain/difficulty_balance", diff_balance);
  std::vector<double> w(static_cast<size_t>(M), 0.0);
  for (int i = 1; i < M; ++i) {
    const double len = (route[i] - route[i - 1]).norm();
    double dens = 1.0;
    if (diff_balance) {
      const Eigen::Vector3d mid = 0.5 * (route[i] + route[i - 1]);
      if (nearRiskZone(mid)) dens += 3.0;
      double g = 0.0;
      if (pm_->terrainElevation(mid.x(), mid.y(), &g)) {
        const double agl = mid.z() - g;
        if (agl < 2.5)
          dens += 3.0 * std::min(1.0, std::max(0.0, (2.5 - agl) / 1.5));
      }
    }
    w[static_cast<size_t>(i)] = w[static_cast<size_t>(i - 1)] + len * dens;
  }
  const double W = w.back();
  const double span_w = W / segments_;

  const double span = S / segments_;

  // Sustainable grade at cruise: the THRUST-LIMITED climb, not just the
  // fpa cap. A junction is pinned at full cruise with a = 0, so the solver
  // has no energy trade there — a uniformly steep leg has kappa == 0 and
  // would otherwise score as the calmest possible candidate while pinning
  // an unsustainable climb as a hard BC (review find).
  double tan_grade_max = std::numeric_limits<double>::infinity();
  if (const auto *dyn = pm_->dynamicsParams()) {
    double um = 100.0;
    if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
      um = node_->get_parameter("optimization/dynamics_unit_xy_m")
               .as_double();
    const double v = cruise * um;
    const double q =
        0.5 * mmp_vehicle_dynamics::airDensity(*dyn, 0.0) * v * v;
    const double W = dyn->mass_kg * dyn->gravity_mps2;
    const double CL = W / std::max(1e-9, q * dyn->wing_area_m2);
    const double D = (dyn->zero_lift_drag_coefficient +
                      dyn->induced_drag_factor * CL * CL) *
                     q * dyn->wing_area_m2;
    const double sg = std::max(
        0.0, std::min(1.0, (dyn->thrust_max_n * (1.0 - dyn->constraint_margin) - D) / W));
    tan_grade_max = std::min(std::tan(dyn->flight_path_angle_max_rad),
                             std::tan(std::asin(sg)));
  }

  // Discrete 3D curvature per interior vertex: turn angle over the mean
  // chord — the a-priori stand-in for the baseline's |a| (= v^2 * kappa).
  std::vector<double> kappa(static_cast<size_t>(M), 0.0);
  for (int i = 1; i + 1 < M; ++i) {
    const Eigen::Vector3d a = route[i] - route[i - 1];
    const Eigen::Vector3d b = route[i + 1] - route[i];
    const double la = a.norm(), lb = b.norm();
    if (la < 1e-9 || lb < 1e-9) continue;
    const double c =
        std::max(-1.0, std::min(1.0, a.dot(b) / (la * lb)));
    kappa[static_cast<size_t>(i)] = std::acos(c) / (0.5 * (la + lb));
  }

  contracts->clear();
  double w_prev = 0.0;
  for (int j = 1; j < segments_; ++j) {
    const double target = j * span_w;
    const double lo = w_prev + 0.2 * span_w;
    const double hi = target + 0.4 * span_w;
    int best = -1, best_any = -1;
    double best_score = std::numeric_limits<double>::infinity();
    double best_any_score = best_score;
    for (int i = 1; i + 1 < M; ++i) {
      if (w[static_cast<size_t>(i)] < lo || w[static_cast<size_t>(i)] > hi)
        continue;
      // Calmness dominates, balance breaks ties: kappa is rad/u (a gentle
      // route sits at ~0.001-0.01, a hard corner at 0.05+), the balance
      // term at most 0.4 — the 100x scale makes a real bend outweigh any
      // imbalance while flat stretches sort by balance. Grade beyond the
      // sustainable cone is penalized on the same scale as a hard bend.
      const double hxy_i = (route[i + 1] - route[i - 1]).head<2>().norm();
      const double grade_i =
          hxy_i > 1e-9
              ? std::abs(route[i + 1].z() - route[i - 1].z()) / hxy_i
              : 1e3;
      const double score =
          kappa[static_cast<size_t>(i)] * 100.0 +
          std::abs(w[static_cast<size_t>(i)] - target) / span_w +
          std::max(0.0, grade_i - tan_grade_max) * 100.0;
      if (score < best_any_score) { best_any = i; best_any_score = score; }
      if (nearRiskZone(route[i])) continue;
      if (score < best_score) { best = i; best_score = score; }
    }
    if (best < 0 && best_any >= 0) {
      log_->warnf("[CHAIN-PAR] junction %d: window blanketed by zones — "
                  "calmest candidate regardless (risk comparison "
                  "contaminated there)", j);
      best = best_any;
    }
    if (best < 0) {
      // No vertex fell inside the arc window at all (very sparse route) —
      // a different failure than zone blanket; the caller falls back.
      log_->warnf("[CHAIN-PAR] junction %d: no route vertex in the arc "
                  "window — route too sparse for %d segments",
                  j, segments_);
      return false;
    }
    Contract c;
    c.t = s[best] / std::max(1e-9, cruise);
    c.pos = route[best];
    Eigen::Vector3d dir = route[best + 1] - route[best - 1];
    if (dir.norm() < 1e-9) dir = Eigen::Vector3d::UnitX();
    const double hxy = dir.head<2>().norm();
    if (hxy > 1e-9 && std::abs(dir.z()) > tan_grade_max * hxy) {
      log_->warnf("[CHAIN-PAR] contract %d grade %.3f > sustainable %.3f — "
                  "tangent sheared onto the climb cone (a hard BC at "
                  "cruise with a=0 has no energy trade)",
                  j, std::abs(dir.z()) / hxy, tan_grade_max);
      dir.z() = std::copysign(tan_grade_max * hxy, dir.z());
    } else if (hxy <= 1e-9) {
      // Near-vertical chord: enter level — the same intent as the arrival
      // contract's degenerate walk-back.
      dir = Eigen::Vector3d::UnitX();
    }
    c.vel = dir.normalized() * cruise;
    c.acc = Eigen::Vector3d::Zero();
    log_->infof("[CHAIN-PAR] contract %d @s=%.1f u (w-share %.2f, target "
                "%.2f): pos=(%.2f, %.2f, %.2f) |v|=%.3f kappa=%.4f rad/u",
                j, s[best], w[static_cast<size_t>(best)] / span_w,
                target / span_w, c.pos.x(), c.pos.y(), c.pos.z(),
                c.vel.norm(), kappa[static_cast<size_t>(best)]);
    contracts->push_back(c);
    w_prev = w[static_cast<size_t>(best)];
  }
  return true;
}

bool SegmentChainPlanner::planRouteParallel(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc,
    const std::vector<Eigen::Vector3d> &waypoints,
    bool start_vel_synthesized, bool run_parallel)
{
  const auto t_wall = std::chrono::steady_clock::now();
  const auto ms_since = [](const std::chrono::steady_clock::time_point &t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  };

  // Single-shot fallback for anything the route mode cannot author.
  const auto fallback = [&](const char *why) {
    log_->warnf("[CHAIN-PAR] %s — falling back to the single-shot plan", why);
    pm_->setStartVelSynthesized(start_vel_synthesized);
    return pm_->planGlobalTraj(start_pos, start_vel, start_acc, waypoints,
                               Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero());
  };

  // === 1. front-end only: commit the route (AGL/bbox/SDF/zone binding
  // happen here exactly once) ===
  log_->infof("[CHAIN-PAR] route-%s plan, %d segments (no baseline)",
              run_parallel ? "parallel" : "sequential", segments_);
  pm_->setStartVelSynthesized(false);
  if (!pm_->planGlobalTraj(start_pos, start_vel, start_acc, waypoints,
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           false, false, nullptr, nullptr,
                           /*front_end_only=*/true)) {
    log_->errorf("[CHAIN-PAR] front-end failed — nothing to author on");
    return false;
  }
  const std::vector<Eigen::Vector3d> route = pm_->lastCommittedRoute();
  const std::vector<double> cap = pm_->lastCommittedCapRef();
  const double fe_ms = ms_since(t_wall);

  // === 2. author contracts + slices on the route ===
  std::vector<Contract> contracts;
  if (!authorContractsFromRoute(route, &contracts))
    return fallback("route too small to author junctions on");
  std::vector<RouteSlice> slices =
      sliceCommittedRoute(route, cap, contracts);
  if (slices.size() != static_cast<size_t>(segments_))
    return fallback("committed route did not slice cleanly");

  // Segment 1's head is the MISSION start: replicate [VEL-ALIGN] (the
  // route is known here) and the [STALL-FLOOR] the bypassed planGlobalTraj
  // path would have applied.
  Eigen::Vector3d v0 = start_vel;
  if (pm_->alignStartVelToRoute() && start_vel_synthesized &&
      route.size() >= 2) {
    Eigen::Vector3d dir = route[1] - route[0];
    dir.z() = 0.0;
    if (dir.head<2>().norm() > 1e-9)
      v0 = dir.normalized() * start_vel.norm();
  }
  if (const auto *dyn = pm_->dynamicsParams()) {
    double um = 100.0;
    if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
      um = node_->get_parameter("optimization/dynamics_unit_xy_m")
               .as_double();
    const double floor =
        dyn->speed_min_mps * (1.0 + dyn->constraint_margin) / um;
    if (floor > 0.0 && v0.norm() < floor) {
      Eigen::Vector3d dir = v0;
      if (dir.norm() < 1e-9 && route.size() >= 2) {
        dir = route[1] - route[0];
        dir.z() = 0.0;
      }
      if (dir.norm() < 1e-9) dir = Eigen::Vector3d::UnitX();
      log_->warnf("[CHAIN-PAR] start speed %.3f below stall floor %.3f — "
                  "raised (same contract as [STALL-FLOOR])",
                  v0.norm(), floor);
      v0 = dir.normalized() * floor;
    }
  }
  const double author_ms = ms_since(t_wall) - fe_ms;

  // === 3. per-worker optimizer instances (serial: setParam snapshots
  // node params; inner OpenMP is SHARED as cores/segments per worker —
  // see the note at the pin below) ===
  rclcpp::Parameter saved_rpt;
  bool had_rpt = node_->has_parameter("optimization/risk_parallel_threads");
  if (had_rpt)
    saved_rpt = node_->get_parameter("optimization/risk_parallel_threads");
  if (run_parallel) {
    // The solver is ALREADY internally parallel (OpenMP over the risk
    // term), so segment threads mostly REDISTRIBUTE cores rather than add
    // compute. Pinning workers to 1 inner thread measured SLOWER than
    // sequential (each solve lost its 32 cores); share the machine
    // instead: cores / segments inner threads per worker.
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const int per =
        std::max(1, static_cast<int>(hw) / std::max(1, segments_));
    log_->infof("[CHAIN-PAR] %u cores / %d workers -> %d inner threads "
                "each", hw, segments_, per);
    node_->set_parameters(
        {rclcpp::Parameter("optimization/risk_parallel_threads", per)});
  }
  std::vector<std::unique_ptr<ego_planner::PolyTrajOptimizer>> opts;
  for (int i = 0; i < segments_; ++i)
    opts.push_back(pm_->makeConfiguredOptimizer());
  if (run_parallel && had_rpt) node_->set_parameters({saved_rpt});
  const bool sup = pm_->zoneAvoidPassNow() == 1;

  // === 4. solve — the same worker body, threaded or looped ===
  std::vector<poly_traj::Trajectory> runs(static_cast<size_t>(segments_));
  std::vector<char> ok(static_cast<size_t>(segments_), 0);
  std::vector<double> solve_ms(static_cast<size_t>(segments_), 0.0);
  const auto worker = [&](int i) {
    const auto t0 = std::chrono::steady_clock::now();
    const bool last = (i + 1 == segments_);
    const size_t ui = static_cast<size_t>(i);
    const Eigen::Vector3d hp = i ? contracts[ui - 1].pos : start_pos;
    const Eigen::Vector3d hv = i ? contracts[ui - 1].vel : v0;
    const Eigen::Vector3d ha = i ? contracts[ui - 1].acc : start_acc;
    const Eigen::Vector3d gp = slices[ui].path.back();
    const Eigen::Vector3d ev =
        last ? Eigen::Vector3d::Zero() : contracts[ui].vel;
    const Eigen::Vector3d ea =
        last ? Eigen::Vector3d::Zero() : contracts[ui].acc;
    ok[ui] = pm_->solveSlice(*opts[ui], slices[ui].path, slices[ui].cap,
                             hp, hv, ha, gp, ev, ea, sup, &runs[ui])
                 ? 1
                 : 0;
    solve_ms[ui] = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
  };
  const auto t_solve = std::chrono::steady_clock::now();
  if (run_parallel) {
    std::vector<std::thread> workers;
    workers.reserve(static_cast<size_t>(segments_));
    for (int i = 0; i < segments_; ++i) workers.emplace_back(worker, i);
    for (auto &w : workers) w.join();
  } else {
    for (int i = 0; i < segments_; ++i) worker(i);
  }
  const double solve_wall_ms = ms_since(t_solve);

  for (int i = 0; i < segments_; ++i) {
    if (!ok[static_cast<size_t>(i)]) {
      // No baseline exists in this mode — degrade to the single-shot plan
      // instead of rejecting the mission: a marginal per-segment gate trip
      // (measured: 28.7% vs the 25% envelope gate) must cost speed, not
      // the flight.
      char why[96];
      snprintf(why, sizeof why, "segment %d/%d failed", i + 1, segments_);
      return fallback(why);
    }
  }

  // === 5. stitch, report, terminal, store, evaluate ===
  poly_traj::Trajectory chained = runs.front();
  for (size_t i = 1; i < runs.size(); ++i) chained.append(runs[i]);

  const poly_traj::Trajectory no_baseline;  // report skips the comparison
  logChainReport(no_baseline, runs, contracts, chained,
                 std::vector<SegmentOverrides>(
                     static_cast<size_t>(segments_)));

  std::vector<double> phase_ends;
  std::vector<std::string> phase_names;
  double acc_t = 0.0;
  for (size_t i = 0; i < runs.size(); ++i) {
    acc_t += runs[i].getTotalDuration();
    phase_ends.push_back(acc_t);
    phase_names.push_back("seg" + std::to_string(i + 1));
  }
  const double t_pre_terminal = chained.getTotalDuration();
  appendTerminalPhase(&chained);
  if (chained.getTotalDuration() > t_pre_terminal + 1e-9) {
    phase_ends.push_back(chained.getTotalDuration());
    phase_names.push_back("terminal");
  }

  const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  // No baseline: both slots carry the chained product (the comparison
  // channel simply mirrors the flight in this mode).
  pm_->traj_.setGlobalTraj(chained, now_s);
  pm_->traj_.setLocalTraj(chained, now_s, pm_->traj_.local_traj.drone_id);
  pm_->publishTrajectoryViz(chained, chained);
  logFinalEvaluation(chained, phase_ends, phase_names);

  std::string per;
  for (size_t i = 0; i < solve_ms.size(); ++i) {
    char b[32];
    snprintf(b, sizeof b, "%s%.0f", i ? "/" : "", solve_ms[i]);
    per += b;
  }
  log_->infof("[CHAIN-PAR] %s: front-end %.0f ms + author %.0f ms + solves "
              "[%s] ms (wall %.0f, max %.0f) => TOTAL %.0f ms | %d pieces, "
              "%.1f s flight",
              run_parallel ? "PARALLEL" : "sequential", fe_ms, author_ms,
              per.c_str(), solve_wall_ms,
              *std::max_element(solve_ms.begin(), solve_ms.end()),
              ms_since(t_wall), chained.getPieceNum(),
              chained.getTotalDuration());
  return true;
}

void SegmentChainPlanner::logFinalEvaluation(
    const poly_traj::Trajectory &flight,
    const std::vector<double> &phase_ends,
    const std::vector<std::string> &phase_names) const
{
  const double T = flight.getTotalDuration();
  if (T <= 1e-9 || phase_ends.empty() ||
      phase_ends.size() != phase_names.size())
    return;

  const mmp_vehicle_dynamics::Parameters *dyn = pm_->dynamicsParams();
  const auto param_or = [&](const char *n, double def) {
    return node_->has_parameter(n) ? node_->get_parameter(n).as_double()
                                   : def;
  };
  const double um_xy = param_or("optimization/dynamics_unit_xy_m", 100.0);
  const double um_z = param_or("optimization/dynamics_unit_z_m", 100.0);
  const double clr_band = param_or("optimization/obstacle_clearance", 0.3);
  const Eigen::Vector3d S(um_xy, um_xy, um_z);

  struct PhaseStat {
    double min_agl{std::numeric_limits<double>::infinity()};
    double min_agl_t{0.0};
    double agl_sum{0.0};
    int n{0}, n_below_band{0}, n_below_ground{0};
    int env_n{0}, env_viol{0};
    double util_peak{0.0};
    mmp_vehicle_dynamics::EnvelopeLimit peak_limit{
        mmp_vehicle_dynamics::EnvelopeLimit::None};
    double risk_max{0.0}, risk_int{0.0};
  };
  std::vector<PhaseStat> st(phase_ends.size());

  const double dt = 0.1;
  const size_t nz = pm_->numRiskZones();
  // Same counting doctrine as the solver's [CONV-REJECT] audit, latch
  // included: the pre-cruise ramp (a rest-start mission legitimately
  // begins below stall) stays out of the statistics, but sub-min-speed
  // dwell AFTER cruise entry counts into BOTH counters — dropping it from
  // both made the evaluator blind to mid-flight speed collapse, the exact
  // ~0%-scored failure the solver's latch was built to kill (review find:
  // a sub-stall arrival contract fed a 300 s helix that scored "0.0%").
  bool reached_cruise = false;
  for (double t = 0.0; t < T; t += dt) {
    size_t ph = 0;
    while (ph + 1 < phase_ends.size() && t >= phase_ends[ph]) ++ph;
    PhaseStat &s = st[ph];
    const Eigen::Vector3d p = flight.getPos(t);
    double g = 0.0;
    pm_->terrainElevation(p.x(), p.y(), &g);  // false: sea level 0
    const double agl = p.z() - g;
    if (agl < s.min_agl) { s.min_agl = agl; s.min_agl_t = t; }
    s.agl_sum += agl;
    ++s.n;
    if (agl < clr_band) ++s.n_below_band;
    if (agl < 0.0) ++s.n_below_ground;
    if (dyn) {
      const auto ev = mmp_vehicle_dynamics::evaluateInverseDynamics(
          *dyn, S.cwiseProduct(p), S.cwiseProduct(flight.getVel(t)),
          S.cwiseProduct(flight.getAcc(t)));
      if (ev.valid && ev.speed_mps >= dyn->speed_min_mps) {
        reached_cruise = true;
        ++s.env_n;
        mmp_vehicle_dynamics::EnvelopeLimit lim;
        const double u =
            mmp_vehicle_dynamics::envelopeUtilization(*dyn, ev, &lim);
        if (!mmp_vehicle_dynamics::isWithinEnvelope(*dyn, ev)) ++s.env_viol;
        if (u > s.util_peak) { s.util_peak = u; s.peak_limit = lim; }
      } else if (reached_cruise) {
        ++s.env_n;
        ++s.env_viol;
      }
    }
    if (nz > 0) {
      double keep = 1.0;
      for (size_t zi = 0; zi < nz; ++zi)
        keep *= 1.0 - pm_->getEffectiveRisk(zi, p);
      const double r = 1.0 - keep;
      s.risk_max = std::max(s.risk_max, r);
      s.risk_int += r * dt;
    }
  }

  log_->infof("[FINAL-EVAL] ===== whole-flight evaluation: %.1f s, %d "
              "pieces, %zu phase(s) =====",
              T, flight.getPieceNum(), phase_ends.size());
  double t0 = 0.0;
  PhaseStat tot;
  for (size_t ph = 0; ph < phase_ends.size(); ++ph) {
    const PhaseStat &s = st[ph];
    if (s.n > 0) {
      // A phase with dynamics on but ZERO cruise-domain samples has no
      // measurement — "0.0%" would print no-data as a clean reading.
      char env[96];
      if (!dyn) {
        snprintf(env, sizeof env, "env model off");
      } else if (s.env_n == 0) {
        snprintf(env, sizeof env, "env NO CRUISE DATA (pre-cruise ramp)");
      } else {
        snprintf(env, sizeof env, "env viol %5.1f%% peak %5.1f%% (%s)",
                 100.0 * s.env_viol / s.env_n, 100.0 * s.util_peak,
                 mmp_vehicle_dynamics::envelopeLimitName(s.peak_limit));
      }
      log_->infof(
          "[FINAL-EVAL] %-9s %6.1f s | AGL min %7.3f@%.0fs mean %6.2f u, "
          "<band %4.1f%%, underground %s | %s | risk max %.3f, "
          "exposure %.1f s",
          phase_names[ph].c_str(), phase_ends[ph] - t0, s.min_agl,
          s.min_agl_t, s.agl_sum / s.n, 100.0 * s.n_below_band / s.n,
          s.n_below_ground ? "YES" : "no", env, s.risk_max, s.risk_int);
    }
    t0 = phase_ends[ph];
    if (s.min_agl < tot.min_agl) {
      tot.min_agl = s.min_agl;
      tot.min_agl_t = s.min_agl_t;
    }
    tot.n += s.n;
    tot.n_below_band += s.n_below_band;
    tot.n_below_ground += s.n_below_ground;
    tot.env_n += s.env_n;
    tot.env_viol += s.env_viol;
    tot.risk_max = std::max(tot.risk_max, s.risk_max);
    tot.risk_int += s.risk_int;
    if (s.util_peak > tot.util_peak) {
      tot.util_peak = s.util_peak;
      tot.peak_limit = s.peak_limit;
    }
  }
  const double viol_pct =
      tot.env_n ? 100.0 * tot.env_viol / tot.env_n : 0.0;
  // A flight that never once reaches cruise is unflyable for this airframe
  // — the solver's own degenerate rule (viol := 1.0), mirrored here so
  // no-data can never read as clean.
  const bool no_cruise = dyn != nullptr && tot.env_n == 0;
  const bool clean =
      tot.n_below_ground == 0 && !no_cruise && viol_pct < 25.0;
  if (clean) {
    log_->infof("[FINAL-EVAL] verdict: CLEAN — AGL min %.3f u, env viol "
                "%.1f%% (peak %.1f%% %s), risk exposure %.1f s",
                tot.min_agl, viol_pct, 100.0 * tot.util_peak,
                mmp_vehicle_dynamics::envelopeLimitName(tot.peak_limit),
                tot.risk_int);
  } else {
    log_->warnf("[FINAL-EVAL] verdict: CHECK — %s%s%s(AGL min %.3f u "
                "@%.0fs, env viol %.1f%%) — the stitched flight carries "
                "hazards no per-solve audit saw",
                tot.n_below_ground ? "TERRAIN OVERLAP " : "",
                no_cruise ? "NEVER REACHES CRUISE " : "",
                !no_cruise && viol_pct >= 25.0 ? "ENVELOPE " : "",
                tot.min_agl, tot.min_agl_t, viol_pct);
  }
}

void SegmentChainPlanner::appendTerminalPhase(
    poly_traj::Trajectory *chained) const
{
  const auto dp = [&](const char *n, auto def) {
    if (!node_->has_parameter(n)) node_->declare_parameter(n, def);
  };
  dp("chain/terminal/enable", false);
  bool enable = false;
  node_->get_parameter("chain/terminal/enable", enable);
  if (!enable) return;

  TerminalHelixParams prm;
  dp("chain/terminal/radius", prm.radius);
  dp("chain/terminal/turns", prm.turns);
  dp("chain/terminal/final_agl", prm.final_agl);
  dp("chain/terminal/right", prm.right);
  node_->get_parameter("chain/terminal/radius", prm.radius);
  node_->get_parameter("chain/terminal/turns", prm.turns);
  node_->get_parameter("chain/terminal/final_agl", prm.final_agl);
  node_->get_parameter("chain/terminal/right", prm.right);

  const int N = chained->getPieceNum();
  const Eigen::Vector3d hp = chained->getJuncPos(N);
  const Eigen::Vector3d hv = chained->getJuncVel(N);
  const Eigen::Vector3d ha = chained->getJuncAcc(N);
  if (ha.norm() > 1e-6) {
    // The helix entry is built for the arrival contract's a = 0; a nonzero
    // handoff acceleration becomes the seam's |dA| verbatim.
    log_->warnf("[CHAIN] terminal handoff acceleration %.4f u/s^2 != 0 — "
                "the helix entry assumes the arrival contract; the seam "
                "will carry exactly this discontinuity", ha.norm());
  }

  double min_agl = 0.0;
  const poly_traj::Trajectory term = TerminalPhase::helixDescent(
      hp, hv, prm,
      [this](double x, double y, double *e) {
        if (pm_->terrainElevation(x, y, e)) return true;
        *e = 0.0;  // water / off-DEM: sea level
        return false;
      },
      &min_agl);
  if (term.getPieceNum() == 0) {
    log_->warnf("[CHAIN] terminal helix degenerate (|v|=%.3f, R=%.1f, "
                "turns=%.2f) — not appended", hv.norm(), prm.radius,
                prm.turns);
    return;
  }

  const double dP = (term.getJuncPos(0) - hp).norm();
  const double dV = (term.getJuncVel(0) - hv).norm();
  const double dA = (term.getJuncAcc(0) - ha).norm();
  const Eigen::Vector3d exit_p = term.getJuncPos(term.getPieceNum());
  double g = 0.0;
  pm_->terrainElevation(exit_p.x(), exit_p.y(), &g);
  log_->infof("[CHAIN-REPORT] terminal: helix R=%.1f u, %.2f turns %s, "
              "%d pieces, %.1f s | handoff seam |dP|=%.3e |dV|=%.3e "
              "|dA|=%.3e | exit AGL %.3f u, min AGL %.3f u",
              prm.radius, prm.turns, prm.right ? "right" : "left",
              term.getPieceNum(), term.getTotalDuration(), dP, dV, dA,
              exit_p.z() - g, min_agl);
  if (min_agl < 0.5 * prm.final_agl) {
    log_->warnf("[CHAIN] PRESCRIBED terminal geometry descends to %.3f u "
                "AGL — no optimizer and no collision audit protects this "
                "phase yet (stage 4); move the helix or shrink the turns",
                min_agl);
  }
  chained->append(term);
}

std::vector<SegmentChainPlanner::SegmentOverrides>
SegmentChainPlanner::readSegmentOverrides() const
{
  std::vector<SegmentOverrides> out(static_cast<size_t>(segments_));
  for (int i = 0; i < segments_; ++i) {
    const std::string pname = "chain/seg" + std::to_string(i + 1) + "/params";
    if (!node_->has_parameter(pname))
      node_->declare_parameter(pname, std::vector<std::string>{});
    std::vector<std::string> specs;
    node_->get_parameter(pname, specs);
    auto &so = out[static_cast<size_t>(i)];
    for (const std::string &spec : specs) {
      const size_t eq = spec.find('=');
      if (eq == std::string::npos || eq == 0 || eq + 1 >= spec.size()) {
        log_->warnf("[CHAIN] seg%d override '%s' is not name=value — skipped",
                    i + 1, spec.c_str());
        continue;
      }
      const std::string name = spec.substr(0, eq);
      const std::string val = spec.substr(eq + 1);
      if (!node_->has_parameter(name)) {
        log_->warnf("[CHAIN] seg%d override '%s': no such parameter — "
                    "skipped (typo, or its consumer never declared it)",
                    i + 1, name.c_str());
        continue;
      }
      if (name.rfind("optimization/", 0) != 0) {
        log_->warnf("[CHAIN] seg%d override '%s': only optimization/* is "
                    "re-read per segment (manager/FSM parameters load at "
                    "startup) — the value will be set but its consumer "
                    "will not see it this mission", i + 1, name.c_str());
      }
      try {
        switch (node_->get_parameter(name).get_type()) {
          case rclcpp::ParameterType::PARAMETER_DOUBLE: {
            // Full-token parse: stod/stoll silently accept trailing garbage
            // ("1.8x" -> 1.8), which would truncate a typo into a plausible
            // value while the report label testifies the typo was applied.
            size_t pos = 0;
            const double d = std::stod(val, &pos);
            if (pos != val.size())
              throw std::invalid_argument("trailing '" + val.substr(pos) + "'");
            so.params.emplace_back(name, d);
            break;
          }
          case rclcpp::ParameterType::PARAMETER_INTEGER: {
            size_t pos = 0;
            const long long v = std::stoll(val, &pos);
            if (pos != val.size())
              throw std::invalid_argument("trailing '" + val.substr(pos) + "'");
            so.params.emplace_back(name, static_cast<int64_t>(v));
            break;
          }
          case rclcpp::ParameterType::PARAMETER_BOOL: {
            // Strict spellings only: "True"/"yes"/"on" silently meaning
            // false is exactly the typo class the label would then lie about.
            if (val == "true" || val == "1")
              so.params.emplace_back(name, true);
            else if (val == "false" || val == "0")
              so.params.emplace_back(name, false);
            else
              throw std::invalid_argument("bool wants true/false/1/0");
            break;
          }
          case rclcpp::ParameterType::PARAMETER_STRING:
            so.params.emplace_back(name, val);
            break;
          default:
            log_->warnf("[CHAIN] seg%d override '%s': unsupported parameter "
                        "type — skipped", i + 1, name.c_str());
            continue;
        }
      } catch (const std::exception &e) {
        log_->warnf("[CHAIN] seg%d override '%s': value parse failed (%s) — "
                    "skipped", i + 1, spec.c_str(), e.what());
        continue;
      }
      so.label += (so.label.empty() ? "" : ", ") + spec;
    }
  }
  return out;
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
  // A junction is a handoff, and a handoff mid-maneuver pins a hard BC in
  // the field's fiercest gradient. r3 taught it inside a zone gauntlet
  // (|a| = 0.111 junction trapped the next solve: bank 55 deg, terrain
  // overlap; |a| = 0.030 six seconds later chained cleanly) — but r4
  // taught that calmness must be the RULE, not the zone-blanketed
  // fallback: a 337 km terrain-following baseline is maneuvering at the
  // equal-time nominal too (|a| = 0.082, 0.84 g), and the segment solve
  // diverged into a 500 s sub-stall iterate. So: among the window's
  // zone-clear candidates take the CALMEST baseline state; only a fully
  // blanketed window falls back to the calmest candidate regardless of
  // zones (and says so — the risk comparison is contaminated there).
  double clear_t = -1.0, clear_a = std::numeric_limits<double>::infinity();
  double any_t = std::max(t_nominal, lo);
  double any_a = traj.getAcc(any_t).norm();
  for (double step = 0.0; step <= 0.40 + 1e-9; step += 0.08) {
    for (const double sgn : {+1.0, -1.0}) {
      const double cand = t_nominal + sgn * step * span;
      if (cand < lo) continue;
      const double a = traj.getAcc(cand).norm();
      if (a < any_a) { any_t = cand; any_a = a; }
      if (!nearRiskZone(traj.getPos(cand)) && a < clear_a) {
        clear_t = cand;
        clear_a = a;
      }
      if (step == 0.0) break;  // nominal has no sign
    }
  }
  if (clear_t >= 0.0) {
    if (std::abs(clear_t - t_nominal) > 1e-9) {
      log_->infof("[CHAIN] junction @%.1f s moved to %.1f s (calmest "
                  "zone-clear state in the window, |a|=%.3f u/s^2)",
                  t_nominal, clear_t, clear_a);
    }
    return clear_t;
  }
  log_->warnf("[CHAIN] junction @%.1f s: window blanketed by zones — taking "
              "the calmest baseline state @%.1f s (|a|=%.3f u/s^2)",
              t_nominal, any_t, any_a);
  return any_t;
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
    const poly_traj::Trajectory &chained,
    const std::vector<SegmentOverrides> &overrides) const
{
  log_->infof("[CHAIN-REPORT] ===== split verification =====");

  // Per-run shape: piece counts and durations reveal a span whose sub-plan
  // diverged wildly from its share of the baseline (e.g. a junction that
  // forced a detour the unsplit optimum never took). Mean speed makes the
  // stage-2 requirement differentiation legible next to its override list.
  for (size_t i = 0; i < runs.size(); ++i) {
    const ArcTable at = buildArcTable(runs[i], 200);
    const double dur = runs[i].getTotalDuration();
    const std::string &lbl =
        (i < overrides.size()) ? overrides[i].label : std::string();
    log_->infof("[CHAIN-REPORT] run %zu/%zu: %d pieces, %.1f s, %.1f u arc, "
                "mean %.3f u/s%s%s",
                i + 1, runs.size(), runs[i].getPieceNum(), dur, at.total,
                dur > 1e-9 ? at.total / dur : 0.0,
                lbl.empty() ? "" : " | overrides: ",
                lbl.empty() ? "" : lbl.c_str());
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
  // [CHAIN-PAR] route mode has no baseline — the comparison is skipped.
  if (baseline.getPieceNum() == 0) {
    log_->infof("[CHAIN-REPORT] (route mode: no baseline to compare "
                "against)");
  } else {
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
  }

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
