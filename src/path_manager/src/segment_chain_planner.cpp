#include "path_manager/segment_chain_planner.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <thread>

#include "path_manager/terminal_phase.h"

namespace path_manager {

namespace {
// Type-tolerant runtime parameter read. Statically-typed declares throw at
// PLAN time when a CLI/YAML override carries the other numeric type
// ("-p x:=-10" is an int, "x: 1" for a bool...) — and a try/catch around
// the declare only converts the crash into silently DISCARDING the
// override (the throw happens before registration; review find). Declaring
// with dynamic_typing accepts any override type, and the switch coerces
// bool/int/double, so operator-typed values can neither kill the node nor
// vanish.
double readNumParam(const rclcpp::Node::SharedPtr &node, const char *name,
                    double def)
{
  try {
    if (!node->has_parameter(name)) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.dynamic_typing = true;
      node->declare_parameter(name, rclcpp::ParameterValue(def), d);
    }
    const rclcpp::Parameter p = node->get_parameter(name);
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE: return p.as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<double>(p.as_int());
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return p.as_bool() ? 1.0 : 0.0;
      default: return def;
    }
  } catch (const std::exception &) {
    return def;
  }
}

// [PHASE] Optimizer-owned override whitelist (frozen v4), TAGGED: soft
// weights may be dropped with a degrade note; hard-max entries are
// REQUIREMENTS — merged conservatively (min) or preserved globally on
// fallback, never silently lost. PathManager-shared keys (obstacle
// clearance, altitude weight/headrooms) are deliberately absent: they
// configure shared machinery and cannot differ per parallel worker.
enum class OvrTag { kSoft, kHardMax };
struct OvrKey { const char *name; OvrTag tag; };
constexpr OvrKey kOverrideWhitelist[] = {
    {"optimization/weight_time", OvrTag::kSoft},
    {"optimization/weight_Risk", OvrTag::kSoft},
    {"optimization/max_vel", OvrTag::kHardMax},
};
const OvrKey *whitelistFind(const std::string &n)
{
  for (const auto &k : kOverrideWhitelist)
    if (n == k.name) return &k;
  return nullptr;
}

// [PHASE] Mission-wide parameter guard for the WORKER CONSTRUCTION window
// (frozen v4: two roles, split). applyEffective() = per-worker config:
// reset to pristine, then set this worker's list — the freshly built
// optimizer snapshots it. restoreNow()/dtor = the exception-safe guarantee
// that mission-wide values are back before the threads fan out (and on any
// throw in between). File-scope on purpose: plan()'s ParamRestore is a
// function-local one-shot and cannot be reused here (review find).
class ScopedMissionParams {
 public:
  ScopedMissionParams(const rclcpp::Node::SharedPtr &node,
                      swarm_formation::LogManager *log)
      : node_(node), log_(log) {}
  void applyEffective(const std::vector<rclcpp::Parameter> &ps)
  {
    resetPristine();
    for (const auto &p : ps)
      if (!saved_.count(p.get_name()) && node_->has_parameter(p.get_name()))
        saved_.emplace(p.get_name(), node_->get_parameter(p.get_name()));
    if (!ps.empty()) node_->set_parameters(ps);
  }
  void resetPristine()
  {
    if (saved_.empty()) return;
    std::vector<rclcpp::Parameter> back;
    back.reserve(saved_.size());
    for (const auto &kv : saved_) back.push_back(kv.second);
    node_->set_parameters(back);
  }
  void restoreNow()
  {
    if (done_) return;
    done_ = true;
    try {
      resetPristine();
    } catch (const std::exception &e) {
      if (log_)
        log_->errorf("[PHASE] mission-wide parameter restore FAILED: %s",
                     e.what());
    }
  }
  ~ScopedMissionParams() { restoreNow(); }

 private:
  rclcpp::Node::SharedPtr node_;
  swarm_formation::LogManager *log_;
  std::map<std::string, rclcpp::Parameter> saved_;
  bool done_{false};
};
}  // namespace

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

PlanResult SegmentChainPlanner::plan(const Eigen::Vector3d &start_pos,
                                     const Eigen::Vector3d &start_vel,
                                     const Eigen::Vector3d &start_acc,
                                     const std::vector<Eigen::Vector3d> &waypoints,
                                     bool start_vel_synthesized,
                                     const ego_planner::TailBoundary &mission_tail)
{
  // [PHASE] Final-boundary validation happens ONCE, here — every exit of
  // planImpl (workers, merge retry, fallbacks, baseline restore) receives
  // the validated tail, so an unmet stated requirement can never leave as
  // a plain success. Frozen policy: invalid + no opt-in -> FAILED; invalid
  // + planning/allow_final_boundary_relaxation -> plan without the tail and
  // return DEGRADED(FINAL_BOUNDARY_RELAXED).
  ego_planner::TailBoundary eff = mission_tail;
  bool relaxed = false;
  std::string relax_why;
  if (mission_tail.anyPrescribed()) {
    std::string problem;
    if (!mission_tail.allFinite()) {
      problem = "non-finite final boundary";
    } else if (mission_tail.prescribe_vel) {
      const double floor_u = pm_->stallFloorUnits();
      if (mission_tail.vel.norm() < floor_u)
        problem = "final speed " + std::to_string(mission_tail.vel.norm()) +
                  " u/s below the stall floor " + std::to_string(floor_u) +
                  " u/s (a fixed-wing terminal state below stall is outside "
                  "the model, like the launch phase)";
    }
    if (!problem.empty()) {
      const bool allow =
          readNumParam(node_, "planning/allow_final_boundary_relaxation",
                       0.0) != 0.0;
      if (!allow) {
        log_->errorf("[PHASE] final boundary REJECTED: %s — set "
                     "planning/allow_final_boundary_relaxation to fly "
                     "without it", problem.c_str());
        return PlanResult::failed(problem);
      }
      log_->warnf("[PHASE] final boundary relaxed (opt-in): %s",
                  problem.c_str());
      eff = ego_planner::TailBoundary{};
      relaxed = true;
      relax_why = problem + " — final boundary relaxed (opt-in)";
    }
  }
  PlanResult r = planImpl(start_pos, start_vel, start_acc, waypoints,
                          start_vel_synthesized, eff);
  if (relaxed) r.degrade(PlanReason::FINAL_BOUNDARY_RELAXED, relax_why);
  return r;
}

PlanResult SegmentChainPlanner::planImpl(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &start_acc,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               bool start_vel_synthesized,
                               const ego_planner::TailBoundary &mission_tail)
{
  // Stage-1 scope: one goal. Multi-waypoint missions need a waypoint-to-span
  // assignment that does not exist yet — fall back to the single-shot plan.
  if (waypoints.size() != 1) {
    log_->warnf("[CHAIN] %zu waypoints — stage 1 chains single-goal missions "
                "only, falling back to the single-shot plan", waypoints.size());
    pm_->setStartVelSynthesized(start_vel_synthesized);
    if (!pm_->planGlobalTraj(start_pos, start_vel, start_acc, waypoints,
                             mission_tail))
      return PlanResult::failed("multi-waypoint single-shot plan failed");
    PlanResult r = PlanResult::success();
    r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
              "multi-waypoint mission — chain not attempted");
    return r;
  }

  // [AUTO-N] chain/segments == 0 sizes the split from the mission itself
  // once its piece count is known (route mode: after the front-end;
  // baseline mode: after the baseline solve): N = round(pieces / target),
  // target = chain/auto_pieces_per_segment (default 70 — the 105-run
  // 6-point sweep's operating point: same mean quality as 55 with the
  // worst case at +1.8% instead of +2.5%, one reject instead of several).
  // Missions under ~1.5 targets do not split at all. A positive
  // chain/segments keeps today's fixed-N behavior; the parameter is read
  // per plan, so it is live-tunable between missions.
  {
    if (!node_->has_parameter("chain/segments"))
      node_->declare_parameter("chain/segments", segments_);
    int req = segments_;
    node_->get_parameter("chain/segments", req);
    auto_segments_ = (req <= 0);
    if (!auto_segments_) segments_ = std::min(16, std::max(2, req));
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
      // [PHASE] Route mode applies WHITELISTED optimizer-owned overrides
      // per worker (phase profiles + seg<i>), in the serial construction
      // window — see planRouteParallel. Non-whitelisted keys are dropped
      // loudly there.
      return planRouteParallel(start_pos, start_vel, start_acc, waypoints,
                               start_vel_synthesized, par, mission_tail);
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
                           mission_tail)) {
    log_->errorf("[CHAIN] baseline plan failed — nothing to chain, nothing "
                 "to fly");
    return PlanResult::failed("baseline plan failed");
  }
  const poly_traj::Trajectory baseline = pm_->traj_.local_traj.traj;
  const double T = baseline.getTotalDuration();
  // Both bail-outs below fly the baseline, and must leave the SAME storage
  // the degrade path leaves: baseline in BOTH slots (at this point the
  // global slot still holds the MINCO seed — the baseline copy normally
  // happens after the segments run) plus viz and the final evaluation.
  const auto fly_baseline = [&]() {
    const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    pm_->traj_.setGlobalTraj(baseline, now_s);
    pm_->traj_.setLocalTraj(baseline, now_s, pm_->traj_.local_traj.drone_id);
    pm_->publishTrajectoryViz(baseline, baseline);
    logFinalEvaluation(baseline, {T}, {"baseline"});
    // A mission genuinely too small to split IS correctly served by the
    // baseline — SUCCESS, not a degradation (outcome matrix, frozen).
    return PlanResult::success();
  };
  if (!resolveAutoSegments(baseline.getPieceNum(), "baseline"))
    return fly_baseline();  // [AUTO-N] mission below the split threshold
  if (T <= 1e-6 || baseline.getPieceNum() < segments_) {
    log_->warnf("[CHAIN] baseline too small to split (%.3f s, %d pieces) — "
                "flying the baseline", T, baseline.getPieceNum());
    return fly_baseline();
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
    ego_planner::TailBoundary seg_tail =
        last ? mission_tail
             : ego_planner::TailBoundary::pinned(contracts[i].vel,
                                                 contracts[i].acc);
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
          seg_tail = ego_planner::TailBoundary::pinned(
              dir.normalized() * seg_vmax[static_cast<size_t>(i)],
              Eigen::Vector3d::Zero());
          log_->infof("[CHAIN] arrival contract at the segment ceiling: "
                      "|v|=%.3f u/s level along the baseline approach",
                      seg_tail.vel.norm());
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
                             seg_tail, /*junction_goal=*/!last,
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
      // Restore BEFORE both readouts: the risk viz and [FINAL-EVAL] must
      // describe this baseline under MISSION-WIDE parameters, not the
      // failed segment's overrides — same order as the success path.
      restore_guard.restore();
      pm_->publishTrajectoryViz(baseline, baseline);
      logFinalEvaluation(baseline, {baseline.getTotalDuration()},
                         {"baseline"});
      PlanResult r = PlanResult::success();
      r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
                "segment " + std::to_string(i + 1) + "/" +
                    std::to_string(segments_) +
                    " failed — baseline restored");
      return r;
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
    if (phase_applied_) {
      phase_names.push_back(i == 0 ? "departure"
                            : i + 1 == runs.size()
                                ? "arrival"
                                : "cruise-" + std::to_string(i));
    } else {
      phase_names.push_back("seg" + std::to_string(i + 1));
    }
  }
  const double t_pre_terminal = chained.getTotalDuration();
  const poly_traj::Trajectory term =
      appendTerminalPhase(&chained, mission_tail.anyPrescribed());
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
  {
    // [CHAIN-VIZ] per-segment colors + labelled seams (+ the terminal
    // handoff as the last junction when a helix was appended).
    std::vector<Eigen::Vector3d> junc;
    for (const auto &c : contracts) junc.push_back(c.pos);
    if (term.getPieceNum() > 0) junc.push_back(term.getJuncPos(0));
    pm_->publishChainSegmentsViz(runs, junc,
                                 term.getPieceNum() > 0 ? &term : nullptr);
  }

  logFinalEvaluation(chained, phase_ends, phase_names);

  const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_wall).count();
  log_->infof("[CHAIN] chained %d segments: %d pieces, %.1f s flight "
              "(baseline %.1f s), planned in %.1f ms wall",
              segments_, chained.getPieceNum(), chained.getTotalDuration(), T,
              wall_ms);
  return PlanResult::success();
}

void SegmentChainPlanner::applyContractJitter(
    const std::vector<Eigen::Vector3d> &route,
    std::vector<Contract> *contracts)
{
  // [JITTER] Sensitivity-experiment hook (all default 0 = off): perturbs the
  // AUTHORED contracts before slicing, to measure how much quality the
  // prescription actually costs. arc_frac slides every junction along the
  // committed route by that fraction of the mean span (position AND tangent
  // re-derived, so the slice endpoints follow); heading_deg then rotates the
  // contract velocity in the horizontal plane; speed_frac scales its
  // magnitude. Loud WARN when active — this is measurement scaffolding,
  // never an operating mode.
  const double heading_deg =
      readNumParam(node_, "chain/jitter/heading_deg", 0.0);
  const double speed_frac = readNumParam(node_, "chain/jitter/speed_frac", 0.0);
  const double arc_frac = readNumParam(node_, "chain/jitter/arc_frac", 0.0);
  if (heading_deg == 0.0 && speed_frac == 0.0 && arc_frac == 0.0) return;
  if (contracts->empty() || route.size() < 2) return;

  log_->warnf("[CHAIN-JITTER] EXPERIMENT: heading %+.1f deg, speed %+.1f%%, "
              "arc %+.2f span — contracts perturbed",
              heading_deg, speed_frac * 100.0, arc_frac);

  // Cumulative arc table of the committed route.
  std::vector<double> s(route.size(), 0.0);
  for (size_t i = 1; i < route.size(); ++i)
    s[i] = s[i - 1] + (route[i] - route[i - 1]).norm();
  const double span = s.back() / static_cast<double>(contracts->size() + 1);
  const auto pointAt = [&](double sq, Eigen::Vector3d *pos,
                           Eigen::Vector3d *tan) {
    sq = std::min(std::max(sq, 0.0), s.back());
    size_t i = 1;
    while (i + 1 < s.size() && s[i] < sq) ++i;
    const double seg = std::max(1e-9, s[i] - s[i - 1]);
    const double t = (sq - s[i - 1]) / seg;
    *pos = route[i - 1] + t * (route[i] - route[i - 1]);
    *tan = (route[i] - route[i - 1]) / seg;
  };
  const double cs = std::cos(heading_deg * M_PI / 180.0);
  const double sn = std::sin(heading_deg * M_PI / 180.0);
  double prev_s = 0.0;
  for (auto &c : *contracts) {
    Eigen::Vector3d pos = c.pos, tan = c.vel.normalized();
    if (arc_frac != 0.0) {
      // Locate the contract on the route (nearest-vertex arc), then slide.
      size_t best = 0;
      double bd = std::numeric_limits<double>::max();
      for (size_t i = 0; i < route.size(); ++i) {
        const double d = (route[i] - c.pos).squaredNorm();
        if (d < bd) { bd = d; best = i; }
      }
      // Keep junction order with a guaranteed gap even at large shifts.
      const double target =
          std::max(s[best] + arc_frac * span, prev_s + 0.1 * span);
      pointAt(target, &pos, &tan);
      prev_s = target;
    }
    const double speed = c.vel.norm() * (1.0 + speed_frac);
    Eigen::Vector3d v = tan;
    if (heading_deg != 0.0) {
      v = Eigen::Vector3d(cs * tan.x() - sn * tan.y(),
                          sn * tan.x() + cs * tan.y(), tan.z());
      v.normalize();
    }
    c.pos = pos;
    c.vel = v * speed;
    // c.acc stays as authored (a = 0).
  }
}

bool SegmentChainPlanner::resolveAutoSegments(int pieces, const char *source)
{
  if (!auto_segments_) return true;
  int target = 70;
  if (!node_->has_parameter("chain/auto_pieces_per_segment"))
    node_->declare_parameter("chain/auto_pieces_per_segment", 70);
  node_->get_parameter("chain/auto_pieces_per_segment", target);
  target = std::max(5, target);  // floor guards absurd targets (N = pieces)
  const int n = (pieces + target / 2) / target;  // round to nearest
  if (n < 2) {
    log_->infof("[CHAIN] auto segments: %d pieces (%s) < ~1.5x target %d — "
                "mission too small to split, flying the single-shot plan",
                pieces, source, target);
    return false;
  }
  segments_ = std::min(16, n);
  log_->infof("[CHAIN] auto segments: %d pieces (%s) / target %d -> N=%d",
              pieces, source, target, segments_);
  return true;
}

SegmentChainPlanner::Contract SegmentChainPlanner::contractFromVertex(
    const std::vector<Eigen::Vector3d> &route, int i, double cruise,
    double tan_grade_max) const
{
  Contract c;
  const int M = static_cast<int>(route.size());
  c.t = 0.0;
  c.pos = route[i];
  Eigen::Vector3d dir =
      route[std::min(i + 1, M - 1)] - route[std::max(i - 1, 0)];
  if (dir.norm() < 1e-9) dir = Eigen::Vector3d::UnitX();
  const double hxy = dir.head<2>().norm();
  if (hxy > 1e-9 && std::abs(dir.z()) > tan_grade_max * hxy)
    dir.z() = std::copysign(tan_grade_max * hxy, dir.z());
  else if (hxy <= 1e-9)
    dir = Eigen::Vector3d::UnitX();
  c.vel = dir.normalized() * cruise;
  c.acc = Eigen::Vector3d::Zero();
  return c;
}

std::vector<Eigen::Vector3d> SegmentChainPlanner::buildDepartureConnector(
    const Eigen::Vector3d &start, const Eigen::Vector3d &v0,
    const Eigen::Vector3d &handoff, double turn_radius_u) const
{
  std::vector<Eigen::Vector3d> pts{start};
  Eigen::Vector2d p = start.head<2>();
  const Eigen::Vector2d tgt = handoff.head<2>();
  Eigen::Vector2d h = v0.head<2>();
  const double step = 2.0;
  if (h.norm() > 1e-6 && turn_radius_u > 1e-6) {
    h.normalize();
    // Turn toward the handoff bearing along a margin-backed circle, then
    // run straight. Anti-parallel start (cross ~ 0) defaults to a left turn.
    Eigen::Vector2d to_t = tgt - p;
    double sgn =
        (h.x() * to_t.y() - h.y() * to_t.x()) >= 0.0 ? 1.0 : -1.0;
    double turned = 0.0;
    while (turned < 2.0 * M_PI) {
      to_t = tgt - p;
      const double d = to_t.norm();
      if (d < step) break;
      to_t /= d;
      const double mis =
          std::atan2(h.x() * to_t.y() - h.y() * to_t.x(), h.dot(to_t));
      if (std::abs(mis) < 0.08) break;
      const double dth =
          sgn * std::min(step / turn_radius_u, std::abs(mis));
      const double cs = std::cos(dth), sn = std::sin(dth);
      h = Eigen::Vector2d(cs * h.x() - sn * h.y(),
                          sn * h.x() + cs * h.y());
      p += h * step;
      turned += std::abs(dth);
      pts.emplace_back(p.x(), p.y(), 0.0);
    }
  }
  while ((tgt - p).norm() > 5.0) {
    p += (tgt - p).normalized() * 5.0;
    pts.emplace_back(p.x(), p.y(), 0.0);
  }
  pts.push_back(handoff);
  // z: smoothstep along cumulative arc (start/handoff exact).
  std::vector<double> s(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i)
    s[i] = s[i - 1] + (pts[i].head<2>() - pts[i - 1].head<2>()).norm();
  const double S = std::max(1e-9, s.back());
  for (size_t i = 0; i < pts.size(); ++i) {
    const double u = s[i] / S;
    pts[i].z() = start.z() + (handoff.z() - start.z()) * (3.0 - 2.0 * u) * u * u;
  }
  pts.front() = start;
  pts.back() = handoff;
  return pts;
}

bool SegmentChainPlanner::validateConnector(
    const std::vector<Eigen::Vector3d> &pts, double tan_grade_max) const
{
  if (pts.size() < 2) return false;
  const double agl_min = 0.8 * pm_->minGoalAgl();
  for (size_t i = 0; i < pts.size(); ++i) {
    double g = 0.0;
    pm_->terrainElevation(pts[i].x(), pts[i].y(), &g);
    if (pts[i].z() < g + agl_min) return false;
    if (nearRiskZone(pts[i])) return false;
    if (i > 0) {
      const double dxy = (pts[i].head<2>() - pts[i - 1].head<2>()).norm();
      const double dz = std::abs(pts[i].z() - pts[i - 1].z());
      if (dxy > 1e-9 && dz / dxy > tan_grade_max * 1.05) return false;
    }
  }
  return true;
}

bool SegmentChainPlanner::authorContractsFromRoute(
    const std::vector<Eigen::Vector3d> &route, const Eigen::Vector3d &v0,
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
  const double W_total = w.back();
  const double span_w = W_total / segments_;

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

  // [PHASE] Departure/arrival handoff selection (frozen v4 rules). All
  // thresholds are ARC-based — vertex counts depend on sampling density.
  // N>=3: pin junction 1 (first sustained-calm vertex past the departure
  // arc) and junction N-1 (last sustained-calm vertex before the arrival
  // arc); middles balance between them. N==2: ONE junction serves both
  // handoffs — the calmest vertex whose calm window holds in BOTH
  // directions. Candidates missing => keep today's balanced junctions and
  // report a phase-semantic degrade (never re-split: N stays).
  phase_note_.clear();
  phase_applied_ = false;
  int dep_idx = -1, arr_idx = -1;
  const bool phase_on =
      readNumParam(node_, "chain/phase/enable", 0.0) != 0.0;
  if (phase_on) {
    const double dep_min =
        readNumParam(node_, "chain/phase/depart_min_arc_u", 30.0);
    const double arr_min =
        readNumParam(node_, "chain/phase/arrive_min_arc_u", 30.0);
    const double cruise_min =
        readNumParam(node_, "chain/phase/min_cruise_arc_u", 60.0);
    const double win = readNumParam(node_, "chain/phase/calm_window_u", 40.0);
    const double kcalm = readNumParam(node_, "chain/phase/kappa_calm", 0.01);
    const double gmax =
        readNumParam(node_, "chain/phase/grade_frac", 0.5) * tan_grade_max;
    const auto vertexCalm = [&](int k) {
      if (kappa[static_cast<size_t>(k)] > kcalm) return false;
      const int km = std::max(0, k - 1), kp = std::min(M - 1, k + 1);
      const double hxy = (route[kp] - route[km]).head<2>().norm();
      const double gr =
          hxy > 1e-9 ? std::abs(route[kp].z() - route[km].z()) / hxy : 1e3;
      if (gr > gmax) return false;
      return !nearRiskZone(route[k]);
    };
    const auto sustained = [&](int i, int dir) {  // +1 fwd, -1 back, 0 both
      const auto leg = [&](int step) {
        double acc = 0.0;
        for (int k = i; k >= 1 && k + 1 < M; k += step) {
          if (!vertexCalm(k)) return false;
          acc += (route[k + 1] - route[k]).norm();
          if (acc >= win) return true;
        }
        return true;  // window truncated by the route end — accept
      };
      if (dir >= 0 && !leg(+1)) return false;
      if (dir <= 0 && !leg(-1)) return false;
      return true;
    };
    // [PHASE-DEP] reachability LOWER-BOUND screening (necessary, not
    // sufficient — the connector solve is the sufficiency check): the
    // handoff must sit beyond the arc a margin-backed minimum-radius turn
    // from the INITIAL heading needs, plus the climb-cone arc for its
    // altitude difference. Sufficiency (lateral room, terrain, vertical
    // speed transition) is delegated to the connector validation + solve.
    phase_tan_grade_ = tan_grade_max;
    phase_turn_radius_u_ = 0.0;
    if (const auto *dyn = pm_->dynamicsParams()) {
      double um = 100.0;
      if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
        um = node_->get_parameter("optimization/dynamics_unit_xy_m")
                 .as_double();
      const double vm = std::max(1e-3, v0.norm()) * um;
      const double nmax = std::max(1.01, dyn->load_factor_max);
      phase_turn_radius_u_ =
          1.5 * vm * vm /
          (dyn->gravity_mps2 * std::sqrt(nmax * nmax - 1.0)) / um;
    }
    const Eigen::Vector2d v0h = v0.head<2>();
    const auto depRequired = [&](int i) {
      double req = dep_min;
      if (phase_turn_radius_u_ > 0.0 && v0h.norm() > 1e-6) {
        Eigen::Vector2d tan_i =
            (route[std::min(i + 1, M - 1)] - route[std::max(i - 1, 0)])
                .head<2>();
        if (tan_i.norm() > 1e-9) {
          const double dpsi = std::abs(std::atan2(
              v0h.normalized().x() * tan_i.normalized().y() -
                  v0h.normalized().y() * tan_i.normalized().x(),
              v0h.normalized().dot(tan_i.normalized())));
          req = std::max(req, phase_turn_radius_u_ * dpsi);
        }
      }
      if (tan_grade_max < 1e8)
        req = std::max(req, std::abs(route[i].z() - route.front().z()) /
                                std::max(1e-6, tan_grade_max));
      return req;
    };
    dep_candidates_.clear();
    arr_candidates_.clear();
    if (segments_ >= 3) {
      for (int i = 1; i + 1 < M && dep_candidates_.size() < 3; ++i)
        if (s[static_cast<size_t>(i)] >= depRequired(i) && sustained(i, +1))
          dep_candidates_.push_back(i);
      if (!dep_candidates_.empty()) dep_idx = dep_candidates_.front();
      for (int i = M - 2; i >= 1 && arr_candidates_.size() < 3; --i)
        if (s[static_cast<size_t>(i)] <= S - arr_min && sustained(i, -1))
          arr_candidates_.push_back(i);
      if (!arr_candidates_.empty()) arr_idx = arr_candidates_.front();
      if (dep_idx < 0 || arr_idx < 0 || arr_idx <= dep_idx ||
          s[static_cast<size_t>(arr_idx)] - s[static_cast<size_t>(dep_idx)] <
              cruise_min) {
        phase_note_ =
            "phase handoffs not found (departure idx " +
            std::to_string(dep_idx) + ", arrival idx " +
            std::to_string(arr_idx) +
            ") — N kept, balanced junctions, phase semantics degraded";
        log_->warnf("[PHASE] %s", phase_note_.c_str());
        dep_idx = arr_idx = -1;
      } else {
        phase_applied_ = true;
        log_->infof("[PHASE] handoffs: departure @s=%.1f u, arrival @s=%.1f "
                    "u (cruise span %.1f u)",
                    s[static_cast<size_t>(dep_idx)],
                    s[static_cast<size_t>(arr_idx)],
                    s[static_cast<size_t>(arr_idx)] -
                        s[static_cast<size_t>(dep_idx)]);
      }
    } else {  // N == 2: single shared handoff, BOTH screenings at once
      double best_k = std::numeric_limits<double>::infinity();
      for (int i = 1; i + 1 < M; ++i) {
        if (s[static_cast<size_t>(i)] < depRequired(i) ||
            s[static_cast<size_t>(i)] > S - arr_min)
          continue;
        if (!sustained(i, 0)) continue;
        if (kappa[static_cast<size_t>(i)] < best_k) {
          best_k = kappa[static_cast<size_t>(i)];
          dep_idx = i;
        }
      }
      if (dep_idx >= 0) {
        phase_applied_ = true;
        dep_candidates_.assign(1, dep_idx);  // N=2: no boundary-move retry
        log_->infof("[PHASE] N=2 shared handoff @s=%.1f u (kappa %.4f)",
                    s[static_cast<size_t>(dep_idx)], best_k);
      } else {
        phase_note_ = "no shared departure/arrival handoff (N=2) — balanced "
                      "junction kept, phase semantics degraded";
        log_->warnf("[PHASE] %s", phase_note_.c_str());
      }
    }
  }

  contracts->clear();
  double w_prev = 0.0;
  for (int j = 1; j < segments_; ++j) {
    int forced = -1;
    if (phase_applied_) {
      if (j == 1 && dep_idx >= 0) forced = dep_idx;
      if (j == segments_ - 1 && arr_idx >= 0) forced = arr_idx;
    }
    double target = j * span_w;
    double lo = w_prev + 0.2 * span_w;
    double hi = target + 0.4 * span_w;
    double span_bal = span_w;
    if (phase_applied_ && segments_ >= 3 && forced < 0) {
      // Middle junctions balance INSIDE the cruise span.
      const double w0 = w[static_cast<size_t>(dep_idx)];
      const double w1 = w[static_cast<size_t>(arr_idx)];
      span_bal = (w1 - w0) / static_cast<double>(segments_ - 2);
      target = w0 + (j - 1) * span_bal;
      lo = std::max(w_prev + 0.2 * span_bal, w0 + 1e-9);
      hi = std::min(target + 0.4 * span_bal, w1 - 1e-9);
    }
    int best = forced, best_any = -1;
    double best_score = std::numeric_limits<double>::infinity();
    double best_any_score = best_score;
    for (int i = 1; best < 0 && i + 1 < M; ++i) {
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
          std::abs(w[static_cast<size_t>(i)] - target) / span_bal +
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

PlanResult SegmentChainPlanner::planRouteParallel(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc,
    const std::vector<Eigen::Vector3d> &waypoints,
    bool start_vel_synthesized, bool run_parallel,
    const ego_planner::TailBoundary &mission_tail)
{
  const auto t_wall = std::chrono::steady_clock::now();
  const auto ms_since = [](const std::chrono::steady_clock::time_point &t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  };

  // Single-shot fallback for anything the route mode cannot author.
  // [PHASE] Effective per-worker override lists (mission-wide < phase
  // profile by position < seg<i>; whitelisted keys only) — filled after N
  // is known, read by the fallback for hard-limit preservation.
  SegmentOverrides prof_dep, prof_cru, prof_arr;
  std::vector<SegmentOverrides> eff;
  std::string soft_drop_note;

  // as_degraded=false marks the one caller where single-shot IS the correct
  // plan (mission below the split threshold), not a repair.
  const auto fallback = [&](const char *why, bool as_degraded = true) {
    log_->warnf("[CHAIN-PAR] %s — falling back to the single-shot plan", why);
    // [PHASE] Hard-tagged requirements survive the fallback CONSERVATIVELY
    // (min across all segments applied mission-wide). Frozen doctrine:
    // preserved -> DEGRADED, lost -> FAILED — a fallback that quietly
    // dropped an explicit hard limit would lie exactly like the old tail
    // sentinel did.
    std::map<std::string, double> hardmin;
    for (const auto &e : eff)
      for (const auto &pp : e.params) {
        const OvrKey *k = whitelistFind(pp.get_name());
        if (!k || k->tag != OvrTag::kHardMax) continue;
        const double v = pp.as_double();
        auto it = hardmin.find(pp.get_name());
        if (it == hardmin.end() || v < it->second)
          hardmin[pp.get_name()] = v;
      }
    ScopedMissionParams fguard(node_, log_);
    if (!hardmin.empty()) {
      try {
        std::vector<rclcpp::Parameter> hp2;
        hp2.reserve(hardmin.size());
        for (const auto &kv : hardmin) hp2.emplace_back(kv.first, kv.second);
        fguard.applyEffective(hp2);
        pm_->initOptimizer(/*force_reinit=*/true);
        pm_->deliverTrajToOptimizer();
        log_->warnf("[PHASE] fallback preserves %zu hard limit(s) "
                    "conservatively (mission-wide min)", hardmin.size());
      } catch (const std::exception &e) {
        return PlanResult::failed(
            std::string("hard-limit preservation failed: ") + e.what());
      }
    }
    pm_->setStartVelSynthesized(start_vel_synthesized);
    const bool fb_ok = pm_->planGlobalTraj(start_pos, start_vel, start_acc,
                                           waypoints, mission_tail);
    if (!hardmin.empty()) {
      fguard.restoreNow();
      try {
        pm_->initOptimizer(/*force_reinit=*/true);
        pm_->deliverTrajToOptimizer();
      } catch (const std::exception &e) {
        log_->errorf("[PHASE] optimizer restore after fallback failed: %s",
                     e.what());
      }
    }
    if (!fb_ok)
      return PlanResult::failed(std::string(why) +
                                "; single-shot fallback failed too");
    PlanResult r = PlanResult::success();
    if (as_degraded) r.degrade(PlanReason::SINGLE_PLAN_FALLBACK, why);
    if (!hardmin.empty())
      r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
                "hard limits preserved conservatively in the single plan");
    return r;
  };

  // === 1. front-end only: commit the route (AGL/bbox/SDF/zone binding
  // happen here exactly once) ===
  if (auto_segments_)
    log_->infof("[CHAIN-PAR] route-%s plan, auto-sized segments "
                "(no baseline)", run_parallel ? "parallel" : "sequential");
  else
    log_->infof("[CHAIN-PAR] route-%s plan, %d segments (no baseline)",
                run_parallel ? "parallel" : "sequential", segments_);
  pm_->setStartVelSynthesized(false);
  if (!pm_->planGlobalTraj(start_pos, start_vel, start_acc, waypoints,
                           ego_planner::TailBoundary{},
                           false, false, nullptr, nullptr,
                           /*front_end_only=*/true)) {
    log_->errorf("[CHAIN-PAR] front-end failed — nothing to author on");
    return PlanResult::failed("front-end failed");
  }
  const std::vector<Eigen::Vector3d> route = pm_->lastCommittedRoute();
  const std::vector<double> cap = pm_->lastCommittedCapRef();
  const double fe_ms = ms_since(t_wall);
  const bool phase_requested =
      readNumParam(node_, "chain/phase/enable", 0.0) != 0.0;
  if (!resolveAutoSegments(static_cast<int>(route.size()) - 1, "route"))
    // Direct plan on a too-small mission is the CORRECT answer — except
    // when phase semantics were requested and cannot be delivered.
    return fallback(phase_requested
                        ? "auto-N below threshold (phase mode: direct)"
                        : "auto-N: mission below the split threshold",
                    /*as_degraded=*/phase_requested);

  // [VEL-ALIGN] resolved BEFORE authoring: the departure handoff screening
  // measures the turn the EFFECTIVE start velocity needs.
  Eigen::Vector3d v0 = start_vel;
  if (pm_->alignStartVelToRoute() && start_vel_synthesized &&
      route.size() >= 2) {
    Eigen::Vector3d dir = route[1] - route[0];
    dir.z() = 0.0;
    if (dir.head<2>().norm() > 1e-9)
      v0 = dir.normalized() * start_vel.norm();
  }

  // === 2. author contracts + slices on the route ===
  std::vector<Contract> contracts;
  if (!authorContractsFromRoute(route, v0, &contracts))
    return fallback("route too small to author junctions on");
  applyContractJitter(route, &contracts);
  std::vector<RouteSlice> slices =
      sliceCommittedRoute(route, cap, contracts);
  if (slices.size() != static_cast<size_t>(segments_))
    return fallback("committed route did not slice cleanly");

  // Segment 1's head is the MISSION start: replicate [VEL-ALIGN] (the
  // route is known here) and the [STALL-FLOOR] the bypassed planGlobalTraj
  // path would have applied.
  if (const auto *dyn = pm_->dynamicsParams()) {
    double um = 100.0;
    if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
      um = node_->get_parameter("optimization/dynamics_unit_xy_m")
               .as_double();
    // Same degenerate-unit guard as dynamicsMinSpeedFloorUnits(): an
    // unguarded division turned dynamics_unit_xy_m=0 into an INFINITE
    // stall floor (audit find).
    const double floor =
        um <= 1e-9
            ? 0.0
            : dyn->speed_min_mps * (1.0 + dyn->constraint_margin) / um;
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
  // [PHASE] Build the effective per-worker override lists: phase profile
  // by POSITION (departure = first, arrival = last, cruise between; only
  // when phase mode is on), then seg<i> on top (explicit index wins).
  // Whitelisted optimizer-owned keys only — everything else is dropped
  // loudly (PathManager-shared values cannot differ per parallel worker).
  prof_dep = readOverrideList("chain/phase/departure/params",
                              "phase-departure");
  prof_cru = readOverrideList("chain/phase/cruise/params", "phase-cruise");
  prof_arr = readOverrideList("chain/phase/arrival/params", "phase-arrival");
  const std::vector<SegmentOverrides> seg_over = readSegmentOverrides();
  eff.assign(static_cast<size_t>(segments_), SegmentOverrides{});
  for (int i = 0; i < segments_; ++i) {
    std::map<std::string, rclcpp::Parameter> m;
    const auto take = [&](const SegmentOverrides &src, const char *origin) {
      for (const auto &pp : src.params) {
        if (!whitelistFind(pp.get_name())) {
          log_->warnf("[PHASE] %s override '%s' is not route-mode "
                      "whitelisted — dropped (baseline flow applies the "
                      "full surface)", origin, pp.get_name().c_str());
          continue;
        }
        m[pp.get_name()] = pp;
      }
    };
    if (phase_requested)
      take(i == 0 ? prof_dep : (i + 1 == segments_ ? prof_arr : prof_cru),
           i == 0 ? "departure" : (i + 1 == segments_ ? "arrival" : "cruise"));
    take(seg_over[static_cast<size_t>(i)], "seg");
    auto &e = eff[static_cast<size_t>(i)];
    for (const auto &kv : m) {
      e.params.push_back(kv.second);
      e.label += (e.label.empty() ? "" : ", ") + kv.first + "=" +
                 kv.second.value_to_string();
    }
  }

  // [PHASE] The baseline-mode ceiling rules, ported (found by the
  // phaseweight harness variant): a profile/seg override that lowers a
  // worker's max_vel below the authored contract speed hands it an
  // unsatisfiable hard BC — the segment fails and the merge ladder drags
  // the whole merged span down to the ceiling. (1) Junction speeds
  // harmonize to min(neighbor ceilings) — shrink only; (2) an unprescribed
  // LAST-segment tail gets a level entry at its own ceiling instead of the
  // mission-wide cruise speed.
  const auto effMaxVel = [&](int i) {
    for (const auto &pp : eff[static_cast<size_t>(i)].params)
      if (pp.get_name() == "optimization/max_vel") return pp.as_double();
    return pm_->maxVel();
  };
  for (size_t c = 0; c < contracts.size(); ++c) {
    const double cap = std::min(effMaxVel(static_cast<int>(c)),
                                effMaxVel(static_cast<int>(c) + 1));
    const double sp = contracts[c].vel.norm();
    if (cap < pm_->maxVel() - 1e-12 && sp > cap + 1e-9) {
      contracts[c].vel *= cap / sp;
      log_->infof("[PHASE] contract %zu speed harmonized to the lowered "
                  "ceiling: %.3f -> %.3f u/s", c + 1, sp, cap);
    }
  }

  // Worker optimizers are built SERIALLY (setParam snapshots node params);
  // the guard resets to pristine between workers and restores mission-wide
  // values — exception-safe — before any thread starts.
  ScopedMissionParams cfg_guard(node_, log_);
  std::vector<std::unique_ptr<ego_planner::PolyTrajOptimizer>> opts;
  for (int i = 0; i < segments_; ++i) {
    cfg_guard.applyEffective(eff[static_cast<size_t>(i)].params);
    if (!eff[static_cast<size_t>(i)].label.empty())
      log_->infof("[PHASE] worker %d effective overrides: %s", i + 1,
                  eff[static_cast<size_t>(i)].label.c_str());
    opts.push_back(pm_->makeConfiguredOptimizer());
  }
  cfg_guard.restoreNow();
  if (run_parallel && had_rpt) node_->set_parameters({saved_rpt});
  const bool sup = pm_->zoneAvoidPassNow() == 1;

  // === 4. solve — the same worker body, threaded or looped ===
  std::vector<poly_traj::Trajectory> runs(static_cast<size_t>(segments_));
  std::vector<char> ok(static_cast<size_t>(segments_), 0);
  std::vector<double> solve_ms(static_cast<size_t>(segments_), 0.0);
  jitter_fail_segment_ = static_cast<int>(
      readNumParam(node_, "chain/jitter/fail_segment", 0.0));
  const auto worker = [&](int i) {
    const auto t0 = std::chrono::steady_clock::now();
    const bool last = (i + 1 == segments_);
    const size_t ui = static_cast<size_t>(i);
    const Eigen::Vector3d hp = i ? contracts[ui - 1].pos : start_pos;
    const Eigen::Vector3d hv = i ? contracts[ui - 1].vel : v0;
    const Eigen::Vector3d ha = i ? contracts[ui - 1].acc : start_acc;
    const Eigen::Vector3d gp = slices[ui].path.back();
    ego_planner::TailBoundary tb =
        last ? mission_tail
             : ego_planner::TailBoundary::pinned(contracts[ui].vel,
                                                 contracts[ui].acc);
    if (last && !tb.prescribe_vel) {
      const double ceil_v = effMaxVel(i);
      if (ceil_v < pm_->maxVel() - 1e-12 && slices[ui].path.size() >= 2) {
        Eigen::Vector3d ad = slices[ui].path.back() -
                             slices[ui].path[slices[ui].path.size() - 2];
        ad.z() = 0.0;
        if (ad.norm() < 1e-6) ad = Eigen::Vector3d::UnitX();
        tb = ego_planner::TailBoundary::pinned(ad.normalized() * ceil_v,
                                               Eigen::Vector3d::Zero());
        log_->infof("[PHASE] arrival tail at the segment ceiling: |v|=%.3f "
                    "u/s level", ceil_v);
      }
    }
    // [JITTER] fault injection for the merge-retry ladder's deterministic
    // test (chain/jitter/fail_segment, 1-based, default 0 = off): the
    // sweeps' organic rejects are session-dependent, so the rescue path
    // needs a fixture that fails on demand.
    if (jitter_fail_segment_ == i + 1 || jitter_fail_segment_ == -1) {
      log_->warnf("[CHAIN-JITTER] EXPERIMENT: segment %d/%d failure "
                  "INJECTED", i + 1, segments_);
      ok[ui] = 0;
      solve_ms[ui] = 0.0;
      return;
    }
    ok[ui] = pm_->solveSlice(*opts[ui], slices[ui].path, slices[ui].cap,
                             hp, hv, ha, gp, tb, sup, &runs[ui])
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

  // [JITTER] One-shot semantics for the failure injection: clear it the
  // moment it fired so a value left behind on a running node (or a stale
  // test yaml) cannot condemn every subsequent mission (review find). The
  // WARN above marks the injected plan; this reset marks the recovery.
  if (jitter_fail_segment_ == -1 ||
      (jitter_fail_segment_ > 0 && jitter_fail_segment_ <= segments_)) {
    node_->set_parameters(
        {rclcpp::Parameter("chain/jitter/fail_segment", 0)});
    log_->warnf("[CHAIN-JITTER] fail_segment fired — cleared to 0 "
                "(one-shot)");
  }

  // [CHAIN-RETRY] Merge ladder before the single-shot fallback. A lone
  // failed segment is usually a junction-placement casualty — the sweeps
  // showed missions that reject at fine splits succeed once the hard
  // stretch is cut less (and the sensitivity experiment showed junction
  // STATE barely matters, so the lever is the junction's existence, not
  // its value). Drop one junction bounding the failed segment and re-solve
  // the merged span once per side; only if both merges fail does the plan
  // degrade to single-shot. Bounded on purpose: exactly one failed
  // segment, at most two extra solves — multi-failure means the corridor
  // is hostile to this split wholesale, and single-shot is the honest
  // answer there.
  {
    std::vector<int> failed;
    for (int i = 0; i < segments_; ++i)
      if (!ok[static_cast<size_t>(i)]) failed.push_back(i);
    const bool merge_on =
        readNumParam(node_, "chain/merge_retry", 1.0) != 0.0;
    if (failed.size() == 1 && merge_on && segments_ >= 2) {
      const int f = failed.front();
      const double peak_max =
          readNumParam(node_, "optimization/audit_envelope_peak_max", 1.30);
      // [PHASE-DEP] design v2: a failed EDGE segment must never dissolve
      // its phase boundary into the cruise (the 548.7 u "departure" worm).
      // Departure: retry with a turn-out CONNECTOR seed that follows the
      // initial velocity — at the same handoff first (N=2: only this), then
      // at farther screened candidates (N>=3). Arrival: move the handoff
      // earlier among its screened candidates. Every retry solve passes the
      // peak-utilization acceptance gate. Exhausted -> DIRECT (single plan,
      // DEGRADED, no phase labels) — never a mislabeled merge.
      bool edge_rescued = false;
      if (phase_applied_ && (f == 0 || f == segments_ - 1)) {
        const bool edge_dep = (f == 0);
        bool rescued = false;
        const auto solveGated =
            [&](ego_planner::PolyTrajOptimizer &o,
                const std::vector<Eigen::Vector3d> &path,
                const std::vector<double> &cp, const Eigen::Vector3d &hp2,
                const Eigen::Vector3d &hv2, const Eigen::Vector3d &ha2,
                const ego_planner::TailBoundary &tb2,
                poly_traj::Trajectory *out2) {
              if (!pm_->solveSlice(o, path, cp, hp2, hv2, ha2, path.back(),
                                   tb2, sup, out2))
                return false;
              if (o.lastEnvPeak() > peak_max) {
                log_->warnf("[PHASE-DEP] retry solve peak %.1f%% > %.0f%% — "
                            "rejected", 100.0 * o.lastEnvPeak(),
                            100.0 * peak_max);
                return false;
              }
              return true;
            };
        const std::vector<int> &cands =
            edge_dep ? dep_candidates_ : arr_candidates_;
        const size_t ci0 = (segments_ == 2 || !edge_dep) ? 1 : 1;
        // Departure at N>=3 additionally retries the ORIGINAL handoff with
        // a connector seed before moving it: the boundary may be fine and
        // only the route-slice seed wrong.
        std::vector<int> attempt;
        if (edge_dep) attempt.push_back(cands.empty() ? -1 : cands[0]);
        for (size_t ci = ci0; ci < cands.size(); ++ci)
          attempt.push_back(cands[ci]);
        for (int c : attempt) {
          if (c < 0 || rescued) break;
          Contract nc = contractFromVertex(route, c, pm_->maxVel(),
                                           phase_tan_grade_);
          std::vector<Contract> trial = contracts;
          trial[edge_dep ? 0 : trial.size() - 1] = nc;
          auto ns = sliceCommittedRoute(route, cap, trial);
          if (ns.size() != static_cast<size_t>(segments_)) continue;
          std::vector<Eigen::Vector3d> edge_path;
          std::vector<double> edge_cap;
          if (edge_dep) {
            edge_path = buildDepartureConnector(start_pos, v0, nc.pos,
                                                phase_turn_radius_u_);
            if (!validateConnector(edge_path, phase_tan_grade_)) {
              log_->warnf("[PHASE-DEP] connector to s-candidate %d failed "
                          "terrain/zone/grade validation — next", c);
              continue;
            }
          } else {
            edge_path = ns.back().path;
            edge_cap = ns.back().cap;
          }
          // Rebuild the two touched optimizers under their profiles.
          ScopedMissionParams eguard(node_, log_);
          eguard.applyEffective(
              eff[static_cast<size_t>(edge_dep ? 0 : segments_ - 1)].params);
          auto o_edge = pm_->makeConfiguredOptimizer();
          eguard.applyEffective(
              eff[static_cast<size_t>(edge_dep ? 1 : segments_ - 2)].params);
          auto o_next = pm_->makeConfiguredOptimizer();
          eguard.restoreNow();
          poly_traj::Trajectory run_edge, run_next;
          bool ok2 = false;
          if (edge_dep) {
            const bool nb_last = (1 == segments_ - 1);
            ego_planner::TailBoundary tb_nb =
                nb_last ? mission_tail
                        : ego_planner::TailBoundary::pinned(trial[1].vel,
                                                            trial[1].acc);
            ok2 = solveGated(*o_edge, edge_path, edge_cap, start_pos, v0,
                             start_acc,
                             ego_planner::TailBoundary::pinned(nc.vel, nc.acc),
                             &run_edge) &&
                  solveGated(*o_next, ns[1].path, ns[1].cap, nc.pos, nc.vel,
                             nc.acc, tb_nb, &run_next);
            if (ok2) {
              contracts = trial;
              slices = ns;
              runs[0] = run_edge;
              runs[1] = run_next;
              ok[0] = ok[1] = 1;
            }
          } else {
            const int lastseg = segments_ - 1;
            const Contract &head_prev =
                contracts[static_cast<size_t>(lastseg - 2 >= 0 ? lastseg - 2
                                                               : 0)];
            ego_planner::TailBoundary tb_last = mission_tail;
            ok2 = solveGated(*o_next, ns[static_cast<size_t>(lastseg - 1)].path,
                             ns[static_cast<size_t>(lastseg - 1)].cap,
                             lastseg - 1 == 0 ? start_pos : head_prev.pos,
                             lastseg - 1 == 0 ? v0 : head_prev.vel,
                             lastseg - 1 == 0 ? start_acc : head_prev.acc,
                             ego_planner::TailBoundary::pinned(nc.vel, nc.acc),
                             &run_next) &&
                  solveGated(*o_edge, edge_path, edge_cap, nc.pos, nc.vel,
                             nc.acc, tb_last, &run_edge);
            if (ok2) {
              contracts = trial;
              slices = ns;
              runs[static_cast<size_t>(lastseg)] = run_edge;
              runs[static_cast<size_t>(lastseg - 1)] = run_next;
              ok[static_cast<size_t>(lastseg)] = 1;
              ok[static_cast<size_t>(lastseg - 1)] = 1;
            }
          }
          if (ok2) {
            rescued = true;
            log_->warnf("[PHASE-DEP] %s boundary rescued at s-candidate %d "
                        "(%s seed) — phase semantics preserved",
                        edge_dep ? "departure" : "arrival", c,
                        edge_dep ? "connector" : "route");
          }
        }
        if (!rescued) {
          phase_applied_ = false;
          PlanResult r = fallback(edge_dep
                                      ? "no reachable departure handoff"
                                      : "no reachable arrival handoff");
          if (r.hasTrajectory())
            r.degrade(PlanReason::PHASE_BOUNDARY_FALLBACK,
                      edge_dep ? "departure boundary undeliverable — direct "
                                 "plan, no phase labels"
                               : "arrival boundary undeliverable — direct "
                                 "plan, no phase labels");
          return r;
        }
        edge_rescued = true;
      }
      if (!edge_rescued) {
      // Neighbor sides, smaller merged span first (keep the re-solve easy).
      std::vector<int> sides;  // neighbor index
      if (f > 0) sides.push_back(f - 1);
      if (f + 1 < segments_) sides.push_back(f + 1);
      // [PHASE-DEP] protected boundaries: the generic merge may never drop
      // the departure (contracts[0]) or arrival (contracts[last]) junction.
      if (phase_applied_) {
        sides.erase(std::remove_if(sides.begin(), sides.end(),
                                   [&](int n) {
                                     const int jd2 = std::min(f, n);
                                     return jd2 == 0 ||
                                            jd2 == static_cast<int>(
                                                       contracts.size()) -
                                                       1;
                                   }),
                    sides.end());
      }
      std::sort(sides.begin(), sides.end(), [&](int a, int b) {
        return slices[static_cast<size_t>(a)].path.size() <
               slices[static_cast<size_t>(b)].path.size();
      });
      bool rescued = false;
      for (int n : sides) {
        const int lo = std::min(f, n), hi = std::max(f, n);
        const size_t ulo = static_cast<size_t>(lo), uhi = static_cast<size_t>(hi);
        const int jd = lo;  // contract index dropped (junction between lo|hi)
        RouteSlice merged;
        merged.path = slices[ulo].path;
        merged.cap = slices[ulo].cap;
        // Slices share the junction vertex — skip the duplicate.
        merged.path.insert(merged.path.end(),
                           slices[uhi].path.begin() + 1, slices[uhi].path.end());
        merged.cap.insert(merged.cap.end(),
                          slices[uhi].cap.begin() + 1, slices[uhi].cap.end());
        const bool m_last = (hi + 1 == segments_);
        const Eigen::Vector3d hp =
            lo ? contracts[static_cast<size_t>(lo - 1)].pos : start_pos;
        const Eigen::Vector3d hv =
            lo ? contracts[static_cast<size_t>(lo - 1)].vel : v0;
        const Eigen::Vector3d ha =
            lo ? contracts[static_cast<size_t>(lo - 1)].acc : start_acc;
        const Eigen::Vector3d gp = merged.path.back();
        const ego_planner::TailBoundary tb =
            m_last ? mission_tail
                   : ego_planner::TailBoundary::pinned(
                         contracts[static_cast<size_t>(hi)].vel,
                         contracts[static_cast<size_t>(hi)].acc);
        log_->warnf("[CHAIN-RETRY] segment %d/%d failed — merging with "
                    "segment %d (junction %d dropped), re-solving the span",
                    f + 1, segments_, n + 1, jd + 1);
        const auto t_retry = std::chrono::steady_clock::now();
        // [PHASE] merged-span profile (frozen): arrival if it contains the
        // last segment, departure if the first, cruise otherwise. seg<i>
        // SOFT overrides are dropped (noted -> degrade); hard-max values
        // merge conservatively (min of both sides).
        SegmentOverrides mo;
        {
          std::map<std::string, rclcpp::Parameter> m;
          if (phase_requested) {
            const SegmentOverrides &prof =
                (hi + 1 == segments_) ? prof_arr
                                      : (lo == 0 ? prof_dep : prof_cru);
            for (const auto &pp : prof.params)
              if (whitelistFind(pp.get_name())) m[pp.get_name()] = pp;
          }
          for (int side : {lo, hi}) {
            for (const auto &pp :
                 eff[static_cast<size_t>(side)].params) {
              const OvrKey *k = whitelistFind(pp.get_name());
              if (!k) continue;
              if (k->tag == OvrTag::kSoft) {
                const auto it = m.find(pp.get_name());
                if (it == m.end() ||
                    it->second.as_double() != pp.as_double())
                  soft_drop_note +=
                      (soft_drop_note.empty() ? "" : ", ") + pp.get_name();
                continue;
              }
              const double v = pp.as_double();
              const auto it = m.find(pp.get_name());
              if (it == m.end() || v < it->second.as_double())
                m[pp.get_name()] = rclcpp::Parameter(pp.get_name(), v);
            }
          }
          for (const auto &kv : m) {
            mo.params.push_back(kv.second);
            mo.label += (mo.label.empty() ? "" : ", ") + kv.first;
          }
        }
        ScopedMissionParams mguard(node_, log_);
        mguard.applyEffective(mo.params);
        auto ropt = pm_->makeConfiguredOptimizer();
        mguard.restoreNow();
        if (!mo.label.empty())
          log_->infof("[PHASE] merged span profile: %s", mo.label.c_str());
        poly_traj::Trajectory mrun;
        if (pm_->solveSlice(*ropt, merged.path, merged.cap, hp, hv, ha, gp,
                            tb, sup, &mrun)) {
          const double retry_ms = ms_since(t_retry);
          runs[ulo] = mrun;
          runs.erase(runs.begin() + static_cast<long>(uhi));
          ok[ulo] = 1;
          ok.erase(ok.begin() + static_cast<long>(uhi));
          solve_ms[ulo] += solve_ms[uhi] + retry_ms;
          solve_ms.erase(solve_ms.begin() + static_cast<long>(uhi));
          slices[ulo] = std::move(merged);
          slices.erase(slices.begin() + static_cast<long>(uhi));
          contracts.erase(contracts.begin() + jd);
          segments_ -= 1;
          log_->warnf("[CHAIN-RETRY] merge rescued the chain: %d segments "
                      "remain (merged solve %.0f ms)", segments_, retry_ms);
          rescued = true;
          break;
        }
        log_->warnf("[CHAIN-RETRY] merged span with segment %d failed too",
                    n + 1);
      }
      if (!rescued) {
        char why[96];
        snprintf(why, sizeof why,
                 "segment %d/%d failed (merge retries exhausted)", f + 1,
                 segments_);
        return fallback(why);
      }
      }  // !edge_rescued (generic merge)
    } else if (!failed.empty()) {
      char why[96];
      snprintf(why, sizeof why, "%zu segments failed", failed.size());
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
    if (phase_applied_) {
      phase_names.push_back(i == 0 ? "departure"
                            : i + 1 == runs.size()
                                ? "arrival"
                                : "cruise-" + std::to_string(i));
    } else {
      phase_names.push_back("seg" + std::to_string(i + 1));
    }
  }
  const double t_pre_terminal = chained.getTotalDuration();
  const poly_traj::Trajectory term =
      appendTerminalPhase(&chained, mission_tail.anyPrescribed());
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
  {
    std::vector<Eigen::Vector3d> junc;
    for (const auto &c : contracts) junc.push_back(c.pos);
    if (term.getPieceNum() > 0) junc.push_back(term.getJuncPos(0));
    pm_->publishChainSegmentsViz(runs, junc,
                                 term.getPieceNum() > 0 ? &term : nullptr);
  }
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
  // A merge rescue that dropped no stated requirement stays SUCCESS
  // (outcome matrix); profile-loss degrades attach at the merge site once
  // phase profiles exist.
  PlanResult result = PlanResult::success();
  if (!phase_note_.empty())
    result.degrade(PlanReason::PHASE_BOUNDARY_FALLBACK, phase_note_);
  if (!soft_drop_note.empty())
    result.degrade(PlanReason::PHASE_BOUNDARY_FALLBACK,
                   "merge dropped soft overrides: " + soft_drop_note);
  return result;
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
  // no-data can never read as clean. The envelope threshold is the SAME
  // configurable gate the per-solve audit uses, not a second number.
  const double viol_max_pct =
      100.0 * param_or("optimization/audit_envelope_violation_max", 0.25);
  const bool no_cruise = dyn != nullptr && tot.env_n == 0;
  // [PHASE-DEP] peak joins the verdict: 0.7% violation TIME hid a 299%
  // instantaneous thrust demand as CLEAN (review find). Same parameter the
  // departure acceptance gate uses.
  const double peak_chk =
      param_or("optimization/audit_envelope_peak_max", 1.30);
  const bool peak_bad = tot.util_peak > peak_chk;
  const bool clean = tot.n_below_ground == 0 && !no_cruise &&
                     viol_pct < viol_max_pct && !peak_bad;
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
                !no_cruise && viol_pct >= viol_max_pct ? "ENVELOPE " : "",
                tot.min_agl, tot.min_agl_t, viol_pct);
  }
}

poly_traj::Trajectory SegmentChainPlanner::appendTerminalPhase(
    poly_traj::Trajectory *chained, bool final_state_prescribed) const
{
  const auto dp = [&](const char *n, auto def) {
    if (!node_->has_parameter(n)) node_->declare_parameter(n, def);
  };
  dp("chain/terminal/enable", false);
  bool enable = false;
  node_->get_parameter("chain/terminal/enable", enable);
  // [PHASE] v1 exclusivity (frozen design): a mission-prescribed final
  // state and the prescribed terminal geometry both claim the trajectory's
  // end — enabling both would make "final state" ambiguous (chain tail vs
  // helix entry). The mission input outranks the yaml toggle; WARN, not
  // ERROR: this is a defined priority, not a malfunction.
  if (enable && final_state_prescribed) {
    log_->warnf("[CHAIN] terminal geometry DISABLED: the mission prescribes "
                "the final state (chain/terminal/enable ignored — the two "
                "modes are exclusive in v1)");
    return {};
  }
  if (!enable) return {};

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
    return {};
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
  if (min_agl < 0.0) {
    // The prescribed phase bypasses [REJECT]/[CONV-REJECT] entirely — this
    // is its collision gate, same criterion as the MINCO audit's
    // below-surface check. Degrade loudly: the chain flies without the
    // helix rather than publishing a trajectory through terrain.
    log_->warnf("[REJECT] PRESCRIBED terminal geometry goes %.3f u BELOW "
                "terrain — helix DISCARDED, the chain flies without it",
                min_agl);
    return {};
  }
  if (min_agl < 0.5 * prm.final_agl) {
    log_->warnf("[CHAIN] PRESCRIBED terminal geometry descends to %.3f u "
                "AGL — no optimizer audit protects this phase; move the "
                "helix or shrink the turns", min_agl);
  }
  chained->append(term);
  return term;
}

SegmentChainPlanner::SegmentOverrides
SegmentChainPlanner::readOverrideList(const std::string &pname,
                                      const std::string &who) const
{
  SegmentOverrides so;
  if (!node_->has_parameter(pname))
    node_->declare_parameter(pname, std::vector<std::string>{});
  std::vector<std::string> specs;
  node_->get_parameter(pname, specs);
  {
    for (const std::string &spec : specs) {
      const size_t eq = spec.find('=');
      if (eq == std::string::npos || eq == 0 || eq + 1 >= spec.size()) {
        log_->warnf("[CHAIN] %s override '%s' is not name=value — skipped",
                    who.c_str(), spec.c_str());
        continue;
      }
      const std::string name = spec.substr(0, eq);
      const std::string val = spec.substr(eq + 1);
      if (!node_->has_parameter(name)) {
        log_->warnf("[CHAIN] %s override '%s': no such parameter — "
                    "skipped (typo, or its consumer never declared it)",
                    who.c_str(), name.c_str());
        continue;
      }
      if (name.rfind("optimization/", 0) != 0) {
        log_->warnf("[CHAIN] %s override '%s': only optimization/* is "
                    "re-read per segment (manager/FSM parameters load at "
                    "startup) — the value will be set but its consumer "
                    "will not see it this mission", who.c_str(), name.c_str());
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
            log_->warnf("[CHAIN] %s override '%s': unsupported parameter "
                        "type — skipped", who.c_str(), name.c_str());
            continue;
        }
      } catch (const std::exception &e) {
        log_->warnf("[CHAIN] %s override '%s': value parse failed (%s) — "
                    "skipped", who.c_str(), spec.c_str(), e.what());
        continue;
      }
      so.label += (so.label.empty() ? "" : ", ") + spec;
    }
  }
  return so;
}

std::vector<SegmentChainPlanner::SegmentOverrides>
SegmentChainPlanner::readSegmentOverrides() const
{
  std::vector<SegmentOverrides> out(static_cast<size_t>(segments_));
  for (int i = 0; i < segments_; ++i)
    out[static_cast<size_t>(i)] = readOverrideList(
        "chain/seg" + std::to_string(i + 1) + "/params",
        "seg" + std::to_string(i + 1));
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
