#include "path_manager/segment_chain_planner.h"

#include <optional>

#include <cstdio>

#include "path_manager/traj_sampling.h"
#include "path_manager/transition_phase.h"

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

// Dense (t, cumulative arc) table + arc->time lookup now live in
// traj_sampling.h: baseline/chain deviation sweeps and the waypoint
// machinery must share ONE arc convention or their reports stop being
// comparable (quintic_hermite.h precedent).
using traj_sampling::ArcTable;
using traj_sampling::buildArcTable;
using traj_sampling::timeAtArcFrac;

}  // namespace

SegmentChainPlanner::SegmentChainPlanner(rclcpp::Node::SharedPtr node,
                                         std::shared_ptr<PathManager> path_manager,
                                         swarm_formation::LogManager *log_manager,
                                         int segments, bool inherit_route)
    : node_(node), pm_(path_manager), log_(log_manager),
      segments_(std::max(2, segments)),
      segments_requested_(std::max(2, segments)),
      inherit_route_(inherit_route) {}

void SegmentChainPlanner::readSegmentsOption()
{
  // [AUTO-N] chain/segments == 0 sizes the split from the mission itself
  // once its piece count is known (route mode: after the front-end;
  // baseline mode: after the baseline solve): N = round(pieces / target),
  // target = chain/auto_pieces_per_segment (default 70 — the 105-run
  // 6-point sweep's operating point: same mean quality as 55 with the
  // worst case at +1.8% instead of +2.5%, one reject instead of several).
  // Missions under ~1.5 targets do not split at all. A positive
  // chain/segments keeps today's fixed-N behavior; the parameter is read
  // per plan, so it is live-tunable between missions.
  if (!node_->has_parameter("chain/segments"))
    node_->declare_parameter("chain/segments", segments_);
  int req = segments_;
  node_->get_parameter("chain/segments", req);
  auto_segments_ = (req <= 0);
  if (!auto_segments_) segments_ = std::min(16, std::max(2, req));
}

void SegmentChainPlanner::resetPlanState()
{
  // [PLAN-STATE] Every member below is per-plan. The merge ladder decrements
  // segments_ in flight and the phase blackboard is written only on the
  // phase-enabled path, so without this a mission inherits the previous
  // mission's N and its screened candidates (review find). Called once at
  // the top of every plan, before anything reads them.
  segments_ = segments_requested_;
  auto_segments_ = false;
  phase_applied_ = false;
  phase_note_.clear();
  last_spans_.clear();
  // Per-plan like the spans beside it: a caller reading lastFlightVerdict()
  // after a plan that never got as far as evaluating must not be handed the
  // previous mission's answer.
  last_verdict_ = FlightVerdict{};
  dep_candidates_.clear();
  arr_candidates_.clear();
  phase_tan_grade_ = 1e9;
  phase_turn_radius_u_ = 0.0;
  jitter_fail_segment_ = 0;
  transition_active_ = false;
}

std::vector<double> SegmentChainPlanner::capAlongRoute(
    const std::vector<Eigen::Vector3d> &route,
    const std::vector<double> &cap,
    const std::vector<Eigen::Vector3d> &pts, int upto_vertex) const
{
  if (cap.size() != route.size() || route.size() < 2 || pts.empty()) return {};
  // The connector spans start -> handoff, so only the route PREFIX up to
  // that vertex is a meaningful neighbourhood. Searching the whole route
  // lets a corridor that doubles back hand a connector point the cap of a
  // segment it never flies near (review find).
  size_t last_seg = route.size() - 2;
  if (upto_vertex > 0)
    last_seg = std::min(last_seg, static_cast<size_t>(upto_vertex - 1));
  std::vector<double> out;
  out.reserve(pts.size());
  for (const auto &p : pts) {
    double best_d2 = std::numeric_limits<double>::max();
    size_t best_seg = 0;
    for (size_t i = 0; i <= last_seg; ++i) {
      const Eigen::Vector3d &a = route[i];
      const Eigen::Vector3d ab = route[i + 1] - a;
      const double len2 = ab.squaredNorm();
      double u = (len2 > 1e-12) ? (p - a).dot(ab) / len2 : 0.0;
      u = std::min(1.0, std::max(0.0, u));
      const double d2 = (a + u * ab - p).squaredNorm();
      if (d2 < best_d2) { best_d2 = d2; best_seg = i; }
    }
    // Two rules, both inherited from how the route's own cap is built:
    //  - never tighten the ceiling at a cut: max of the segment's ends
    //  - the ceiling is never below the path's OWN altitude (planFrontEnd
    //    seeds cap_ref from max(raw_z, route z), an invariant every
    //    route-seeded solve holds). A connector climbs off the route, so
    //    the sampled value alone can land BELOW the point it is capping —
    //    which would price the seed's own altitude as a violation.
    out.push_back(std::max(std::max(cap[best_seg], cap[best_seg + 1]), p.z()));
  }
  return out;
}

void SegmentChainPlanner::degradeForVerdict(PlanResult *r,
                                            const FlightVerdict &fv,
                                            const char *what) const
{
  // One place, four callers (both baseline exits, the multi-waypoint
  // single-shot, the direct fallback). These each grew their own copy of the
  // envelope degrade and none of them grew the standoff one; a shared helper
  // is how the next verdict field reaches all of them instead of three of
  // them.
  if (!r || !fv.evaluated) return;
  // envelope_bad, NOT !clean. !clean is true for an unevaluated zone policy
  // and for an authored-zone contact too, and reporting either of those as
  // STITCHED_ENVELOPE_BUDGET put a machine-readable reason on the result
  // that was simply false — and, because degrade() resolves by enum order,
  // it OVERWROTE the reason that was true.
  if (fv.envelope_bad) {
    char why[192];
    snprintf(why, sizeof why,
             "%s whole-flight reading exceeds the envelope budget "
             "(viol %.1f%%, peak %.1f%%)",
             what, fv.viol_pct, 100.0 * fv.util_peak);
    r->degrade(PlanReason::STITCHED_ENVELOPE_BUDGET, why);
  }
  if (fv.zone_standoff_n > 0) {
    char why[192];
    snprintf(why, sizeof why,
             "%s entered the routing standoff shell around a HARD_AVOID zone "
             "(%d samples) without entering the authored volume",
             what, fv.zone_standoff_n);
    log_->warnf("[STITCH-GATE] %s", why);
    r->degrade(PlanReason::STITCHED_ZONE_STANDOFF, why);
  }
}

void SegmentChainPlanner::invalidateStoredTrajectory() const
{
  // A refused plan must not leave a flyable-looking trajectory behind:
  // ReplanFSM::planFromGlobalTraj publishes and executes traj_.local_traj
  // on (duration > 0 && start_time > 0) alone, so a FAILED outcome that
  // stored one first would be picked up by the next state transition
  // (review find). Zeroing the two fields it tests is the whole contract.
  pm_->traj_.local_traj.duration = 0.0;
  pm_->traj_.local_traj.start_time = 0.0;
  // ...and erase what was already drawn. The direct paths publish the tube
  // and the risk band from INSIDE planGlobalTraj, before this gate can run,
  // and those channels are latched — so a refused flight stayed on screen
  // as the current plan, and was handed to any RViz that connected later.
  // The stitched paths draw nothing before judging, so this is a no-op for
  // them; the guarantee has to hold for every caller either way.
  pm_->clearTrajectoryViz();
}

PlanResult SegmentChainPlanner::plan(const StartHead &head,
                                     const std::vector<Eigen::Vector3d> &waypoints,
                                     const ego_planner::TailBoundary &mission_tail)
{
  // Unpacked once, at the entry. The body reads these names; what changed is
  // that they come from ONE value whose combinations are all legal, instead
  // of three booleans a caller assembled by hand — an assembly that had
  // already produced an illegal one (a prescribed acceleration discarded
  // because the velocity arrived in the scalar form).
  const Eigen::Vector3d &start_pos = head.pos_u;
  const Eigen::Vector3d &start_vel = head.vel_u;
  const Eigen::Vector3d &start_acc = head.acc_u;
  const bool start_vel_synthesized =
      head.src == StartStateSource::STATED_SPEED;
  const bool start_vel_commanded =
      head.src == StartStateSource::STATED_VECTOR ||
      head.src == StartStateSource::TEST_INJECTED;
  const bool start_acc_commanded = head.acc_prescribed;
  (void)start_vel_synthesized;
  // [PLAN-STATE] First statement of the only public entry: no member may
  // carry a previous mission's value into this one. The envelope rejection
  // below returns early, so the reset has to precede it.
  resetPlanState();
  // [AUTO-N] the segment-count option is interpreted HERE, before any
  // mission-shape branch, so the transition coordinator and the plain
  // chain see the same N policy (review find: the transition branch ran
  // on the reset defaults — 2 segments, no cruise span).
  readSegmentsOption();
  // [ENVELOPE] Contract 1 (2026-08-08): an EXPLICITLY commanded initial
  // velocity outside the cruise model's validity region never reaches the
  // cruise pipeline. No clamp — rewriting an operator's stated launch
  // state into a different flyable one is how a 549 u worm once flew as
  // "SUCCESS". Contract 2 (2026-08-08): the classifier now distinguishes
  // the TRANSITION regime (outside cruise, inside the transition model's
  // own validity) and dispatches it to the coordinator — behind
  // transition/enable, default off, so the frozen contract-1 outcome is
  // unchanged until the transition review passes. Synthesized/
  // trajectory-derived starts are our own states and keep the
  // [STALL-FLOOR] clamp doctrine.
  StartRegime regime = StartRegime::CRUISE_VALID;
  std::string regime_why;
  // ONE gate for BOTH stated forms. A scalar-stated head used to take a
  // different branch below and be judged by statedStartSpeedProblem, which
  // checked the margin-backed cruise floor and nothing else — so initial_speed 400 m/s
  // planned while initial_velocity [400,0,0] was refused for exceeding the
  // handoff ceiling, the flight-path cone was never applied to the scalar
  // form at all, and TRANSITION_REQUIRED was unreachable from it: a level
  // 60 m/s stated as a vector dispatched to the coordinator while the same
  // 60 m/s stated as a scalar did not. The two forms differ in what you may
  // SAY, never in what is accepted.
  if (start_vel_commanded || start_vel_synthesized) {
    // start_acc_commanded ALONE. It used to be ANDed with
    // start_vel_commanded, which meant a prescribed acceleration was
    // discarded whenever the velocity arrived in the SCALAR form —
    // use_initial_speed + use_initial_acceleration is a legal message, and
    // its acceleration reached the optimizer head with no envelope judgment
    // at all. The two claims are independent on the wire and are
    // independent here: the acceleration bool says whether an acceleration
    // was prescribed, and nothing about which velocity form carried it.
    regime = classifyStartState(start_pos, start_vel, start_acc,
                                start_acc_commanded, &regime_why);
    if (regime == StartRegime::UNSUPPORTED) {
      log_->errorf("[ENVELOPE] commanded initial state REJECTED: %s "
                   "(INITIAL_MODE_UNSUPPORTED)", regime_why.c_str());
      return PlanResult::failedBecause(
          PlanReason::INITIAL_MODE_UNSUPPORTED,
          "initial state unsupported: " + regime_why);
    }
    if (regime == StartRegime::TRANSITION_REQUIRED) {
      bool enabled = false;
      if (!node_->has_parameter("transition/enable"))
        node_->declare_parameter("transition/enable", false);
      node_->get_parameter("transition/enable", enabled);
      if (!enabled) {
        log_->errorf("[ENVELOPE] commanded initial state REJECTED: %s — "
                     "transition regime recognized but transition/enable "
                     "is off (INITIAL_MODE_UNSUPPORTED)",
                     regime_why.c_str());
        return PlanResult::failedBecause(
            PlanReason::INITIAL_MODE_UNSUPPORTED,
            "initial state unsupported: " + regime_why +
                " (transition disabled)");
      }
    }
  }
  // [PHASE] Final-boundary validation happens ONCE, here — every exit of
  // planImpl (workers, merge retry, fallbacks, baseline restore) receives
  // the validated tail, so an unmet stated requirement can never leave as
  // a plain success. Frozen policy: invalid + no opt-in -> FAILED; invalid
  // + planning/allow_final_boundary_relaxation -> plan without the tail and
  // return DEGRADED(FINAL_BOUNDARY_RELAXED). Same envelope validator as the
  // initial state: a terminal state the model cannot fly (sub-stall, above
  // max speed, outside the flight-path cone) is prescribed like the launch
  // phase — outside the region, not plannable.
  ego_planner::TailBoundary eff = mission_tail;
  bool relaxed = false;
  std::string relax_why;
  if (mission_tail.anyPrescribed()) {
    std::string problem;
    if (!mission_tail.allFinite()) {
      problem = "non-finite final boundary";
    } else if (mission_tail.prescribe_vel) {
      const std::string ep = pm_->stateEnvelopeProblem(mission_tail.vel);
      if (!ep.empty())
        problem = "final velocity outside the cruise validity region: " + ep;
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
  if (regime == StartRegime::TRANSITION_REQUIRED) {
    // Tail validation above applies to EVERY mission shape — the
    // transition dispatch happens after it so a bad final boundary can
    // never slip out through the new path.
    PlanResult r = planTransitionMission(head, waypoints, eff);
    if (relaxed) r.degrade(PlanReason::FINAL_BOUNDARY_RELAXED, relax_why);
    return r;
  }
  PlanResult r = planImpl(head, waypoints, eff);
  if (relaxed) r.degrade(PlanReason::FINAL_BOUNDARY_RELAXED, relax_why);
  return r;
}

PlanResult SegmentChainPlanner::planImpl(const StartHead &head,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               const ego_planner::TailBoundary &mission_tail)
{
  const Eigen::Vector3d &start_pos = head.pos_u;
  const Eigen::Vector3d &start_vel = head.vel_u;
  const Eigen::Vector3d &start_acc = head.acc_u;
  const bool start_vel_synthesized =
      head.src == StartStateSource::STATED_SPEED;
  (void)start_vel_synthesized;
  // Stage-1 scope: one goal. Multi-waypoint missions need a waypoint-to-span
  // assignment that does not exist yet — fall back to the single-shot plan.
  if (waypoints.size() != 1) {
    log_->warnf("[CHAIN] %zu waypoints — stage 1 chains single-goal missions "
                "only, falling back to the single-shot plan", waypoints.size());
    if (!pm_->planGlobalTraj(head, waypoints, mission_tail))
      return PlanResult::failed("multi-waypoint single-shot plan failed");
    // Same contract as every other direct product: judged before it is
    // accepted. "Chain not attempted" is a statement about how the flight
    // was produced, never about whether it was checked.
    log_->infof("[PLAN-MODE] direct");
    const poly_traj::Trajectory &mw = pm_->traj_.local_traj.traj;
    const FlightVerdict mv = evaluateFlight(
        mw, {{mw.getTotalDuration(), PhaseKind::CRUISE, "direct"}});
    // A multi-leg front end runs one zone search PER LEG and
    // zonePolicySnapshot is plan-wide-or-nothing (path_manager.cpp), so
    // every multi-waypoint mission with a zone ANYWHERE reports
    // policy_measurable=false. Two wrong answers are available here and this
    // gate has now had both:
    //   refuse on it  -> a zone 50 km off the route kills the mission
    //   degrade on it -> the FSM EXECUTES a flight nobody checked against
    //                    the zones, which is fail-open however loudly it is
    //                    logged. "We could not check" is not a safety
    //                    argument, and one test with one distant zone is
    //                    evidence about that zone, not about the policy.
    // Until a per-leg snapshot exists, the default is fail-CLOSED and the
    // other behaviour is an explicit, named opt-in someone has to turn on.
    bool allow_unmeasured =
        readNumParam(node_, "manager/allow_unmeasured_zone_policy", 0.0) != 0.0;
    const bool unmeasured_blocks = !mv.policy_measurable && !allow_unmeasured;
    if (!mv.evaluated || mv.unflyableMeasured() || unmeasured_blocks) {
      char why[256];
      snprintf(why, sizeof why,
               "multi-waypoint single-shot flight is unflyable (%s%s%s%s%s)",
               !mv.evaluated ? "not evaluated" : "",
               mv.underground ? "terrain overlap; " : "",
               mv.no_cruise ? "never reaches cruise; " : "",
               mv.zone_hard_n > 0 ? "authored zone entered; " : "",
               unmeasured_blocks
                   ? "zone policy could not be evaluated on a multi-leg "
                     "mission — set manager/allow_unmeasured_zone_policy to "
                     "fly it uncleared" : "");
      log_->errorf("[STITCH-GATE] %s", why);
      invalidateStoredTrajectory();
      return PlanResult::failedBecause(PlanReason::STITCHED_FLIGHT_UNSAFE,
                                       why);
    }
    PlanResult r = PlanResult::success();
    r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
              "multi-waypoint mission — chain not attempted");
    if (!mv.policy_measurable) {
      // Only reachable with the opt-in above set.
      const char *why2 =
          "zone policy was NOT evaluated: a multi-leg front end cannot "
          "produce a plan-wide zone snapshot, so this flight is uncleared "
          "with respect to risk zones (flown because "
          "manager/allow_unmeasured_zone_policy is set)";
      log_->warnf("[STITCH-GATE] %s", why2);
      r.degrade(PlanReason::ZONE_POLICY_UNEVALUATED, why2);
    }
    degradeForVerdict(&r, mv, "multi-waypoint single-shot flight");
    return r;
  }

  // [AUTO-N] chain/segments interpretation moved to readSegmentsOption()
  // (shared with the transition coordinator, run at plan() entry).

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
      return planRouteParallel(head, waypoints, par, mission_tail);
    } else if (par) {
      log_->warnf("[CHAIN] chain/parallel needs chain/author_from_route "
                  "(the baseline is inherently sequential) — running the "
                  "sequential baseline flow");
    }
  }

  const auto t_wall = std::chrono::steady_clock::now();

  // === Baseline: the unsplit mission, exactly today's behavior ===
  // N is not known yet in auto mode — it is sized from THIS solve's piece
  // count a few lines below. Printing segments_ here announced a number the
  // plan never used (review find; it only looked right before resetPlanState
  // because the member still carried the previous mission's resolved N).
  if (auto_segments_)
    log_->infof("[CHAIN] baseline plan (unsplit mission; segment count sized "
                "from its piece count)");
  else
    log_->infof("[CHAIN] baseline plan (unsplit mission; %d chained segments "
                "follow)", segments_);
  if (!pm_->planGlobalTraj(head, waypoints, mission_tail)) {
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
  // One gate, two baseline exits. Both used to store and publish first and
  // keep evaluateFlight only for its log line; a whole-flight refusal that
  // the stitched path enforces must not be escapable by bailing out to the
  // baseline.
  FlightVerdict baseline_verdict{};
  const auto baseline_gate = [&](const poly_traj::Trajectory &b)
      -> std::optional<PlanResult> {
    const auto bv = evaluateFlight(
        b, {{b.getTotalDuration(), PhaseKind::CRUISE, "baseline"}});
    baseline_verdict = bv;
    // !evaluated must fail too. unflyable() is false when nothing was
    // judged, so testing it alone lets a flight the evaluator could not
    // read pass as safe — the exact fail-open this gate exists to remove.
    if (bv.evaluated && !bv.unflyable()) return std::nullopt;
    char why[176];
    snprintf(why, sizeof why,
             "baseline flight is unflyable (%s%s%s%s%s) — whole-flight "
             "evaluation",
             !bv.evaluated ? "not evaluated" : "",
             bv.underground ? "terrain overlap; " : "",
             bv.no_cruise ? "never reaches cruise; " : "",
             bv.zone_hard_n > 0 ? "authored zone entered; " : "",
             !bv.evaluated ? "" :
                 (!bv.policy_measurable ? "zone policy unevaluated" : ""));
    log_->errorf("[STITCH-GATE] %s", why);
    invalidateStoredTrajectory();
    return PlanResult::failedBecause(PlanReason::STITCHED_FLIGHT_UNSAFE, why);
  };

  const auto fly_baseline = [&]() {
    // Explicit, so a consumer never has to infer the mode from the ABSENCE
    // of a chain line — the same fail-open shape as inferring zone safety
    // from a missing warning.
    log_->infof("[PLAN-MODE] direct");
    // EVALUATE BEFORE STORING. This path used to store, publish, and then
    // call evaluateFlight for its log line only, discarding the answer — so
    // a baseline that touched a HARD_AVOID volume, or one whose zone policy
    // could not be judged, was returned as a success and was publishable.
    // The stitched path refuses exactly that, and a bail-out to the baseline
    // must not be the way around the refusal.
    if (const auto bad = baseline_gate(baseline)) return *bad;
    const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    pm_->traj_.setGlobalTraj(baseline, now_s);
    pm_->traj_.setLocalTraj(baseline, now_s, pm_->traj_.local_traj.drone_id);
    pm_->publishTrajectoryViz(baseline, baseline);
    // A mission genuinely too small to split IS correctly served by the
    // baseline — SUCCESS, not a degradation (outcome matrix, frozen).
    PlanResult r = PlanResult::success();
    // ...but a baseline whose whole-flight reading is worse than clean must
    // SAY so. Only unflyable() refuses here; everything else was returning a
    // plain SUCCESS while FINAL-EVAL printed CHECK, so the two channels
    // disagreed about the same flight.
    degradeForVerdict(&r, baseline_verdict, "baseline");
    return r;
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
    const RouteSlice *slice =
        (i < static_cast<int>(slices.size())) ? &slices[i] : nullptr;
    // Segment 0's head is the mission head; every later segment's head is
    // the previous segment's tail — a CHAIN_JUNCTION, which the policy
    // leaves verbatim.
    StartHead seg_head;
    seg_head.src = (i == 0) ? head.src : StartStateSource::CHAIN_JUNCTION;
    seg_head.pos_u = head_pos;
    seg_head.vel_u = head_vel;
    seg_head.acc_u = head_acc;
    seg_head.acc_prescribed = (i == 0) ? head.acc_prescribed : true;
    if (!pm_->planGlobalTraj(seg_head, goal,
                             seg_tail, /*junction_goal=*/!last,
                             slice ? &slice->path : nullptr,
                             slice ? &slice->cap : nullptr)) {
      // Degrade loudly to the baseline: the mission still flies, and the
      // failed experiment is visible in the log, not in the sky.
      log_->warnf("[CHAIN] segment %d/%d FAILED — restoring and flying the "
                  "baseline", i + 1, segments_);
      // Restore BEFORE both readouts: the risk viz and [FINAL-EVAL] must
      // describe this baseline under MISSION-WIDE parameters, not the
      // failed segment's overrides — same order as the success path. The
      // gate therefore runs after the restore and before any storing.
      restore_guard.restore();
      if (const auto bad = baseline_gate(baseline)) return *bad;
      const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
      pm_->traj_.setGlobalTraj(baseline, now_s);
      pm_->traj_.setLocalTraj(baseline, now_s, pm_->traj_.local_traj.drone_id);
      pm_->publishTrajectoryViz(baseline, baseline);
      PlanResult r = PlanResult::success();
      r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
                "segment " + std::to_string(i + 1) + "/" +
                    std::to_string(segments_) +
                    " failed — baseline restored");
      degradeForVerdict(&r, baseline_verdict, "restored baseline");
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
  std::vector<PhaseSpan> spans;
  double acc_t = 0.0;
  for (size_t i = 0; i < runs.size(); ++i) {
    acc_t += runs[i].getTotalDuration();
    // All chained spans are CRUISE-judged today; the transition coordinator
    // will prepend a TRANSITION span. Kind is the judgment contract, the
    // name is display only.
    spans.push_back({acc_t, PhaseKind::CRUISE,
                     phase_applied_
                         ? (i == 0 ? std::string("departure")
                            : i + 1 == runs.size()
                                ? std::string("arrival")
                                : "cruise-" + std::to_string(i))
                         : "seg" + std::to_string(i + 1)});
  }
  const double t_pre_terminal = chained.getTotalDuration();
  const poly_traj::Trajectory term =
      appendTerminalPhase(&chained, mission_tail.anyPrescribed());
  if (chained.getTotalDuration() > t_pre_terminal + 1e-9) {
    spans.push_back(
        {chained.getTotalDuration(), PhaseKind::TERMINAL, "terminal"});
  }

  // [STITCH-GATE] Judged before storage — see the route-mode twin: a refused
  // flight must not reach traj_ or RViz.
  //
  // Baseline mode differs from route mode in what it can do about it: a
  // fully solved, per-solve-audited baseline is sitting right here. Killing
  // the mission while holding a flyable trajectory would contradict this
  // mode's whole doctrine ("on any segment failure the baseline is restored
  // and flown"), so an unflyable STITCH degrades onto the baseline instead
  // of failing (review find). Route mode has no baseline, so there FAILED
  // is the only honest answer.
  const FlightVerdict fv = evaluateFlight(chained, spans);
  if (!fv.evaluated || fv.unflyable()) {
    // UNEVALUATED joins unflyable here too (fail-closed): the baseline is
    // still the honest repair — a judged-good baseline beats refusing the
    // mission because the STITCH could not be judged.
    log_->errorf("[STITCH-GATE] stitched flight %s (%s%s) — restoring the "
                 "baseline",
                 fv.evaluated ? "is unflyable" : "could not be evaluated",
                 fv.underground ? "terrain overlap" : "",
                 fv.no_cruise ? (fv.underground ? ", never reaches cruise"
                                                : "never reaches cruise")
                              : "");
    PlanResult r = fly_baseline();
    if (r.hasTrajectory())
      r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
                "stitched chain failed the whole-flight evaluation — "
                "baseline flown instead");
    else
      invalidateStoredTrajectory();
    return r;
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

  const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_wall).count();
  log_->infof("[CHAIN] chained %d segments: %d pieces, %.1f s flight "
              "(baseline %.1f s), planned in %.1f ms wall",
              segments_, chained.getPieceNum(), chained.getTotalDuration(), T,
              wall_ms);
  return stitchedVerdictResult(fv, PlanResult::success());
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
  // Keep this in step with optimizer_params.yaml — the yaml wins when it is
  // loaded, so a divergent fallback only shows up for an embedder that does
  // not load it, which is the worst place to discover a different default.
  int target = 35;
  if (!node_->has_parameter("chain/auto_pieces_per_segment"))
    node_->declare_parameter("chain/auto_pieces_per_segment", 35);
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

std::vector<double> SegmentChainPlanner::routeArcTable(
    const std::vector<Eigen::Vector3d> &route)
{
  std::vector<double> s(route.size(), 0.0);
  for (size_t i = 1; i < route.size(); ++i)
    s[i] = s[i - 1] + (route[i] - route[i - 1]).norm();
  return s;
}

std::vector<double> SegmentChainPlanner::routeCurvature(
    const std::vector<Eigen::Vector3d> &route)
{
  const int M = static_cast<int>(route.size());
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
  return kappa;
}

SegmentChainPlanner::HandoffScreenResult
SegmentChainPlanner::screenHandoffCandidates(
    const std::vector<Eigen::Vector3d> &route, const Eigen::Vector3d &v0,
    const ReachLimits &rl, double arr_min_arc_u) const
{
  HandoffScreenResult out;
  const int M = static_cast<int>(route.size());
  if (M < 3) return out;
  const std::vector<double> s = routeArcTable(route);
  const std::vector<double> kappa = routeCurvature(route);
  const double S = s.back();
  const auto vertexCalm = [&](int k) {
    if (kappa[static_cast<size_t>(k)] > rl.kappa_calm) return false;
    const int km = std::max(0, k - 1), kp = std::min(M - 1, k + 1);
    const double hxy = (route[kp] - route[km]).head<2>().norm();
    const double gr =
        hxy > 1e-9 ? std::abs(route[kp].z() - route[km].z()) / hxy : 1e3;
    if (gr > rl.grade_max) return false;
    return !nearRiskZone(route[k]);
  };
  const auto sustained = [&](int i, int dir) {  // +1 fwd, -1 back, 0 both
    const auto leg = [&](int step) {
      double acc = 0.0;
      for (int k = i; k >= 1 && k + 1 < M; k += step) {
        if (!vertexCalm(k)) return false;
        acc += (route[k + 1] - route[k]).norm();
        if (acc >= rl.calm_window_u) return true;
      }
      return true;  // window truncated by the route end — accept
    };
    if (dir >= 0 && !leg(+1)) return false;
    if (dir <= 0 && !leg(-1)) return false;
    return true;
  };
  const Eigen::Vector2d v0h = v0.head<2>();
  const auto depRequired = [&](int i) {
    double req = rl.dep_min_arc_u;
    if (rl.turn_radius_u > 0.0 && v0h.norm() > 1e-6) {
      Eigen::Vector2d tan_i =
          (route[std::min(i + 1, M - 1)] - route[std::max(i - 1, 0)])
              .head<2>();
      if (tan_i.norm() > 1e-9) {
        const double dpsi = std::abs(std::atan2(
            v0h.normalized().x() * tan_i.normalized().y() -
                v0h.normalized().y() * tan_i.normalized().x(),
            v0h.normalized().dot(tan_i.normalized())));
        req = std::max(req, rl.turn_radius_u * dpsi);
      }
    }
    if (rl.tan_grade_max < 1e8)
      req = std::max(req, std::abs(route[i].z() - route.front().z()) /
                              std::max(1e-6, rl.tan_grade_max));
    // Speed-change arc (transition only; 0 for the cruise-to-cruise
    // screen): the entry must also be far enough to close the speed gap.
    return req + rl.accel_arc_u;
  };
  for (int i = 1;
       i + 1 < M && out.dep_candidates.size() < rl.max_candidates; ++i)
    if (s[static_cast<size_t>(i)] >= depRequired(i) && sustained(i, +1))
      out.dep_candidates.push_back(i);
  for (int i = M - 2;
       i >= 1 && out.arr_candidates.size() < rl.max_candidates; --i)
    if (s[static_cast<size_t>(i)] <= S - arr_min_arc_u && sustained(i, -1))
      out.arr_candidates.push_back(i);
  double best_k = std::numeric_limits<double>::infinity();
  for (int i = 1; i + 1 < M; ++i) {
    if (s[static_cast<size_t>(i)] < depRequired(i) ||
        s[static_cast<size_t>(i)] > S - arr_min_arc_u)
      continue;
    if (!sustained(i, 0)) continue;
    if (kappa[static_cast<size_t>(i)] < best_k) {
      best_k = kappa[static_cast<size_t>(i)];
      out.shared_idx = i;
    }
  }
  if (out.shared_idx >= 0) out.shared_kappa = best_k;
  return out;
}

bool SegmentChainPlanner::authorContractsFromRoute(
    const std::vector<Eigen::Vector3d> &route, const Eigen::Vector3d &v0,
    std::vector<Contract> *contracts) const
{
  const int M = static_cast<int>(route.size());
  if (M < segments_ + 1 || !contracts) return false;
  const std::vector<double> s = routeArcTable(route);
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
  const std::vector<double> kappa = routeCurvature(route);

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
    dep_candidates_.clear();
    arr_candidates_.clear();
    // Candidates are taken FIRST-FIT (nearest qualifying vertex), not
    // scored — a ranking needs a reachability model that does not exist
    // yet (contract 2 §12), so the only knob here is how many the retry
    // ladder may try. Each extra candidate costs a full solve.
    // Clamped on BOTH sides: an unclamped huge value (a stray 1e300, or
    // YAML's .inf) casts to 0 and would silently switch handoff selection
    // off entirely (review find). 1 means "authored handoff only, no retry"
    // — the retry ladder starts at candidate index 1.
    const size_t max_cand = static_cast<size_t>(std::min(
        16.0, std::max(1.0,
                       readNumParam(node_, "chain/phase/max_candidates", 3.0))));
    ReachLimits rl;
    rl.dep_min_arc_u = dep_min;
    rl.calm_window_u = win;
    rl.kappa_calm = kcalm;
    rl.grade_max = gmax;
    rl.tan_grade_max = tan_grade_max;
    rl.turn_radius_u = phase_turn_radius_u_;
    rl.accel_arc_u = 0.0;  // cruise-to-cruise legacy screen: no speed change
    rl.max_candidates = max_cand;
    const HandoffScreenResult hs =
        screenHandoffCandidates(route, v0, rl, arr_min);
    if (segments_ >= 3) {
      dep_candidates_ = hs.dep_candidates;
      if (!dep_candidates_.empty()) dep_idx = dep_candidates_.front();
      arr_candidates_ = hs.arr_candidates;
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
      dep_idx = hs.shared_idx;
      if (dep_idx >= 0) {
        phase_applied_ = true;
        dep_candidates_.assign(1, dep_idx);  // N=2: no boundary-move retry
        log_->infof("[PHASE] N=2 shared handoff @s=%.1f u (kappa %.4f)",
                    s[static_cast<size_t>(dep_idx)], hs.shared_kappa);
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
    // A FORCED junction (the phase departure/arrival handoff) is not up
    // for selection — the screening already chose it. Only free
    // junctions are searched, and they are searched across the WHOLE
    // window: the loop used to carry `best < 0` in its condition, so it
    // stopped at the first admissible vertex and the scoring below —
    // calmness first, balance breaking ties — never ran. The effect was
    // systematic, every free junction landing on the window's lower
    // edge. Measured on the live mission: junction 2 sat at arc 253.1 u
    // where the window opened at 244.1 and the balance target was 1085,
    // leaving one segment carrying 78% of the route and 1017 ms of the
    // 1428 ms plan while its siblings took 1-8 ms.
    for (int i = 1; forced < 0 && i + 1 < M; ++i) {
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

// [S13] The route-parallel plan decomposed into its two real stages, so a
// transition coordinator can call them SEPARATELY: commit the global route
// once, run a transition toward an entry point ON it, cut the route there,
// and hand planOverRoute the slice with the transition end state as the
// head. planRouteParallel itself is now just the no-transition composition
// of the two.

SegmentChainPlanner::StartRegime SegmentChainPlanner::classifyStartState(
    const Eigen::Vector3d &pos_u, const Eigen::Vector3d &vel_u,
    const Eigen::Vector3d &acc_u, bool acc_prescribed,
    std::string *why) const
{
  const auto unsupported = [&](const std::string &w) {
    if (why) *why = w;
    return StartRegime::UNSUPPORTED;
  };
  // Finiteness is judged regardless of prescription — garbage is garbage.
  if (!pos_u.allFinite() || !vel_u.allFinite() || !acc_u.allFinite())
    return unsupported("non-finite commanded state");
  // A stated REST start, from either form, in EVERY configuration. This is
  // model-independent on purpose: a zero head has no direction, the
  // first-leg synthesis and every downstream normalization divide by it,
  // and no parameter set makes it flyable. Justifying rest-refusal with the
  // margin-backed cruise floor instead would leave it unrefused under
  // optimization/dynamics_enable: false, where the floor does not exist.
  // Legal to STATE — parseStartClaim accepts it — and refused here, by
  // name, rather than silently raised to the floor.
  if (vel_u.squaredNorm() <= 0.0)
    return unsupported(
        "stated initial speed is zero — a rest start has no direction and "
        "this stack cannot fly one; the launch phase needs a transition "
        "planner, not a clamp");
  // Prescribed: the full PVA is the operator's claim — judge all of it.
  // Unprescribed: the internal acc value is NOT evidence; the velocity
  // state alone decides the regime.
  const std::string prob =
      acc_prescribed ? pm_->pvaEnvelopeProblem(pos_u, vel_u, acc_u)
                     : pm_->stateEnvelopeProblem(vel_u);
  if (prob.empty()) return StartRegime::CRUISE_VALID;
  if (why) *why = prob;
  const auto *dyn = pm_->dynamicsParams();
  if (!dyn || !mmp_vehicle_dynamics::parametersAreValid(*dyn))
    return unsupported(prob +
                       "; no valid assumption parameter set to transition "
                       "under");
  double um = 100.0, uz = 100.0;
  if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
    um = node_->get_parameter("optimization/dynamics_unit_xy_m").as_double();
  if (node_->has_parameter("optimization/dynamics_unit_z_m"))
    uz = node_->get_parameter("optimization/dynamics_unit_z_m").as_double();
  const Eigen::Vector3d v_si(vel_u.x() * um, vel_u.y() * um,
                             vel_u.z() * uz);
  const double V = v_si.norm();
  // [SPEED-BOUNDARY] same numeric-equality rule as the envelope limits
  // (path_manager.h): a speed commanded exactly at the boundary lands on the
  // passing side in every direction.
  if (PathManager::belowSpeedBoundary(V, dyn->model_activation_speed_mps))
    return unsupported(prob + "; below the transition model's activation "
                              "speed");
  if (PathManager::aboveSpeedBoundary(V, dyn->speed_max_mps))
    return unsupported(prob + "; above the model ceiling — the model is "
                              "undefined there, a transition claim would "
                              "be a physics claim");
  const double gamma = std::asin(
      std::min(1.0, std::max(-1.0, v_si.z() / std::max(V, 1e-9))));
  // The TRANSITION POLICY gamma bound — the same single definition the
  // generator pre-guards and the polynomial validator enforce
  // (TransitionLimits default member), never a duplicated literal.
  if (std::abs(gamma) >
      transition_phase::TransitionLimits{}.max_abs_gamma_rad)
    return unsupported(prob + "; flight-path angle outside the "
                              "transition policy cone");
  return StartRegime::TRANSITION_REQUIRED;
}

bool SegmentChainPlanner::cutAtArc(const std::vector<Eigen::Vector3d> &route,
                                   const std::vector<double> &cap,
                                   double s_cut,
                                   std::vector<Eigen::Vector3d> *out_route,
                                   std::vector<double> *out_cap) const
{
  if (!out_route || !out_cap || route.size() < 2 ||
      cap.size() != route.size() || !std::isfinite(s_cut) || s_cut < 0.0)
    return false;
  const std::vector<double> s = routeArcTable(route);
  if (s_cut >= s.back()) return false;  // nothing left to chain over
  size_t i = 0;
  while (i + 2 < route.size() && s[i + 1] <= s_cut) ++i;
  const double seg = s[i + 1] - s[i];
  const double u = seg > 1e-12 ? (s_cut - s[i]) / seg : 0.0;
  const Eigen::Vector3d v0 = route[i] + u * (route[i + 1] - route[i]);
  if (!v0.allFinite()) return false;
  out_route->clear();
  out_cap->clear();
  // cap never tightens across the cut, floored by the vertex's own z.
  const double cap0 = std::max({cap[i], cap[i + 1], v0.z()});
  if ((v0 - route[i + 1]).norm() < 1e-3) {
    out_route->push_back(route[i + 1]);
    out_cap->push_back(std::max(cap0, cap[i + 1]));
    ++i;
  } else {
    out_route->push_back(v0);
    out_cap->push_back(cap0);
  }
  for (size_t k = i + 1; k < route.size(); ++k) {
    out_route->push_back(route[k]);
    out_cap->push_back(cap[k]);
  }
  return out_route->size() >= 2;
}

PlanResult SegmentChainPlanner::planTransitionMission(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail)
{
  const Eigen::Vector3d &start_pos = head.pos_u;
  const Eigen::Vector3d &start_vel = head.vel_u;
  const Eigen::Vector3d &start_acc = head.acc_u;
  const bool start_acc_commanded = head.acc_prescribed;
  namespace tp = transition_phase;
  // The section-13 sequence, owned end to end. transition_active_ holds
  // for the whole scope: every fallback() below the coordinator returns
  // FAILED instead of re-planning the regime this function exists for.
  struct ActiveGuard {
    SegmentChainPlanner *p;
    ~ActiveGuard() { p->setTransitionActive(false); }
  } guard{this};
  setTransitionActive(true);

  const auto fail = [&](PlanReason why, const std::string &msg) {
    log_->errorf("[S13] transition mission FAILED: %s", msg.c_str());
    return PlanResult::failedBecause(why, msg);
  };
  const auto *dyn = pm_->dynamicsParams();
  if (!dyn || !mmp_vehicle_dynamics::parametersAreValid(*dyn))
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "no valid assumption parameter set");
  double um = 100.0, uz = 100.0;
  if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
    um = node_->get_parameter("optimization/dynamics_unit_xy_m").as_double();
  if (node_->has_parameter("optimization/dynamics_unit_z_m"))
    uz = node_->get_parameter("optimization/dynamics_unit_z_m").as_double();
  if (um <= 1e-9 || uz <= 1e-9)
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "degenerate dynamics unit scale");
  bool run_parallel = false;
  if (!node_->has_parameter("chain/parallel"))
    node_->declare_parameter("chain/parallel", false);
  node_->get_parameter("chain/parallel", run_parallel);

  // [1] route commit — one front-end pass from the mission start.
  std::vector<Eigen::Vector3d> route;
  std::vector<double> cap;
  double fe_ms = 0.0;
  if (!commitRoute(head, waypoints, run_parallel,
                   &route, &cap, &fe_ms))
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "route commit failed");

  // [2] zone policy snapshot — fail-closed precondition. INVALID (pass 0,
  // stale data, multi-leg search) means the transition cannot be judged.
  const auto snap = pm_->zonePolicySnapshot();
  if (!snap.valid)
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "zone policy snapshot invalid — transition cannot be "
                "judged against the committed policy");

  // [3] entry screening — the SAME frozen [PHASE] predicates plus the
  // speed-change arc a cruise-to-cruise screen never needed.
  const double cruise = pm_->maxVel();
  double tan_grade_max = std::numeric_limits<double>::infinity();
  double turn_radius_u = 0.0, accel_arc_u = 0.0;
  {
    const double v_c = cruise * um;
    const double q =
        0.5 * mmp_vehicle_dynamics::airDensity(*dyn, 0.0) * v_c * v_c;
    const double W = dyn->mass_kg * dyn->gravity_mps2;
    const double CL = W / std::max(1e-9, q * dyn->wing_area_m2);
    const double D = (dyn->zero_lift_drag_coefficient +
                      dyn->induced_drag_factor * CL * CL) *
                     q * dyn->wing_area_m2;
    const double sg = std::max(
        0.0,
        std::min(1.0, (dyn->thrust_max_n * (1.0 - dyn->constraint_margin) -
                       D) /
                          W));
    tan_grade_max = std::min(std::tan(dyn->flight_path_angle_max_rad),
                             std::tan(std::asin(sg)));
    const Eigen::Vector3d v0_si(start_vel.x() * um, start_vel.y() * um,
                                start_vel.z() * uz);
    const double vm = std::max(1e-3, v0_si.norm());
    const double nmax = std::max(1.01, dyn->load_factor_max);
    turn_radius_u = 1.5 * vm * vm /
                    (dyn->gravity_mps2 * std::sqrt(nmax * nmax - 1.0)) / um;
    const double a_avail = std::max(
        0.5, (dyn->thrust_max_n * (1.0 - dyn->constraint_margin) - D) /
                 dyn->mass_kg);
    accel_arc_u =
        std::max(0.0, (v_c * v_c - vm * vm) / (2.0 * a_avail)) / um;
  }
  ReachLimits rl;
  rl.dep_min_arc_u = readNumParam(node_, "chain/phase/depart_min_arc_u", 30.0);
  rl.calm_window_u = readNumParam(node_, "chain/phase/calm_window_u", 40.0);
  rl.kappa_calm = readNumParam(node_, "chain/phase/kappa_calm", 0.01);
  rl.grade_max =
      readNumParam(node_, "chain/phase/grade_frac", 0.5) * tan_grade_max;
  rl.tan_grade_max = tan_grade_max;
  rl.turn_radius_u = turn_radius_u;
  rl.accel_arc_u = accel_arc_u;
  rl.max_candidates = 3;
  const HandoffScreenResult hs = screenHandoffCandidates(
      route, start_vel, rl,
      readNumParam(node_, "chain/phase/arrive_min_arc_u", 30.0));
  if (hs.dep_candidates.empty())
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "no reachable cruise entry candidate on the committed "
                "route");
  const std::vector<double> arc = routeArcTable(route);
  std::vector<tp::EntryCandidate> cands;
  for (int idx : hs.dep_candidates) {
    tp::EntryCandidate e;
    e.route_vertex = idx;
    e.route_start_s = arc[static_cast<size_t>(idx)];
    const Eigen::Vector3d &p = route[static_cast<size_t>(idx)];
    e.pos_m = Eigen::Vector3d(p.x() * um, p.y() * um, p.z() * uz);
    const Eigen::Vector3d d_u =
        route[static_cast<size_t>(std::min<int>(idx + 1,
                                                (int)route.size() - 1))] -
        route[static_cast<size_t>(std::max(idx - 1, 0))];
    Eigen::Vector3d d_si(d_u.x() * um, d_u.y() * um, d_u.z() * uz);
    if (d_si.norm() < 1e-9) d_si = Eigen::Vector3d::UnitX();
    e.tangent = d_si.normalized();
    cands.push_back(e);
  }

  // [4] generation — pure component; closures compose the snapshot
  // policy, terrain and the handoff envelope (SI <-> planner units here,
  // never inside the component).
  tp::TransitionRequest req;
  req.initial_pos_m =
      Eigen::Vector3d(start_pos.x() * um, start_pos.y() * um,
                      start_pos.z() * uz);
  req.initial_vel_mps =
      Eigen::Vector3d(start_vel.x() * um, start_vel.y() * um,
                      start_vel.z() * uz);
  req.initial_acc_mps2 =
      Eigen::Vector3d(start_acc.x() * um, start_acc.y() * um,
                      start_acc.z() * uz);
  // The message contract's use_initial_acceleration bool, plumbed all the
  // way — the VALUE never decides prescription (review find: a numeric-0
  // sentinel cannot express "prescribed exactly zero").
  req.initial_acc_prescribed = start_acc_commanded;
  req.entry_candidates = cands;
  req.limits.dyn = *dyn;
  req.limits.unit_xy_m = um;
  req.limits.unit_z_m = uz;
  req.limits.min_agl_m = pm_->minGoalAgl() * uz;
  req.limits.end_speed_min_mps =
      dyn->speed_min_mps * (1.0 + dyn->constraint_margin);
  // The SAME ceiling the envelope validator judges by — computed in one
  // place, or the generator captures end speeds the judge refuses.
  req.limits.end_speed_max_mps =
      std::min(dyn->speed_max_mps, pm_->effectiveHandoffMaxMps());
  const auto toU = [um, uz](const Eigen::Vector3d &p_m) {
    return Eigen::Vector3d(p_m.x() / um, p_m.y() / um, p_m.z() / uz);
  };
  req.zone_probe = [this, &snap, toU](const Eigen::Vector3d &p_m) {
    const Eigen::Vector3d p_u = toU(p_m);
    for (size_t i = 0; i < snap.zones.size(); ++i) {
      switch (pm_->zoneContact(snap, i, p_u)) {
        case PathManager::ZoneContactResult::CLEAR:
          break;
        case PathManager::ZoneContactResult::CONTACT:
          // Policy lives in the disposition: only HARD_AVOID contact
          // disqualifies; soft crossings the global 3-pass chose (or
          // endpoint exemptions) stay traversable, exposure measured.
          if (snap.zones[i].disposition ==
              PathManager::ZoneDisposition::HARD_AVOID)
            return pm_->zoneContactAuthored(snap, i, p_u)
                       ? tp::ZoneProbe::CONTACT_AUTHORED
                       : tp::ZoneProbe::CONTACT_STANDOFF;
          break;
        case PathManager::ZoneContactResult::STALE:
        case PathManager::ZoneContactResult::INVALID:
          return tp::ZoneProbe::STALE_OR_INVALID;
      }
    }
    return tp::ZoneProbe::CLEAR;
  };
  req.zone_exposure_raw = [this, &snap, toU](const Eigen::Vector3d &p_m) {
    const Eigen::Vector3d p_u = toU(p_m);
    double sum = 0.0;
    for (size_t i = 0; i < snap.zones.size(); ++i) {
      double e = 0.0;
      if (pm_->zoneExposureRaw(snap, i, p_u, &e)) sum += e;
    }
    return sum;
  };
  if (pm_->hasTerrainData()) {
    req.terrain_z = [this, um, uz](double x_m, double y_m, double *elev_m) {
      double elev_u = 0.0;
      if (!pm_->terrainElevation(x_m / um, y_m / um, &elev_u)) return false;
      if (elev_m) *elev_m = elev_u * uz;
      return true;
    };
  }  // else: null closure — the component refuses when an AGL floor is
     // required without a terrain SOURCE (fail-closed precondition).
  req.pva_problem = [this, toU, um, uz](const Eigen::Vector3d &p_m,
                                        const Eigen::Vector3d &v_mps,
                                        const Eigen::Vector3d &a_mps2) {
    return pm_->pvaEnvelopeProblem(
        toU(p_m),
        Eigen::Vector3d(v_mps.x() / um, v_mps.y() / um, v_mps.z() / uz),
        Eigen::Vector3d(a_mps2.x() / um, a_mps2.y() / um,
                        a_mps2.z() / uz));
  };

  const tp::TransitionResult tr = tp::generate(req);
  log_->infof(
      "[S13] transition audit: enumerated %d, winner %d | disq fin %d "
      "pre %d rep %d sat %d ter %d zone %d limits %d time %d pva %d "
      "adapter %d | dwell %.2f s, winner risk max %.3g int %.3g (search "
      "%.3g/%.3g), adapter err p/v/a %.3g/%.3g/%.3g, start acc: "
      "repro err %.3g / model %.3g m/s^2",
      tr.audit.candidates_enumerated, tr.audit.winner_primitive_id,
      tr.audit.disq_finiteness, tr.audit.disq_preguard,
      tr.audit.disq_representable, tr.audit.disq_saturated,
      tr.audit.disq_terrain, tr.audit.disq_zone, tr.audit.disq_limits,
      tr.audit.disq_timeout, tr.audit.disq_end_pva, tr.audit.disq_adapter,
      tr.audit.dwell_achieved_s, tr.audit.risk_max, tr.audit.risk_integral,
      tr.audit.search_risk_max, tr.audit.search_risk_integral,
      tr.audit.adapter_max_pos_err_m, tr.audit.adapter_max_vel_err_mps,
      tr.audit.adapter_max_acc_err_mps2, tr.audit.start_acc_repro_err_mps2,
      tr.audit.start_acc_model_mps2);
  if (!tr.ok)
    return fail(tr.any_candidate_reached_adapter
                    ? PlanReason::TRANSITION_ADAPTER_UNSOUND
                    : PlanReason::TRANSITION_GENERATION_FAILED,
                tr.reason);

  // Coordinator-side re-check of the returned end PVA (belt and braces:
  // the component gated it through the same closure, but the handoff is
  // THIS seam's contract).
  const Eigen::Vector3d end_pos_u = toU(tr.end_pos_m);
  const Eigen::Vector3d end_vel_u(tr.end_vel_mps.x() / um,
                                  tr.end_vel_mps.y() / um,
                                  tr.end_vel_mps.z() / uz);
  const Eigen::Vector3d end_acc_u(tr.end_acc_mps2.x() / um,
                                  tr.end_acc_mps2.y() / um,
                                  tr.end_acc_mps2.z() / uz);
  const std::string end_prob =
      pm_->pvaEnvelopeProblem(end_pos_u, end_vel_u, end_acc_u);
  if (!end_prob.empty())
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "transition end PVA failed the handoff re-check: " +
                    end_prob);

  // [5] cut — materialize the sub-route ONCE; route_start_s never leaks
  // further downstream.
  std::vector<Eigen::Vector3d> sub_route;
  std::vector<double> sub_cap;
  if (!cutAtArc(route, cap, tr.route_start_s, &sub_route, &sub_cap))
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "route cut at the entry arc failed");

  // [6] chain over the remainder with the transition prefix: junction
  // gate, leading TRANSITION span, full-flight judgment, storage and viz
  // all live inside planOverRoute.
  TransitionPrefix prefix;
  prefix.traj = tr.traj;
  prefix.junction_pva_tol_u = 1e-6;
  // [HEAD-POLICY] The cruise planner's head is the transition arc's END
  // state. TRANSITION_HANDOFF says so, and applyHeadPolicy refuses to
  // re-aim or floor it BECAUSE OF THAT — it already passed the handoff gate
  // and its terminal PVA was validated by the generator. Previously this
  // was expressed as "the caller holds a transition pointer", which is the
  // same fact stored twice in two places that could disagree.
  StartHead handoff;
  handoff.src = StartStateSource::TRANSITION_HANDOFF;
  handoff.pos_u = end_pos_u;
  handoff.vel_u = end_vel_u;
  handoff.acc_u = end_acc_u;
  handoff.acc_prescribed = true;   // the generator produced a full PVA
  return planOverRoute(sub_route, sub_cap, fe_ms, handoff, waypoints,
                       run_parallel, mission_tail, &prefix);
}

PlanResult SegmentChainPlanner::planRouteParallel(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    bool run_parallel, const ego_planner::TailBoundary &mission_tail)
{
  std::vector<Eigen::Vector3d> route;
  std::vector<double> cap;
  double fe_ms = 0.0;
  if (!commitRoute(head, waypoints, run_parallel,
                   &route, &cap, &fe_ms))
    return PlanResult::failed("front-end failed");
  return planOverRoute(route, cap, fe_ms, head, waypoints, run_parallel,
                       mission_tail);
}

// [S13] Stage 1: the ONE front-end pass that commits the global route
// (AGL/bbox/SDF/zone binding happen here exactly once — route decided once,
// the r3 homotopy lesson).
bool SegmentChainPlanner::commitRoute(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    bool run_parallel, std::vector<Eigen::Vector3d> *route,
    std::vector<double> *cap, double *fe_ms)
{
  const Eigen::Vector3d &start_pos = head.pos_u;
  const Eigen::Vector3d &start_vel = head.vel_u;
  const Eigen::Vector3d &start_acc = head.acc_u;
  const auto t0 = std::chrono::steady_clock::now();
  if (auto_segments_)
    log_->infof("[CHAIN-PAR] route-%s plan, auto-sized segments "
                "(no baseline)", run_parallel ? "parallel" : "sequential");
  else
    log_->infof("[CHAIN-PAR] route-%s plan, %d segments (no baseline)",
                run_parallel ? "parallel" : "sequential", segments_);
  if (!pm_->planGlobalTraj(head, waypoints, ego_planner::TailBoundary{},
                           false, nullptr, nullptr,
                           /*front_end_only=*/true)) {
    log_->errorf("[CHAIN-PAR] front-end failed — nothing to author on");
    return false;
  }
  *route = pm_->lastCommittedRoute();
  *cap = pm_->lastCommittedCapRef();
  if (fe_ms)
    *fe_ms = std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - t0)
                 .count();
  return true;
}

// [S13] Stage 2: author + solve + merge + stitch over an ALREADY COMMITTED
// route (or a slice of one). The head PVA is whatever the caller hands in —
// today the mission start, under contract 2 the transition end state.
PlanResult SegmentChainPlanner::planOverRoute(
    const std::vector<Eigen::Vector3d> &route, const std::vector<double> &cap,
    double fe_ms, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    bool run_parallel,
    const ego_planner::TailBoundary &mission_tail,
    const TransitionPrefix *transition)
{
  const Eigen::Vector3d &start_pos = head.pos_u;
  const Eigen::Vector3d &start_vel = head.vel_u;
  const Eigen::Vector3d &start_acc = head.acc_u;
  const bool start_vel_synthesized =
      head.src == StartStateSource::STATED_SPEED;
  const bool start_acc_commanded = head.acc_prescribed;
  (void)start_vel_synthesized;
  // [S13] Fail-closed INPUT contract (review find): once a coordinator can
  // hand this function an external route slice and head PVA, a mismatch
  // must be a FAILED plan, not a silent degradation — sliceCommittedRoute
  // in particular quietly abandons the altitude cap when sizes disagree,
  // which on a transition slice would judge the solve under the wrong
  // altitude regime. Head/route TANGENT alignment is deliberately NOT
  // required: a head velocity pointing away from the route is a legitimate
  // departure case (the connector's whole job).
  {
    const char *bad = nullptr;
    if (route.size() < 2) {
      bad = "route has fewer than 2 vertices";
    } else if (cap.size() != route.size()) {
      bad = "cap size does not match route size (silent cap abandonment "
            "downstream)";
    } else if ((route.front() - start_pos).norm() > 1e-2) {
      bad = "head position does not sit on the route start";
    } else {
      for (const auto &v : route)
        if (!v.allFinite()) { bad = "non-finite route vertex"; break; }
      if (!bad)
        for (double c : cap)
          if (!std::isfinite(c)) { bad = "non-finite cap value"; break; }
      if (!bad && (!start_pos.allFinite() || !start_vel.allFinite() ||
                   !start_acc.allFinite()))
        bad = "non-finite head PVA";
    }
    if (bad) {
      log_->errorf("[CHAIN-PAR] planOverRoute input contract violated: %s",
                   bad);
      return PlanResult::failed(std::string("planOverRoute input: ") + bad);
    }
  }
  // [S13] RE-ENTRY safety, targeted at what actually leaks: the transition
  // coordinator retries planOverRoute with a different entry candidate, and
  // between two calls the ONLY plan-scoped residue is (a) the phase
  // blackboard this function writes and (b) segments_, which the merge
  // ladder decrements mid-plan. Both are scoped here — the blackboard reset
  // now, segments_ restored on every exit — instead of a wholesale state
  // refactor nothing else needs (plan() already resets at its own entry).
  phase_applied_ = false;
  phase_note_.clear();
  dep_candidates_.clear();
  arr_candidates_.clear();
  phase_tan_grade_ = 1e9;
  phase_turn_radius_u_ = 0.0;
  struct SegmentsRestore {
    int &ref;
    int saved;
    ~SegmentsRestore() { ref = saved; }
  } segments_restore{segments_, segments_};
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
    // [S13] With a transition ACTIVE the single-shot fallback is forbidden
    // outright (contract §10/§13): it would re-plan FROM THE MISSION START
    // with the cruise model — re-planning the very regime the transition
    // exists to handle, and duplicating the transition span. Installed
    // BEFORE any coordinator exists so the guard cannot be forgotten when
    // one arrives.
    if (transition_active_) {
      log_->errorf("[CHAIN-PAR] %s — single-shot fallback FORBIDDEN while a "
                   "transition is active", why);
      return PlanResult::failed(
          std::string(why) +
          "; single-shot fallback is forbidden while a transition is active");
    }
    log_->warnf("[CHAIN-PAR] %s — falling back to the single-shot plan", why);
    // [PHASE] Hard-tagged requirements survive the fallback CONSERVATIVELY
    // (min across all segments applied mission-wide). Frozen doctrine:
    // preserved -> DEGRADED, lost -> FAILED — a fallback that quietly
    // dropped an explicit hard limit would lie exactly like the old tail
    // sentinel did.
    std::map<std::string, double> hardmin;
    const auto collect = [&](const SegmentOverrides &e) {
      for (const auto &pp : e.params) {
        const OvrKey *k = whitelistFind(pp.get_name());
        if (!k || k->tag != OvrTag::kHardMax) continue;
        const double v = pp.as_double();
        auto it = hardmin.find(pp.get_name());
        if (it == hardmin.end() || v < it->second)
          hardmin[pp.get_name()] = v;
      }
    };
    if (!eff.empty()) {
      for (const auto &e : eff) collect(e);
    } else {
      // [PHASE] The EARLY fallbacks (front-end failure, auto-N below the
      // split threshold) run before eff is built, so scanning eff alone
      // preserved nothing and the direct plan quietly flew past an operator's
      // hard limit — the exact lie this preservation exists to prevent
      // (review find). Read the declared profiles directly instead.
      collect(readOverrideList("chain/phase/departure/params",
                               "phase-departure"));
      collect(readOverrideList("chain/phase/cruise/params", "phase-cruise"));
      collect(readOverrideList("chain/phase/arrival/params", "phase-arrival"));
      for (const auto &so : readSegmentOverrides()) collect(so);
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
    const bool fb_ok = pm_->planGlobalTraj(head, waypoints, mission_tail);
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
    // [ENVELOPE] Contract 1 (2026-08-08): a phase-mode direct fallback is
    // the one product that never meets a whole-flight audit (the stitched
    // chain gets [FINAL-EVAL]; this path returns before it). Within-envelope
    // inputs can still produce an unflyable direct solve — the worm's honest
    // sibling — so the SAME fitness evaluation gates it: pass -> DEGRADED as
    // before, fail -> FAILED(DIRECT_FALLBACK_UNSAFE). "No phase labels"
    // must never mean "no gate".
    // UNCONDITIONAL. This gate used to run only with chain/phase/enable on,
    // so with the flag off a direct product was delivered with no
    // [ZONE-AUDIT], no refusal and no [PLAN-MODE] line at all. Phase labels
    // decide what the spans are CALLED, never whether the flight is judged,
    // and a safety gate behind a feature flag is one config edit from
    // absent.
    // CORRECTION to the commit that made this change: it claimed the A/B ran
    // with phase off and had therefore been measuring ungated direct
    // products. That was wrong — optimizer_params.yaml:27 has
    // chain/phase/enable: true and the harness does not override it, so the
    // gate was live in all 176 archived rows. The change stands on its own
    // reasoning; the motivation given for it did not.
    log_->infof("[PLAN-MODE] direct");
    FlightVerdict direct_verdict{};
    {
      // The LOCAL slot carries the optimized trajectory that is actually
      // flown; the global slot is the pre-L-BFGS seed (path_manager.cpp
      // stores out_global there as the comparison channel). Judging the
      // seed would gate on a trajectory nobody flies — review find.
      const poly_traj::Trajectory &fly = pm_->traj_.local_traj.traj;
      const FlightVerdict fv = evaluateFlight(
          fly, {{fly.getTotalDuration(), PhaseKind::CRUISE, "direct"}});
      direct_verdict = fv;
      // Fail-CLOSED (review find): a flight the evaluator could not judge
      // is as unflyable as one it condemned — "no verdict" must never read
      // as "clean" for a product that skipped every whole-flight audit.
      // unflyable() adds the hard-zone and unmeasurable-policy conditions,
      // which !clean alone does not carry to a refusal.
      if (!fv.evaluated || fv.unflyable() || !fv.clean) {
        char why2[192];
        if (!fv.evaluated) {
          snprintf(why2, sizeof why2,
                   "direct fallback could not be evaluated by the flight "
                   "fitness gate (degenerate product)");
        } else {
          snprintf(why2, sizeof why2,
                   "direct fallback failed the flight fitness gate (%s%s%s%senv "
                   "viol %.1f%%, peak %.1f%%)",
                   fv.underground ? "terrain overlap, " : "",
                   fv.no_cruise ? "never reaches cruise, " : "",
                   fv.zone_hard_n > 0 ? "authored zone entered, " : "",
                   !fv.policy_measurable ? "zone policy unevaluated, " : "",
                   fv.viol_pct, 100.0 * fv.util_peak);
        }
        log_->errorf("[ENVELOPE] %s — FAILED (DIRECT_FALLBACK_UNSAFE)", why2);
        invalidateStoredTrajectory();
        return PlanResult::failedBecause(
            PlanReason::DIRECT_FALLBACK_UNSAFE,
            std::string(why) + "; " + why2);
      }
    }
    PlanResult r = PlanResult::success();
    if (as_degraded) r.degrade(PlanReason::SINGLE_PLAN_FALLBACK, why);
    if (!hardmin.empty())
      r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
                "hard limits preserved conservatively in the single plan");
    // The fourth caller the helper's own comment promised and did not have.
    // Without it the same trajectory was DEGRADED when the chain delivered
    // it and SUCCESS when the direct path did — the standoff shell went
    // unreported on 21 of the archived 176 runs' worth of direct products.
    degradeForVerdict(&r, direct_verdict, "direct fallback");
    return r;
  };

  const bool phase_requested =
      readNumParam(node_, "chain/phase/enable", 0.0) != 0.0;
  if (!resolveAutoSegments(static_cast<int>(route.size()) - 1, "route"))
    // Direct plan on a too-small mission is the CORRECT answer — except
    // when phase semantics were requested and cannot be delivered.
    return fallback(phase_requested
                        ? "auto-N below threshold (phase mode: direct)"
                        : "auto-N: mission below the split threshold",
                    /*as_degraded=*/phase_requested);
  // The restore target is the CONFIRMED N — auto resolution just decided
  // it (review find: saving before this point made the RAII revert an
  // auto-resolved N=2 back to the entry value after a successful plan,
  // breaking segments() for every route+auto combination). A mid-plan
  // merge decrement still restores to this confirmed value.
  segments_restore.saved = segments_;

  // [HEAD-POLICY] resolved BEFORE authoring: the departure handoff
  // screening measures the turn the EFFECTIVE start velocity needs. The
  // rule itself lives in start_state.h — this block used to re-implement
  // both [VEL-ALIGN] and [STALL-FLOOR], re-deriving the floor arithmetic by
  // hand instead of calling the accessor, and the two copies had drifted:
  // the numeric-equality rule at the speed boundaries reached the other one
  // and not this. A transition-prescribed head is exempt STRUCTURALLY — it
  // already passed the handoff gate — and that exemption is now a SOURCE
  // rather than a local pointer test.
  StartHead head0;
  head0.src = transition != nullptr
                  ? StartStateSource::TRANSITION_HANDOFF
                  : (start_vel_synthesized ? StartStateSource::STATED_SPEED
                                           : StartStateSource::CHAIN_JUNCTION);
  head0.pos_u = start_pos;
  head0.vel_u = start_vel;
  head0.acc_u = start_acc;
  // The real value, not a hardcoded false: a prescribed acceleration pins
  // the frame the direction lives in, so it must stop the re-aim. Hardcoding
  // false here let a STATED_SPEED head with a prescribed acceleration be
  // re-aimed out from under it.
  head0.acc_prescribed = start_acc_commanded;
  double um_head = 100.0;
  if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
    um_head = node_->get_parameter("optimization/dynamics_unit_xy_m")
                  .as_double();
  // No pointer test. TRANSITION_HANDOFF is neither re-aimed nor floored
  // BECAUSE OF WHAT IT IS, and applyHeadPolicy knows that from the source —
  // passing floor_u = 0 to suppress the rule encoded the same fact a second
  // time, in a place that could disagree with the first.
  const double floor_head = pm_->cruiseFloorUnits();
  const double eps_head =
      um_head > 1e-9 ? PathManager::kSpeedBoundaryEpsMps / um_head : 0.0;
  const HeadPolicy hp0 = applyHeadPolicy(
      head0, route, floor_head, pm_->alignStartVelToRoute(), eps_head);
  Eigen::Vector3d v0 = hp0.vel_u;
  if (hp0.floored)
    log_->warnf("[CHAIN-PAR] start speed %.3f below the margin-backed cruise "
                "floor %.3f — raised (same contract as [STALL-FLOOR])",
                start_vel.norm(), hp0.floor_u);
  if (hp0.floor_declined)
    log_->infof("[HEAD-POLICY] head speed %.3f u/s is below the "
                "margin-backed cruise floor %.3f u/s and was KEPT — source "
                "%s may not be rewritten",
                v0.norm(), hp0.floor_u, sourceName(head0.src));

  // === 2. author contracts + slices on the route ===
  std::vector<Contract> contracts;
  if (!authorContractsFromRoute(route, v0, &contracts))
    return fallback("route too small to author junctions on");
  applyContractJitter(route, &contracts);
  std::vector<RouteSlice> slices =
      sliceCommittedRoute(route, cap, contracts);
  if (slices.size() != static_cast<size_t>(segments_))
    return fallback("committed route did not slice cleanly");

  const double author_ms = ms_since(t_wall);

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
      //
      // [PHASE-N2] Stated policy, not an accident of an empty list: at N=2
      // ONE junction serves both handoffs, dual-screened in both directions.
      // Moving it for the arrival would silently invalidate the departure
      // screening that also accepted it, so the arrival edge has NO retry
      // here — it goes direct with an honest note. (Before this was written
      // down, arr_candidates_ was simply empty at N=2 and the ladder fell
      // through reporting "no reachable arrival handoff", which reads as a
      // screening failure rather than a policy.)
      bool edge_rescued = false;
      if (phase_applied_ && segments_ == 2 && f == segments_ - 1) {
        phase_applied_ = false;
        log_->warnf("[PHASE-N2] arrival segment failed at N=2: the single "
                    "shared handoff serves BOTH phases, so moving it is not "
                    "available — direct plan, no phase labels");
        PlanResult r = fallback("N=2 shared handoff admits no arrival retry");
        if (r.hasTrajectory())
          r.degrade(PlanReason::PHASE_BOUNDARY_FALLBACK,
                    "N=2 shared handoff admits no arrival retry — direct "
                    "plan, no phase labels");
        return r;
      }
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
            // The connector is not a route slice, so it has no cap of its
            // own — sample one off the route (review find: an empty cap
            // silently demoted this solve to the scalar altitude band while
            // every route-seeded solve used the arc-varying one).
            edge_cap = capAlongRoute(route, cap, edge_path, c);
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

  std::vector<PhaseSpan> spans;
  double acc_t = 0.0;
  for (size_t i = 0; i < runs.size(); ++i) {
    acc_t += runs[i].getTotalDuration();
    // All chained spans are CRUISE-judged today; the transition coordinator
    // will prepend a TRANSITION span. Kind is the judgment contract, the
    // name is display only.
    spans.push_back({acc_t, PhaseKind::CRUISE,
                     phase_applied_
                         ? (i == 0 ? std::string("departure")
                            : i + 1 == runs.size()
                                ? std::string("arrival")
                                : "cruise-" + std::to_string(i))
                         : "seg" + std::to_string(i + 1)});
  }
  const double t_pre_terminal = chained.getTotalDuration();
  const poly_traj::Trajectory term =
      appendTerminalPhase(&chained, mission_tail.anyPrescribed());
  if (chained.getTotalDuration() > t_pre_terminal + 1e-9) {
    spans.push_back(
        {chained.getTotalDuration(), PhaseKind::TERMINAL, "terminal"});
  }

  if (transition != nullptr) {
    // [S13] G5b C2 junction: the solved chain head must sit EXACTLY on
    // the transition tail (planner units). The seam jerk jump is
    // MEASURED and logged only — its cap is a section-8 number that must
    // not be self-calibrated from generated output.
    poly_traj::Trajectory tt = transition->traj;
    const double Tt = tt.getTotalDuration();
    const double dP = (tt.getPos(Tt) - chained.getPos(0.0)).norm();
    const double dV = (tt.getVel(Tt) - chained.getVel(0.0)).norm();
    const double dA = (tt.getAcc(Tt) - chained.getAcc(0.0)).norm();
    const double djerk = (tt.getJer(Tt) - chained.getJer(0.0)).norm();
    log_->infof("[S13] junction seam: dP %.3g dV %.3g dA %.3g u (tol %.3g)"
                ", jerk jump %.3g u/s^3 (measured, ungated)",
                dP, dV, dA, transition->junction_pva_tol_u, djerk);
    if (!(dP <= transition->junction_pva_tol_u &&
          dV <= transition->junction_pva_tol_u &&
          dA <= transition->junction_pva_tol_u)) {
      invalidateStoredTrajectory();
      return PlanResult::failedBecause(
          PlanReason::TRANSITION_JUNCTION_UNSOUND,
          "chain head is not on the transition tail");
    }
    // Prepend: the FULL flight is judged, stored and visualized — with a
    // leading TRANSITION span (measured, excluded from cruise envelope
    // statistics; gates deferred by the anti-circularity doctrine).
    poly_traj::Trajectory full = transition->traj;
    full.append(chained);
    std::vector<PhaseSpan> full_spans;
    full_spans.push_back({Tt, PhaseKind::TRANSITION, "transition"});
    for (PhaseSpan sp : spans) {
      sp.t_end += Tt;
      full_spans.push_back(sp);
    }
    chained = full;
    spans = full_spans;
  }

  // [STITCH-GATE] JUDGE BEFORE STORING. The verdict has to precede
  // setLocalTraj and the viz publish: a refused flight left in traj_ is a
  // trajectory the next state transition can pick up, and one already drawn
  // in RViz tells the operator it was accepted. FAILED leaves both untouched.
  last_spans_ = spans;
  const FlightVerdict stitched_fv =
      evaluateFlight(chained, spans);
  if (!stitched_fv.evaluated || stitched_fv.unflyable())
    return stitchedVerdictResult(stitched_fv, PlanResult::success());

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

  std::string per;
  for (size_t i = 0; i < solve_ms.size(); ++i) {
    char b[32];
    snprintf(b, sizeof b, "%s%.0f", i ? "/" : "", solve_ms[i]);
    per += b;
  }
  log_->infof("[PLAN-MODE] chain");
  log_->infof("[CHAIN-PAR] %s: front-end %.0f ms + author %.0f ms + solves "
              "[%s] ms (wall %.0f, max %.0f) => TOTAL %.0f ms | %d pieces, "
              "%.1f s flight",
              run_parallel ? "PARALLEL" : "sequential", fe_ms, author_ms,
              per.c_str(), solve_wall_ms,
              *std::max_element(solve_ms.begin(), solve_ms.end()),
              fe_ms + ms_since(t_wall), chained.getPieceNum(),
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
  return stitchedVerdictResult(stitched_fv, result);
}

PlanResult SegmentChainPlanner::stitchedVerdictResult(
    const FlightVerdict &fv, PlanResult ok_result) const
{
  // Fail-closed on UNEVALUATED (review find: unflyable() is false when
  // evaluated is false, so a malformed span set slid a stitched product
  // through as SUCCESS with the trajectory already stored): a flight the
  // evaluator could not judge is refused like one it condemned, and
  // whatever was stored is invalidated.
  if (!fv.evaluated) {
    log_->errorf("[STITCH-GATE] whole-flight evaluation did not run "
                 "(malformed spans or degenerate flight) — FAILED");
    invalidateStoredTrajectory();
    return PlanResult::failedBecause(
        PlanReason::STITCHED_FLIGHT_UNSAFE,
        "stitched flight could not be evaluated (span contract violation "
        "or degenerate product)");
  }
  // [STITCH-GATE] Frozen policy for the whole-flight verdict on the product
  // the caller is about to fly. Two tiers, because the two failure kinds are
  // not the same kind of thing:
  //   terrain overlap / never reaches cruise -> FAILED. There is no margin
  //     to spend here; the trajectory passes through the ground or the
  //     airframe cannot sustain the flight at all.
  //   envelope over-utilization -> DEGRADED. A limit budget is something an
  //     operator can knowingly exceed, and every span already passed its own
  //     per-solve audit; the stitched reading is the honest whole-flight
  //     number, reported loudly rather than silently discarded (which is
  //     what happened before this gate existed).
  // A degenerate product that could not be judged keeps its result: unlike
  // the direct fallback (a repair, judged fail-closed), this trajectory
  // already cleared the per-solve audits that produced it.
  if (fv.unflyable()) {
    char why[176];
    snprintf(why, sizeof why,
             "stitched flight is unflyable (%s%s%s%s) — whole-flight "
             "evaluation, which no per-solve audit performs",
             fv.underground ? "terrain overlap; " : "",
             fv.no_cruise ? "never reaches cruise; " : "",
             fv.zone_hard_n > 0 ? "authored zone entered; " : "",
             !fv.policy_measurable ? "zone policy unevaluated" : "");
    log_->errorf("[STITCH-GATE] %s", why);
    // Nothing stored on this path today, but the guarantee has to hold for
    // every caller — an earlier stage (baseline solve, direct fallback) may
    // already have left a trajectory in traj_.
    invalidateStoredTrajectory();
    return PlanResult::failedBecause(PlanReason::STITCHED_FLIGHT_UNSAFE, why);
  }
  if (fv.evaluated && fv.envelope_bad) {
    char why[176];
    snprintf(why, sizeof why,
             "stitched flight exceeds the envelope budget (viol %.1f%%, peak "
             "%.1f%%) — flyable, but the whole-flight reading is worse than "
             "any per-solve audit saw",
             fv.viol_pct, 100.0 * fv.util_peak);
    log_->warnf("[STITCH-GATE] %s", why);
    ok_result.degrade(PlanReason::STITCHED_ENVELOPE_BUDGET, why);
  }
  // Reported as its own thing, never folded into the envelope reason: the
  // caller has to be able to tell "hotter than requested" from "closer to a
  // zone than the route planner wanted", and only one of those is about the
  // airframe.
  if (fv.evaluated && fv.zone_standoff_n > 0) {
    char why[176];
    snprintf(why, sizeof why,
             "flight entered the routing standoff shell around a HARD_AVOID "
             "zone (%d samples) without entering the authored volume",
             fv.zone_standoff_n);
    log_->warnf("[STITCH-GATE] %s", why);
    ok_result.degrade(PlanReason::STITCHED_ZONE_STANDOFF, why);
  }
  return ok_result;
}

SegmentChainPlanner::FlightVerdict SegmentChainPlanner::evaluateFlight(
    const poly_traj::Trajectory &flight,
    const std::vector<PhaseSpan> &spans) const
{
  // Clear FIRST, so an early return can never leave the PREVIOUS flight's
  // verdict readable through lastFlightVerdict(). A regression that asserts
  // on a stale verdict is worse than one that has none: it reports on a
  // flight that is not the one under test.
  last_verdict_ = FlightVerdict{};
  const double T = flight.getTotalDuration();
  if (T <= 1e-9 || spans.empty()) return {};
  // [S13] Span input contract, fail-closed to UNEVALUATED (the direct
  // fallback gate treats that as FAILED; the stitched paths build their
  // own spans, so a violation here is a programming error to surface):
  //  - every t_end finite and strictly increasing
  //  - the last t_end covers the whole flight
  //  - TRANSITION only as the LEADING contiguous spans (a transition in
  //    the middle of a cruise flight is not a thing this contract knows)
  //  - TERMINAL only as the last span
  {
    double prev = 0.0;
    bool cruise_seen = false;
    for (size_t i = 0; i < spans.size(); ++i) {
      const PhaseSpan &sp = spans[i];
      if (!std::isfinite(sp.t_end) || sp.t_end <= prev + 1e-9) {
        log_->errorf("[FINAL-EVAL] span contract: t_end not finite/strictly "
                     "increasing at span %zu", i);
        return {};
      }
      prev = sp.t_end;
      if (sp.kind == PhaseKind::TRANSITION) {
        if (cruise_seen) {
          log_->errorf("[FINAL-EVAL] span contract: TRANSITION after a "
                       "non-transition span (%zu)", i);
          return {};
        }
      } else {
        cruise_seen = true;
      }
      if (sp.kind == PhaseKind::TERMINAL && i + 1 != spans.size()) {
        log_->errorf("[FINAL-EVAL] span contract: TERMINAL not last (%zu)",
                     i);
        return {};
      }
    }
    if (std::abs(spans.back().t_end - T) > 1e-6) {
      log_->errorf("[FINAL-EVAL] span contract: last t_end %.6f != flight "
                   "duration %.6f", spans.back().t_end, T);
      return {};
    }
  }

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
    // Zone contact of the delivered flight. Containment alone is NOT the
    // question — policy lives in the per-zone DISPOSITION, exactly as the
    // transition candidate filter reads it: only HARD_AVOID contact is a
    // violation, while SOFT_UNAVOIDABLE / SOFT_ENDPOINT / SOFT_FALLBACK are
    // crossings the 3-pass policy deliberately allowed. Counting every
    // containment as "hard" reported 3 hard contacts on an r5 flight whose
    // final field was pass-2 all-soft — a permitted crossing dressed up as a
    // safety breach. The two are kept apart and both are reported.
    // Sampled at the dt below (0.1 s), so these are SAMPLE counts.
    int zone_hard_n{0};       // inside the AUTHORED volume (q < 1.0)
    double zone_hard_s{0.0};
    int zone_standoff_n{0};   // in the 1.0-1.05 routing standoff shell
    double zone_standoff_s{0.0};
    int zone_soft_n{0};
    double zone_soft_s{0.0};
    bool zone_contact_measurable{true};
    // [ZONE-CONTACT] WHERE and HOW DEEP, not just how many. A count alone
    // cannot separate a 2 m graze of the 1.05x standoff shell from a
    // traverse of the authored volume, and those two call for opposite
    // responses. q < 1 is inside the authored ellipsoid; the gate fires at
    // q < 1.05, so q_min is the number that says which one happened.
    double zone_hard_t_first{-1.0};
    double zone_hard_t_last{-1.0};
    int zone_hard_runs{0};       // contiguous contact intervals
    bool zone_hard_prev{false};  // sampler state behind the run count
    // Closest approach to any HARD_AVOID zone over the whole span, contact
    // or not. Distinct from zone_hard_qmin, which only exists when a contact
    // occurred.
    double zone_clear_qmin{1e9};
    Eigen::Vector3d zone_clear_p{Eigen::Vector3d::Zero()};
    double zone_clear_t{-1.0};
    int zone_clear_zone{-1};
    double zone_hard_qmin{1e9};
    Eigen::Vector3d zone_hard_qmin_p{Eigen::Vector3d::Zero()};
    double zone_hard_qmin_vis{-1.0};
    int zone_hard_qmin_zone{-1};
  };
  std::vector<PhaseStat> st(spans.size());

  const double dt = 0.1;
  const size_t nz = pm_->numRiskZones();
  const auto eval_snap = pm_->zonePolicySnapshot();
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
    while (ph + 1 < spans.size() && t >= spans[ph].t_end) ++ph;
    PhaseStat &s = st[ph];
    // Judgment selection by CONTRACT TYPE, never by the display name. A
    // TRANSITION span is outside the cruise model by definition: its
    // samples never enter the cruise envelope statistics (its own
    // evaluator attaches with the generator, per ADR-0002 — v0 limits are
    // experimental, not gates). Terrain/zone/risk below stay: those are
    // model-agnostic.
    const bool cruise_judged = spans[ph].kind != PhaseKind::TRANSITION;
    const Eigen::Vector3d p = flight.getPos(t);
    double g = 0.0;
    pm_->terrainElevation(p.x(), p.y(), &g);  // false: sea level 0
    const double agl = p.z() - g;
    if (agl < s.min_agl) { s.min_agl = agl; s.min_agl_t = t; }
    s.agl_sum += agl;
    ++s.n;
    if (agl < clr_band) ++s.n_below_band;
    if (agl < 0.0) ++s.n_below_ground;
    if (dyn && cruise_judged) {
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

      bool hard = false, standoff = false, soft = false;
      int hard_zi = -1;
      double hard_q = 1e9;
      // Closest approach to a HARD_AVOID zone, tracked on EVERY sample and
      // not only on contact. "No contact" is a threshold answer; how close
      // the flight came is the measurement, and without it two products on
      // the same route cannot be compared at all — which is exactly the
      // question a contact on one and not the other raises.
      for (size_t zi = 0; zi < nz; ++zi) {
        if (zi >= eval_snap.zones.size() ||
            eval_snap.zones[zi].disposition !=
                PathManager::ZoneDisposition::HARD_AVOID)
          continue;
        const double q = pm_->getZoneEllipsoidRadius(zi, p);
        if (q < s.zone_clear_qmin) {
          s.zone_clear_qmin = q;
          s.zone_clear_p = p;
          s.zone_clear_t = t;
          s.zone_clear_zone = static_cast<int>(zi);
        }
      }
      for (size_t zi = 0; zi < nz; ++zi) {
        switch (pm_->zoneContact(eval_snap, zi, p)) {
          case PathManager::ZoneContactResult::CONTACT:
            if (zi < eval_snap.zones.size() &&
                eval_snap.zones[zi].disposition ==
                    PathManager::ZoneDisposition::HARD_AVOID) {
              // WHICH BAND. zoneContact() reports the 1.05x standoff volume,
              // which is the surface a ROUTE is kept out of; only the
              // authored volume refuses a finished flight.
              if (pm_->zoneContactAuthored(eval_snap, zi, p)) hard = true;
              else standoff = true;
              // Only on contact, so the deep-sample cost never touches the
              // common path.
              const double q = pm_->getZoneEllipsoidRadius(zi, p);
              if (q < hard_q) { hard_q = q; hard_zi = static_cast<int>(zi); }
            } else {
              soft = true;
            }
            break;
          case PathManager::ZoneContactResult::STALE:
          case PathManager::ZoneContactResult::INVALID:
            // A snapshot that cannot be judged must not read as "clear" —
            // that is how a zero contact count becomes a claim nobody
            // checked. The verdict below refuses CLEAN on this.
            s.zone_contact_measurable = false;
            break;
          case PathManager::ZoneContactResult::CLEAR:
            break;
        }
      }
      if (standoff) { ++s.zone_standoff_n; s.zone_standoff_s += dt; }
      // The contact-detail fields track EITHER band, so the emitted line can
      // say which one the deepest sample was in; the counters above are what
      // decide refusal.
      if (hard || standoff) {
        if (hard) { ++s.zone_hard_n; s.zone_hard_s += dt; }
        if (s.zone_hard_t_first < 0.0) s.zone_hard_t_first = t;
        s.zone_hard_t_last = t;
        if (!s.zone_hard_prev) ++s.zone_hard_runs;
        if (hard_q < s.zone_hard_qmin) {
          s.zone_hard_qmin = hard_q;
          s.zone_hard_qmin_p = p;
          s.zone_hard_qmin_zone = hard_zi;
          s.zone_hard_qmin_vis =
              hard_zi >= 0 ? pm_->getRiskVisibility(hard_zi, p) : -1.0;
        }
      }
      s.zone_hard_prev = hard || standoff;
      if (soft) { ++s.zone_soft_n; s.zone_soft_s += dt; }
    }
  }

  log_->infof("[FINAL-EVAL] ===== whole-flight evaluation: %.1f s, %d "
              "pieces, %zu phase(s) =====",
              T, flight.getPieceNum(), spans.size());
  double t0 = 0.0;
  PhaseStat tot;
  for (size_t ph = 0; ph < spans.size(); ++ph) {
    const PhaseStat &s = st[ph];
    if (s.n > 0) {
      // A phase with dynamics on but ZERO cruise-domain samples has no
      // measurement — "0.0%" would print no-data as a clean reading.
      char env[96];
      if (!dyn) {
        snprintf(env, sizeof env, "env model off");
      } else if (spans[ph].kind == PhaseKind::TRANSITION) {
        snprintf(env, sizeof env,
                 "env n/a (TRANSITION — cruise judgment not applicable)");
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
          "exposure %.1f s, zone contact hard %d (%.1f s) soft %d (%.1f s) "
          "@0.1s sampling%s",
          spans[ph].name.c_str(), spans[ph].t_end - t0, s.min_agl,
          s.min_agl_t, s.agl_sum / s.n, 100.0 * s.n_below_band / s.n,
          s.n_below_ground ? "YES" : "no", env, s.risk_max, s.risk_int,
          s.zone_hard_n, s.zone_hard_s, s.zone_soft_n, s.zone_soft_s,
          s.zone_contact_measurable ? "" : " [UNMEASURABLE: stale/invalid "
                                           "zone snapshot]");
    }
    t0 = spans[ph].t_end;
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
    tot.zone_hard_n += s.zone_hard_n;
    tot.zone_hard_s += s.zone_hard_s;
    tot.zone_standoff_n += s.zone_standoff_n;
    tot.zone_standoff_s += s.zone_standoff_s;
    tot.zone_hard_runs += s.zone_hard_runs;
    if (s.zone_hard_t_first >= 0.0 &&
        (tot.zone_hard_t_first < 0.0 ||
         s.zone_hard_t_first < tot.zone_hard_t_first))
      tot.zone_hard_t_first = s.zone_hard_t_first;
    if (s.zone_hard_t_last > tot.zone_hard_t_last)
      tot.zone_hard_t_last = s.zone_hard_t_last;
    if (s.zone_hard_qmin < tot.zone_hard_qmin) {
      tot.zone_hard_qmin = s.zone_hard_qmin;
      tot.zone_hard_qmin_p = s.zone_hard_qmin_p;
      tot.zone_hard_qmin_vis = s.zone_hard_qmin_vis;
      tot.zone_hard_qmin_zone = s.zone_hard_qmin_zone;
    }
    if (s.zone_clear_qmin < tot.zone_clear_qmin) {
      tot.zone_clear_qmin = s.zone_clear_qmin;
      tot.zone_clear_p = s.zone_clear_p;
      tot.zone_clear_t = s.zone_clear_t;
      tot.zone_clear_zone = s.zone_clear_zone;
    }
    tot.zone_soft_n += s.zone_soft_n;
    tot.zone_soft_s += s.zone_soft_s;
    tot.zone_contact_measurable &= s.zone_contact_measurable;
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
  // A flight that entered a HARD_AVOID volume, or one whose zone policy could
  // not be judged at all, must not read CLEAN. Leaving these out of the
  // condition is what let the new metric be printed beside a CLEAN verdict it
  // had no influence over — the number was reported, not enforced. An
  // unjudgeable snapshot is treated like a contact rather than like clear:
  // "we could not check" is not evidence of safety.
  const bool zone_bad = tot.zone_hard_n > 0 || !tot.zone_contact_measurable;
  // Envelope alone, kept apart from the zone reasons: a caller that has to
  // NAME the cause cannot recover it from `clean`.
  const bool envelope_bad = viol_pct >= viol_max_pct || peak_bad;
  const bool clean = tot.n_below_ground == 0 && !no_cruise &&
                     !envelope_bad && !zone_bad;
  if (clean) {
    log_->infof("[FINAL-EVAL] verdict: CLEAN — AGL min %.3f u, env viol "
                "%.1f%% (peak %.1f%% %s), risk exposure %.1f s, "
                "zone contact hard 0 standoff %d (%.1f s) soft %d (%.1f s) "
                "@0.1s sampling",
                tot.min_agl, viol_pct, 100.0 * tot.util_peak,
                mmp_vehicle_dynamics::envelopeLimitName(tot.peak_limit),
                tot.risk_int, tot.zone_standoff_n, tot.zone_standoff_s,
                tot.zone_soft_n, tot.zone_soft_s);
  } else {
    log_->warnf("[FINAL-EVAL] verdict: CHECK — %s%s%s%s%s(AGL min %.3f u "
                "@%.0fs, env viol %.1f%%) — the stitched flight carries "
                "hazards no per-solve audit saw",
                tot.n_below_ground ? "TERRAIN OVERLAP " : "",
                no_cruise ? "NEVER REACHES CRUISE " : "",
                tot.zone_hard_n > 0 ? "AUTHORED ZONE ENTERED " : "",
                !tot.zone_contact_measurable ? "ZONE POLICY UNEVALUATED " : "",
                !no_cruise && viol_pct >= viol_max_pct ? "ENVELOPE " : "",
                tot.min_agl, tot.min_agl_t, viol_pct);
  }
  FlightVerdict v;
  v.evaluated = true;
  v.clean = clean;
  v.envelope_bad = envelope_bad;
  v.underground = tot.n_below_ground != 0;
  v.no_cruise = no_cruise;
  v.zone_hard_n = tot.zone_hard_n;
  v.zone_standoff_n = tot.zone_standoff_n;
  v.zone_soft_n = tot.zone_soft_n;
  v.policy_measurable = tot.zone_contact_measurable;
  v.viol_pct = viol_pct;
  v.util_peak = tot.util_peak;
  // Machine-readable, always emitted, one line. A reader (and the A/B
  // parser) must not have to infer zone safety from the ABSENCE of a
  // warning word — that is fail-open by construction: a reworded log or a
  // dropped call both read as "measurable, no contact".
  log_->infof("[ZONE-AUDIT] hard=%d hard_s=%.1f standoff=%d standoff_s=%.1f "
              "soft=%d soft_s=%.1f measurable=%s sample_dt=%.2f "
              "risk_max=%.4f risk_exposure_s=%.1f",
              v.zone_hard_n, tot.zone_hard_s,
              v.zone_standoff_n, tot.zone_standoff_s,
              v.zone_soft_n, tot.zone_soft_s,
              v.policy_measurable ? "true" : "false", dt,
              tot.risk_max, tot.risk_int);
  // The count above says a breach happened; this says WHICH breach. Without
  // it, a 2 m graze of the 1.05x standoff shell and a traverse of the
  // authored volume are the same line — and they are not the same event.
  // q_min < 1.00 means the flight was inside the zone the mission authored;
  // 1.00 <= q_min < 1.05 means it stayed in the routing standoff margin,
  // where the optimizer's risk term is zero by construction and risk_max
  // above is therefore 0.0000 whatever the flight did.
  // runs= is summed per phase, so a contact straddling a phase boundary
  // reads as two.
  // Emitted for every flight that had a HARD_AVOID zone to stay clear of,
  // whether or not it touched anything. q_min is on the same scale the two
  // thresholds use: < 1.00 entered the authored volume, < 1.05 entered the
  // routing standoff, and anything above is the margin that was actually
  // kept.
  if (tot.zone_clear_zone >= 0) {
    log_->infof("[ZONE-CLEARANCE] zone=%d q_min=%.5f t=%.2fs "
                "at (%.2f, %.2f, %.2f)",
                tot.zone_clear_zone, tot.zone_clear_qmin, tot.zone_clear_t,
                tot.zone_clear_p.x(), tot.zone_clear_p.y(),
                tot.zone_clear_p.z());
  }
  if (tot.zone_hard_n > 0 || tot.zone_standoff_n > 0) {
    log_->infof("[ZONE-CONTACT] zone=%d t=[%.2f, %.2f]s runs=%d q_min=%.5f "
                "%s vis=%.3f at (%.2f, %.2f, %.2f)",
                tot.zone_hard_qmin_zone, tot.zone_hard_t_first,
                tot.zone_hard_t_last, tot.zone_hard_runs, tot.zone_hard_qmin,
                tot.zone_hard_qmin < 1.0 ? "INSIDE-AUTHORED" : "standoff-shell",
                tot.zone_hard_qmin_vis, tot.zone_hard_qmin_p.x(),
                tot.zone_hard_qmin_p.y(), tot.zone_hard_qmin_p.z());
  }
  // LAST, and only once every field is filled: an earlier assignment left
  // viol_pct/util_peak at zero in the recorded copy.
  last_verdict_ = v;
  return v;
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
