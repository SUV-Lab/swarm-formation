#include "path_manager/segment_chain_planner.h"
#include "path_manager/optimizer_worker_profile.h"
#include "path_manager/direct_plan_settings.h"

#include <optional>

#include <cstdio>

#include "path_manager/traj_sampling.h"
#include "path_manager/route_slice_batch.h"
#include "path_manager/flight_assembly.h"
#include "path_manager/flight_evaluator.h"
#include "path_manager/transition_phase.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <thread>

#include "path_manager/terminal_phase.h"

namespace path_manager {

namespace {
// A malformed explicit hard requirement cannot be discarded like an optional
// soft-profile typo. Entry adapters turn this configuration refusal into an
// owned FAILED result without adopting an intermediate solve.
class InvalidHardProfile : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

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
using OvrTag = OptimizerWorkerParameterKind;
using OvrKey = OptimizerWorkerParameter;
const OvrKey *whitelistFind(const std::string &name)
{
  const auto *key = findOptimizerWorkerParameter(name);
  return key && key->tag != OvrTag::kThreadBudget ? key : nullptr;
}

// Lowered arrival speed supplies only unspecified boundary components.
// Velocity and acceleration are independent operator claims.
ego_planner::TailBoundary completeArrivalTail(
    ego_planner::TailBoundary tail, const Eigen::Vector3d &default_velocity)
{
  if (!tail.prescribe_vel) {
    tail.vel = default_velocity;
    tail.prescribe_vel = true;
  }
  if (!tail.prescribe_acc) {
    tail.acc.setZero();
    tail.prescribe_acc = true;
  }
  return tail;
}

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
      segments_requested_(std::max(2, segments)),
      inherit_route_(inherit_route)
{
  last_plan_.segments = segments_requested_;
}

void SegmentChainPlanner::readSegmentsOption(PlanContext &state)
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
    node_->declare_parameter("chain/segments", state.segments);
  int req = state.segments;
  node_->get_parameter("chain/segments", req);
  state.auto_segments = (req <= 0);
  if (!state.auto_segments) state.segments = std::min(16, std::max(2, req));
  state.confirmed_segments = state.segments;
}

void SegmentChainPlanner::captureDiagnostics(PlanContext &state) const
{
  last_plan_.segments = state.confirmed_segments;
  last_plan_.auto_segments = state.auto_segments;
  last_plan_.head_policy = state.head_policy;
  last_plan_.verdict = state.verdict;
  last_plan_.spans = std::move(state.spans);
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

void SegmentChainPlanner::recordExecutionInvalidation(PlanningComputation &computed) const
{
  // Record the gate's execution effect; calculation never clears a live slot.
  // The backend or compatibility plan wrapper applies it with the final result.
  computed.invalidate_execution = true;
}

PlanningComputation SegmentChainPlanner::compute(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail)
{
  pm_->beginPlanningComputation();
  PlanContext state(segments_requested_);
  legacy_transition_active_ = false;
  PlanningComputation computed;
  try {
    computed.status = planInto(state, computed, head, waypoints, mission_tail);
  } catch (const InvalidHardProfile &error) {
    computed = PlanningComputation{};
    computed.status = PlanResult::failed(error.what());
    state.verdict = FlightVerdict{};
    state.spans.clear();
  } catch (...) {
    captureDiagnostics(state);
    throw;
  }
  computed.evaluation = state.verdict;
  computed.phases = state.spans;
  captureDiagnostics(state);
  return pm_->sealPlanningComputation(std::move(computed));
}

PlanResult SegmentChainPlanner::plan(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail)
{
  auto computed = compute(head, waypoints, mission_tail);
  if (!pm_->applyPlanningComputation(computed).applied)
    return PlanResult::failed("computed chain plan could not be adopted");
  return computed.status;
}

PlanResult SegmentChainPlanner::planInto(PlanContext &state,
    PlanningComputation &computed, const StartHead &head,
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
  const bool start_acc_commanded = head.acc_prescribed;
  // Read below by the one gate that still distinguishes the two stated
  // forms; the other two functions derived the same locals and never used
  // them, which is how a reader concludes the boolean still means something
  // there.
  const bool start_vel_synthesized =
      head.src == StartStateSource::STATED_SPEED;
  const bool start_vel_commanded =
      head.src == StartStateSource::STATED_VECTOR ||
      head.src == StartStateSource::TEST_INJECTED;
  // The entry owns a fresh PlanContext before any validation can refuse it.
  // Earlier requests' diagnostics are only a compatibility snapshot.
  // [HEAD-POLICY] fail-CLOSED at the entry. StartHead{} default-constructs
  // to UNSPECIFIED, so "the provenance is a required parameter" is a
  // compile-time guarantee about the CALL and not about the VALUE — a
  // caller can still pass a default-constructed head. Refuse it here rather
  // than plan from a state nobody described.
  if (head.src == StartStateSource::UNSPECIFIED)
    return PlanResult::failedBecause(
        PlanReason::INITIAL_STATE_UNSPECIFIED,
        "start state has no source — the planner will not invent one");
  if (!head.pos_u.allFinite() || !head.vel_u.allFinite() ||
      !head.acc_u.allFinite())
    return PlanResult::failedBecause(
        PlanReason::INITIAL_STATE_MALFORMED,
        "start state is not finite");
  // [AUTO-N] the segment-count option is interpreted HERE, before any
  // mission-shape branch, so the transition coordinator and the plain
  // chain see the same N policy (review find: the transition branch ran
  // on the reset defaults — 2 segments, no cruise span).
  readSegmentsOption(state);
  // [ENVELOPE] Contract 1 (2026-08-08): an EXPLICITLY commanded initial
  // velocity outside the cruise model's validity region never reaches the
  // cruise pipeline. No clamp — rewriting an operator's stated launch
  // state into a different flyable one is how a 549 u worm once flew as
  // "SUCCESS". Contract 2 (2026-08-08): the classifier now distinguishes
  // the TRANSITION regime (outside cruise, inside the transition model's
  // own validity) and dispatches it to the coordinator — behind
  // transition/enable, which SHIPS ON (optimizer_params.yaml, commit
  // 37fc51a "transition ships on"). With it off the frozen contract-1
  // outcome is what remains, and initfail/initaccfail/initceiling/pvaprobe
  // pin that disabled behaviour explicitly. Synthesized/
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
    PlanResult r = planTransitionMission(state, computed, head, waypoints, eff);
    if (relaxed) r.degrade(PlanReason::FINAL_BOUNDARY_RELAXED, relax_why);
    return r;
  }
  PlanResult r = planImpl(state, computed, head, waypoints, eff);
  if (relaxed) r.degrade(PlanReason::FINAL_BOUNDARY_RELAXED, relax_why);
  return r;
}

PlanResult SegmentChainPlanner::planImpl(PlanContext &state,
    PlanningComputation &computed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail)
{
  // Multi-waypoint requests retain the existing single-shot policy.
  if (waypoints.size() != 1)
    return planMultiWaypoint(state, computed, head, waypoints, mission_tail);

  // [AUTO-N] chain/segments interpretation moved to readSegmentsOption(state)
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
      // window — see planFromRoute. Non-whitelisted keys are dropped
      // loudly there.
      return planFromRoute(state, computed, head, waypoints, par, mission_tail);
    } else if (par) {
      log_->warnf("[CHAIN] chain/parallel needs chain/author_from_route "
                  "(the baseline is inherently sequential) — running the "
                  "sequential baseline flow");
    }
  }

  return planBaselineMission(state, computed, head, waypoints, mission_tail);
}

PlanResult SegmentChainPlanner::planMultiWaypoint(
    PlanContext &state, PlanningComputation &computed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail)
{
  log_->warnf("[CHAIN] %zu waypoints — stage 1 chains single-goal missions "
              "only, falling back to the single-shot plan", waypoints.size());
  auto product = pm_->computeGlobalTrajectory(head, waypoints, mission_tail);
  if (!product)
    return PlanResult::failed("multi-waypoint single-shot plan failed");
  computed.recordProduct(std::move(*product));
  // Same contract as every other direct product: judged before it is
  // accepted. "Chain not attempted" is a statement about how the flight
  // was produced, never about whether it was checked.
  log_->infof("[PLAN-MODE] direct");
  const poly_traj::Trajectory &mw = computed.product->trajectory;
  const FlightVerdict mv = evaluateFlightInContext(state,
      mw, {{mw.getTotalDuration(), PhaseKind::CRUISE, "direct"}},
      &pm_->lastPieceLeg(), pm_->lastCommittedRouteEpoch());
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
             "multi-waypoint single-shot flight is unflyable (%s%s%s%s%s%s)",
             !mv.evaluated ? "not evaluated" : "",
             mv.underground ? "terrain overlap; " : "",
             mv.no_cruise ? "never reaches cruise; " : "",
             mv.zone_hard_n > 0 ? "authored zone entered; " : "",
             mv.zone_junction_hard_n > 0
                 ? "leg handover inside an authored zone; " : "",
             unmeasured_blocks
                 ? "zone policy could not be evaluated on a multi-leg "
                   "mission — set manager/allow_unmeasured_zone_policy to "
                   "fly it uncleared" : "");
    log_->errorf("[STITCH-GATE] %s", why);
    recordExecutionInvalidation(computed);
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

PlanResult SegmentChainPlanner::planBaselineMission(
    PlanContext &state, PlanningComputation &computed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail)
{
  const auto t_wall = std::chrono::steady_clock::now();

  // === Baseline: the unsplit mission, exactly today's behavior ===
  // N is not known yet in auto mode — it is sized from THIS solve's piece
  // count a few lines below. Log the fixed count only after it is known.
  if (state.auto_segments)
    log_->infof("[CHAIN] baseline plan (unsplit mission; segment count sized "
                "from its piece count)");
  else
    log_->infof("[CHAIN] baseline plan (unsplit mission; %d chained segments "
                "follow)", state.segments);
  auto baseline_product = pm_->computeGlobalTrajectory(head, waypoints, mission_tail);
  if (!baseline_product) {
    log_->errorf("[CHAIN] baseline plan failed — nothing to chain, nothing "
                 "to fly");
    return PlanResult::failed("baseline plan failed");
  }
  computed.recordProduct(std::move(*baseline_product));
  const poly_traj::Trajectory baseline = computed.product->trajectory;
  const double T = baseline.getTotalDuration();
  // Both bail-outs return the baseline with the same comparison reference
  // as the degrade path. Until selected, the calculation product's reference
  // is the MINCO seed. Adoption into execution storage happens in the caller.
  // One gate, two baseline exits. Both used to store and publish first and
  // keep evaluateFlight only for its log line; a whole-flight refusal that
  // the stitched path enforces must not be escapable by bailing out to the
  // baseline.
  FlightVerdict baseline_verdict{};
  const auto baseline_gate = [&](const poly_traj::Trajectory &b)
      -> std::optional<PlanResult> {
    const auto bv = evaluateFlightInContext(state,
        b, {{b.getTotalDuration(), PhaseKind::CRUISE, "baseline"}},
        &pm_->lastPieceLeg(), pm_->lastCommittedRouteEpoch());
    baseline_verdict = bv;
    // !evaluated must fail too. unflyable() is false when nothing was
    // judged, so testing it alone lets a flight the evaluator could not
    // read pass as safe — the exact fail-open this gate exists to remove.
    if (bv.evaluated && !bv.unflyable()) return std::nullopt;
    char why[176];
    snprintf(why, sizeof why,
             "baseline flight is unflyable (%s%s%s%s%s%s) — whole-flight "
             "evaluation",
             !bv.evaluated ? "not evaluated" : "",
             bv.underground ? "terrain overlap; " : "",
             bv.no_cruise ? "never reaches cruise; " : "",
             bv.zone_hard_n > 0 ? "authored zone entered; " : "",
             bv.zone_junction_hard_n > 0
                 ? "leg handover inside an authored zone; " : "",
             !bv.evaluated ? "" :
                 (!bv.policy_measurable ? "zone policy unevaluated" : ""));
    log_->errorf("[STITCH-GATE] %s", why);
    recordExecutionInvalidation(computed);
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
    computed.recordProduct({baseline, baseline, now_s, now_s});
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
  if (!resolveAutoSegments(state, baseline.getPieceNum(), "baseline"))
    return fly_baseline();  // [AUTO-N] mission below the split threshold
  if (T <= 1e-6 || baseline.getPieceNum() < state.segments) {
    log_->warnf("[CHAIN] baseline too small to split (%.3f s, %d pieces) — "
                "flying the baseline", T, baseline.getPieceNum());
    return fly_baseline();
  }

  BaselineSegments prepared = prepareBaselineSegments(state, head, baseline);
  const auto &seg_over = prepared.overrides;
  const auto &contracts = prepared.contracts;
  BaselineRunResult solved = solveBaselineSegments(
      state, computed, head, waypoints, mission_tail, baseline, prepared);
  if (solved.failed_segment >= 0) {
    const int i = solved.failed_segment;
    if (const auto bad = baseline_gate(baseline)) return *bad;
    const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    computed.recordProduct({baseline, baseline, now_s, now_s});
    pm_->publishTrajectoryViz(baseline, baseline);
    PlanResult r = PlanResult::success();
    r.degrade(PlanReason::SINGLE_PLAN_FALLBACK,
              "segment " + std::to_string(i + 1) + "/" +
                  std::to_string(state.segments) +
                  " failed — baseline restored");
    degradeForVerdict(&r, baseline_verdict, "restored baseline");
    return r;
  }
  const auto &runs = solved.runs;
  auto &chained = solved.chained;

  // Report on the CHAIN portion (baseline deviation stays apples-to-apples),
  // then append the optional prescribed terminal phase before final judgment.
  logChainReport(baseline, runs, contracts, chained, seg_over);
  std::vector<PhaseSpan> spans;
  double acc_t = 0.0;
  for (size_t i = 0; i < runs.size(); ++i) {
    acc_t += runs[i].getTotalDuration();
    // All chained spans are CRUISE-judged today; the transition coordinator
    // will prepend a TRANSITION span. Kind is the judgment contract, the
    // name is display only.
    spans.push_back({acc_t, PhaseKind::CRUISE,
                     state.phase_applied
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
  // Stitched: the manager's map describes ONE segment solve, not this
  // concatenation. Saying so is the contract; letting a size comparison
  // decide it was a coincidence that held only while the counts differed.
  const FlightVerdict fv = evaluateFlightInContext(state, chained, spans, nullptr, 0);
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
      recordExecutionInvalidation(computed);
    return r;
  }

  const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  // The owned comparison reference is the baseline OPTIMIZED trajectory.
  // The execution adapter later applies global before local bookkeeping.
  computed.recordProduct({chained, baseline, now_s, now_s});
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
              state.segments, chained.getPieceNum(), chained.getTotalDuration(), T,
              wall_ms);
  return stitchedVerdictResult(computed, fv, PlanResult::success());
}

SegmentChainPlanner::BaselineSegments
SegmentChainPlanner::prepareBaselineSegments(
    const PlanContext &state, const StartHead &head,
    const poly_traj::Trajectory &baseline) const
{
  BaselineSegments prepared;
  const double T = baseline.getTotalDuration();
  const auto &start_vel = head.vel_u;
  // === [STAGE-2] per-segment requirement overrides (read before the
  // contracts: a slow segment's ceiling must shape its junction speeds) ===
  auto &seg_over = prepared.overrides;
  seg_over = readSegmentOverrides(state);
  // Effective speed ceiling per segment: its optimization/max_vel override,
  // else the mission-wide value. A junction pinned FASTER than a bounding
  // segment's ceiling is a self-contradictory hard BC — the solve brakes
  // through a state its own feasibility term condemns and the envelope
  // audit prices the transient as violations (measured: 34.7% > the 30%
  // hard gate on a 1.8 u/s segment fed 2.0 u/s boundaries).
  double &vmax_global = prepared.global_max_velocity;
  if (node_->has_parameter("optimization/max_vel"))
    vmax_global = node_->get_parameter("optimization/max_vel").as_double();
  auto &seg_vmax = prepared.max_velocity;
  seg_vmax.assign(static_cast<size_t>(state.segments), vmax_global);
  for (int i = 0; i < state.segments; ++i)
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
  auto &contracts = prepared.contracts;
  const double span = T / state.segments;
  double t_prev = 0.0;
  for (int i = 1; i < state.segments; ++i) {
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
  auto &slices = prepared.slices;
  auto &route_epoch = prepared.route_epoch;
  if (inherit_route_) {
    slices = sliceCommittedRoute(pm_->lastCommittedRoute(),
                                 pm_->lastCommittedCapRef(), contracts,
                                 pm_->lastCommittedRouteEdgeLeg());
    // [LEG-POLICY] The epoch these slices' tags were minted in. Every segment
    // hands it back so PathManager can prove no front end has run since — the
    // segment solves take the inherited-route branch, which cannot mint tags
    // of its own.
    route_epoch = pm_->lastCommittedRouteEpoch();
    if (slices.size() != static_cast<size_t>(state.segments)) {
      log_->warnf("[CHAIN] committed route did not slice cleanly (%zu/%d) — "
                  "falling back to per-span front-end search",
                  slices.size(), state.segments);
      slices.clear();
    } else {
      for (size_t j = 0; j < slices.size(); ++j) {
        log_->infof("[CHAIN] slice %zu: %zu vertices (cap %zu)",
                    j + 1, slices[j].path.size(), slices[j].cap.size());
      }
    }
  }

  return prepared;
}

SegmentChainPlanner::BaselineRunResult
SegmentChainPlanner::solveBaselineSegments(
    const PlanContext &state, PlanningComputation &computed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail,
    const poly_traj::Trajectory &baseline, const BaselineSegments &prepared)
{
  BaselineRunResult result;
  const auto &start_pos = head.pos_u;
  const auto &start_vel = head.vel_u;
  const auto &start_acc = head.acc_u;
  const auto &seg_over = prepared.overrides;
  const auto &seg_vmax = prepared.max_velocity;
  const auto &contracts = prepared.contracts;
  const auto &slices = prepared.slices;
  const auto route_epoch = prepared.route_epoch;
  const double vmax_global = prepared.global_max_velocity;
  bool any_over = false;
  for (const auto &so : seg_over) any_over |= !so.params.empty();
  // === Chained segment runs (sequential; each is a full pipeline run) ===
  auto &runs = result.runs;
  for (int i = 0; i < state.segments; ++i) {
    const bool last = (i + 1 == state.segments);
    DirectPlanSettings segment_settings;
    if (any_over) {
      // Each segment overlays the unchanged mission settings. Preserve the
      // existing last-wins rule for duplicate entries without node writes.
      std::map<std::string, rclcpp::Parameter> effective;
      for (const auto &parameter : seg_over[i].params)
        effective[parameter.get_name()] = parameter;
      for (const auto &entry : effective)
        segment_settings.overrides.push_back(entry.second);
      if (!seg_over[i].label.empty())
        log_->infof("[CHAIN] segment %d/%d requirement overrides: %s",
                    i + 1, state.segments, seg_over[i].label.c_str());
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
    // ceiling. Supply an unspecified final velocity at that ceiling along
    // the baseline approach, preserving any prescribed final acceleration.
    if (last && !seg_tail.prescribe_vel &&
        seg_vmax[static_cast<size_t>(i)] < vmax_global - 1e-12) {
      const Eigen::Vector3d base_tail =
          baseline.getJuncVel(baseline.getPieceNum());
      if (base_tail.norm() > seg_vmax[static_cast<size_t>(i)] + 1e-9) {
        Eigen::Vector3d dir = base_tail;
        dir.z() = 0.0;  // level, like the arrival contract (no-op: the
                        // baseline tail is already level by that contract)
        if (dir.norm() > 1e-9) {
          seg_tail = completeArrivalTail(
              seg_tail, dir.normalized() * seg_vmax[static_cast<size_t>(i)]);
          log_->infof("[CHAIN] arrival contract at the segment ceiling: "
                      "|v|=%.3f u/s level along the baseline approach",
                      seg_tail.vel.norm());
        }
      }
    }

    log_->infof("[CHAIN] segment %d/%d: head (%.2f, %.2f, %.2f) |v|=%.3f -> "
                "%s (%.2f, %.2f, %.2f)",
                i + 1, state.segments, head_pos.x(), head_pos.y(), head_pos.z(),
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
    path_manager::PathManager::RouteProvenance prov;
    if (slice && !slice->edge_leg.empty()) {
      prov.edge_leg = &slice->edge_leg;
      prov.epoch = route_epoch;
    }
    // With any segment profile active, every segment gets a fresh session,
    // including a segment whose own override list is empty.
    auto segment_product = any_over
        ? pm_->computeGlobalTrajectory(
              segment_settings, seg_head, goal, seg_tail, /*junction_goal=*/!last,
              slice ? &slice->path : nullptr, slice ? &slice->cap : nullptr,
              prov.edge_leg ? &prov : nullptr)
        : pm_->computeGlobalTrajectory(
              seg_head, goal, seg_tail, /*junction_goal=*/!last,
              slice ? &slice->path : nullptr, slice ? &slice->cap : nullptr,
              prov.edge_leg ? &prov : nullptr);
    if (!segment_product) {
      log_->warnf("[CHAIN] segment %d/%d FAILED — restoring and flying the "
                  "baseline", i + 1, state.segments);
      result.failed_segment = i;
      return result;
    }
    computed.recordProduct(std::move(*segment_product));
    runs.push_back(computed.product->trajectory);
  }

  // === Stitch the owned product ===
  auto &chained = result.chained;
  chained = runs.front();
  for (size_t i = 1; i < runs.size(); ++i) chained.append(runs[i]);

  // Segment sessions leave mission settings and model untouched, so terminal
  // construction and whole-flight evaluation still use the mission context.
  return result;
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

bool SegmentChainPlanner::resolveAutoSegments(PlanContext &state, int pieces, const char *source)
{
  if (!state.auto_segments) return true;
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
  state.segments = std::min(16, n);
  state.confirmed_segments = state.segments;
  log_->infof("[CHAIN] auto segments: %d pieces (%s) / target %d -> N=%d",
              pieces, source, target, state.segments);
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

bool SegmentChainPlanner::authorContractsFromRoute(PlanContext &state,
    const std::vector<Eigen::Vector3d> &route, const Eigen::Vector3d &v0,
    std::vector<Contract> *contracts) const
{
  const int M = static_cast<int>(route.size());
  if (M < state.segments + 1 || !contracts) return false;
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
  const double span_w = W_total / state.segments;

  // Sustainable grade at cruise: the THRUST-LIMITED climb, not just the
  // fpa cap. A junction is pinned at full cruise with a = 0, so the solver
  // has no energy trade there — a uniformly steep leg has kappa == 0 and
  // would otherwise score as the calmest possible candidate while pinning
  // an unsustainable climb as a hard BC (review find).
  double tan_grade_max = std::numeric_limits<double>::infinity();
  if (const auto model = pm_->vehicleModel()) {
    // readNumParam declares-if-absent, so this can never read a literal while
    // the real value is declared a moment later. ReplanFSM now declares both
    // unit names in its constructor, which is before this planner exists.
    const double um =
        readNumParam(node_, "optimization/dynamics_unit_xy_m", 100.0);
    const double v = cruise * um;
    tan_grade_max = model->planningGeometry(v, 0.0, 1.5).climb_grade;
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
    state.phase_tan_grade = tan_grade_max;
    state.phase_turn_radius_u = 0.0;
    if (const auto model = pm_->vehicleModel()) {
      const double um =
          readNumParam(node_, "optimization/dynamics_unit_xy_m", 100.0);
      const double vm = std::max(1e-3, v0.norm()) * um;
      state.phase_turn_radius_u =
          model->planningGeometry(cruise * um, vm, 1.5).turn_radius_m / um;
    }
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
    rl.turn_radius_u = state.phase_turn_radius_u;
    rl.accel_arc_u = 0.0;  // cruise-to-cruise legacy screen: no speed change
    rl.max_candidates = max_cand;
    const HandoffScreenResult hs =
        screenHandoffCandidates(route, v0, rl, arr_min);
    if (state.segments >= 3) {
      state.dep_candidates = hs.dep_candidates;
      if (!state.dep_candidates.empty()) dep_idx = state.dep_candidates.front();
      state.arr_candidates = hs.arr_candidates;
      if (!state.arr_candidates.empty()) arr_idx = state.arr_candidates.front();
      if (dep_idx < 0 || arr_idx < 0 || arr_idx <= dep_idx ||
          s[static_cast<size_t>(arr_idx)] - s[static_cast<size_t>(dep_idx)] <
              cruise_min) {
        state.phase_note =
            "phase handoffs not found (departure idx " +
            std::to_string(dep_idx) + ", arrival idx " +
            std::to_string(arr_idx) +
            ") — N kept, balanced junctions, phase semantics degraded";
        log_->warnf("[PHASE] %s", state.phase_note.c_str());
        dep_idx = arr_idx = -1;
      } else {
        state.phase_applied = true;
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
        state.phase_applied = true;
        state.dep_candidates.assign(1, dep_idx);  // N=2: no boundary-move retry
        log_->infof("[PHASE] N=2 shared handoff @s=%.1f u (kappa %.4f)",
                    s[static_cast<size_t>(dep_idx)], hs.shared_kappa);
      } else {
        state.phase_note = "no shared departure/arrival handoff (N=2) — balanced "
                      "junction kept, phase semantics degraded";
        log_->warnf("[PHASE] %s", state.phase_note.c_str());
      }
    }
  }

  contracts->clear();
  double w_prev = 0.0;
  for (int j = 1; j < state.segments; ++j) {
    int forced = -1;
    if (state.phase_applied) {
      if (j == 1 && dep_idx >= 0) forced = dep_idx;
      if (j == state.segments - 1 && arr_idx >= 0) forced = arr_idx;
    }
    double target = j * span_w;
    double lo = w_prev + 0.2 * span_w;
    double hi = target + 0.4 * span_w;
    double span_bal = span_w;
    if (state.phase_applied && state.segments >= 3 && forced < 0) {
      // Middle junctions balance INSIDE the cruise span.
      const double w0 = w[static_cast<size_t>(dep_idx)];
      const double w1 = w[static_cast<size_t>(arr_idx)];
      span_bal = (w1 - w0) / static_cast<double>(state.segments - 2);
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
                  j, state.segments);
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

// The transition coordinator commits the route once, builds a transition to
// an entry on it, then passes the remaining slice and handoff to computeOverRoute.
// planFromRoute composes route preparation and computation without a transition.
// Both defer execution application to the backend or compatibility plan wrapper.

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
  if (pm_->vehicleModel() && !pm_->dynamicsParams())
    return unsupported(prob + "; vehicle model does not support the fixed-wing "
                              "3DOF transition generator");
  const auto *dyn = pm_->dynamicsParams();
  if (!dyn || !mmp_vehicle_dynamics::parametersAreValid(*dyn))
    return unsupported(prob +
                       "; no valid assumption parameter set to transition "
                       "under");
  const double um =
      readNumParam(node_, "optimization/dynamics_unit_xy_m", 100.0);
  const double uz =
      readNumParam(node_, "optimization/dynamics_unit_z_m", 100.0);
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
                                   std::vector<double> *out_cap,
                                   const std::vector<size_t> &edge_leg,
                                   std::vector<size_t> *out_edge_leg) const
{
  if (out_edge_leg) out_edge_leg->clear();
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
  if (out_route->size() < 2) return false;
  // `i` now names the route edge the surviving head begins on, in BOTH
  // branches above: the coincident case advanced it past the edge it
  // consumed, the clipped case left it on the edge it cut.
  if (out_edge_leg && edge_leg.size() + 1 == route.size() &&
      i < edge_leg.size()) {
    out_edge_leg->assign(edge_leg.begin() + i, edge_leg.end());
    if (out_edge_leg->size() + 1 != out_route->size()) out_edge_leg->clear();
  }
  return true;
}

PlanResult SegmentChainPlanner::planTransitionMission(PlanContext &state,
    PlanningComputation &computed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail)
{
  const Eigen::Vector3d &start_pos = head.pos_u;
  const Eigen::Vector3d &start_vel = head.vel_u;
  const Eigen::Vector3d &start_acc = head.acc_u;
  const bool start_acc_commanded = head.acc_prescribed;
  namespace tp = transition_phase;
  // This request owns the transition flag; every nested fallback sees it.
  // No mutable planner flag needs to be restored at any exit.
  state.transition_active = true;

  const auto fail = [&](PlanReason why, const std::string &msg) {
    log_->errorf("[S13] transition mission FAILED: %s", msg.c_str());
    return PlanResult::failedBecause(why, msg);
  };
  const auto *dyn = pm_->dynamicsParams();
  if (!dyn || !mmp_vehicle_dynamics::parametersAreValid(*dyn))
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "no valid assumption parameter set");
  const double um =
      readNumParam(node_, "optimization/dynamics_unit_xy_m", 100.0);
  const double uz =
      readNumParam(node_, "optimization/dynamics_unit_z_m", 100.0);
  if (um <= 1e-9 || uz <= 1e-9)
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "degenerate dynamics unit scale");
  // [LEG-POLICY] SCOPE, decided before anything is attempted.
  //
  // The coordinator screens its entry candidates and judges its arc against
  // ONE plan-wide zone policy, and a multi-leg front end does not have one.
  // That is a property of the REQUEST, not of anything the planner might
  // discover, so it is settled here rather than after a route commit — a
  // front end that failed for its own reasons would otherwise report
  // TRANSITION_GENERATION_FAILED for an input that was never supported, and
  // the operator would go looking for a geometry problem.
  //
  // Zone-free multi-waypoint transitions are NOT refused: the snapshot is
  // valid then, there is nothing to attribute, and the flight is judged whole
  // exactly as a single-goal one is.
  if (waypoints.size() > 1 && pm_->numRiskZones() > 0)
    return fail(PlanReason::TRANSITION_MULTI_LEG_UNSUPPORTED,
                "a transition mission with " +
                    std::to_string(waypoints.size()) +
                    " waypoints and a risk zone is not supported: the "
                    "coordinator judges its entry against ONE plan-wide "
                    "zone policy, and a multi-leg front end has none");

  bool run_parallel = false;
  if (!node_->has_parameter("chain/parallel"))
    node_->declare_parameter("chain/parallel", false);
  node_->get_parameter("chain/parallel", run_parallel);

  // [1] route commit — one front-end pass from the mission start.
  auto committed = commitRouteInContext(state, head, waypoints, run_parallel);
  if (!committed)
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "route commit failed");
  const auto &route = committed->path;
  const auto &cap = committed->cap;
  const auto &route_edge_leg = committed->edge_leg;

  // [2] zone policy snapshot — fail-closed precondition. INVALID (pass 0,
  // stale data, multi-leg search) means the transition cannot be judged.
  const auto snap = pm_->zonePolicySnapshot();
  // The multi-leg case was refused at entry, so anything invalid here is a
  // real generation precondition — pass 0, or a search that ran on stale
  // zone/terrain data.
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
    const Eigen::Vector3d v0_si(start_vel.x() * um, start_vel.y() * um,
                                start_vel.z() * uz);
    const double vm = std::max(1e-3, v0_si.norm());
    const auto geometry = pm_->vehicleModel()->planningGeometry(v_c, vm, 1.5);
    tan_grade_max = geometry.climb_grade;
    turn_radius_u = geometry.turn_radius_m / um;
    accel_arc_u = geometry.acceleration_distance_m / um;
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
  // The per-gate tally is the ONLY thing that says which gate emptied the
  // candidate set. It used to omit disq_zone_standoff, so a run wiped out
  // by the 1.05x routing shell alone was indistinguishable from one wiped
  // out inside the authored volume. Zone count rides along because a
  // mis-paired scenario (zones that belong to another mission) presents
  // exactly like a generator regression.
  char audit_buf[640];
  std::snprintf(
      audit_buf, sizeof(audit_buf),
      "enumerated %d, winner %d, zones %zu | disq fin %d "
      "pre %d rep %d sat %d ter %d zone %d zone_standoff %d limits %d "
      "time %d pva %d adapter %d",
      tr.audit.candidates_enumerated, tr.audit.winner_primitive_id,
      pm_->numRiskZones(), tr.audit.disq_finiteness, tr.audit.disq_preguard,
      tr.audit.disq_representable, tr.audit.disq_saturated,
      tr.audit.disq_terrain, tr.audit.disq_zone,
      tr.audit.disq_zone_standoff, tr.audit.disq_limits,
      tr.audit.disq_timeout, tr.audit.disq_end_pva, tr.audit.disq_adapter);
  const std::string audit_str(audit_buf);
  log_->infof(
      "[S13] transition audit: %s | dwell %.2f s, winner risk max %.3g int %.3g (search "
      "%.3g/%.3g), adapter err p/v/a %.3g/%.3g/%.3g, start acc: "
      "repro err %.3g / model %.3g m/s^2",
      audit_str.c_str(),
      tr.audit.dwell_achieved_s, tr.audit.risk_max, tr.audit.risk_integral,
      tr.audit.search_risk_max, tr.audit.search_risk_integral,
      tr.audit.adapter_max_pos_err_m, tr.audit.adapter_max_vel_err_mps,
      tr.audit.adapter_max_acc_err_mps2, tr.audit.start_acc_repro_err_mps2,
      tr.audit.start_acc_model_mps2);
  if (!tr.ok)
    return fail(tr.any_candidate_reached_adapter
                    ? PlanReason::TRANSITION_ADAPTER_UNSOUND
                    : PlanReason::TRANSITION_GENERATION_FAILED,
                tr.reason + " [" + audit_str + "]");

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
  std::vector<size_t> sub_edge_leg;
  if (!cutAtArc(route, cap, tr.route_start_s, &sub_route, &sub_cap,
                route_edge_leg, &sub_edge_leg))
    return fail(PlanReason::TRANSITION_GENERATION_FAILED,
                "route cut at the entry arc failed");

  // [6] chain over the remainder with the transition prefix: junction
  // gate, leading TRANSITION span, full-flight judgment and preview all live
  // inside computeOverRoute; execution adoption remains with the caller.
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
  committed->path = std::move(sub_route);
  committed->cap = std::move(sub_cap);
  committed->edge_leg = std::move(sub_edge_leg);
  return computeOverRoute(state, computed, *committed, handoff, waypoints, run_parallel,
                       mission_tail, &prefix);
}

PlanResult SegmentChainPlanner::planFromRoute(PlanContext &state,
    PlanningComputation &computed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    bool run_parallel, const ego_planner::TailBoundary &mission_tail)
{
  const auto committed = commitRouteInContext(state, head, waypoints, run_parallel);
  if (!committed) return PlanResult::failed("front-end failed");
  return computeOverRoute(state, computed, *committed, head, waypoints, run_parallel, mission_tail);
}

std::optional<CommittedRoute> SegmentChainPlanner::commitRoute(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    bool run_parallel)
{
  PlanContext state(last_plan_.segments);
  state.auto_segments = last_plan_.auto_segments;
  return commitRouteInContext(state, head, waypoints, run_parallel);
}

// [S13] Stage 1: the ONE front-end pass that commits the global route
// (AGL/bbox/SDF/zone binding happen here exactly once — route decided once,
// the r3 homotopy lesson).
std::optional<CommittedRoute> SegmentChainPlanner::commitRouteInContext(
    const PlanContext &state, const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    bool run_parallel)
{
  const auto t0 = std::chrono::steady_clock::now();
  if (state.auto_segments)
    log_->infof("[CHAIN-PAR] route-%s plan, auto-sized segments "
                "(no baseline)", run_parallel ? "parallel" : "sequential");
  else
    log_->infof("[CHAIN-PAR] route-%s plan, %d segments (no baseline)",
                run_parallel ? "parallel" : "sequential", state.segments);
  auto committed = pm_->planCommittedRoute(head, waypoints);
  if (!committed) {
    log_->errorf("[CHAIN-PAR] front-end failed — nothing to author on");
    return std::nullopt;
  }
  committed->front_end_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  return committed;
}

bool SegmentChainPlanner::commitRoute(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    bool run_parallel, std::vector<Eigen::Vector3d> *route,
    std::vector<double> *cap, double *fe_ms,
    std::vector<size_t> *edge_leg, uint64_t *epoch)
{
  auto committed = commitRoute(head, waypoints, run_parallel);
  if (!committed) return false;
  *route = std::move(committed->path);
  *cap = std::move(committed->cap);
  if (edge_leg) *edge_leg = std::move(committed->edge_leg);
  if (epoch) *epoch = committed->epoch;
  if (fe_ms) *fe_ms = committed->front_end_ms;
  return true;
}

PlanResult SegmentChainPlanner::planRouteFallback(PlanContext &state,
    PlanningComputation &computed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail,
    const std::vector<SegmentOverrides> &eff, const char *why,
    bool as_degraded)
{
  // [S13] With a transition ACTIVE the single-shot fallback is forbidden
  // outright (contract §10/§13): it would re-plan FROM THE MISSION START
  // with the cruise model — re-planning the very regime the transition
  // exists to handle, and duplicating the transition span. Installed
  // BEFORE any coordinator exists so the guard cannot be forgotten when
  // one arrives.
  if (state.transition_active) {
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
    // Early fallbacks have no workers yet. Resolve the SAME active profiles
    // first: phase enable, positional selection, segment precedence and the
    // last value of a duplicate key must not change merely because this
    // mission was too short to split or its first search failed.
    const bool phase_requested =
        readNumParam(node_, "chain/phase/enable", 0.0) != 0.0;
    const auto profiles = readRouteWorkerProfiles(state, phase_requested);
    for (const auto &profile : profiles.effective) collect(profile);
  }
  std::optional<PlannedTrajectory> fallback_product;
  if (!hardmin.empty()) {
    DirectPlanSettings settings;
    settings.overrides.reserve(hardmin.size());
    for (const auto &limit : hardmin)
      settings.overrides.emplace_back(limit.first, limit.second);
    try {
      fallback_product = pm_->computeGlobalTrajectory(
          settings, head, waypoints, mission_tail);
    } catch (const DirectPlanConfigurationError &e) {
      return PlanResult::failed(
          std::string("hard-limit preservation failed: ") + e.what());
    }
    log_->warnf("[PHASE] fallback preserves %zu hard limit(s) "
                "conservatively (mission-wide min)", hardmin.size());
  } else {
    fallback_product = pm_->computeGlobalTrajectory(head, waypoints, mission_tail);
  }
  const bool fb_ok = fallback_product.has_value();
  if (fallback_product) computed.recordProduct(std::move(*fallback_product));
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
    // Judge the owned optimized trajectory, not its pre-L-BFGS comparison
    // reference. The caller will adopt these into local/global execution
    // storage only after the calculation has returned.
    const poly_traj::Trajectory &fly = computed.product->trajectory;
    const FlightVerdict fv = evaluateFlightInContext(state,
        fly, {{fly.getTotalDuration(), PhaseKind::CRUISE, "direct"}},
        &pm_->lastPieceLeg(), pm_->lastCommittedRouteEpoch());
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
                 "direct fallback failed the flight fitness gate (%s%s%s%s%senv "
                 "viol %.1f%%, peak %.1f%%)",
                 fv.underground ? "terrain overlap, " : "",
                 fv.no_cruise ? "never reaches cruise, " : "",
                 fv.zone_hard_n > 0 ? "authored zone entered, " : "",
                 fv.zone_junction_hard_n > 0
                     ? "leg handover inside an authored zone, " : "",
                 !fv.policy_measurable ? "zone policy unevaluated, " : "",
                 fv.viol_pct, 100.0 * fv.util_peak);
      }
      log_->errorf("[ENVELOPE] %s — FAILED (DIRECT_FALLBACK_UNSAFE)", why2);
      recordExecutionInvalidation(computed);
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
}

SegmentChainPlanner::RouteWorkerSetup
SegmentChainPlanner::readRouteWorkerProfiles(
    const PlanContext &state, bool phase_requested) const
{
  RouteWorkerSetup setup;
  auto &prof_dep = setup.departure;
  auto &prof_cru = setup.cruise;
  auto &prof_arr = setup.arrival;
  auto &eff = setup.effective;
  // [PHASE] Build the effective per-worker override lists: phase profile
  // by POSITION (departure = first, arrival = last, cruise between; only
  // when phase mode is on), then seg<i> on top (explicit index wins).
  // Whitelisted optimizer-owned keys only — everything else is dropped
  // loudly (PathManager-shared values cannot differ per parallel worker).
  if (phase_requested) {
    prof_dep = readOverrideList("chain/phase/departure/params",
                                "phase-departure");
    if (state.segments > 2)
      prof_cru = readOverrideList("chain/phase/cruise/params", "phase-cruise");
    prof_arr = readOverrideList("chain/phase/arrival/params", "phase-arrival");
  }
  const std::vector<SegmentOverrides> seg_over = readSegmentOverrides(state);
  eff.assign(static_cast<size_t>(state.segments), SegmentOverrides{});
  for (int i = 0; i < state.segments; ++i) {
    std::map<std::string, rclcpp::Parameter> m;
    std::map<std::string, std::string> invalid_hard;
    const auto take = [&](const SegmentOverrides &src, const char *origin) {
      for (const auto &pp : src.params) {
        if (!whitelistFind(pp.get_name())) {
          log_->warnf("[PHASE] %s override '%s' is not route-mode "
                      "whitelisted — dropped (baseline flow applies the "
                      "full surface)", origin, pp.get_name().c_str());
          continue;
        }
        m[pp.get_name()] = pp;
        invalid_hard.erase(pp.get_name());
      }
      for (const auto &error : src.invalid_hard) {
        m.erase(error.first);
        invalid_hard[error.first] = error.second;
      }
    };
    if (phase_requested)
      take(i == 0 ? prof_dep : (i + 1 == state.segments ? prof_arr : prof_cru),
           i == 0 ? "departure" : (i + 1 == state.segments ? "arrival" : "cruise"));
    take(seg_over[static_cast<size_t>(i)], "seg");
    if (!invalid_hard.empty())
      throw InvalidHardProfile(invalid_hard.begin()->second);
    auto &e = eff[static_cast<size_t>(i)];
    for (const auto &kv : m) {
      e.params.push_back(kv.second);
      e.label += (e.label.empty() ? "" : ", ") + kv.first + "=" +
                 kv.second.value_to_string();
    }
  }

  return setup;
}

SegmentChainPlanner::RouteWorkerSetup
SegmentChainPlanner::prepareRouteWorkers(const PlanContext &state, std::vector<Contract> &contracts,
                                          bool run_parallel,
                                          bool phase_requested)
{
  RouteWorkerSetup setup = readRouteWorkerProfiles(state, phase_requested);
  auto &eff = setup.effective;
  auto &opts = setup.optimizers;
  // Only the initial concurrent batch shares the inner OpenMP budget.
  // Retry optimizers retain the mission thread setting, matching the original
  // post-restoration construction order.
  std::optional<int> inner_threads;
  if (run_parallel) {
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const int per =
        std::max(1, static_cast<int>(hw) / std::max(1, state.segments));
    log_->infof("[CHAIN-PAR] %u cores / %d workers -> %d inner threads "
                "each", hw, state.segments, per);
    inner_threads = per;
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

  // Each optimizer reads pristine mission parameters plus its own value
  // overlay. No temporary node parameter changes escape worker construction.
  for (int i = 0; i < state.segments; ++i) {
    auto overrides = eff[static_cast<size_t>(i)].params;
    if (inner_threads)
      overrides.emplace_back("optimization/risk_parallel_threads", *inner_threads);
    if (!eff[static_cast<size_t>(i)].label.empty())
      log_->infof("[PHASE] worker %d effective overrides: %s", i + 1,
                  eff[static_cast<size_t>(i)].label.c_str());
    opts.push_back(pm_->makeConfiguredOptimizer(overrides));
  }
  return setup;
}

std::vector<int> SegmentChainPlanner::selectMergeNeighbors(const PlanContext &state,
    int failed, const std::vector<RouteSlice> &slices,
    size_t contract_count) const
{
  std::vector<int> sides;
  if (failed > 0) sides.push_back(failed - 1);
  if (failed + 1 < state.segments) sides.push_back(failed + 1);
  // A generic merge must never dissolve a departure or arrival boundary.
  if (state.phase_applied) {
    sides.erase(std::remove_if(sides.begin(), sides.end(), [&](int neighbor) {
                  const int junction = std::min(failed, neighbor);
                  return junction == 0 ||
                         junction == static_cast<int>(contract_count) - 1;
                }),
                sides.end());
  }
  // Try the smaller neighboring slice first, using the existing ordering.
  std::sort(sides.begin(), sides.end(), [&](int a, int b) {
    return slices[static_cast<size_t>(a)].path.size() <
           slices[static_cast<size_t>(b)].path.size();
  });
  return sides;
}

SegmentChainPlanner::MergeRetryCandidate
SegmentChainPlanner::prepareMergeRetry(const PlanContext &state,
    int failed, int neighbor, const std::vector<RouteSlice> &slices,
    const std::vector<Contract> &contracts, const StartHead &effective_head,
    const ego_planner::TailBoundary &mission_tail,
    bool suppress_crossing_exempt) const
{
  MergeRetryCandidate candidate;
  candidate.lo = std::min(failed, neighbor);
  candidate.hi = std::max(failed, neighbor);
  const size_t lo = static_cast<size_t>(candidate.lo);
  const size_t hi = static_cast<size_t>(candidate.hi);
  auto &request = candidate.request;
  request.path = slices[lo].path;
  request.cap = slices[lo].cap;
  // Adjacent slices share their junction vertex: append it only once.
  request.path.insert(request.path.end(), slices[hi].path.begin() + 1,
                       slices[hi].path.end());
  request.cap.insert(request.cap.end(), slices[hi].cap.begin() + 1,
                      slices[hi].cap.end());
  request.head_pos = lo ? contracts[lo - 1].pos : effective_head.pos_u;
  request.head_vel = lo ? contracts[lo - 1].vel : effective_head.vel_u;
  request.head_acc = lo ? contracts[lo - 1].acc : effective_head.acc_u;
  request.goal_pos = request.path.back();
  request.tail = candidate.hi + 1 == state.segments
                     ? mission_tail
                     : ego_planner::TailBoundary::pinned(contracts[hi].vel,
                                                         contracts[hi].acc);
  request.suppress_crossing_exempt = suppress_crossing_exempt;
  return candidate;
}

SegmentChainPlanner::MergeRetryProfile
SegmentChainPlanner::mergedRetryProfile(const PlanContext &state,
    const MergeRetryCandidate &candidate, const RouteWorkerSetup &workers,
    bool phase_requested) const
{
  MergeRetryProfile profile;
  std::map<std::string, rclcpp::Parameter> values;
  // Arrival wins when a merge spans both ends; otherwise use departure or
  // cruise by position. Only soft weights come from the raw phase profile:
  // its hard maximum may have been superseded by a segment override.
  if (phase_requested) {
    const SegmentOverrides &phase =
        candidate.hi + 1 == state.segments ? workers.arrival
        : candidate.lo == 0           ? workers.departure
                                      : workers.cruise;
    for (const auto &parameter : phase.params) {
      const OvrKey *key = whitelistFind(parameter.get_name());
      if (key && key->tag == OvrTag::kSoft)
        values[parameter.get_name()] = parameter;
    }
  }
  double max_velocity = std::numeric_limits<double>::infinity();
  for (int side : {candidate.lo, candidate.hi}) {
    // Preserve each original worker's actual limit, including an inherited
    // default absent from its sparse override list. The manager's cruise
    // speed and freshly read ROS parameters are not this configured value.
    const double worker_max_velocity =
        workers.optimizers[static_cast<size_t>(side)]->configuredMaxVelocity();
    if (!std::isfinite(worker_max_velocity) || worker_max_velocity <= 0.0)
      throw InvalidHardProfile(
          "invalid hard profile for merge worker: optimization/max_vel must be finite and positive");
    max_velocity = std::min(max_velocity, worker_max_velocity);
    for (const auto &parameter :
         workers.effective[static_cast<size_t>(side)].params) {
      const OvrKey *key = whitelistFind(parameter.get_name());
      if (!key || key->tag != OvrTag::kSoft) continue;
      const auto existing = values.find(parameter.get_name());
      if (existing == values.end() ||
          existing->second.as_double() != parameter.as_double())
        profile.soft_drop_note +=
            (profile.soft_drop_note.empty() ? "" : ", ") +
            parameter.get_name();
    }
  }
  // max_vel is the route-worker whitelist's sole hard maximum.
  values["optimization/max_vel"] =
      rclcpp::Parameter("optimization/max_vel", max_velocity);
  for (const auto &entry : values) {
    profile.overrides.params.push_back(entry.second);
    profile.overrides.label +=
        (profile.overrides.label.empty() ? "" : ", ") + entry.first;
  }
  return profile;
}

std::optional<SegmentChainPlanner::PhaseRetryCandidate>
SegmentChainPlanner::preparePhaseRetry(const PlanContext &state,
    const CommittedRoute &route, int vertex, bool departure,
    const std::vector<Contract> &contracts, const StartHead &effective_head,
    const ego_planner::TailBoundary &mission_tail,
    bool suppress_crossing_exempt) const
{
  PhaseRetryCandidate candidate;
  candidate.departure = departure;
  const Contract junction = contractFromVertex(
      route.path, vertex, pm_->maxVel(), state.phase_tan_grade);
  candidate.contracts = contracts;
  candidate.contracts[departure ? 0 : candidate.contracts.size() - 1] = junction;
  candidate.slices = sliceCommittedRoute(route.path, route.cap,
                                         candidate.contracts, route.edge_leg);
  if (candidate.slices.size() != static_cast<size_t>(state.segments))
    return std::nullopt;

  auto &edge = candidate.edge;
  auto &neighbor = candidate.neighbor;
  edge.suppress_crossing_exempt = suppress_crossing_exempt;
  neighbor.suppress_crossing_exempt = suppress_crossing_exempt;
  if (departure) {
    edge.path = buildDepartureConnector(effective_head.pos_u,
                                        effective_head.vel_u,
                                        junction.pos, state.phase_turn_radius_u);
    if (!validateConnector(edge.path, state.phase_tan_grade)) {
      log_->warnf("[PHASE-DEP] connector to s-candidate %d failed "
                  "terrain/zone/grade validation — next", vertex);
      return std::nullopt;
    }
    // The connector is not a route slice. Preserve its arc-varying cap by
    // sampling the original committed route through this handoff vertex.
    edge.cap = capAlongRoute(route.path, route.cap, edge.path, vertex);
    edge.head_pos = effective_head.pos_u;
    edge.head_vel = effective_head.vel_u;
    edge.head_acc = effective_head.acc_u;
    edge.tail = ego_planner::TailBoundary::pinned(junction.vel, junction.acc);
    neighbor.path = candidate.slices[1].path;
    neighbor.cap = candidate.slices[1].cap;
    neighbor.head_pos = junction.pos;
    neighbor.head_vel = junction.vel;
    neighbor.head_acc = junction.acc;
    neighbor.tail = 1 == state.segments - 1
                        ? mission_tail
                        : ego_planner::TailBoundary::pinned(
                              candidate.contracts[1].vel,
                              candidate.contracts[1].acc);
  } else {
    const int last = state.segments - 1;
    const Contract &previous =
        contracts[static_cast<size_t>(last - 2 >= 0 ? last - 2 : 0)];
    edge.path = candidate.slices.back().path;
    edge.cap = candidate.slices.back().cap;
    edge.head_pos = junction.pos;
    edge.head_vel = junction.vel;
    edge.head_acc = junction.acc;
    edge.tail = mission_tail;
    neighbor.path = candidate.slices[static_cast<size_t>(last - 1)].path;
    neighbor.cap = candidate.slices[static_cast<size_t>(last - 1)].cap;
    neighbor.head_pos = last - 1 == 0 ? effective_head.pos_u : previous.pos;
    neighbor.head_vel = last - 1 == 0 ? effective_head.vel_u : previous.vel;
    neighbor.head_acc = last - 1 == 0 ? effective_head.acc_u : previous.acc;
    neighbor.tail = ego_planner::TailBoundary::pinned(junction.vel, junction.acc);
  }
  edge.goal_pos = edge.path.back();
  neighbor.goal_pos = neighbor.path.back();
  return candidate;
}

std::optional<SegmentChainPlanner::PhaseRetrySolution>
SegmentChainPlanner::solvePhaseRetry(const PlanContext &state,
    const PhaseRetryCandidate &candidate,
    const std::vector<SegmentOverrides> &effective_overrides,
    const RouteSliceConfig &config, double peak_max) const
{
  // Configuration order stays edge then neighbor, even when arrival must
  // solve its neighbor first. Both snapshots finish before either solve.
  auto edge_optimizer = pm_->makeConfiguredOptimizer(
      effective_overrides[static_cast<size_t>(
          candidate.departure ? 0 : state.segments - 1)].params);
  auto neighbor_optimizer = pm_->makeConfiguredOptimizer(
      effective_overrides[static_cast<size_t>(
          candidate.departure ? 1 : state.segments - 2)].params);
  const auto solve_accepted = [&](ego_planner::PolyTrajOptimizer &optimizer,
                                  const RouteSliceRequest &request,
                                  poly_traj::Trajectory *trajectory) {
    RouteSliceResult solved = solveRouteSlice(optimizer, request, config);
    if (!solved.hasTrajectory()) return false;
    *trajectory = std::move(solved.trajectory);
    if (solved.envelope_peak > peak_max) {
      log_->warnf("[PHASE-DEP] retry solve peak %.1f%% > %.0f%% — "
                  "rejected", 100.0 * solved.envelope_peak, 100.0 * peak_max);
      return false;
    }
    return true;
  };
  PhaseRetrySolution solution;
  // Short circuit is intentional: a rejected first solve prevents the
  // second solve, and neither trajectory is adopted unless both pass.
  const bool accepted = candidate.departure
      ? solve_accepted(*edge_optimizer, candidate.edge, &solution.edge) &&
            solve_accepted(*neighbor_optimizer, candidate.neighbor,
                            &solution.neighbor)
      : solve_accepted(*neighbor_optimizer, candidate.neighbor,
                        &solution.neighbor) &&
            solve_accepted(*edge_optimizer, candidate.edge, &solution.edge);
  if (!accepted) return std::nullopt;
  return solution;
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
    const TransitionPrefix *transition,
    const std::vector<size_t> &route_edge_leg, uint64_t route_epoch)
{
  return planOverRoute(CommittedRoute{route, cap, route_edge_leg, route_epoch,
                                      fe_ms},
                        head, waypoints, run_parallel, mission_tail, transition);
}

PlanResult SegmentChainPlanner::planOverRoute(
    const CommittedRoute &committed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints, bool run_parallel,
    const ego_planner::TailBoundary &mission_tail,
    const TransitionPrefix *transition)
{
  pm_->beginPlanningComputation();
  PlanContext state(last_plan_.segments);
  state.auto_segments = last_plan_.auto_segments;
  state.transition_active = legacy_transition_active_;
  PlanningComputation computed;
  try {
    computed.status = computeOverRoute(state, computed, committed, head, waypoints,
                                      run_parallel, mission_tail, transition);
  } catch (const InvalidHardProfile &error) {
    computed = PlanningComputation{};
    computed.status = PlanResult::failed(error.what());
    state.verdict = FlightVerdict{};
    state.spans.clear();
  } catch (...) {
    captureDiagnostics(state);
    throw;
  }
  computed.evaluation = state.verdict;
  computed.phases = state.spans;
  captureDiagnostics(state);
  computed = pm_->sealPlanningComputation(std::move(computed));
  if (!pm_->applyPlanningComputation(computed).applied)
    return PlanResult::failed("computed route plan could not be adopted");
  return computed.status;
}

PlanResult SegmentChainPlanner::computeOverRoute(
    PlanContext &parent, PlanningComputation &computed,
    const CommittedRoute &committed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints, bool run_parallel,
    const ego_planner::TailBoundary &mission_tail,
    const TransitionPrefix *transition)
{
  PlanContext attempt(parent.segments);
  attempt.auto_segments = parent.auto_segments;
  // A supplied prefix must survive every successful result. Protect it
  // from direct fallback without requiring a second, mutable caller flag.
  attempt.transition_active = parent.transition_active || transition != nullptr;
  const auto return_diagnostics = [&] {
    parent.segments = parent.confirmed_segments = attempt.confirmed_segments;
    parent.head_policy = attempt.head_policy;
    parent.verdict = attempt.verdict;
    parent.spans = std::move(attempt.spans);
  };
  try {
    PlanResult result = computeRouteAttempt(attempt, computed, committed, head,
                                            waypoints, run_parallel,
                                            mission_tail, transition);
    return_diagnostics();
    return result;
  } catch (...) {
    return_diagnostics();
    throw;
  }
}

PlanResult SegmentChainPlanner::computeRouteAttempt(PlanContext &state,
    PlanningComputation &computed, const CommittedRoute &committed,
    const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints, bool run_parallel,
    const ego_planner::TailBoundary &mission_tail,
    const TransitionPrefix *transition)
{
  const auto &route = committed.path;
  const auto &cap = committed.cap;
  const auto &route_edge_leg = committed.edge_leg;
  const double fe_ms = committed.front_end_ms;
  const Eigen::Vector3d &start_vel = head.vel_u;
  if (const auto invalid = validateRouteAttempt(committed, head, transition))
    return *invalid;
  RouteWork work;
  work.started = std::chrono::steady_clock::now();
  work.run_parallel = run_parallel;
  const auto t_wall = work.started;
  const auto ms_since = [](const std::chrono::steady_clock::time_point &t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  };

  // Single-shot fallback for anything the route mode cannot author.
  // [PHASE] Effective per-worker override lists (mission-wide < phase
  // profile by position < seg<i>; whitelisted keys only) — filled after N
  // is known, read by the fallback for hard-limit preservation.
  auto &workers = work.workers;
  const auto &eff = workers.effective;

  // as_degraded=false marks the one caller where single-shot IS the correct
  // plan (mission below the split threshold), not a repair.
  const auto fallback = [&](const char *why, bool as_degraded = true) {
    return planRouteFallback(state, computed, head, waypoints, mission_tail, eff, why,
                             as_degraded);
  };

  const bool phase_requested =
      readNumParam(node_, "chain/phase/enable", 0.0) != 0.0;
  if (!resolveAutoSegments(state, static_cast<int>(route.size()) - 1, "route"))
    // Direct plan on a too-small mission is the CORRECT answer — except
    // when phase semantics were requested and cannot be delivered.
    return fallback(phase_requested
                        ? "auto-N below threshold (phase mode: direct)"
                        : "auto-N: mission below the split threshold",
                    /*as_degraded=*/phase_requested);
  // [HEAD-POLICY] resolved BEFORE authoring: the departure handoff
  // screening measures the turn the EFFECTIVE start velocity needs. The
  // rule itself lives in start_state.h — this block used to re-implement
  // both [VEL-ALIGN] and [STALL-FLOOR], re-deriving the floor arithmetic by
  // hand instead of calling the accessor, and the two copies had drifted:
  // the numeric-equality rule at the speed boundaries reached the other one
  // and not this. A transition-prescribed head is exempt STRUCTURALLY — it
  // already passed the handoff gate — and that exemption is now a SOURCE
  // rather than a local pointer test.
  // THE HEAD WE WERE GIVEN. Rebuilding it here — which this function did
  // until now, from a boolean derived two frames up — collapsed
  // TRAJECTORY_DERIVED, STATED_VECTOR and TEST_INJECTED all into
  // CHAIN_JUNCTION, so a trajectory-derived head silently lost its floor
  // correction and two operator forms were misclassified. Threading the
  // value and then re-deriving it at the far end is worse than not
  // threading it: it looks correct.
  //
  // (the prefix/source invariant is checked at the entry, above every
  // fallback that could discard the prefix before it was validated)
  const double um_head =
      readNumParam(node_, "optimization/dynamics_unit_xy_m", 100.0);
  // No pointer test. TRANSITION_HANDOFF is neither re-aimed nor floored
  // BECAUSE OF WHAT IT IS, and applyHeadPolicy knows that from the source —
  // passing floor_u = 0 to suppress the rule encoded the same fact a second
  // time, in a place that could disagree with the first.
  const double floor_head = pm_->cruiseFloorUnits();
  const double eps_head =
      um_head > 1e-9 ? PathManager::kSpeedBoundaryEpsMps / um_head : 0.0;
  const HeadPolicy hp0 = applyHeadPolicy(
      head, route, floor_head, pm_->alignStartVelToRoute(), eps_head);
  state.head_policy = hp0;
  Eigen::Vector3d v0 = hp0.vel_u;
  if (hp0.floored)
    log_->warnf("[CHAIN-PAR] start speed %.3f below the margin-backed cruise "
                "floor %.3f — raised (same contract as [STALL-FLOOR])",
                start_vel.norm(), hp0.floor_u);
  if (hp0.floor_declined)
    log_->infof("[HEAD-POLICY] head speed %.3f u/s is below the "
                "margin-backed cruise floor %.3f u/s and was KEPT — source "
                "%s may not be rewritten",
                v0.norm(), hp0.floor_u, sourceName(head.src));

  // === 2. author contracts + slices on the route ===
  auto &contracts = work.contracts;
  if (!authorContractsFromRoute(state, route, v0, &contracts))
    return fallback("route too small to author junctions on");
  applyContractJitter(route, &contracts);
  auto &slices = work.slices;
  slices = sliceCommittedRoute(route, cap, contracts, route_edge_leg);
  if (slices.size() != static_cast<size_t>(state.segments))
    return fallback("committed route did not slice cleanly");

  work.author_ms = ms_since(t_wall);

  workers = prepareRouteWorkers(state, contracts, run_parallel, phase_requested);
  const bool sup = pm_->suppressCrossingExemptNow();
  // Snapshot the manager-owned slice settings after configuration. max_vel
  // is manager/max_vel; each optimizer retains its own optimization/max_vel
  // override. Altitude settings cannot differ per worker. The computation
  // can now use these same values without reading the manager.
  const RouteSliceConfig slice_config = pm_->sliceSolveConfig();

  StartHead effective_head = head;
  effective_head.vel_u = v0;
  work.solved = solveRouteSegments(state, work, effective_head, mission_tail,
                                    slice_config, sup);

  if (const auto fallback_result = repairRouteSegments(
          state, computed, committed, head, waypoints, mission_tail,
          work, slice_config, sup, phase_requested))
    return *fallback_result;
  return finishRouteFlight(state, computed, work, mission_tail, transition,
                           fe_ms);
}


std::optional<PlanResult> SegmentChainPlanner::validateRouteAttempt(
    const CommittedRoute &committed, const StartHead &head,
    const TransitionPrefix *transition) const
{
  const auto &route = committed.path;
  const auto &cap = committed.cap;
  const auto &start_pos = head.pos_u;
  const auto &start_vel = head.vel_u;
  const auto &start_acc = head.acc_u;
  // [HEAD-POLICY] The head's own contract, checked HERE rather than beside
  // the policy call. The auto-N fallback below returns before that point on
  // a short route, so a prefix/source disagreement could be discarded with
  // the prefix instead of refused — a contract violation resolved by
  // quietly preferring one of the two disagreeing facts.
  if (head.src == StartStateSource::UNSPECIFIED)
    return PlanResult::failedBecause(
        PlanReason::INITIAL_STATE_UNSPECIFIED,
        "start state has no source — the planner will not invent one");
  if (!head.pos_u.allFinite() || !head.vel_u.allFinite() ||
      !head.acc_u.allFinite())
    return PlanResult::failedBecause(PlanReason::INITIAL_STATE_MALFORMED,
                                     "start state is not finite");
  if ((transition != nullptr) !=
      (head.src == StartStateSource::TRANSITION_HANDOFF)) {
    log_->errorf("[HEAD-POLICY] transition prefix %s but head source is %s — "
                 "the same fact disagrees with itself",
                 transition != nullptr ? "PRESENT" : "absent",
                 sourceName(head.src));
    return PlanResult::failedBecause(
        PlanReason::TRANSITION_ADAPTER_UNSOUND,
        "transition prefix and start-state source disagree");
  }
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
  return std::nullopt;
}

SegmentChainPlanner::RouteRunResults SegmentChainPlanner::solveRouteSegments(
    PlanContext &state, const RouteWork &work, const StartHead &effective_head,
    const ego_planner::TailBoundary &mission_tail,
    const RouteSliceConfig &slice_config, bool sup)
{
  const auto &contracts = work.contracts;
  const auto &slices = work.slices;
  const auto &eff = work.workers.effective;
  const auto &opts = work.workers.optimizers;
  const bool run_parallel = work.run_parallel;
  const auto &start_pos = effective_head.pos_u;
  const auto &start_acc = effective_head.acc_u;
  const auto &v0 = effective_head.vel_u;
  // Boundary/tail policy is prepared serially; the batch function only
  // solves the supplied values on independent configured optimizers.
  std::vector<RouteSliceRequest> requests;
  std::vector<char> skip(static_cast<size_t>(state.segments), 0);
  requests.reserve(static_cast<size_t>(state.segments));
  state.jitter_fail_segment = static_cast<int>(
      readNumParam(node_, "chain/jitter/fail_segment", 0.0));
  for (int i = 0; i < state.segments; ++i) {
    const bool last = (i + 1 == state.segments);
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
      double ceil_v = pm_->maxVel();
      for (const auto &parameter : eff[ui].params)
        if (parameter.get_name() == "optimization/max_vel")
          ceil_v = parameter.as_double();
      if (ceil_v < pm_->maxVel() - 1e-12 && slices[ui].path.size() >= 2) {
        Eigen::Vector3d ad = slices[ui].path.back() -
                             slices[ui].path[slices[ui].path.size() - 2];
        ad.z() = 0.0;
        if (ad.norm() < 1e-6) ad = Eigen::Vector3d::UnitX();
        tb = completeArrivalTail(tb, ad.normalized() * ceil_v);
        log_->infof("[PHASE] arrival tail at the segment ceiling: |v|=%.3f "
                    "u/s level", ceil_v);
      }
    }
    requests.push_back(
        {slices[ui].path, slices[ui].cap, hp, hv, ha, gp, tb, sup});
    if (state.jitter_fail_segment == i + 1 || state.jitter_fail_segment == -1) {
      log_->warnf("[CHAIN-JITTER] EXPERIMENT: segment %d/%d failure "
                  "INJECTED", i + 1, state.segments);
      skip[ui] = 1;
    }
  }
  RouteSliceBatchResult solved = solveRouteSliceBatch(
      opts, std::move(requests), slice_config, run_parallel, skip);
  RouteRunResults result;
  result.wall_ms = solved.wall_ms;
  auto &runs = result.runs;
  auto &ok = result.ok;
  auto &solve_ms = result.solve_ms;
  runs.reserve(solved.runs.size());
  ok.reserve(solved.runs.size());
  solve_ms.reserve(solved.runs.size());
  for (auto &run : solved.runs) {
    ok.push_back(run.result.hasTrajectory() ? 1 : 0);
    solve_ms.push_back(run.solve_ms);
    runs.push_back(std::move(run.result.trajectory));
  }

  // [JITTER] One-shot semantics for the failure injection: clear it the
  // moment it fired so a value left behind on a running node (or a stale
  // test yaml) cannot condemn every subsequent mission (review find). The
  // WARN above marks the injected plan; this reset marks the recovery.
  if (state.jitter_fail_segment == -1 ||
      (state.jitter_fail_segment > 0 && state.jitter_fail_segment <= state.segments)) {
    node_->set_parameters(
        {rclcpp::Parameter("chain/jitter/fail_segment", 0)});
    log_->warnf("[CHAIN-JITTER] fail_segment fired — cleared to 0 "
                "(one-shot)");
  }

  return result;
}

std::optional<PlanResult> SegmentChainPlanner::repairRouteSegments(
    PlanContext &state, PlanningComputation &computed,
    const CommittedRoute &committed, const StartHead &head,
    const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &mission_tail, RouteWork &work,
    const RouteSliceConfig &slice_config, bool sup, bool phase_requested)
{
  auto &contracts = work.contracts;
  auto &slices = work.slices;
  auto &workers = work.workers;
  const auto &eff = workers.effective;
  auto &runs = work.solved.runs;
  auto &ok = work.solved.ok;
  auto &solve_ms = work.solved.solve_ms;
  auto &soft_drop_note = work.soft_drop_note;
  const auto &v0 = state.head_policy.vel_u;
  const auto ms_since = [](const std::chrono::steady_clock::time_point &t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0).count();
  };
  const auto fallback = [&](const char *why) {
    return planRouteFallback(state, computed, head, waypoints, mission_tail,
                             eff, why, true);
  };
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
    for (int i = 0; i < state.segments; ++i)
      if (!ok[static_cast<size_t>(i)]) failed.push_back(i);
    const bool merge_on =
        readNumParam(node_, "chain/merge_retry", 1.0) != 0.0;
    if (failed.size() == 1 && merge_on && state.segments >= 2) {
      const int f = failed.front();
      const double peak_max =
          readNumParam(node_, "optimization/audit_envelope_peak_max", 1.30);
      StartHead retry_head = head;
      retry_head.vel_u = v0;
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
      // down, state.arr_candidates was simply empty at N=2 and the ladder fell
      // through reporting "no reachable arrival handoff", which reads as a
      // screening failure rather than a policy.)
      bool edge_rescued = false;
      if (state.phase_applied && state.segments == 2 && f == state.segments - 1) {
        state.phase_applied = false;
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
      if (state.phase_applied && (f == 0 || f == state.segments - 1)) {
        const bool edge_dep = (f == 0);
        bool rescued = false;
        const std::vector<int> &cands =
            edge_dep ? state.dep_candidates : state.arr_candidates;
        // Departure at N>=3 additionally retries the ORIGINAL handoff with
        // a connector seed before moving it: the boundary may be fine and
        // only the route-slice seed wrong.
        std::vector<int> attempt;
        if (edge_dep) attempt.push_back(cands.empty() ? -1 : cands[0]);
        for (size_t ci = 1; ci < cands.size(); ++ci)
          attempt.push_back(cands[ci]);
        for (int c : attempt) {
          if (c < 0 || rescued) break;
          auto candidate = preparePhaseRetry(state, committed, c, edge_dep, contracts,
                                             retry_head, mission_tail, sup);
          if (!candidate) continue;
          auto solution =
              solvePhaseRetry(state, *candidate, eff, slice_config, peak_max);
          if (!solution) continue;

          // Commit the complete two-segment repair together. The original
          // per-worker timing remains unchanged for phase-edge rescues.
          contracts = std::move(candidate->contracts);
          slices = std::move(candidate->slices);
          const size_t edge = static_cast<size_t>(edge_dep ? 0 : state.segments - 1);
          const size_t neighbor =
              static_cast<size_t>(edge_dep ? 1 : state.segments - 2);
          runs[edge] = std::move(solution->edge);
          runs[neighbor] = std::move(solution->neighbor);
          ok[edge] = ok[neighbor] = 1;
          rescued = true;
          log_->warnf("[PHASE-DEP] %s boundary rescued at s-candidate %d "
                      "(%s seed) — phase semantics preserved",
                      edge_dep ? "departure" : "arrival", c,
                      edge_dep ? "connector" : "route");
        }
        if (!rescued) {
          state.phase_applied = false;
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
        const auto sides = selectMergeNeighbors(state, f, slices, contracts.size());
        bool rescued = false;
        for (int neighbor : sides) {
          auto candidate = prepareMergeRetry(state, f, neighbor, slices, contracts,
                                              retry_head, mission_tail, sup);
          const size_t lo = static_cast<size_t>(candidate.lo);
          const size_t hi = static_cast<size_t>(candidate.hi);
          log_->warnf("[CHAIN-RETRY] segment %d/%d failed — merging with "
                      "segment %d (junction %d dropped), re-solving the span",
                      f + 1, state.segments, neighbor + 1, candidate.lo + 1);
          // As before, retry timing starts after geometry preparation and
          // includes profile construction, optimizer setup and the solve.
          const auto t_retry = std::chrono::steady_clock::now();
          const auto profile =
              mergedRetryProfile(state, candidate, workers, phase_requested);
          // Preserve notes from every attempted merge, including failed
          // candidates before a later successful repair.
          if (!profile.soft_drop_note.empty())
            soft_drop_note += (soft_drop_note.empty() ? "" : ", ") +
                              profile.soft_drop_note;
          auto optimizer = pm_->makeConfiguredOptimizer(profile.overrides.params);
          if (!profile.overrides.label.empty())
            log_->infof("[PHASE] merged span profile: %s",
                        profile.overrides.label.c_str());
          RouteSliceResult solved = solveRouteSlice(*optimizer, candidate.request,
                                                     slice_config);
          if (solved.hasTrajectory()) {
            const double retry_ms = ms_since(t_retry);
            runs[lo] = std::move(solved.trajectory);
            runs.erase(runs.begin() + static_cast<long>(hi));
            ok[lo] = 1;
            ok.erase(ok.begin() + static_cast<long>(hi));
            solve_ms[lo] += solve_ms[hi] + retry_ms;
            solve_ms.erase(solve_ms.begin() + static_cast<long>(hi));
            // A merged slice had no per-edge attribution before this
            // extraction; route tags must not become polynomial provenance.
            slices[lo] = RouteSlice{std::move(candidate.request.path),
                                    std::move(candidate.request.cap), {}};
            slices.erase(slices.begin() + static_cast<long>(hi));
            contracts.erase(contracts.begin() + candidate.lo);
            state.segments -= 1;
            log_->warnf("[CHAIN-RETRY] merge rescued the chain: %d segments "
                        "remain (merged solve %.0f ms)", state.segments, retry_ms);
            rescued = true;
            break;
          }
          log_->warnf("[CHAIN-RETRY] merged span with segment %d failed too",
                      neighbor + 1);
        }
        if (!rescued) {
          char why[96];
          snprintf(why, sizeof why,
                   "segment %d/%d failed (merge retries exhausted)", f + 1,
                   state.segments);
          return fallback(why);
        }
      }  // !edge_rescued (generic merge)
    } else if (!failed.empty()) {
      char why[96];
      snprintf(why, sizeof why, "%zu segments failed", failed.size());
      return fallback(why);
    }
  }

  return std::nullopt;
}

PlanResult SegmentChainPlanner::finishRouteFlight(
    PlanContext &state, PlanningComputation &computed, RouteWork &work,
    const ego_planner::TailBoundary &mission_tail,
    const TransitionPrefix *transition, double fe_ms)
{
  const auto &contracts = work.contracts;
  const auto &runs = work.solved.runs;
  const auto &solve_ms = work.solved.solve_ms;
  const auto ms_since = [](const std::chrono::steady_clock::time_point &t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0).count();
  };
  // Assemble values first; the existing evaluation still gates storage
  // and visualization of the accepted whole flight below.
  AssembledFlight assembled = stitchRouteFlight(runs, state.phase_applied);
  auto &chained = assembled.trajectory;
  auto &spans = assembled.phases;
  const poly_traj::Trajectory no_baseline;  // report skips the comparison
  logChainReport(no_baseline, runs, contracts, chained,
                 std::vector<SegmentOverrides>(
                     static_cast<size_t>(state.segments)));

  const double t_pre_terminal = chained.getTotalDuration();
  const poly_traj::Trajectory term =
      appendTerminalPhase(&chained, mission_tail.anyPrescribed());
  if (chained.getTotalDuration() > t_pre_terminal + 1e-9) {
    spans.push_back(
        {chained.getTotalDuration(), PhaseKind::TERMINAL, "terminal"});
  }

  if (transition != nullptr) {
    auto joined = prependFlightPrefix(std::move(assembled), transition->traj,
                                       transition->junction_pva_tol_u);
    log_->infof("[S13] junction seam: dP %.3g dV %.3g dA %.3g u (tol %.3g)"
                ", jerk jump %.3g u/s^3 (measured, ungated)",
                joined.position_error, joined.velocity_error,
                joined.acceleration_error, transition->junction_pva_tol_u,
                joined.jerk_jump);
    if (!joined.joined) {
      recordExecutionInvalidation(computed);
      return PlanResult::failedBecause(
          PlanReason::TRANSITION_JUNCTION_UNSOUND,
          "chain head is not on the transition tail");
    }
    assembled = std::move(joined.flight);
  }

  // [STITCH-GATE] JUDGE BEFORE STORING. The verdict has to precede
  // setLocalTraj and the viz publish: a refused flight left in traj_ is a
  // trajectory the next state transition can pick up, and one already drawn
  // in RViz tells the operator it was accepted. FAILED leaves both untouched.
  const FlightVerdict stitched_fv =
      evaluateFlightInContext(state, chained, spans, nullptr, 0);  // stitched: see planImpl
  if (!stitched_fv.evaluated || stitched_fv.unflyable())
    return stitchedVerdictResult(computed, stitched_fv, PlanResult::success());

  const double now_s = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  // No baseline: the owned reference mirrors the chained product, as the
  // comparison channel does when the caller adopts this mode.
  computed.recordProduct({chained, chained, now_s, now_s});
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
              work.run_parallel ? "PARALLEL" : "sequential", fe_ms, work.author_ms,
              per.c_str(), work.solved.wall_ms,
              *std::max_element(solve_ms.begin(), solve_ms.end()),
              fe_ms + ms_since(work.started), chained.getPieceNum(),
              chained.getTotalDuration());
  // A merge rescue that dropped no stated requirement stays SUCCESS
  // (outcome matrix); profile-loss degrades attach at the merge site once
  // phase profiles exist.
  PlanResult result = PlanResult::success();
  if (!state.phase_note.empty())
    result.degrade(PlanReason::PHASE_BOUNDARY_FALLBACK, state.phase_note);
  if (!work.soft_drop_note.empty())
    result.degrade(PlanReason::PHASE_BOUNDARY_FALLBACK,
                   "merge dropped soft overrides: " + work.soft_drop_note);
  return stitchedVerdictResult(computed, stitched_fv, result);
}

PlanResult SegmentChainPlanner::verdictResultForTest(const FlightVerdict &fv)
{
  PlanningComputation computed;
  computed.status = stitchedVerdictResult(computed, fv, PlanResult::success());
  // A passing synthetic verdict has no product to adopt. Refusing verdicts
  // retain the harness's historical execution invalidation side effect.
  if (computed.invalidate_execution)
    pm_->applyPlanningComputation(pm_->sealPlanningComputation(computed));
  return computed.status;
}

PlanResult SegmentChainPlanner::stitchedVerdictResult(
    PlanningComputation &computed, const FlightVerdict &fv,
    PlanResult ok_result) const
{
  // Fail-closed on UNEVALUATED (review find: unflyable() is false when
  // evaluated is false, so a malformed span set slid a stitched product
  // through as SUCCESS with the trajectory already stored): a flight the
  // evaluator could not judge is refused like one it condemned, and
  // execution invalidation is recorded for the caller to apply.
  if (!fv.evaluated) {
    log_->errorf("[STITCH-GATE] whole-flight evaluation did not run "
                 "(malformed spans or degenerate flight) — FAILED");
    recordExecutionInvalidation(computed);
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
  if (fv.unflyable()) {
    char why[176];
    snprintf(why, sizeof why,
             "stitched flight is unflyable (%s%s%s%s%s) — whole-flight "
             "evaluation, which no per-solve audit performs",
             fv.underground ? "terrain overlap; " : "",
             fv.no_cruise ? "never reaches cruise; " : "",
             fv.zone_hard_n > 0 ? "authored zone entered; " : "",
             fv.zone_junction_hard_n > 0
                 ? "leg handover inside an authored zone; " : "",
             !fv.policy_measurable ? "zone policy unevaluated" : "");
    log_->errorf("[STITCH-GATE] %s", why);
    // Record the refusal even if no product exists yet. The execution
    // adapter reproduces the legacy withdrawal, including after an earlier
    // baseline/direct solve, without exposing an intermediate flight.
    recordExecutionInvalidation(computed);
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
    const poly_traj::Trajectory &flight, const std::vector<PhaseSpan> &spans,
    const std::vector<size_t> *piece_leg, uint64_t piece_epoch) const
{
  PlanContext state(last_plan_.segments);
  last_plan_.verdict = FlightVerdict{};
  last_plan_.spans.clear();
  const FlightVerdict verdict =
      evaluateFlightInContext(state, flight, spans, piece_leg, piece_epoch);
  last_plan_.verdict = state.verdict;
  last_plan_.spans = std::move(state.spans);
  return verdict;
}

SegmentChainPlanner::FlightVerdict SegmentChainPlanner::evaluateFlightInContext(PlanContext &state,
    const poly_traj::Trajectory &flight,
    const std::vector<PhaseSpan> &spans,
    const std::vector<size_t> *piece_leg_in, uint64_t piece_epoch) const
{
  state.verdict = FlightVerdict{};
  state.spans.clear();
  const auto emit = [&](const FlightEvaluationReport &report) {
    for (const auto &line : formatFlightEvaluation(report)) {
      switch (line.level) {
        case FlightEvaluationLogLevel::INFO:
          log_->infof("%s", line.text.c_str()); break;
        case FlightEvaluationLogLevel::WARNING:
          log_->warnf("%s", line.text.c_str()); break;
        case FlightEvaluationLogLevel::ERROR:
          log_->errorf("%s", line.text.c_str()); break;
      }
    }
  };
  FlightEvaluationReport rejected;
  rejected.issue = validateFlightPhases(flight, spans, &rejected.issue_span);
  if (rejected.issue != FlightEvaluationIssue::NONE) {
    rejected.phases = spans;
    rejected.duration = flight.getTotalDuration();
    emit(rejected);
    return {};
  }
  FlightEvaluationInput input{flight, spans};
  const auto param_or = [&](const char *name, double fallback) {
    return readNumParam(node_, name, fallback);
  };
  auto &settings = input.settings;
  settings.unit_xy_m = param_or("optimization/dynamics_unit_xy_m", settings.unit_xy_m);
  settings.unit_z_m = param_or("optimization/dynamics_unit_z_m", settings.unit_z_m);
  settings.obstacle_clearance = param_or("optimization/obstacle_clearance", settings.obstacle_clearance);
  settings.envelope_violation_max = param_or("optimization/audit_envelope_violation_max", settings.envelope_violation_max);
  settings.envelope_peak_max = param_or("optimization/audit_envelope_peak_max", settings.envelope_peak_max);
  input.model = pm_->vehicleModel();
  input.plan_policy = pm_->zonePolicySnapshot();
  input.leg_policies = pm_->legPolicySnapshots();
  input.piece_leg = piece_leg_in;
  input.piece_epoch = piece_epoch;
  input.policy_epoch = pm_->zonePolicyEpoch();
  auto &environment = input.environment;
  environment.zone_count = pm_->numRiskZones();
  environment.terrain_elevation = [this](double x, double y, double *z) {
    return pm_->terrainElevation(x, y, z);
  };
  environment.effective_risk = [this](size_t zone, const Eigen::Vector3d &p) {
    return pm_->getEffectiveRisk(zone, p);
  };
  environment.ellipsoid_radius = [this](size_t zone, const Eigen::Vector3d &p) {
    return pm_->getZoneEllipsoidRadius(zone, p);
  };
  environment.risk_visibility = [this](size_t zone, const Eigen::Vector3d &p) {
    return pm_->getRiskVisibility(zone, p);
  };
  environment.zone_contact = [this](const ZonePolicySnapshot &policy, size_t zone,
                                     const Eigen::Vector3d &p) {
    return pm_->zoneContact(policy, zone, p);
  };
  environment.authored_contact = [this](const ZonePolicySnapshot &policy, size_t zone,
                                         const Eigen::Vector3d &p) {
    return pm_->zoneContactAuthored(policy, zone, p);
  };
  const auto report = path_manager::evaluateFlight(input);
  emit(report);
  state.verdict = report.verdict;
  if (state.verdict.evaluated) state.spans = spans;
  return state.verdict;
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
  // Keep the final occurrence authoritative even for malformed hard values:
  // an earlier invalid value superseded by a valid one is not active, while
  // a final invalid value cannot silently resurrect an earlier valid value.
  std::map<std::string, std::string> hard_errors;
  if (!node_->has_parameter(pname))
    node_->declare_parameter(pname, std::vector<std::string>{});
  std::vector<std::string> specs;
  node_->get_parameter(pname, specs);
  {
    for (const std::string &spec : specs) {
      const size_t eq = spec.find('=');
      if (eq == std::string::npos || eq == 0 || eq + 1 >= spec.size()) {
        const std::string name = spec.substr(0, eq);
        const OvrKey *key = whitelistFind(name);
        if (key && key->tag == OvrTag::kHardMax)
          hard_errors[name] = "invalid hard profile " + who + " '" + spec +
                              "': hard maximum requires name=value";
        else
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
                    "startup) — its consumer will not see this per-call "
                    "value this mission", who.c_str(), name.c_str());
      }
      const OvrKey *key = whitelistFind(name);
      const bool hard_max = key && key->tag == OvrTag::kHardMax;
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
            if (hard_max && (!std::isfinite(d) || !(d > 0.0)))
              throw std::invalid_argument("hard maximum must be finite and positive");
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
        if (hard_max)
          hard_errors[name] = "invalid hard profile " + who + " '" + spec +
                              "': " + e.what();
        else
          log_->warnf("[CHAIN] %s override '%s': value parse failed (%s) — "
                      "skipped", who.c_str(), spec.c_str(), e.what());
        continue;
      }
      if (hard_max) hard_errors.erase(name);
      so.label += (so.label.empty() ? "" : ", ") + spec;
    }
  }
  so.invalid_hard = std::move(hard_errors);
  return so;
}

std::vector<SegmentChainPlanner::SegmentOverrides>
SegmentChainPlanner::readSegmentOverrides(const PlanContext &state) const
{
  std::vector<SegmentOverrides> out(static_cast<size_t>(state.segments));
  for (int i = 0; i < state.segments; ++i) {
    auto &profile = out[static_cast<size_t>(i)];
    profile = readOverrideList(
        "chain/seg" + std::to_string(i + 1) + "/params",
        "seg" + std::to_string(i + 1));
    // No later profile can override a segment entry. This also validates
    // baseline-mode profiles, which do not use route phase composition.
    if (!profile.invalid_hard.empty())
      throw InvalidHardProfile(profile.invalid_hard.begin()->second);
  }
  return out;
}

std::vector<SegmentChainPlanner::RouteSlice>
SegmentChainPlanner::sliceCommittedRoute(
    const std::vector<Eigen::Vector3d> &route,
    const std::vector<double> &cap,
    const std::vector<Contract> &contracts,
    const std::vector<size_t> &edge_leg) const
{
  if (route.size() < 2) return {};
  const bool has_cap = cap.size() == route.size();
  // [LEG-POLICY] Provenance is sliced alongside the geometry or not at all.
  const bool has_leg = edge_leg.size() + 1 == route.size();

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
    // `leg` is the route edge the pushed point ARRIVES along; a cut splits an
    // edge and both halves keep it, so the two clipped ends name the route
    // edges their cuts landed on.
    auto push = [&](const Eigen::Vector3d &p, double cp, size_t leg) {
      if (!s.path.empty() && (p - s.path.back()).norm() < 1e-3) {
        // Coincident with the previous vertex: keep the larger cap so the
        // dedup never tightens the ceiling.
        if (has_cap && !s.cap.empty())
          s.cap.back() = std::max(s.cap.back(), cp);
        return;
      }
      if (!s.path.empty() && has_leg) s.edge_leg.push_back(leg);
      s.path.push_back(p);
      if (has_cap) s.cap.push_back(cp);
    };
    push(a.pos, a.cap, 0);   // first point of the slice: no edge yet
    for (size_t i = a.seg + 1; i <= b.seg; ++i)
      push(route[i], has_cap ? cap[i] : 0.0,
           has_leg ? edge_leg[i - 1] : 0);
    push(b.pos, b.cap, has_leg ? edge_leg[b.seg] : 0);
    if (s.path.size() < 2) return {};  // degenerate span
    if (has_leg && s.edge_leg.size() + 1 != s.path.size()) return {};
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
