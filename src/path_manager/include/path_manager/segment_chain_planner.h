#ifndef SEGMENT_CHAIN_PLANNER_H
#define SEGMENT_CHAIN_PLANNER_H

#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include "path_manager/path_manager.h"
#include "path_manager/planning_result.h"

namespace path_manager {

// [CHAIN] Stage 1 of the trajectory phase-split work: plan the whole mission
// once (the baseline — today's single-shot behavior), sample junction
// contracts (full PVA) from that trajectory at equal time fractions, then
// re-plan each span with the SAME pipeline (fm2 + MINCO + L-BFGS), chaining
// the runs through the contracts: run N's tail BC and run N+1's head BC are
// the SAME prescribed state, so the stitched trajectory is C2 at every seam
// by construction — the report only confirms it.
//
// The contracts are sampled from the baseline rather than synthesized
// (level-cruise at the junction chord) because the baseline already knows
// what the unsplit optimum does there — climbing, banking — and pinning that
// exact state avoids manufacturing artificial level-outs at the seams. It
// also makes the comparison honest: baseline and chain share their junction
// states, so any shape difference between them is the split itself.
//
// Owned by the FSM, beside the state machine, not inside it: the state
// machine decides WHEN to plan; this component knows only how to turn one
// mission into N chained pipeline runs. PathManager stays a pure single-run
// pipeline throughout.
//
// Scope (deliberate, stage 1): single-goal missions, sequential runs. On any
// segment failure the baseline is restored and flown — the experiment
// degrades to today's behavior, loudly.
class SegmentChainPlanner {
public:
  // inherit_route (default on): chained runs re-solve a SLICE of the
  // baseline's committed front-end route instead of re-running FM2 on their
  // span. A sub-mission's own eikonal can pick a different route homotopy
  // than the baseline took there (observed on r3: west over 1,080 m terrain
  // instead of the baseline's eastern saddle — line-search death + terrain
  // overlap). Off = re-litigate the front end per span, which introduces a
  // variable other than the split itself. Orthogonal to the [STAGE-2]
  // per-segment overrides: those apply with inherit_route either way, and
  // they cannot vary FM conditions (only optimization/* re-reads per
  // segment).
  SegmentChainPlanner(rclcpp::Node::SharedPtr node,
                      std::shared_ptr<PathManager> path_manager,
                      swarm_formation::LogManager *log_manager,
                      int segments, bool inherit_route = true);

  // Chained plan of one mission. Same contract as planGlobalTraj: on true,
  // path_manager->traj_ holds a flyable local trajectory (the chained result,
  // or the baseline when the chain degraded). Under chain mode the GLOBAL
  // slot holds the baseline OPTIMIZED trajectory — not the MINCO seed — so
  // /planning/initial_trajectory becomes the baseline-vs-chain comparison
  // channel in RViz.
  //
  // start_vel_synthesized is forwarded to the baseline run and the first
  // segment ([VEL-ALIGN] applies to both the same way); later segments start
  // from a contract state, which is trajectory-derived and never re-aimed.
  // Returns the planner<->FSM contract (planning_result.h): FAILED = do not
  // fly, SUCCESS = all stated requirements met, DEGRADED = flyable under an
  // approved relaxation (reason + detail say which).
  // mission_tail: the mission's FINAL boundary ([PHASE]). Validated once
  // at this entry (shared envelope validator, finiteness); invalid without
  // the planning/allow_final_boundary_relaxation opt-in fails the plan.
  // start_vel_commanded ([ENVELOPE], contract 1): the start velocity is an
  // EXPLICIT operator input (use_initial_velocity / test injection), not a
  // synthesized or trajectory-derived state. Commanded inputs are validated
  // against the cruise envelope at this entry — outside it the plan FAILS
  // (INITIAL_MODE_UNSUPPORTED) before the front end or any optimizer runs;
  // they are never clamped. Non-commanded starts keep the [STALL-FLOOR]
  // clamp doctrine (our own proxy states may be repaired, inputs may not).
  PlanResult plan(const Eigen::Vector3d &start_pos,
                  const Eigen::Vector3d &start_vel,
                  const Eigen::Vector3d &start_acc,
                  const std::vector<Eigen::Vector3d> &waypoints,
                  bool start_vel_synthesized,
                  const ego_planner::TailBoundary &mission_tail =
                      ego_planner::TailBoundary{},
                  bool start_vel_commanded = false);

  int segments() const { return segments_; }

  // [S13] planRouteParallel decomposed (contract 2 §13): commitRoute runs
  // the ONE front-end pass and hands back the committed route + cap;
  // planOverRoute authors/solves/stitches over an already committed route
  // (or a slice of one) from an arbitrary head PVA, with a fail-closed
  // input contract (sizes, finiteness, head-on-route). A transition
  // coordinator calls them separately with the transition in between;
  // planRouteParallel is their no-transition composition. Public: the
  // coordinator and the harness drive these seams directly.
  bool commitRoute(const Eigen::Vector3d &start_pos,
                   const Eigen::Vector3d &start_vel,
                   const Eigen::Vector3d &start_acc,
                   const std::vector<Eigen::Vector3d> &waypoints,
                   bool run_parallel, std::vector<Eigen::Vector3d> *route,
                   std::vector<double> *cap, double *fe_ms);
  PlanResult planOverRoute(const std::vector<Eigen::Vector3d> &route,
                           const std::vector<double> &cap, double fe_ms,
                           const Eigen::Vector3d &start_pos,
                           const Eigen::Vector3d &start_vel,
                           const Eigen::Vector3d &start_acc,
                           const std::vector<Eigen::Vector3d> &waypoints,
                           bool start_vel_synthesized, bool run_parallel,
                           const ego_planner::TailBoundary &mission_tail);
  // [S13] Which JUDGMENT applies to a span of the flight. The evaluator
  // selects the model by this CONTRACT TYPE — never by parsing the display
  // name (review point: labels are for output; a typo in a string must not
  // silently change physics).
  //   CRUISE     full cruise-model envelope judgment (today's audit)
  //   TERMINAL   prescribed helix — judged under the cruise model today
  //              (distinct kind so a dedicated judgment can attach later)
  //   TRANSITION outside the cruise model BY DEFINITION: excluded from the
  //              cruise envelope statistics and MEASURED only — its own
  //              gate arrives with the transition evaluator (ADR-0002
  //              freezes the v0 limits as experimental, not acceptance).
  //              Terrain/zone/risk checks still apply: those are
  //              model-agnostic.
  enum class PhaseKind { CRUISE, TERMINAL, TRANSITION };
  struct PhaseSpan {
    double t_end{0.0};
    PhaseKind kind{PhaseKind::CRUISE};
    std::string name;  // display only
  };

  struct FlightVerdict {
    bool evaluated{false};  // false: degenerate input, nothing was judged
    bool clean{true};
    bool underground{false};
    bool no_cruise{false};
    double viol_pct{0.0};
    double util_peak{0.0};
    // The two counts above that no trajectory may ever fly with, whatever
    // produced it. Envelope over-utilization is graded separately: it is a
    // margin the caller may knowingly spend, terrain is not.
    bool unflyable() const { return evaluated && (underground || no_cruise); }
  };
  FlightVerdict evaluateFlight(const poly_traj::Trajectory &flight,
                               const std::vector<PhaseSpan> &spans) const;
  // [STITCH-GATE] Turns a whole-flight verdict into the plan outcome for a
  // STITCHED product: unflyable -> FAILED(STITCHED_FLIGHT_UNSAFE), envelope
  // budget exceeded -> the given result degraded, otherwise unchanged.
  PlanResult stitchedVerdictResult(const FlightVerdict &fv,
                                   PlanResult ok_result) const;

  // [S13] Coordinator-owned: while true, every single-shot fallback inside
  // the route mode returns FAILED instead of re-planning from the mission
  // start (contract §10 — the cruise planner must never re-plan the regime
  // the transition exists to handle). Cleared by resetPlanState at every
  // plan() entry; the coordinator re-asserts it after entry.
  void setTransitionActive(bool on) { transition_active_ = on; }

private:
  // Junction contract: the shared boundary state between two adjacent runs.
  struct Contract {
    double t;  // baseline trajectory time the state was sampled at
    Eigen::Vector3d pos, vel, acc;
  };

  // Junction time whose sampled position stays clear of every risk zone's
  // moat + GNRON taper reach. A junction inside that band would earn
  // barrier/taper exemptions the baseline never had — the split would CHANGE
  // the risk field, not just the trajectory. Candidates stay within ±40% of
  // the nominal SPAN (T/segments, not total duration — a total-duration
  // window let adjacent junctions cross once segments > 4) and above
  // t_prev + 20% span, so accepted junction times are monotone with a
  // guaranteed gap by construction. When nothing in the window clears, the
  // nominal time is kept and the caller logs the contamination warning.
  double clearJunctionTime(double t_nominal, double t_prev, double span,
                           const poly_traj::Trajectory &traj) const;
  bool nearRiskZone(const Eigen::Vector3d &p) const;

  // [STAGE-2] Per-segment requirement overrides, from chain/seg<i>/params
  // (string array of "param=value"). Values parse against the DECLARED
  // parameter's type; unknown names skip loudly. Only optimization/* takes
  // effect per segment — the segment run is preceded by a forced optimizer
  // re-init, whose setParam re-reads the whole optimizer parameter surface;
  // manager/FSM-side parameters load at startup and are warned about.
  // Mission-wide values are restored on every exit path: the baseline, the
  // next mission, and any fallback all plan with pristine parameters.
  struct SegmentOverrides {
    std::vector<rclcpp::Parameter> params;
    std::string label;  // "a=b, c=d" for the report; empty = none
  };
  std::vector<SegmentOverrides> readSegmentOverrides() const;
  // One "name=value" string-array parameter parsed against declared types
  // (the seg<i> parser, generalized for the phase profiles).
  SegmentOverrides readOverrideList(const std::string &pname,
                                    const std::string &who) const;

  // [CHAIN-PAR] Route-parallel mode (chain/author_from_route [+
  // chain/parallel]): NO baseline solve. One front-end-only pass commits
  // the route (~0.3 s even at 337 km); junction contracts are authored on
  // it — zone-clear, low-CURVATURE (the a-priori calm: a = v^2 * kappa)
  // and arc-balanced — as (vertex, 3D tangent x cruise, a = 0), which at
  // calm vertices is measurably what the unsplit optimum flies there
  // (r4: |a| = 0.002). Segments then solve on per-worker optimizer
  // instances, concurrently when chain/parallel is set. There is no
  // baseline to restore; failures repair through the retry ladders or the
  // gated single-shot fallback (DEGRADED), which a transition-active plan
  // forbids outright.
  PlanResult planRouteParallel(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &start_acc,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               bool start_vel_synthesized, bool run_parallel,
                               const ego_planner::TailBoundary &mission_tail);

  // plan() minus the final-boundary validation (which must run exactly
  // once): every internal exit path receives the validated tail.
  PlanResult planImpl(const Eigen::Vector3d &start_pos,
                      const Eigen::Vector3d &start_vel,
                      const Eigen::Vector3d &start_acc,
                      const std::vector<Eigen::Vector3d> &waypoints,
                      bool start_vel_synthesized,
                      const ego_planner::TailBoundary &mission_tail);

  // [STAGE-4] Whole-flight JUDGMENT of the FINAL stitched product — the one
  // artifact no per-solve audit ever sees whole (and the terminal phase not
  // at all): terrain clearance, flight-envelope utilization (shared
  // mmp_vehicle_dynamics model) and OR-combined risk exposure, sampled at
  // 10 Hz, per phase and total.
  //
  // The RETURN VALUE is the authority; the per-phase log lines it emits are
  // a readout of the same numbers, never a second opinion (review find: the
  // stitched chain used to discard this verdict entirely while the direct
  // fallback gated on it, so the identical evidence decided differently
  // depending on which product produced it).
  //
  // Contract-2 note: every sample here is judged by the CRUISE model. A
  // transition phase is by definition outside it, so per-phase model
  // selection has to land here before any transition trajectory does —
  // see docs/transition_phase_contract.md §4.

  // [STAGE-3] Optional PRESCRIBED terminal phase (chain/terminal/enable):
  // a helix descent of genuinely different character — analytic geometry,
  // no optimizer — appended after the chain from its handoff state (level,
  // cruise, a = 0 by the arrival contract, which is exactly the state a
  // curvature-ramp helix entry continues from with zero seam error). Logs
  // its own [CHAIN-REPORT] block. Returns the appended terminal trajectory
  // (empty when disabled/degenerate/discarded) so the caller can paint it
  // in the per-segment viz.
  // final_state_prescribed: mission tail active -> terminal geometry is
  // skipped with a WARN (v1 exclusivity; the mission input outranks the
  // yaml toggle).
  poly_traj::Trajectory appendTerminalPhase(poly_traj::Trajectory *chained,
                                            bool final_state_prescribed) const;

  // Stage-1 seam verification + baseline comparison ([CHAIN-REPORT]).
  void logChainReport(const poly_traj::Trajectory &baseline,
                      const std::vector<poly_traj::Trajectory> &runs,
                      const std::vector<Contract> &contracts,
                      const poly_traj::Trajectory &chained,
                      const std::vector<SegmentOverrides> &overrides) const;

  // One chained run's share of the baseline's committed route: the vertices
  // between two cut points, with the exact contract positions injected as
  // the slice endpoints (so route endpoint == boundary condition) and the
  // cap reference carried in lockstep.
  struct RouteSlice {
    std::vector<Eigen::Vector3d> path;
    std::vector<double> cap;
  };
  // Cuts the committed route at every contract position by forward polyline
  // projection (each cut searched from the previous cut's segment on, so
  // slices advance monotonically along the route). Returns one slice per
  // segment; empty result means the geometry did not slice cleanly (caller
  // falls back to per-span front-end search).
  std::vector<RouteSlice> sliceCommittedRoute(
      const std::vector<Eigen::Vector3d> &route,
      const std::vector<double> &cap,
      const std::vector<Contract> &contracts) const;

  // [CHAIN-PAR] see planRouteParallel. Contract.t carries pseudo-time
  // (arc / cruise) for the logs. v0 = the mission's effective start velocity
  // — the departure handoff screening needs it ([PHASE-DEP] design v2).
  bool authorContractsFromRoute(const std::vector<Eigen::Vector3d> &route,
                                const Eigen::Vector3d &v0,
                                std::vector<Contract> *contracts) const;

  // [PHASE-DEP] design v2 (departure worm fix): contract geometry at a
  // route vertex (tangent sheared onto the climb cone, cruise speed, a=0).
  Contract contractFromVertex(const std::vector<Eigen::Vector3d> &route,
                              int i, double cruise,
                              double tan_grade_max) const;
  // Per-vertex ceiling reference for a path that is NOT a route slice (the
  // departure connector). Each point takes the cap of its nearest route
  // segment, by the same max(cap[i], cap[i+1]) rule sliceCommittedRoute
  // applies at a cut — an empty cap makes the solver fall back to the
  // scalar altitude band, so a connector solve would be judged under a
  // different altitude regime than every route-seeded solve (review find).
  // upto_vertex > 0 limits the search to the route prefix ending at that
  // vertex (the connector's handoff) — a corridor that doubles back would
  // otherwise donate a far segment's ceiling to a nearby connector point.
  std::vector<double> capAlongRoute(
      const std::vector<Eigen::Vector3d> &route,
      const std::vector<double> &cap,
      const std::vector<Eigen::Vector3d> &pts,
      int upto_vertex = -1) const;
  // Zeroes the fields ReplanFSM tests before executing a stored trajectory.
  // Called on every FAILED exit that ran after the product was stored.
  void invalidateStoredTrajectory() const;
  // Clears every per-plan member so nothing survives into the next mission
  // (review find: segments_ is decremented mid-plan by the merge ladder and
  // the phase blackboard is only reset on the phase-enabled path).
  void resetPlanState();
  // Turn-out seed following the INITIAL velocity: arc (radius = margin x
  // R_min) chasing the handoff bearing, then a straight leg, z smoothstep.
  // The route slice must NOT seed the departure solve when the initial
  // heading disagrees with the route — that is the worm generator.
  std::vector<Eigen::Vector3d> buildDepartureConnector(
      const Eigen::Vector3d &start, const Eigen::Vector3d &v0,
      const Eigen::Vector3d &handoff, double turn_radius_u) const;
  // Terrain-clearance / risk-zone / grade-cone sampling of a connector.
  bool validateConnector(const std::vector<Eigen::Vector3d> &pts,
                         double tan_grade_max) const;

  // [AUTO-N] resolves chain/segments==0 into a mission-sized N from the
  // piece count; returns false when the mission is too small to split
  // (the caller flies the single-shot result instead).
  bool resolveAutoSegments(int pieces, const char *source);

  // [JITTER] measurement-only contract perturbation (chain/jitter/*, all
  // default 0 = off) for the junction sensitivity experiment.
  void applyContractJitter(const std::vector<Eigen::Vector3d> &route,
                           std::vector<Contract> *contracts);
  // [JITTER] deterministic worker-failure injection (1-based; 0 = off) for
  // testing the merge-retry ladder.
  int jitter_fail_segment_{0};
  // [S13] see setTransitionActive.
  bool transition_active_{false};

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PathManager> pm_;
  swarm_formation::LogManager *log_;  // FSM-owned, outlives this component
  // Working segment count for the CURRENT plan: resolved per plan from
  // segments_requested_ (or auto-sizing) and mutated in flight by the merge
  // ladder. Never read across plans — resetPlanState restores it.
  int segments_;
  // The construction-time request, the one value that outlives a plan.
  const int segments_requested_;
  bool inherit_route_;
  bool auto_segments_{false};
  // [PHASE] set during route authoring (const method -> mutable): whether
  // the departure/arrival handoff rules actually pinned junctions, and the
  // human-readable fallback note when they could not (planRouteParallel
  // attaches it to the PlanResult as PHASE_BOUNDARY_FALLBACK).
  mutable bool phase_applied_{false};
  mutable std::string phase_note_;
  // [PHASE-DEP] screened handoff candidates (route vertex indices, first =
  // the authored one) + the physics the edge-retry ladder reuses.
  mutable std::vector<int> dep_candidates_, arr_candidates_;
  mutable double phase_tan_grade_{1e9};
  mutable double phase_turn_radius_u_{0.0};
};

}  // namespace path_manager

#endif  // SEGMENT_CHAIN_PLANNER_H
