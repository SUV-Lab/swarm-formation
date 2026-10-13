#ifndef SEGMENT_CHAIN_PLANNER_H
#define SEGMENT_CHAIN_PLANNER_H

#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <vector>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include "path_manager/path_manager.h"
#include "path_manager/planning_result.h"
#include "path_manager/flight_evaluation.h"
#include "path_manager/committed_route.h"

namespace path_manager {

// Coordinates the existing baseline and route-authored planning modes.
// RosPlanningBackend owns this component and applies its computed execution
// effects through PathManager. ReplanFSM retains ROS command handling,
// publication and execution state. Route mode commits one front-end route, prepares
// boundary contracts, solves independent segments sequentially or in
// parallel, then applies the existing retry/fallback and whole-flight gates.
// Geometry, units, three-pass routing policy and phase semantics are shared
// with PathManager rather than implemented again here.
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

  // Calculate an owned result without adopting into the execution slot.
  // StartHead supplies PVA (u, u/s, u/s^2), provenance and whether acceleration
  // was prescribed. Unspecified provenance and non-finite states are refused.
  // Provenance controls validation, transition dispatch and the existing
  // re-aim/floor policy; later segment heads are validated junction states.
  // mission_tail is the final boundary, subject to the existing validation
  // and planning/allow_final_boundary_relaxation policy.
  // The status is PlanResult, not bool: FAILED has no executable result;
  // SUCCESS/DEGRADED retain their existing gates and relaxation reasons.
  // For baseline chain mode, the reference is the optimized baseline rather
  // than the MINCO seed. Route chain mode retains its full-flight reference.
  // The result owns its trajectories, phases and verdict after this planner
  // is destroyed. Its sealed execution context is valid only for the same
  // live manager, current environment/execution revision, and a single apply;
  // a later computation supersedes it. No ID is assigned during calculation.
  // Failed products are diagnostics, never executable: use hasTrajectory().
  // ROS parameter/environment adapters and legacy preview publication remain
  // synchronous; serialize this call with updates to the same manager. An
  // existing environment-change hook may still withdraw an active flight.
  PlanningComputation compute(
      const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail = {});

  // Compatibility: compute, then immediately apply execution effects through
  // PathManager, including any recorded failure invalidation. This wrapper
  // does not publish the execution trajectory; that remains with ReplanFSM.
  PlanResult plan(const StartHead &head,
                  const std::vector<Eigen::Vector3d> &waypoints,
                  const ego_planner::TailBoundary &mission_tail =
                      ego_planner::TailBoundary{});

  int segments() const { return last_plan_.segments; }

  // Public route seams: commitRoute prepares a route and cap; planOverRoute
  // computes over a committed route (or slice), then applies execution effects.
  // The private planFromRoute and transition coordinator instead compose
  // commitRouteInContext and computeOverRoute without applying execution.
  // The transition path inserts its prefix and slices the committed route
  // before computing the remaining flight. Route admission checks sizes,
  // finiteness and head-on-route consistency.
  std::optional<CommittedRoute> commitRoute(
      const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
      bool run_parallel);
  // Compatibility output-parameter adapter; unchanged outputs on failure.
  bool commitRoute(const StartHead &head,
                   const std::vector<Eigen::Vector3d> &waypoints,
                   bool run_parallel, std::vector<Eigen::Vector3d> *route,
                   std::vector<double> *cap, double *fe_ms,
                   std::vector<size_t> *edge_leg = nullptr,
                   uint64_t *epoch = nullptr);
  // [S13] A transition product to PREPEND: planOverRoute then judges the
  // C2 junction (chain head vs transition tail, planner units), shifts
  // every span behind a leading TRANSITION span, and stores/visualizes
  // the FULL flight. The prescribed head also disables the [VEL-ALIGN]/
  // [STALL-FLOOR] rewrites structurally — a trajectory-derived head that
  // already passed the handoff gate must never be re-aimed or floored.
  struct TransitionPrefix {
    poly_traj::Trajectory traj;      // planner units, quintic pieces
    double junction_pva_tol_u{1e-6};
  };
  PlanResult planOverRoute(const std::vector<Eigen::Vector3d> &route,
                           const std::vector<double> &cap, double fe_ms,
                           const StartHead &head,
                           const std::vector<Eigen::Vector3d> &waypoints,
                           bool run_parallel,
                           const ego_planner::TailBoundary &mission_tail,
                           const TransitionPrefix *transition = nullptr,
                           // [LEG-POLICY] Provenance of `route`, sliced with
                           // it. `epoch` is what proves the manager's per-leg
                           // captures still describe these tags.
                           const std::vector<size_t> &route_edge_leg = {},
                           uint64_t route_epoch = 0);
  PlanResult planOverRoute(
      const CommittedRoute &route, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints, bool run_parallel,
      const ego_planner::TailBoundary &mission_tail,
      const TransitionPrefix *transition = nullptr);
  // Preserve existing caller names while sharing plain result types with
  // the whole-mission library entry point.
  using PhaseKind = path_manager::PhaseKind;
  using PhaseSpan = path_manager::PhaseSpan;
  using FlightVerdict = path_manager::FlightVerdict;

  // [LEG-POLICY] `piece_leg` is the caller's statement about what it is
  // handing over: one leg per MINCO piece of THIS trajectory, or nullptr for
  // "no per-piece attribution exists here". It is a parameter rather than a
  // read of PathManager::lastPieceLeg() because that member is a global with
  // respect to this function: whether attribution engaged was decided by a
  // size comparison alone, so a stitched flight that happened to have as many
  // pieces as the last single solve would have been judged under that solve's
  // map. Single-solve callers pass the manager's map; stitch sites pass
  // nullptr and are refused in the usual way when zones make that matter.
  FlightVerdict evaluateFlight(const poly_traj::Trajectory &flight,
                               const std::vector<PhaseSpan> &spans,
                               const std::vector<size_t> *piece_leg,
                               uint64_t piece_epoch) const;
  // Spans of the most recent completed evaluation, paired with its verdict.
  // Empty after an entry refusal or an evaluation that could not run. A
  // failed flight gate may still leave diagnostics for its rejected candidate.
  const std::vector<PhaseSpan> &lastPhaseSpans() const {
    return last_plan_.spans;
  }
  // The whole-flight verdict of the LAST evaluation. Regressions assert on
  // this rather than on log text: a log-string check passes when the log is
  // missing, when the check never ran, and when an older file is read by
  // mistake — three ways to certify a safety property nobody measured.
  const FlightVerdict &lastFlightVerdict() const { return last_plan_.verdict; }
  // [HEAD-POLICY] What the head policy actually DID on the last plan. A
  // regression that asserts "the plan flew" cannot tell whether the floor
  // correction was applied — the solver may cope with a sub-floor head
  // anyway — so re-deriving the source at the far end of the call chain
  // went undetected. The decision is observable rather than inferred.
  const HeadPolicy &lastHeadPolicy() const { return last_plan_.head_policy; }
  // Test seam: judge a SYNTHESISED flight. The refusal branch for a
  // hard-zone contact cannot be reached by planning — the 3-pass exists to
  // prevent exactly that trajectory — so the regression has to supply one.
  FlightVerdict evaluateFlightForTest(
      const poly_traj::Trajectory &flight,
      const std::vector<PhaseSpan> &spans,
      const std::vector<size_t> *piece_leg = nullptr,
      uint64_t piece_epoch = 0) const {
    return evaluateFlight(flight, spans, piece_leg, piece_epoch);
  }
  // ...and the mapping from that verdict to what the caller receives, so a
  // regression can pin BOTH halves: that a hard contact is detected, and
  // that detecting it refuses the flight and drops any stored trajectory.
  PlanResult verdictResultForTest(const FlightVerdict &fv);

  // [S13] Start-state regime classifier (enum, never string-matched): the
  // single entry gate plan() dispatches on. TRANSITION_REQUIRED = outside
  // the cruise envelope but inside the transition model's own validity
  // (activation..ceiling speed band, flight path short of the singularity
  // cone). Over the model ceiling is UNSUPPORTED: the model is undefined
  // there and claiming a transition would be a physics claim v1 forbids.
  enum class StartRegime { CRUISE_VALID, TRANSITION_REQUIRED, UNSUPPORTED };
  // acc_prescribed mirrors the message bool: an UNPRESCRIBED internal
  // acc value (the FSM's 0) is never classification evidence — judgment
  // then uses the velocity state alone (review find: internal a=0 at an
  // in-cone climb reads as an unflyable hold and flipped the regime).
  StartRegime classifyStartState(const Eigen::Vector3d &pos_u,
                                 const Eigen::Vector3d &vel_u,
                                 const Eigen::Vector3d &acc_u,
                                 bool acc_prescribed,
                                 std::string *why) const;
  // [S13] Coordinator-owned: while true, every single-shot fallback inside
  // the route mode returns FAILED instead of re-planning from the mission
  // start (contract §10 — the cruise planner must never re-plan the regime
  // the transition exists to handle). The compatibility adapter applies it to
  // explicit-route calls; a full plan() always starts a fresh request.
  void setTransitionActive(bool on) { legacy_transition_active_ = on; }

  // Clip the head of a committed route at an arc length. PUBLIC only so the
  // tag-suffix rule can be pinned by a pure-function regression: it is const,
  // reads no member state that a caller could disturb, and returns everything
  // it computes. The alternative was to leave that rule verified solely
  // through a consumer that does not exist yet.
  // [S13] Materialize the sub-route from an arc coordinate ONCE — the cut
  // vertex uses the same lerp arithmetic that produced the entry point, so
  // the sub-route head matches the prescribed head to machine precision.
  // cap0 never tightens (max of the straddling caps and the vertex's own
  // z); >= 2 vertices or false.
  bool cutAtArc(const std::vector<Eigen::Vector3d> &route,
                const std::vector<double> &cap, double s_cut,
                std::vector<Eigen::Vector3d> *out_route,
                std::vector<double> *out_cap,
                // [LEG-POLICY] The cut lands INSIDE one route edge and both
                // sides of it keep that edge's leg, so the surviving tail is
                // a suffix of the tag list — no re-derivation, and the first
                // arc interval is the only one clamped.
                const std::vector<size_t> &edge_leg = {},
                std::vector<size_t> *out_edge_leg = nullptr) const;

private:
  // One synchronous request owns all scratch state. Route retries get fresh
  // child contexts; confirmed_segments is the configured/auto-resolved count,
  // while segments may shrink after a successful merge in that attempt.
  struct PlanContext {
    explicit PlanContext(int count)
        : segments(count), confirmed_segments(count) {}
    int segments;
    int confirmed_segments;
    bool auto_segments{false};
    bool transition_active{false};
    bool phase_applied{false};
    std::string phase_note;
    std::vector<int> dep_candidates, arr_candidates;
    double phase_tan_grade{1e9};
    double phase_turn_radius_u{0.0};
    int jitter_fail_segment{0};
    HeadPolicy head_policy{};
    FlightVerdict verdict{};
    std::vector<PhaseSpan> spans;
  };
  struct PlanDiagnostics {
    int segments{2};
    bool auto_segments{false};
    HeadPolicy head_policy{};
    FlightVerdict verdict{};
    std::vector<PhaseSpan> spans;
  };
  void captureDiagnostics(PlanContext &state) const;
  std::optional<CommittedRoute> commitRouteInContext(
      const PlanContext &state, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints, bool run_parallel);
  FlightVerdict evaluateFlightInContext(
      PlanContext &state, const poly_traj::Trajectory &flight,
      const std::vector<PhaseSpan> &spans,
      const std::vector<size_t> *piece_leg, uint64_t piece_epoch) const;
  PlanResult computeRouteAttempt(
      PlanContext &state, PlanningComputation &computed,
      const CommittedRoute &route, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints, bool run_parallel,
      const ego_planner::TailBoundary &mission_tail,
      const TransitionPrefix *transition);
  PlanResult planInto(PlanContext &state, PlanningComputation &computed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail);
  PlanResult computeOverRoute(PlanContext &state,
      PlanningComputation &computed, const CommittedRoute &route,
      const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
      bool run_parallel, const ego_planner::TailBoundary &mission_tail,
      const TransitionPrefix *transition = nullptr);
  // [AUTO-N] chain/segments option interpretation — ONE shared step, run
  // at the fresh request entry so EVERY mission shape (plain chain and
  // the transition coordinator alike) sees the same N policy. Leaving it
  // inside planImpl let the transition branch run on the reset defaults:
  // the live smoke chained 2 segments with no cruise span while the
  // config said auto-sized (review find).
  void readSegmentsOption(PlanContext &state);
  // [S13] The ONE coordinator function owning the section-13 sequence:
  // commit -> snapshot -> entry screening -> generate -> cut -> chain with
  // the transition prefix. the request carries transition_active throughout,
  // so every fallback inside returns FAILED.
  PlanResult planTransitionMission(PlanContext &state,
      PlanningComputation &computed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail);
  // [STITCH-GATE] Turns a whole-flight verdict into the plan outcome for a
  // STITCHED product: unflyable -> FAILED(STITCHED_FLIGHT_UNSAFE), envelope
  // budget exceeded -> the given result degraded, otherwise unchanged.
  // PRIVATE since the coordinator internalized it; the harness reaches it
  // through verdictResultForTest above, and through planOverRoute end to end.
  PlanResult stitchedVerdictResult(
      PlanningComputation &computed, const FlightVerdict &fv,
      PlanResult ok_result) const;
  // Applies the NON-refusing half of the verdict (envelope budget, zone
  // standoff) to a result that is already a success. Shared by the exits
  // that do not go through stitchedVerdictResult.
  void degradeForVerdict(PlanResult *r, const FlightVerdict &fv,
                         const char *what) const;
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
  // effect per segment through an independent optimizer session and local
  // altitude/clearance settings; manager/FSM-side parameters load at startup
  // and are warned about. Profiles never change shared mission parameters,
  // so the baseline, later missions and final evaluation keep their settings.
  struct SegmentOverrides {
    std::vector<rclcpp::Parameter> params;
    std::string label;  // "a=b, c=d" for the report; empty = none
    // Parse errors are overlaid with values. A higher-precedence valid value
    // may replace an invalid phase value; an invalid final value may not
    // resurrect an earlier valid one. Validate only the effective profile.
    std::map<std::string, std::string> invalid_hard;
  };
  // Profiles and the optimizers configured from them form one preparation
  // result. Each worker consumes its own value overlay without changing node
  // parameter values. Initial parallel workers share the core budget.
  struct RouteWorkerSetup {
    SegmentOverrides departure, cruise, arrival;
    std::vector<SegmentOverrides> effective;
    std::vector<std::unique_ptr<ego_planner::PolyTrajOptimizer>> optimizers;
  };
  // Shared resolution for real workers and early direct fallback.
  RouteWorkerSetup readRouteWorkerProfiles(const PlanContext &state,
                                           bool phase_requested) const;
  RouteWorkerSetup prepareRouteWorkers(const PlanContext &state, std::vector<Contract> &contracts,
                                       bool run_parallel,
                                       bool phase_requested);
  PlanResult planRouteFallback(PlanContext &state,
      PlanningComputation &computed,
      const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail,
      const std::vector<SegmentOverrides> &effective_overrides,
      const char *why, bool as_degraded);
  std::vector<SegmentOverrides> readSegmentOverrides(const PlanContext &state) const;
  // One "name=value" string-array parameter parsed against declared types
  // (the seg<i> parser, generalized for the phase profiles).
  SegmentOverrides readOverrideList(const std::string &pname,
                                    const std::string &who) const;

  // [CHAIN-PAR] Route-based mode (chain/author_from_route [+
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
  PlanResult planFromRoute(PlanContext &state,
      PlanningComputation &computed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints, bool run_parallel,
      const ego_planner::TailBoundary &mission_tail);

  // plan() minus the final-boundary validation (which must run exactly
  // once): every internal exit path receives the validated tail.
  PlanResult planImpl(PlanContext &state,
      PlanningComputation &computed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints,
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
    // [LEG-POLICY] Which leg authored each edge of THIS slice, carried over
    // from the committed route's own tags. A cut splits one route edge in
    // two and both halves keep its leg; nothing is re-derived from geometry.
    // Empty when the committed route had no provenance to slice.
    std::vector<size_t> edge_leg;
  };
  struct BaselineSegments {
    std::vector<SegmentOverrides> overrides;
    std::vector<double> max_velocity;
    double global_max_velocity{std::numeric_limits<double>::infinity()};
    std::vector<Contract> contracts;
    std::vector<RouteSlice> slices;
    uint64_t route_epoch{0};
  };
  struct BaselineRunResult {
    std::vector<poly_traj::Trajectory> runs;
    poly_traj::Trajectory chained;
    int failed_segment{-1};
  };
  struct RouteRunResults {
    std::vector<poly_traj::Trajectory> runs;
    std::vector<char> ok;
    std::vector<double> solve_ms;
    double wall_ms{0.0};
  };
  struct RouteWork {
    std::chrono::steady_clock::time_point started;
    bool run_parallel{false};
    double author_ms{0.0};
    std::vector<Contract> contracts;
    std::vector<RouteSlice> slices;
    RouteWorkerSetup workers;
    RouteRunResults solved;
    std::string soft_drop_note;
  };
  PlanResult planMultiWaypoint(
      PlanContext &state, PlanningComputation &computed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail);
  PlanResult planBaselineMission(
      PlanContext &state, PlanningComputation &computed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail);
  BaselineSegments prepareBaselineSegments(
      const PlanContext &state, const StartHead &head,
      const poly_traj::Trajectory &baseline) const;
  BaselineRunResult solveBaselineSegments(
      const PlanContext &state, PlanningComputation &computed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail,
      const poly_traj::Trajectory &baseline, const BaselineSegments &prepared);
  std::optional<PlanResult> validateRouteAttempt(
      const CommittedRoute &committed, const StartHead &head,
      const TransitionPrefix *transition) const;
  RouteRunResults solveRouteSegments(
      PlanContext &state, const RouteWork &work, const StartHead &effective_head,
      const ego_planner::TailBoundary &mission_tail,
      const RouteSliceConfig &slice_config, bool suppress_crossing_exempt);
  std::optional<PlanResult> repairRouteSegments(
      PlanContext &state, PlanningComputation &computed,
      const CommittedRoute &committed, const StartHead &head,
      const std::vector<Eigen::Vector3d> &waypoints,
      const ego_planner::TailBoundary &mission_tail, RouteWork &work,
      const RouteSliceConfig &slice_config, bool suppress_crossing_exempt,
      bool phase_requested);
  PlanResult finishRouteFlight(
      PlanContext &state, PlanningComputation &computed, RouteWork &work,
      const ego_planner::TailBoundary &mission_tail,
      const TransitionPrefix *transition, double front_end_ms);
  // One proposed repair owns its geometry and boundary conditions. The
  // coordinator adopts it only after the solve accepts the entire repair.
  struct MergeRetryCandidate {
    int lo, hi;
    RouteSliceRequest request;
  };
  struct MergeRetryProfile {
    SegmentOverrides overrides;
    std::string soft_drop_note;
  };
  std::vector<int> selectMergeNeighbors(const PlanContext &state,
      int failed, const std::vector<RouteSlice> &slices,
      size_t contract_count) const;
  MergeRetryCandidate prepareMergeRetry(const PlanContext &state,
      int failed, int neighbor, const std::vector<RouteSlice> &slices,
      const std::vector<Contract> &contracts, const StartHead &effective_head,
      const ego_planner::TailBoundary &mission_tail,
      bool suppress_crossing_exempt) const;
  MergeRetryProfile mergedRetryProfile(const PlanContext &state,
      const MergeRetryCandidate &candidate, const RouteWorkerSetup &workers,
      bool phase_requested) const;

  struct PhaseRetryCandidate {
    bool departure;
    std::vector<Contract> contracts;
    std::vector<RouteSlice> slices;
    RouteSliceRequest edge, neighbor;
  };
  struct PhaseRetrySolution {
    poly_traj::Trajectory edge, neighbor;
  };
  std::optional<PhaseRetryCandidate> preparePhaseRetry(const PlanContext &state,
      const CommittedRoute &route, int vertex, bool departure,
      const std::vector<Contract> &contracts, const StartHead &effective_head,
      const ego_planner::TailBoundary &mission_tail,
      bool suppress_crossing_exempt) const;
  std::optional<PhaseRetrySolution> solvePhaseRetry(const PlanContext &state,
      const PhaseRetryCandidate &candidate,
      const std::vector<SegmentOverrides> &effective_overrides,
      const RouteSliceConfig &config, double peak_max) const;

  // Cuts the committed route at every contract position by forward polyline
  // projection (each cut searched from the previous cut's segment on, so
  // slices advance monotonically along the route). Returns one slice per
  // segment; empty result means the geometry did not slice cleanly (caller
  // falls back to per-span front-end search).
  std::vector<RouteSlice> sliceCommittedRoute(
      const std::vector<Eigen::Vector3d> &route,
      const std::vector<double> &cap,
      const std::vector<Contract> &contracts,
      const std::vector<size_t> &edge_leg = {}) const;

  // [CHAIN-PAR] see planFromRoute. Contract.t carries pseudo-time
  // (arc / cruise) for the logs. v0 = the mission's effective start velocity
  // — the departure handoff screening needs it ([PHASE-DEP] design v2).
  // [S13] Handoff screening (frozen v4 [PHASE] rules) extracted whole so
  // the transition coordinator reuses the SAME predicates with a
  // speed-change arc term (accel_arc_u). Const and value-returning: no
  // member state is touched; the caller owns logging and assignment.
  struct ReachLimits {
    double dep_min_arc_u{30.0};
    double calm_window_u{40.0};
    double kappa_calm{0.01};
    double grade_max{1e9};      // grade_frac * tan_grade_max
    double tan_grade_max{1e9};
    double turn_radius_u{0.0};
    double accel_arc_u{0.0};    // transition speed-change arc; 0 = legacy
    size_t max_candidates{3};
  };
  struct HandoffScreenResult {
    std::vector<int> dep_candidates;  // first-fit forward (N>=3 rule)
    std::vector<int> arr_candidates;  // first-fit backward (N>=3 rule)
    int shared_idx{-1};             // N==2 rule: calmest both-way vertex
    double shared_kappa{0.0};
  };
  static std::vector<double> routeArcTable(
      const std::vector<Eigen::Vector3d> &route);
  static std::vector<double> routeCurvature(
      const std::vector<Eigen::Vector3d> &route);
  HandoffScreenResult screenHandoffCandidates(
      const std::vector<Eigen::Vector3d> &route, const Eigen::Vector3d &v0,
      const ReachLimits &rl, double arr_min_arc_u) const;
  bool authorContractsFromRoute(PlanContext &state, const std::vector<Eigen::Vector3d> &route,
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
  // Records a refusal's execution invalidation effect, applied only by the
  // execution adapter. Does not clear stored trajectory during calculation.
  void recordExecutionInvalidation(PlanningComputation &computed) const;
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
  bool resolveAutoSegments(PlanContext &state, int pieces, const char *source);

  // [JITTER] measurement-only contract perturbation (chain/jitter/*, all
  // default 0 = off) for the junction sensitivity experiment.
  void applyContractJitter(const std::vector<Eigen::Vector3d> &route,
                           std::vector<Contract> *contracts);
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PathManager> pm_;
  swarm_formation::LogManager *log_;  // FSM-owned, outlives this component
  const int segments_requested_;
  bool inherit_route_;
  // Compatibility readout and explicit-route adapter option only. Private
  // calculation helpers receive PlanContext and never consume this snapshot.
  mutable PlanDiagnostics last_plan_;
  bool legacy_transition_active_{false};

};

}  // namespace path_manager

#endif  // SEGMENT_CHAIN_PLANNER_H
