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
  PlanResult plan(const Eigen::Vector3d &start_pos,
                  const Eigen::Vector3d &start_vel,
                  const Eigen::Vector3d &start_acc,
                  const std::vector<Eigen::Vector3d> &waypoints,
                  bool start_vel_synthesized);

  int segments() const { return segments_; }

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

  // [CHAIN-PAR] Route-parallel mode (chain/author_from_route [+
  // chain/parallel]): NO baseline solve. One front-end-only pass commits
  // the route (~0.3 s even at 337 km); junction contracts are authored on
  // it — zone-clear, low-CURVATURE (the a-priori calm: a = v^2 * kappa)
  // and arc-balanced — as (vertex, 3D tangent x cruise, a = 0), which at
  // calm vertices is measurably what the unsplit optimum flies there
  // (r4: |a| = 0.002). Segments then solve on per-worker optimizer
  // instances, concurrently when chain/parallel is set. No baseline means
  // no fallback: a failed segment fails the mission plan.
  PlanResult planRouteParallel(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &start_acc,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               bool start_vel_synthesized, bool run_parallel);

  // [STAGE-4] Whole-flight evaluation of the FINAL stitched product — the
  // one artifact no per-solve audit ever sees whole (and the terminal
  // phase not at all): terrain clearance, flight-envelope utilization
  // (shared mmp_vehicle_dynamics model) and OR-combined risk exposure,
  // sampled at 10 Hz, per phase and total, ending in a CLEAN/CHECK
  // verdict line. Informational, never a gate: publication already
  // happened under the per-solve audits; this block is where the stitched
  // flight's remaining blind spots become visible.
  void logFinalEvaluation(const poly_traj::Trajectory &flight,
                          const std::vector<double> &phase_ends,
                          const std::vector<std::string> &phase_names) const;

  // [STAGE-3] Optional PRESCRIBED terminal phase (chain/terminal/enable):
  // a helix descent of genuinely different character — analytic geometry,
  // no optimizer — appended after the chain from its handoff state (level,
  // cruise, a = 0 by the arrival contract, which is exactly the state a
  // curvature-ramp helix entry continues from with zero seam error). Logs
  // its own [CHAIN-REPORT] block. Returns the appended terminal trajectory
  // (empty when disabled/degenerate/discarded) so the caller can paint it
  // in the per-segment viz.
  poly_traj::Trajectory appendTerminalPhase(poly_traj::Trajectory *chained) const;

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
  // (arc / cruise) for the logs.
  bool authorContractsFromRoute(const std::vector<Eigen::Vector3d> &route,
                                std::vector<Contract> *contracts) const;

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

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PathManager> pm_;
  swarm_formation::LogManager *log_;  // FSM-owned, outlives this component
  int segments_;
  bool inherit_route_;
  bool auto_segments_{false};
};

}  // namespace path_manager

#endif  // SEGMENT_CHAIN_PLANNER_H
