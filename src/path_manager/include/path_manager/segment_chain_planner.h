#ifndef SEGMENT_CHAIN_PLANNER_H
#define SEGMENT_CHAIN_PLANNER_H

#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include "path_manager/path_manager.h"

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
  // overlap). Off = re-litigate the front end per span (the stage-2 mode,
  // where per-span FM conditions are the point).
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
  bool plan(const Eigen::Vector3d &start_pos,
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

  // Stage-1 seam verification + baseline comparison ([CHAIN-REPORT]).
  void logChainReport(const poly_traj::Trajectory &baseline,
                      const std::vector<poly_traj::Trajectory> &runs,
                      const std::vector<Contract> &contracts,
                      const poly_traj::Trajectory &chained) const;

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

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PathManager> pm_;
  swarm_formation::LogManager *log_;  // FSM-owned, outlives this component
  int segments_;
  bool inherit_route_;
};

}  // namespace path_manager

#endif  // SEGMENT_CHAIN_PLANNER_H
