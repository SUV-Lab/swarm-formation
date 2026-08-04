#include "path_manager/segment_chain_planner.h"

#include <chrono>

namespace path_manager {

SegmentChainPlanner::SegmentChainPlanner(rclcpp::Node::SharedPtr node,
                                         std::shared_ptr<PathManager> path_manager,
                                         swarm_formation::LogManager *log_manager,
                                         int segments)
    : node_(node), pm_(path_manager), log_(log_manager),
      segments_(std::max(2, segments)) {}

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
  for (int i = 1; i < segments_; ++i) {
    Contract c;
    c.t = clearJunctionTime(T * i / segments_, baseline);
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
    // Contract heads are trajectory-derived states — never re-aim them.
    pm_->setStartVelSynthesized(i == 0 ? start_vel_synthesized : false);
    if (!pm_->planGlobalTraj(head_pos, head_vel, head_acc, goal,
                             end_vel, end_acc, /*junction_goal=*/!last)) {
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

double SegmentChainPlanner::clearJunctionTime(
    double t, const poly_traj::Trajectory &traj) const
{
  if (!nearRiskZone(traj.getPos(t))) return t;
  const double T = traj.getTotalDuration();
  for (double step = 0.02; step <= 0.10 + 1e-9; step += 0.02) {
    for (const double sgn : {+1.0, -1.0}) {
      const double cand = t + sgn * step * T;
      if (cand <= 0.05 * T || cand >= 0.95 * T) continue;
      if (!nearRiskZone(traj.getPos(cand))) {
        log_->infof("[CHAIN] junction @%.1f s nudged to %.1f s (clear of "
                    "zone moat+taper)", t, cand);
        return cand;
      }
    }
  }
  return t;  // the caller logs the contamination warning
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
  (void)baseline;
  (void)contracts;
  (void)chained;
  // Seam continuity: both sides of every junction were solved against the
  // SAME hard PVA contract, so these deltas are solver arithmetic, not
  // geometry — anything above ~1e-6 means a BC was mangled on the way in.
  for (size_t i = 0; i + 1 < runs.size(); ++i) {
    const auto &a = runs[i];
    const auto &b = runs[i + 1];
    const Eigen::Vector3d dp = a.getJuncPos(a.getPieceNum()) - b.getJuncPos(0);
    const Eigen::Vector3d dv = a.getJuncVel(a.getPieceNum()) - b.getJuncVel(0);
    const Eigen::Vector3d da = a.getJuncAcc(a.getPieceNum()) - b.getJuncAcc(0);
    log_->infof("[CHAIN-REPORT] seam %zu: |dP|=%.3e u |dV|=%.3e u/s "
                "|dA|=%.3e u/s^2", i + 1, dp.norm(), dv.norm(), da.norm());
  }
}

}  // namespace path_manager
