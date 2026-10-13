#pragma once

#include <cstdint>
#include <functional>

#include "path_manager/planning_computation.h"
#include "path_optimizer/plan_container.hpp"

namespace path_manager {

enum class ExecutionRevocationReason {
  OBSTACLE_BLOCKED, TERRAIN_BLOCKED, POLICY_UNEVALUATED
};

// Captured while the withdrawn trajectory is still current. Publication
// adapters do not have to query mutable planner state to identify an ABORT.
struct ExecutionRevocation {
  uint64_t trajectory_id{0};
  int drone_id{-1};
  uint64_t environment_generation{0};
  ExecutionRevocationReason reason{ExecutionRevocationReason::OBSTACLE_BLOCKED};
  Eigen::Vector3d hit{Eigen::Vector3d::Zero()};
  double detected_time{0.0};
};

// Owns execution identity, adopted values and pending-computation validity.
// No node, clock, environment or visualization is retained. Calls are serial;
// environment generation and clock points are supplied by the adapter.
// Returned views are const and borrowed until the next mutation/destruction.
// The legacy trajectory data types remain read-only compatibility views.
class PlanningExecutionStore {
 public:
  using RevocationObserver = std::function<void(const ExecutionRevocation &)>;
  explicit PlanningExecutionStore(int drone_id);
  PlanningExecutionStore(const PlanningExecutionStore &) = delete;
  PlanningExecutionStore &operator=(const PlanningExecutionStore &) = delete;

  const ego_planner::LocalTrajData &current() const { return current_; }
  const ego_planner::GlobalTrajData &reference() const { return reference_; }
  int droneId() const { return current_.drone_id; }
  uint64_t identityCounter() const { return next_identity_; }

  void invalidatePending();
  PlanningComputation seal(PlanningComputation value,
                           uint64_t environment_generation) const;
  ExecutionReceipt apply(const PlanningComputation &value,
                         uint64_t environment_generation);
  // The legacy emergency command replaces only the active trajectory. It
  // consumes one ID and leaves the reference trajectory unchanged.
  ExecutionReceipt adoptEmergencyTrajectory(const poly_traj::Trajectory &trajectory,
                                             double start_time);

  void setRevocationObserver(RevocationObserver observer);
  // Notify first, then invalidate the active slot and every pending result.
  // An inactive slot produces no event but retains the legacy invalidation
  // effect. An observer must not reenter/mutate this store; exceptions retain
  // the pre-invalidation state, matching the existing synchronous hook.
  void revoke(ExecutionRevocationReason reason, const Eigen::Vector3d &hit,
              double detected_time, uint64_t environment_generation);

 private:
  void adoptCurrent(const poly_traj::Trajectory &trajectory, double start_time);
  void invalidateCurrent();
  ego_planner::LocalTrajData current_{};
  ego_planner::GlobalTrajData reference_{};
  uint64_t next_identity_{0};
  const uint64_t context_;
  uint64_t revision_{0};
  RevocationObserver observer_;
};

}  // namespace path_manager
