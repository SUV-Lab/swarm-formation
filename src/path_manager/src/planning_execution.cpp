#include "path_manager/planning_execution.h"
#include "path_manager/route_search_backend.h"

#include <limits>
#include <utility>

namespace path_manager {

PlanningExecutionStore::PlanningExecutionStore(int drone_id)
    : context_(nextRouteSearchIdentity())
{
  current_.start_pos.setZero();
  current_.drone_id = drone_id;
  reference_.last_glb_t_of_lc_tgt = -1.0;
}

void PlanningExecutionStore::invalidatePending() { ++revision_; }

PlanningComputation PlanningExecutionStore::seal(
    PlanningComputation value, uint64_t environment_generation) const
{
  value.execution_context = context_;
  value.execution_revision = revision_;
  value.previous_identity_counter = next_identity_;
  value.environment_generation = environment_generation;
  return value;
}

void PlanningExecutionStore::adoptCurrent(
    const poly_traj::Trajectory &trajectory, double start_time)
{
  current_.traj_id = ++next_identity_;
  current_.duration = trajectory.getTotalDuration();
  current_.start_pos = trajectory.getJuncPos(0);
  current_.start_time = start_time;
  current_.traj = trajectory;
}

void PlanningExecutionStore::invalidateCurrent()
{
  current_.duration = 0.0;
  current_.start_time = 0.0;
}

ExecutionReceipt PlanningExecutionStore::apply(
    const PlanningComputation &value, uint64_t environment_generation)
{
  if (value.execution_context != context_ ||
      value.execution_revision != revision_ ||
      value.previous_identity_counter != next_identity_ ||
      value.environment_generation != environment_generation)
    return {};
  if ((value.status.hasTrajectory() &&
       (!value.product || value.invalidate_execution)) ||
      (value.product && (value.identity_advances == 0 ||
       value.product->trajectory.getPieceNum() == 0 ||
       value.product->reference_trajectory.getPieceNum() == 0)) ||
      (!value.product && value.identity_advances != 0) ||
      (!value.status.hasTrajectory() && value.product &&
       !value.invalidate_execution) ||
      value.identity_advances >
          std::numeric_limits<uint64_t>::max() - next_identity_)
    return {};

  ++revision_;  // Accepted no-product failures also consume their context.
  if (value.product) {
    const auto &product = *value.product;
    next_identity_ += value.identity_advances - 1;
    reference_.traj = product.reference_trajectory;
    reference_.duration = product.reference_trajectory.getTotalDuration();
    reference_.global_start_time = product.reference_start_time;
    reference_.glb_t_of_lc_tgt = product.reference_start_time;
    reference_.last_glb_t_of_lc_tgt = -1.0;
    current_.duration = 0.0;
    adoptCurrent(product.trajectory, product.start_time);
  }
  // A post-solve refusal keeps the old diagnostic product/ID effect but
  // never grants execution. This is not an environment ABORT notification.
  if (value.invalidate_execution) invalidateCurrent();
  return {true, current_.traj_id, current_.drone_id};
}

ExecutionReceipt PlanningExecutionStore::adoptEmergencyTrajectory(
    const poly_traj::Trajectory &trajectory, double start_time)
{
  if (trajectory.getPieceNum() == 0 ||
      next_identity_ == std::numeric_limits<uint64_t>::max()) return {};
  ++revision_;
  adoptCurrent(trajectory, start_time);
  return {true, current_.traj_id, current_.drone_id};
}

void PlanningExecutionStore::setRevocationObserver(RevocationObserver observer)
{ observer_ = std::move(observer); }

void PlanningExecutionStore::revoke(
    ExecutionRevocationReason reason, const Eigen::Vector3d &hit,
    double detected_time, uint64_t environment_generation)
{
  if (observer_ && current_.duration > 0.0) {
    const ExecutionRevocation event{current_.traj_id, current_.drone_id,
        environment_generation, reason, hit, detected_time};
    observer_(event);
  }
  ++revision_;
  invalidateCurrent();
}

}  // namespace path_manager
