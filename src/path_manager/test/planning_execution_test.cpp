#include "path_manager/planning_execution.h"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace pm = path_manager;
namespace {
int failures = 0;
void check(bool value, const std::string &description)
{
  std::cout << (value ? "[OK] " : "[FAIL] ") << description << '\n';
  if (!value) ++failures;
}
poly_traj::Trajectory flight(double x)
{
  poly_traj::CoefficientMat c = poly_traj::CoefficientMat::Zero();
  c.col(5) = Eigen::Vector3d(x, 2.0, 3.0);
  c(0, 4) = 2.0;
  return poly_traj::Trajectory({4.0}, {c});
}
std::vector<double> coefficients(const poly_traj::Trajectory &trajectory)
{
  std::vector<double> values;
  for (const auto &piece : trajectory) {
    values.push_back(piece.getDuration());
    const auto &matrix = piece.getCoeffMat();
    values.insert(values.end(), matrix.data(), matrix.data() + matrix.size());
  }
  return values;
}
pm::PlanningComputation candidate(pm::PlanningExecutionStore &store,
                                  uint64_t environment, uint64_t advances = 1)
{
  pm::PlanningComputation value;
  value.status = pm::PlanResult::success();
  value.recordProduct({flight(10.0), flight(20.0), 101.0, 100.0});
  value.identity_advances = advances;
  return store.seal(std::move(value), environment);
}

void adoptionAndIdentity()
{
  pm::PlanningExecutionStore store(7), foreign(7);
  check(store.current().traj_id == 0 && store.current().duration == 0.0 &&
            store.current().start_time == 0.0 && store.current().end_time == 0.0 &&
            store.current().start_pos == Eigen::Vector3d::Zero() &&
            store.reference().last_glb_t_of_lc_tgt == -1.0,
        "execution owner starts with defined empty views");
  auto value = candidate(store, 11, 5);
  const auto expected = coefficients(value.product->trajectory);
  check(!foreign.apply(value, 11).applied && !store.apply(value, 12).applied &&
            store.identityCounter() == 0,
        "foreign and environment-stale computations change no state");
  const auto receipt = store.apply(value, 11);
  check(receipt.applied && receipt.trajectory_id == 5 && receipt.drone_id == 7 &&
            store.identityCounter() == 5 && store.current().start_time == 101.0 &&
            store.reference().global_start_time == 100.0 &&
            store.reference().glb_t_of_lc_tgt == 100.0,
        "adoption preserves accumulated historical IDs and both clock points");
  check(!store.apply(value, 11).applied && store.identityCounter() == 5,
        "an adopted computation cannot be replayed");
  value.product->trajectory = flight(999.0);
  check(coefficients(store.current().traj) == expected,
        "execution owns coefficients independently of the calculation result");

  auto pending = candidate(store, 11);
  store.invalidatePending();
  check(!store.apply(pending, 11).applied && store.current().duration == 4.0,
        "new calculation/configuration supersedes pending results without withdrawing execution");
  auto failure = store.seal({}, 11);
  check(store.apply(failure, 11).applied && !store.apply(failure, 11).applied &&
            store.identityCounter() == 5 && coefficients(store.current().traj) == expected,
        "early failure consumes its receipt but no trajectory identity");

  auto refused = candidate(store, 11, 3);
  refused.status = pm::PlanResult::failed("post-solve refusal");
  check(!store.apply(refused, 11).applied && store.identityCounter() == 5,
        "a failed intermediate cannot be adopted as executable");
  refused.invalidate_execution = true;
  check(store.apply(refused, 11).applied && store.identityCounter() == 8 &&
            store.current().traj_id == 8 && store.current().duration == 0.0 &&
            store.current().start_time == 0.0 &&
            coefficients(store.current().traj) == coefficients(refused.product->trajectory),
        "post-solve refusal retains the historical product/ID and clears execution validity");

  auto malformed = candidate(store, 11);
  malformed.invalidate_execution = true;
  check(!store.apply(malformed, 11).applied, "successful invalidated candidates are refused");
  malformed = candidate(store, 11); malformed.identity_advances = 0;
  check(!store.apply(malformed, 11).applied, "product without identity advance is refused");
  malformed = candidate(store, 11); malformed.product.reset();
  check(!store.apply(malformed, 11).applied, "missing product with success/identity is refused");
  malformed = candidate(store, 11); malformed.product->reference_trajectory.clear();
  check(!store.apply(malformed, 11).applied, "empty reference product is refused");
  auto overflow = candidate(store, 11, std::numeric_limits<uint64_t>::max());
  check(!store.apply(overflow, 11).applied && store.identityCounter() == 8,
        "identity overflow is refused before adopting any fields");
}

void revocationAndRecovery()
{
  pm::PlanningExecutionStore store(3);
  auto first = candidate(store, 17);
  check(store.apply(first, 17).applied, "revocation fixture is adopted");
  auto pending = candidate(store, 17);
  int events = 0;
  pm::ExecutionRevocation captured;
  store.setRevocationObserver([&](const pm::ExecutionRevocation &event) {
    ++events;
    captured = event;
    check(store.current().duration == 4.0 && store.current().start_time == 101.0 &&
              event.trajectory_id == store.current().traj_id,
          "observer sees the named executable trajectory before withdrawal");
  });
  store.revoke(pm::ExecutionRevocationReason::TERRAIN_BLOCKED,
               Eigen::Vector3d(1.0, 2.0, 3.0), 2.5, 18);
  check(events == 1 && captured.trajectory_id == 1 && captured.drone_id == 3 &&
            captured.environment_generation == 18 && captured.detected_time == 2.5 &&
            captured.reason == pm::ExecutionRevocationReason::TERRAIN_BLOCKED &&
            captured.hit == Eigen::Vector3d(1.0, 2.0, 3.0),
        "revocation event owns the original identity, environment, reason and detection data");
  check(store.current().duration == 0.0 && store.current().start_time == 0.0 &&
            !store.apply(pending, 17).applied,
        "withdrawal invalidates execution and pending candidates without allocating an ID");
  store.revoke(pm::ExecutionRevocationReason::POLICY_UNEVALUATED,
               Eigen::Vector3d::Zero(), 0.0, 18);
  check(events == 1 && store.identityCounter() == 1,
        "repeated withdrawal does not announce an inactive trajectory");
  auto recovery = candidate(store, 18);
  check(store.apply(recovery, 18).applied && store.current().traj_id == 2,
        "a fresh computation recovers with a new identity");

  store.setRevocationObserver([](const pm::ExecutionRevocation &) {
    throw std::runtime_error("observer failure");
  });
  auto before_throw = candidate(store, 18);
  bool threw = false;
  try {
    store.revoke(pm::ExecutionRevocationReason::OBSTACLE_BLOCKED,
                 Eigen::Vector3d::Zero(), 0.0, 18);
  } catch (const std::runtime_error &) { threw = true; }
  check(threw && store.current().duration == 4.0 && store.apply(before_throw, 18).applied,
        "observer exception retains the legacy hook-before-invalidation state");
  store.setRevocationObserver({});
  const auto reference = coefficients(store.reference().traj);
  const auto reference_start = store.reference().global_start_time;
  auto before_stop = candidate(store, 18);
  const auto stop = store.adoptEmergencyTrajectory(flight(77.0), 222.0);
  check(stop.applied && stop.trajectory_id == 4 && store.current().start_time == 222.0 &&
            coefficients(store.reference().traj) == reference &&
            store.reference().global_start_time == reference_start &&
            !store.apply(before_stop, 18).applied,
        "emergency replacement consumes one ID, preserves reference and supersedes pending plans");
}
}  // namespace

int main()
{
  static_assert(!std::is_copy_constructible_v<pm::PlanningExecutionStore>);
  static_assert(std::is_const_v<std::remove_reference_t<decltype(
      std::declval<pm::PlanningExecutionStore &>().current())>>);
  adoptionAndIdentity();
  revocationAndRecovery();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
