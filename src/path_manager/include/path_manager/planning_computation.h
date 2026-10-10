#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "path_manager/flight_evaluation.h"
#include "path_manager/planning_result.h"
#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {

// An owned calculation product, before execution identity is assigned.
// Coordinates and coefficients retain the existing planner units. These
// timestamps preserve the ROS backend's existing pre/post-solve clock points;
// they are metadata, not an execution command or relative trajectory time.
struct PlannedTrajectory {
  poly_traj::Trajectory trajectory;
  poly_traj::Trajectory reference_trajectory;
  double start_time{0.0};
  double reference_start_time{0.0};
};

// Compatibility effects are separate from the numerical product. Historical
// baseline/segment solves consumed identities before the final flight existed.
// Collect that count without touching execution storage during calculation.
struct PlanningComputation {
  PlanResult status;
  // On a post-solve refusal this can retain a rejected intermediate product
  // solely to reproduce invalidated legacy storage. It is NOT a flyable result:
  // consumers must use hasTrajectory(); MissionPlanResult hides it on failure.
  std::optional<PlannedTrajectory> product;
  uint64_t identity_advances{0};
  bool invalidate_execution{false};
  FlightVerdict evaluation;
  std::vector<PhaseSpan> phases;
  uint64_t environment_generation{0};

  // Filled by the owning manager once calculation finishes. Adoption is
  // serialized and rejects another manager, stale environment/execution, or
  // reuse. These fields identify a live compatibility context, not a snapshot.
  uint64_t execution_context{0};
  uint64_t execution_revision{0};
  uint64_t previous_identity_counter{0};

  bool hasTrajectory() const {
    return status.hasTrajectory() && product.has_value();
  }
  void recordProduct(PlannedTrajectory value) {
    product = std::move(value);
    ++identity_advances;
    invalidate_execution = false;
  }
};

struct ExecutionReceipt {
  bool applied{false};
  uint64_t trajectory_id{0};
  int drone_id{-1};
};

}  // namespace path_manager
