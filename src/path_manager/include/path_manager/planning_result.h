#ifndef PATH_MANAGER_PLANNING_RESULT_H
#define PATH_MANAGER_PLANNING_RESULT_H

#include <string>

namespace path_manager {

// [PHASE] The planner <-> FSM contract for one mission plan. A bool cannot
// say "there is a flyable trajectory, but a stated requirement was relaxed
// under an approved policy" — and a log line is not a return value (review
// find: the FSM only ever read the bool, so DEGRADED existed nowhere the
// caller could see). Lives in path_manager, not the FSM header, so planners
// do not depend on the state machine.
//
// Outcome doctrine (2026-08-06 design review, frozen):
//   SUCCESS  — every stated requirement met.
//   DEGRADED — a flyable trajectory exists; a SOFT requirement was dropped
//              or an approved relaxation applied. The FSM executes it and
//              reports the reason. (soft profile loss, safe single-shot
//              fallback, phase-boundary fallback, opt-in final-boundary
//              relaxation, phase-mode mission too short for phases)
//   FAILED   — no trajectory the caller may fly: planning failed outright,
//              an explicit final state is invalid without the relaxation
//              opt-in, or a HARD limit could not be preserved (a hard
//              requirement is never traded for a flight).
enum class PlanOutcome { FAILED, SUCCESS, DEGRADED };

// Single reason, priority-ordered (higher wins when several apply):
//   FINAL_BOUNDARY_RELAXED > SINGLE_PLAN_FALLBACK > PHASE_BOUNDARY_FALLBACK
// `detail` carries the full story when reasons stack.
enum class PlanReason {
  NONE,
  PHASE_BOUNDARY_FALLBACK,
  SINGLE_PLAN_FALLBACK,
  FINAL_BOUNDARY_RELAXED,
};

struct PlanResult {
  PlanOutcome outcome{PlanOutcome::FAILED};
  PlanReason reason{PlanReason::NONE};
  std::string detail;

  bool hasTrajectory() const { return outcome != PlanOutcome::FAILED; }

  static PlanResult success() { return {PlanOutcome::SUCCESS, PlanReason::NONE, {}}; }
  static PlanResult failed(std::string why)
  {
    return {PlanOutcome::FAILED, PlanReason::NONE, std::move(why)};
  }
  // Priority-respecting degrade accumulator: outcome drops to DEGRADED,
  // the strongest reason wins, details concatenate.
  void degrade(PlanReason r, const std::string &why)
  {
    if (outcome == PlanOutcome::FAILED) return;  // FAILED is terminal
    outcome = PlanOutcome::DEGRADED;
    if (static_cast<int>(r) > static_cast<int>(reason)) reason = r;
    if (!detail.empty()) detail += "; ";
    detail += why;
  }
};

}  // namespace path_manager

#endif  // PATH_MANAGER_PLANNING_RESULT_H
