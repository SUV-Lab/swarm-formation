#ifndef PATH_MANAGER_ROUTE_SLICE_BATCH_H
#define PATH_MANAGER_ROUTE_SLICE_BATCH_H

#include <memory>
#include <vector>

#include "path_manager/route_slice_solver.h"

namespace path_manager {

struct RouteSliceRun {
  RouteSliceResult result;
  // Time inside the slice solve, excluding coordinator request preparation.
  double solve_ms{0.0};
  // A skipped slot has no optimizer verdict: its default result is only an
  // empty placeholder. Attempted input failures still set this to true.
  bool attempted{false};
};

struct RouteSliceBatchResult {
  // Input order is retained regardless of completion order. Each result owns
  // its trajectory independently of the requests and optimizer instances.
  std::vector<RouteSliceRun> runs;
  // Worker dispatch through completion/join; excludes request preparation,
  // argument validation and allocation of the result slots.
  double wall_ms{0.0};
};

// Synchronously solve prepared slices, one exclusive optimizer per request.
// Requests are owned by this call; optimizers and their configured environment
// are borrowed and must remain alive/stable until it returns. The caller may
// reuse the optimizer instances afterwards. No manager, parameter reads,
// logging, retry policy or trajectory publication is performed here.
//
// Empty skip means run every request; otherwise nonzero slots are skipped.
// The coordinator owns the reason (for example, deterministic fault injection)
// and its reporting. Sizes must match and every optimizer must be non-null;
// invalid arguments throw std::invalid_argument before any solve starts.
//
// Exceptions retain the existing worker semantics: sequential solver calls
// propagate; parallel solver calls are expected not to throw (an uncaught
// worker exception terminates, rather than becoming an optimizer failure).
// If starting a thread fails, already-started workers are joined before the
// launch exception propagates. All successful launches join before return.
RouteSliceBatchResult solveRouteSliceBatch(
    const std::vector<std::unique_ptr<ego_planner::PolyTrajOptimizer>> &optimizers,
    std::vector<RouteSliceRequest> requests, const RouteSliceConfig &config,
    bool run_parallel, const std::vector<char> &skip = {});

}  // namespace path_manager

#endif  // PATH_MANAGER_ROUTE_SLICE_BATCH_H
