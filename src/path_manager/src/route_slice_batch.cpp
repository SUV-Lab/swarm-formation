#include "path_manager/route_slice_batch.h"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace path_manager {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Also joins during unwinding if a later thread could not be constructed.
// This does not catch or reinterpret exceptions from inside a worker.
struct JoinWorkers {
  std::vector<std::thread> &workers;
  ~JoinWorkers() {
    for (auto &worker : workers)
      if (worker.joinable()) worker.join();
  }
};

}  // namespace

RouteSliceBatchResult solveRouteSliceBatch(
    const std::vector<std::unique_ptr<ego_planner::PolyTrajOptimizer>> &optimizers,
    std::vector<RouteSliceRequest> requests, const RouteSliceConfig &config,
    bool run_parallel, const std::vector<char> &skip)
{
  if (optimizers.size() != requests.size())
    throw std::invalid_argument("route slice batch needs one optimizer per request");
  if (!skip.empty() && skip.size() != requests.size())
    throw std::invalid_argument("route slice batch skip size does not match requests");
  for (const auto &optimizer : optimizers)
    if (!optimizer)
      throw std::invalid_argument("route slice batch optimizer must not be null");

  RouteSliceBatchResult batch;
  batch.runs.resize(requests.size());
  if (requests.empty()) return batch;

  const auto worker = [&](size_t i) {
    if (!skip.empty() && skip[i]) return;
    RouteSliceRun &run = batch.runs[i];
    run.attempted = true;
    const auto start = Clock::now();
    run.result = solveRouteSlice(*optimizers[i], std::move(requests[i]), config);
    run.solve_ms = elapsedMs(start);
  };

  const auto start = Clock::now();
  if (run_parallel) {
    std::vector<std::thread> workers;
    workers.reserve(requests.size());
    JoinWorkers join{workers};
    for (size_t i = 0; i < requests.size(); ++i)
      workers.emplace_back(worker, i);
  } else {
    for (size_t i = 0; i < requests.size(); ++i) worker(i);
  }
  batch.wall_ms = elapsedMs(start);
  return batch;
}

}  // namespace path_manager
