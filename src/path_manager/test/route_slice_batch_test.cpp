// Real optimizer regression for ordered batch results, partial failures,
// independent input/output ownership, and sequential/parallel equivalence.
#include "path_manager/path_manager.h"
#include "path_manager/route_slice_batch.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

using Optimizers =
    std::vector<std::unique_ptr<ego_planner::PolyTrajOptimizer>>;
using path_manager::RouteSliceBatchResult;
using path_manager::RouteSliceRequest;
using path_manager::RouteSliceStatus;
using path_manager::solveRouteSliceBatch;

// Capture scalar values independently, so shared scratch storage cannot make
// both sides of a lifetime comparison change together.
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

bool sameRequest(const RouteSliceRequest &a, const RouteSliceRequest &b)
{
  if (a.path.size() != b.path.size() || a.cap != b.cap ||
      a.suppress_crossing_exempt != b.suppress_crossing_exempt ||
      a.tail.prescribe_vel != b.tail.prescribe_vel ||
      a.tail.prescribe_acc != b.tail.prescribe_acc)
    return false;
  for (size_t i = 0; i < a.path.size(); ++i)
    if ((a.path[i] - b.path[i]).norm() != 0.0) return false;
  return (a.head_pos - b.head_pos).norm() == 0.0 &&
         (a.head_vel - b.head_vel).norm() == 0.0 &&
         (a.head_acc - b.head_acc).norm() == 0.0 &&
         (a.goal_pos - b.goal_pos).norm() == 0.0 &&
         (a.tail.vel - b.tail.vel).norm() == 0.0 &&
         (a.tail.acc - b.tail.acc).norm() == 0.0;
}

Optimizers configuredWorkers(path_manager::PathManager &manager, size_t count)
{
  Optimizers workers;
  workers.reserve(count);
  for (size_t i = 0; i < count; ++i)
    workers.push_back(manager.makeConfiguredOptimizer());
  return workers;
}

template <typename Call>
void checkInvalidArgument(Call call, const std::string &description)
{
  bool rejected = false;
  try {
    call();
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, description);
}

class BlockedField : public path_planner::sdf::IDistanceField {
 public:
  float getDistance(const Eigen::Vector3d &) const override { return -1.0f; }
  bool getDistanceAndGradient(const Eigen::Vector3d &, float *distance,
                              Eigen::Vector3d *gradient) const override {
    *distance = -1.0f;
    gradient->setZero();
    return true;
  }
  bool hasData() const override { return true; }
};

void checkAdmission(bool parallel)
{
  const std::string mode = parallel ? "parallel" : "sequential";
  const Optimizers none;
  const auto empty = solveRouteSliceBatch(none, {}, {}, parallel);
  check(empty.runs.empty() && empty.wall_ms == 0.0,
        mode + " empty batch needs no workers");

  RouteSliceRequest valid_route;
  valid_route.path = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()};
  Optimizers one;
  one.push_back(std::make_unique<ego_planner::PolyTrajOptimizer>());
  // These workers deliberately have no node or providers. Validation and a
  // skipped slot must return before entering any optimizer, not partially run
  // earlier valid-looking slots before discovering the malformed batch.
  checkInvalidArgument(
      [&] { solveRouteSliceBatch(one, {valid_route, valid_route}, {}, parallel); },
      mode + " rejects mismatched worker count before a solve");
  checkInvalidArgument(
      [&] { solveRouteSliceBatch(one, {valid_route}, {}, parallel, {0, 0}); },
      mode + " rejects mismatched skip count before a solve");

  Optimizers missing;
  missing.push_back(std::make_unique<ego_planner::PolyTrajOptimizer>());
  missing.push_back(nullptr);
  checkInvalidArgument(
      [&] { solveRouteSliceBatch(missing, {valid_route, valid_route}, {}, parallel); },
      mode + " rejects a later null worker before running the first slot");
  checkInvalidArgument(
      [&] { solveRouteSliceBatch(missing, {valid_route, valid_route}, {},
                                parallel, {0, 1}); },
      mode + " validates null workers even when their slots are skipped");

  const auto skipped = solveRouteSliceBatch(one, {valid_route}, {}, parallel, {7});
  check(skipped.runs.size() == 1 && !skipped.runs[0].attempted &&
            skipped.runs[0].solve_ms == 0.0 &&
            !skipped.runs[0].result.hasTrajectory() &&
            skipped.runs[0].result.trajectory.getPieceNum() == 0,
        mode + " skipped slot has no optimizer verdict or trajectory");
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: route_slice_batch_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  checkAdmission(false);
  checkAdmission(true);

  RouteSliceBatchResult retained;
  std::vector<std::vector<double>> retained_coefficients;
  std::vector<Eigen::Vector3d> retained_midpoints;
  std::vector<double> retained_peaks;
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", argv[1]});
    options.parameter_overrides({
        rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
    auto node = rclcpp::Node::make_shared("route_slice_batch_test", options);
    node->declare_parameter("drone_id", 0);
    node->declare_parameter("enable_debug_logs", true);
    path_manager::PathManager manager(node);
    const auto config = manager.sliceSolveConfig();

    std::vector<RouteSliceRequest> requests(3);
    for (size_t i = 0; i < requests.size(); ++i) {
      auto &request = requests[i];
      request.head_pos = Eigen::Vector3d(20.0 + 10.0 * i, 30.0 + 30.0 * i, 4.0);
      request.goal_pos = request.head_pos + Eigen::Vector3d(80.0 + 20.0 * i, 0.0, 0.0);
      request.head_vel = Eigen::Vector3d(config.max_vel, 0.0, 0.0);
      request.tail = ego_planner::TailBoundary::pinned(
          request.head_vel, Eigen::Vector3d::Zero());
      request.path = {request.head_pos, request.goal_pos};
      request.cap = {4.0, 4.0};
      request.suppress_crossing_exempt = true;
    }
    const auto original_requests = requests;
    auto sequential_workers = configuredWorkers(manager, requests.size());
    auto parallel_workers = configuredWorkers(manager, requests.size());
    const auto sequential = solveRouteSliceBatch(
        sequential_workers, requests, config, false);
    retained = solveRouteSliceBatch(parallel_workers, requests, config, true);
    check(sequential.runs.size() == requests.size() &&
              retained.runs.size() == requests.size(),
          "both batch modes return exactly one ordered slot per request");
    bool all_success = sequential.runs.size() == requests.size() &&
                       retained.runs.size() == requests.size();
    for (size_t i = 0; i < requests.size() && all_success; ++i)
      all_success = sequential.runs[i].result.hasTrajectory() &&
                    retained.runs[i].result.hasTrajectory();
    check(all_success, "real optimizers solve every distinct straight-route fixture");
    if (!all_success) {
      rclcpp::shutdown();
      return 1;
    }
    check(std::isfinite(sequential.wall_ms) && sequential.wall_ms >= 0.0 &&
              std::isfinite(retained.wall_ms) && retained.wall_ms >= 0.0,
          "batch timing is finite in both execution modes");
    for (size_t i = 0; i < requests.size(); ++i) {
      const auto &serial = sequential.runs[i];
      const auto &parallel = retained.runs[i];
      const auto &trajectory = parallel.result.trajectory;
      check(serial.attempted && parallel.attempted &&
                std::isfinite(serial.solve_ms) && serial.solve_ms >= 0.0 &&
                std::isfinite(parallel.solve_ms) && parallel.solve_ms >= 0.0,
            "successful slot " + std::to_string(i) + " records its own attempt and timing");
      check(coefficients(serial.result.trajectory) == coefficients(trajectory) &&
                serial.result.envelope_peak == parallel.result.envelope_peak,
            "slot " + std::to_string(i) + " has identical sequential/parallel coefficients, durations and evaluation");
      check((trajectory.getJuncPos(0) - requests[i].head_pos).norm() < 1e-7 &&
                (trajectory.getJuncPos(trajectory.getPieceNum()) -
                 requests[i].goal_pos).norm() < 1e-7,
            "slot " + std::to_string(i) + " retains its input position in the batch");
      check(sameRequest(requests[i], original_requests[i]),
            "slot " + std::to_string(i) + " leaves the caller's complete request unchanged");
      retained_coefficients.push_back(coefficients(trajectory));
      retained_midpoints.push_back(trajectory.getPos(0.5 * trajectory.getTotalDuration()));
      retained_peaks.push_back(parallel.result.envelope_peak);
    }

    for (bool parallel : {false, true}) {
      const std::string mode = parallel ? "parallel" : "sequential";
      RouteSliceRequest one_point;
      one_point.path.push_back(Eigen::Vector3d::Zero());
      const std::vector<RouteSliceRequest> mixed = {
          requests[0], {}, requests[2], one_point, requests[1]};
      BlockedField blocked;
      auto workers = configuredWorkers(manager, mixed.size());
      workers[4]->setSDFManager(&blocked);
      workers[4]->setCollisionReject(true);
      const auto partial = solveRouteSliceBatch(workers, mixed, config, parallel);
      const bool complete = partial.runs.size() == mixed.size();
      check(complete, mode + " partial failure keeps every result slot");
      if (!complete) continue;
      check(partial.runs[0].result.hasTrajectory() &&
                partial.runs[2].result.hasTrajectory() &&
                coefficients(partial.runs[0].result.trajectory) == retained_coefficients[0] &&
                coefficients(partial.runs[2].result.trajectory) == retained_coefficients[2],
            mode + " successful siblings retain their exact trajectories around failed slices");
      check(partial.runs[1].attempted && partial.runs[3].attempted &&
                partial.runs[1].result.status == RouteSliceStatus::ROUTE_TOO_SHORT &&
                partial.runs[3].result.status == RouteSliceStatus::ROUTE_TOO_SHORT &&
                partial.runs[1].result.trajectory.getPieceNum() == 0 &&
                partial.runs[3].result.trajectory.getPieceNum() == 0,
            mode + " empty and one-point slices are attempted input failures with no trajectory");
      check(partial.runs[4].attempted &&
                partial.runs[4].result.status == RouteSliceStatus::OPTIMIZER_FAILED &&
                partial.runs[4].result.trajectory.getPieceNum() == 0,
            mode + " real collision rejection is a distinct attempted optimizer failure");
      for (size_t i = 0; i < mixed.size(); ++i)
        check(sameRequest(mixed[i], i == 0 ? requests[0] :
                                     i == 2 ? requests[2] :
                                     i == 4 ? requests[1] :
                                     i == 3 ? one_point : RouteSliceRequest{}),
              mode + " mixed input slot " + std::to_string(i) + " remains unchanged");
    }

    auto next_requests = requests;
    for (auto &request : next_requests) {
      request.goal_pos.x() += 20.0;
      request.path.back() = request.goal_pos;
    }
    const auto reused = solveRouteSliceBatch(
        parallel_workers, next_requests, config, true);
    bool owned_after_reuse = reused.runs.size() == requests.size();
    for (size_t i = 0; i < requests.size() && owned_after_reuse; ++i)
      owned_after_reuse = reused.runs[i].result.hasTrajectory() &&
          coefficients(retained.runs[i].result.trajectory) == retained_coefficients[i] &&
          retained.runs[i].result.envelope_peak == retained_peaks[i];
    check(owned_after_reuse,
          "reusing every worker for different routes preserves retained results and evaluations");
  }

  bool owned_after_destruction = retained.runs.size() == retained_coefficients.size() &&
                                 !retained.runs.empty();
  for (size_t i = 0; i < retained.runs.size() && owned_after_destruction; ++i) {
    const auto &result = retained.runs[i].result;
    owned_after_destruction = result.hasTrajectory() &&
        coefficients(result.trajectory) == retained_coefficients[i] &&
        (result.trajectory.getPos(0.5 * result.trajectory.getTotalDuration()) -
         retained_midpoints[i]).norm() == 0.0 &&
        result.envelope_peak == retained_peaks[i];
  }
  check(owned_after_destruction,
        "all retained batch results remain evaluable after workers, manager and node destruction");
  rclcpp::shutdown();
  std::cout << (failures == 0 ? "PASS: " : "FAIL: ") << failures
            << " failed check(s)\n";
  return failures == 0 ? 0 : 1;
}
