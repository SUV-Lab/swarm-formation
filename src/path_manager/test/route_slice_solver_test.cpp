// Exercise the real optimizer boundary: independent input/output ownership,
// failure classification and compatibility with existing callers. Algorithm
// preservation is also checked by the existing chain/RViz regression runs.
#include "path_manager/path_manager.h"
#include "path_manager/route_slice_solver.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char *message)
{
  std::cout << (ok ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!ok) ++failures;
}

// Store numbers separately from Trajectory so a lifetime regression cannot
// pass by comparing two views of the same optimizer scratch storage.
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

// A deliberately blocked environment. The solver can finish, but its actual
// collision audit must reject the result; this is not an injected bool.
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

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: route_slice_solver_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  using path_manager::RouteSliceRequest;
  using path_manager::RouteSliceResult;
  using path_manager::RouteSliceStatus;
  using path_manager::solveRouteSlice;

  // No configuration is needed for a rejected route; the optimizer must not
  // be entered (its node/provider pointers have not been initialized).
  ego_planner::PolyTrajOptimizer unconfigured;
  const auto empty = solveRouteSlice(unconfigured, {}, {});
  check(empty.status == RouteSliceStatus::ROUTE_TOO_SHORT &&
            !empty.hasTrajectory() && empty.trajectory.getPieceNum() == 0,
        "empty route has a distinct failure and no trajectory");
  RouteSliceRequest one_point;
  one_point.path.push_back(Eigen::Vector3d::Zero());
  const auto one = solveRouteSlice(unconfigured, one_point, {});
  check(one.status == RouteSliceStatus::ROUTE_TOO_SHORT &&
            one.trajectory.getPieceNum() == 0,
        "one-point route is rejected before optimizer access");

  RouteSliceResult retained;
  std::vector<double> retained_coefficients;
  Eigen::Vector3d midpoint = Eigen::Vector3d::Zero();
  double midpoint_time = 0.0;
  double retained_peak = 0.0;
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", argv[1]});
    // The test owns concurrency so runs on different machines agree.
    options.parameter_overrides({
        rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
    auto node = rclcpp::Node::make_shared("route_slice_solver_test", options);
    node->declare_parameter("drone_id", 0);
    node->declare_parameter("enable_debug_logs", true);
    path_manager::PathManager manager(node);
    auto optimizer = manager.makeConfiguredOptimizer();
    const auto config = manager.sliceSolveConfig();

    RouteSliceRequest request;
    request.head_pos = Eigen::Vector3d(20.0, 30.0, 4.0);
    request.goal_pos = Eigen::Vector3d(100.0, 30.0, 4.0);
    request.head_vel = Eigen::Vector3d(config.max_vel, 0.0, 0.0);
    request.tail = ego_planner::TailBoundary::pinned(
        request.head_vel, Eigen::Vector3d::Zero());
    request.path = {request.head_pos, request.goal_pos};
    request.cap = {4.0, 4.0};
    request.suppress_crossing_exempt = true;

    retained = solveRouteSlice(*optimizer, request, config);
    check(retained.hasTrajectory() && retained.trajectory.getPieceNum() >= 2,
          "real optimizer returns an owned trajectory from a two-point slice");
    if (retained.hasTrajectory()) {
      retained_coefficients = coefficients(retained.trajectory);
      retained_peak = retained.envelope_peak;
      midpoint_time = 0.5 * retained.trajectory.getTotalDuration();
      midpoint = retained.trajectory.getPos(midpoint_time);
      check(std::isfinite(retained_peak) &&
                retained_peak == optimizer->lastEnvPeak(),
            "result captures the successful solve's envelope evaluation");
      check(request.path.size() == 2 && request.cap == std::vector<double>({4.0, 4.0}) &&
                (request.path.front() - request.head_pos).norm() == 0.0 &&
                (request.path.back() - request.goal_pos).norm() == 0.0,
            "optimizer midpoint insertion leaves the caller's request unchanged");

      // Reject a full manager profile at the independent-worker boundary.
      // A valid key before the unsupported keys must not partially apply.
      const std::vector<std::string> profile_keys = {
          "optimization/weight_time", "optimization/obstacle_clearance",
          "optimization/alt_cap_headroom"};
      const auto pristine_parameters = node->get_parameters(profile_keys);
      size_t parameter_writes = 0;
      const auto parameter_probe = node->add_on_set_parameters_callback(
          [&](const std::vector<rclcpp::Parameter> &) {
            ++parameter_writes;
            rcl_interfaces::msg::SetParametersResult accepted;
            accepted.successful = true;
            return accepted;
          });
      bool unsupported_rejected = false;
      try {
        manager.makeConfiguredOptimizer({
            rclcpp::Parameter("optimization/weight_time", 1234.0),
            rclcpp::Parameter("optimization/obstacle_clearance", 8.0),
            rclcpp::Parameter("optimization/alt_cap_headroom", 9.0)});
      } catch (const std::invalid_argument &error) {
        unsupported_rejected = std::string(error.what()).find(
            "optimization/obstacle_clearance") != std::string::npos;
      }
      check(unsupported_rejected,
            "worker rejects shared clearance override with the unsupported key");
      const auto after_rejection = manager.sliceSolveConfig();
      check(after_rejection.max_vel == config.max_vel &&
                after_rejection.alt_cap_headroom == config.alt_cap_headroom &&
                after_rejection.alt_floor_headroom == config.alt_floor_headroom &&
                after_rejection.ground_height == config.ground_height &&
                after_rejection.weight_altitude == config.weight_altitude,
            "rejected worker profile leaves every slice setting unchanged");
      check(parameter_writes == 0 &&
                node->get_parameters(profile_keys) == pristine_parameters,
            "rejected worker profile makes no transient or retained ROS writes");

      auto compatibility_optimizer = manager.makeConfiguredOptimizer();
      poly_traj::Trajectory compatibility;
      const bool compatible = manager.solveSlice(
          *compatibility_optimizer, request.path, request.cap,
          request.head_pos, request.head_vel, request.head_acc, request.goal_pos,
          request.tail, request.suppress_crossing_exempt, &compatibility);
      check(compatible && coefficients(compatibility) == retained_coefficients,
            "real solve after rejected profile preserves exact durations and coefficients");
      check(!manager.solveSlice(*compatibility_optimizer, {}, {},
                                 request.head_pos, request.head_vel,
                                 request.head_acc, request.goal_pos, request.tail,
                                 false, &compatibility) &&
                coefficients(compatibility) == retained_coefficients,
            "failed compatibility call preserves its caller's existing output");
      check(!manager.solveSlice(*compatibility_optimizer, request.path,
                                 request.cap, request.head_pos, request.head_vel,
                                 request.head_acc, request.goal_pos, request.tail,
                                 false, nullptr),
            "compatibility adapter retains the null-output guard");

      RouteSliceRequest next = request;
      next.goal_pos.x() += 20.0;
      next.path.back() = next.goal_pos;
      const auto reused = solveRouteSlice(*optimizer, next, config);
      check(reused.hasTrajectory() &&
                coefficients(retained.trajectory) == retained_coefficients &&
                retained.envelope_peak == retained_peak,
            "a later solve cannot change a retained trajectory or evaluation");

      // Preserve the existing optimizer fallback for malformed cap pairing.
      next.cap = {4.0};
      const auto scalar_cap = solveRouteSlice(*optimizer, next, config);
      check(scalar_cap.hasTrajectory(),
            "cap-size mismatch retains the optimizer's scalar-cap fallback");

      BlockedField blocked;
      optimizer->setSDFManager(&blocked);
      optimizer->setCollisionReject(true);
      const auto rejected = solveRouteSlice(*optimizer, request, config);
      check(rejected.status == RouteSliceStatus::OPTIMIZER_FAILED &&
                !rejected.hasTrajectory() && rejected.trajectory.getPieceNum() == 0,
            "real collision rejection returns optimizer failure and no trajectory");
      check(coefficients(retained.trajectory) == retained_coefficients,
            "a rejected solve cannot overwrite an earlier successful result");
      optimizer->setSDFManager(nullptr);
    }
  }
  check(retained.hasTrajectory() &&
            coefficients(retained.trajectory) == retained_coefficients &&
            (retained.trajectory.getPos(midpoint_time) - midpoint).norm() == 0.0 &&
            retained.envelope_peak == retained_peak,
        "result remains evaluable after optimizer, manager and node destruction");
  rclcpp::shutdown();
  return failures == 0 ? 0 : 1;
}
