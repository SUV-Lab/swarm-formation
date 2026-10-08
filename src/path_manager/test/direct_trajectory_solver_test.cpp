// Real optimizer regression for the extracted direct-solve boundary. Compare
// with the prior low-level invocation, including failure-side diagnostics.
#include "path_manager/direct_trajectory_solver.h"
#include "path_manager/path_manager.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

using path_manager::DirectAltitudeConfig;
using path_manager::DirectTrajectoryRequest;
using path_manager::DirectTrajectoryResult;
using path_manager::DirectTrajectoryStatus;
using path_manager::computeDirectAltitudeBand;
using path_manager::solveDirectTrajectory;

bool near(double a, double b) { return std::abs(a - b) < 1e-12; }

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

bool samePath(const std::vector<Eigen::Vector3d> &a,
              const std::vector<Eigen::Vector3d> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if ((a[i] - b[i]).norm() != 0.0) return false;
  return true;
}

bool sameRequest(const DirectTrajectoryRequest &a,
                 const DirectTrajectoryRequest &b)
{
  return samePath(a.path, b.path) && samePath(a.waypoints, b.waypoints) &&
         a.cap_ref == b.cap_ref && a.edge_leg == b.edge_leg &&
         (a.head_pos - b.head_pos).norm() == 0.0 &&
         (a.head_vel - b.head_vel).norm() == 0.0 &&
         (a.head_acc - b.head_acc).norm() == 0.0 &&
         a.tail.prescribe_vel == b.tail.prescribe_vel &&
         a.tail.prescribe_acc == b.tail.prescribe_acc &&
         (a.tail.vel - b.tail.vel).norm() == 0.0 &&
         (a.tail.acc - b.tail.acc).norm() == 0.0 &&
         a.max_vel == b.max_vel &&
         a.suppress_crossing_exempt == b.suppress_crossing_exempt &&
         a.band.z_lo == b.band.z_lo && a.band.z_hi == b.band.z_hi &&
         a.band.weight == b.band.weight &&
         a.band.path_max_z_denoised == b.band.path_max_z_denoised &&
         a.band.path_max_z == b.band.path_max_z;
}

struct LegacySolve {
  bool accepted{false};
  poly_traj::Trajectory seed, trajectory;
  std::vector<Eigen::Vector3d> optimizer_path;
  std::vector<size_t> piece_leg;
};

// The pre-extraction optimizeStage backend invocation. It deliberately does
// not call the new kernel, and retains the backend's scratch seed on refusal
// so the test can distinguish that diagnostic from an accepted trajectory.
LegacySolve legacySolve(ego_planner::PolyTrajOptimizer &optimizer,
                        const DirectTrajectoryRequest &request)
{
  LegacySolve result;
  optimizer.setAltitudeBand(request.band.z_lo, request.band.z_hi,
                             request.band.weight);
  optimizer.setSuppressCrossingExempt(request.suppress_crossing_exempt);
  result.optimizer_path = request.path;
  result.accepted = optimizer.optimizeFromPath(
      result.optimizer_path, request.head_pos, request.head_vel, request.head_acc,
      request.waypoints, request.max_vel, result.seed, result.trajectory,
      request.cap_ref, request.tail, request.edge_leg, &result.piece_leg);
  return result;
}

void compareLegacy(const DirectTrajectoryResult &result,
                    const LegacySolve &legacy, const std::string &label)
{
  check(result.hasTrajectory() == legacy.accepted,
        label + " retains the backend acceptance decision");
  check(samePath(result.optimizer_path, legacy.optimizer_path) &&
            result.piece_leg == legacy.piece_leg,
        label + " retains the exact edited path and piece-to-leg diagnostics");
  if (legacy.accepted) {
    check(coefficients(result.seed) == coefficients(legacy.seed) &&
              coefficients(result.trajectory) == coefficients(legacy.trajectory),
          label + " returns identical seed/final coefficients and durations");
  } else {
    check(result.seed.getPieceNum() == 0 && result.trajectory.getPieceNum() == 0,
          label + " exposes no seed or final trajectory after refusal");
  }
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

void checkAltitudeBand()
{
  const std::vector<Eigen::Vector3d> path = {
      {0.0, 0.0, 2.0}, {1.0, 0.0, 6.0}, {2.0, 0.0, 3.0}};
  const Eigen::Vector3d head(0.0, 0.0, 1.0), goal(2.0, 0.0, 4.0);
  DirectAltitudeConfig config{8.0, 0.4, 1.25, -1.0, 17.0};
  const auto restored = computeDirectAltitudeBand(path, head, goal, config);
  check(near(restored.path_max_z_denoised, 6.0) &&
            near(restored.path_max_z, 8.0) && near(restored.z_hi, 8.4) &&
            near(restored.z_lo, -0.25) && restored.weight == 17.0,
        "direct altitude band restores the raw crest while retaining path/endpoint floor");

  config.raw_max_z = 5.0;
  config.ground_height = 2.0;
  const auto cushion = computeDirectAltitudeBand(path, head, goal, config);
  check(near(cushion.path_max_z, 6.0) && near(cushion.z_hi, 6.4) &&
            near(cushion.z_lo, 2.625),
        "lower raw crest cannot tighten the cap and enabled ground adds its cushion");

  const auto endpoints = computeDirectAltitudeBand(
      {}, Eigen::Vector3d(0.0, 0.0, 5.0), Eigen::Vector3d(1.0, 0.0, 9.0),
      DirectAltitudeConfig{7.0, 0.4, 0.2, -1.0, 3.0});
  check(near(endpoints.z_hi, 9.4) && near(endpoints.z_lo, 4.8),
        "endpoint heights remain part of the direct altitude envelope");

  DirectTrajectoryRequest request;
  request.path = {{0.0, 0.0, 4.0}, {1.0, 0.0, 4.0}};
  request.head_pos = request.path.front();
  request.waypoints = {request.path.back()};
  request.cap_ref = {10000.0};  // Deliberate size mismatch: scalar fallback.
  request.band = computeDirectAltitudeBand(
      request.path, request.head_pos, request.waypoints.back(),
      DirectAltitudeConfig{4.3, 0.4, 0.2, -1.0, 17.0});
  check(near(request.band.path_max_z_denoised, 4.0) &&
            near(request.band.path_max_z, 4.3) && near(request.band.z_hi, 4.7) &&
            request.band.z_hi < request.cap_ref.front(),
        "a cap-reference outlier does not inflate the direct scalar ceiling");
}

void checkAdmission()
{
  ego_planner::PolyTrajOptimizer unconfigured;
  DirectTrajectoryRequest short_request;
  short_request.waypoints = {Eigen::Vector3d::Ones()};
  const auto empty = solveDirectTrajectory(unconfigured, short_request);
  check(empty.status == DirectTrajectoryStatus::ROUTE_TOO_SHORT &&
            !empty.hasTrajectory() && empty.optimizer_path.empty() &&
            empty.piece_leg.empty() && empty.seed.getPieceNum() == 0 &&
            empty.trajectory.getPieceNum() == 0,
        "empty path is rejected before unconfigured optimizer access");
  short_request.path = {Eigen::Vector3d::Zero()};
  const auto one = solveDirectTrajectory(unconfigured, short_request);
  check(one.status == DirectTrajectoryStatus::ROUTE_TOO_SHORT &&
            samePath(one.optimizer_path, short_request.path) &&
            one.piece_leg.empty() && one.seed.getPieceNum() == 0 &&
            one.trajectory.getPieceNum() == 0,
        "one-point input retains its diagnostic path without entering optimizer");
  DirectTrajectoryRequest no_waypoints;
  no_waypoints.path = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()};
  const auto missing = solveDirectTrajectory(unconfigured, no_waypoints);
  check(missing.status == DirectTrajectoryStatus::NO_WAYPOINTS &&
            !missing.hasTrajectory() &&
            samePath(missing.optimizer_path, no_waypoints.path) &&
            missing.seed.getPieceNum() == 0 && missing.trajectory.getPieceNum() == 0,
        "missing mission goal is rejected before optimizer access");
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: direct_trajectory_solver_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  checkAltitudeBand();
  checkAdmission();

  DirectTrajectoryResult retained;
  std::vector<double> retained_seed, retained_final;
  std::vector<Eigen::Vector3d> retained_path;
  std::vector<size_t> retained_piece_leg;
  Eigen::Vector3d seed_midpoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d final_midpoint = Eigen::Vector3d::Zero();
  double seed_time = 0.0, final_time = 0.0;
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", argv[1]});
    options.parameter_overrides({
        rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
    auto node = rclcpp::Node::make_shared("direct_trajectory_solver_test", options);
    node->declare_parameter("drone_id", 0);
    node->declare_parameter("enable_debug_logs", true);
    path_manager::PathManager manager(node);
    auto optimizer = manager.makeConfiguredOptimizer();
    auto legacy_optimizer = manager.makeConfiguredOptimizer();
    const auto shared_config = manager.sliceSolveConfig();
    const DirectAltitudeConfig band_config{
        4.2, shared_config.alt_cap_headroom, shared_config.alt_floor_headroom,
        shared_config.ground_height, shared_config.weight_altitude};

    DirectTrajectoryRequest request;
    request.head_pos = Eigen::Vector3d(20.0, 30.0, 4.0);
    request.head_vel = Eigen::Vector3d(shared_config.max_vel, 0.0, 0.0);
    request.waypoints = {Eigen::Vector3d(100.0, 30.0, 4.0)};
    request.path = {request.head_pos, request.waypoints.back()};
    request.cap_ref = {4.0, 4.0};
    request.edge_leg = {7};
    request.tail = ego_planner::TailBoundary::pinned(
        request.head_vel, Eigen::Vector3d::Zero());
    request.band = computeDirectAltitudeBand(
        request.path, request.head_pos, request.waypoints.back(), band_config);
    request.max_vel = shared_config.max_vel;
    request.suppress_crossing_exempt = true;
    const auto original = request;

    retained = solveDirectTrajectory(*optimizer, request);
    const auto legacy = legacySolve(*legacy_optimizer, request);
    compareLegacy(retained, legacy, "normal solve");
    check(retained.hasTrajectory() && retained.seed.getPieceNum() >= 2 &&
              retained.trajectory.getPieceNum() >= 2,
          "real direct optimizer supplies both seed and final trajectories");
    if (!retained.hasTrajectory()) {
      rclcpp::shutdown();
      return 1;
    }
    check(sameRequest(request, original),
          "midpoint insertion leaves every caller request field unchanged");
    check(retained.optimizer_path.size() > request.path.size() &&
              retained.piece_leg.size() ==
                  static_cast<size_t>(retained.trajectory.getPieceNum()) &&
              !retained.piece_leg.empty() &&
              std::all_of(retained.piece_leg.begin(), retained.piece_leg.end(),
                          [](size_t leg) { return leg == 7; }),
          "inserted path vertices split the original edge into correctly attributed pieces");

    retained_seed = coefficients(retained.seed);
    retained_final = coefficients(retained.trajectory);
    retained_path = retained.optimizer_path;
    retained_piece_leg = retained.piece_leg;
    seed_time = 0.5 * retained.seed.getTotalDuration();
    final_time = 0.5 * retained.trajectory.getTotalDuration();
    seed_midpoint = retained.seed.getPos(seed_time);
    final_midpoint = retained.trajectory.getPos(final_time);

    DirectTrajectoryRequest next = request;
    next.waypoints.back().x() += 20.0;
    next.path.back() = next.waypoints.back();
    next.edge_leg = {9};
    const auto reused = solveDirectTrajectory(*optimizer, next);
    check(reused.hasTrajectory() && coefficients(retained.seed) == retained_seed &&
              coefficients(retained.trajectory) == retained_final &&
              samePath(retained.optimizer_path, retained_path) &&
              retained.piece_leg == retained_piece_leg,
          "optimizer reuse cannot rewrite retained trajectories or diagnostics");

    next.cap_ref = {10000.0};
    // A climbing head makes the scalar ceiling observable in the solve;
    // an exactly level straight flight would not exercise its penalty.
    next.head_vel.z() = 0.2;
    next.head_vel.x() = std::sqrt(next.max_vel * next.max_vel - 0.2 * 0.2);
    const auto scalar = solveDirectTrajectory(*optimizer, next);
    const auto scalar_legacy = legacySolve(*legacy_optimizer, next);
    compareLegacy(scalar, scalar_legacy, "cap-size scalar fallback");
    check(scalar.hasTrajectory() && near(next.band.path_max_z, 4.2) &&
              near(next.band.z_hi, 4.2 + shared_config.alt_cap_headroom) &&
              next.band.z_hi < next.cap_ref.front(),
          "real scalar-cap fallback uses the raw crest band despite cap-reference outlier");
    auto inflated_request = next;
    inflated_request.band.z_hi =
        next.cap_ref.front() + shared_config.alt_cap_headroom;
    auto inflated_optimizer = manager.makeConfiguredOptimizer();
    const auto inflated = legacySolve(*inflated_optimizer, inflated_request);
    check(scalar.hasTrajectory() &&
              (!inflated.accepted ||
               coefficients(scalar.trajectory) != coefficients(inflated.trajectory)),
          "climbing fixture distinguishes the direct ceiling from erroneous cap-outlier inflation");

    DirectTrajectoryRequest bad_tags = request;
    bad_tags.edge_leg = {7, 8};
    const auto untagged = solveDirectTrajectory(*optimizer, bad_tags);
    const auto untagged_legacy = legacySolve(*legacy_optimizer, bad_tags);
    compareLegacy(untagged, untagged_legacy, "mismatched edge tags");
    check(untagged.hasTrajectory() && untagged.piece_leg.empty(),
          "mismatched provenance preserves the backend's whole-map drop rule");

    BlockedField blocked;
    optimizer->setSDFManager(&blocked);
    optimizer->setCollisionReject(true);
    legacy_optimizer->setSDFManager(&blocked);
    legacy_optimizer->setCollisionReject(true);
    const auto rejected = solveDirectTrajectory(*optimizer, request);
    const auto rejected_legacy = legacySolve(*legacy_optimizer, request);
    compareLegacy(rejected, rejected_legacy, "real collision rejection");
    check(rejected.status == DirectTrajectoryStatus::OPTIMIZER_FAILED &&
              rejected.optimizer_path.size() > request.path.size() &&
              rejected.piece_leg.empty() && rejected_legacy.seed.getPieceNum() > 0,
          "rejection preserves edited path and cleared mapping while withholding backend scratch seed");
    check(sameRequest(request, original) &&
              coefficients(retained.seed) == retained_seed &&
              coefficients(retained.trajectory) == retained_final &&
              samePath(retained.optimizer_path, retained_path) &&
              retained.piece_leg == retained_piece_leg,
          "a rejected solve leaves input and earlier accepted results untouched");
    optimizer->setSDFManager(nullptr);
    legacy_optimizer->setSDFManager(nullptr);
  }

  check(retained.hasTrajectory() && coefficients(retained.seed) == retained_seed &&
            coefficients(retained.trajectory) == retained_final &&
            samePath(retained.optimizer_path, retained_path) &&
            retained.piece_leg == retained_piece_leg &&
            (retained.seed.getPos(seed_time) - seed_midpoint).norm() == 0.0 &&
            (retained.trajectory.getPos(final_time) - final_midpoint).norm() == 0.0,
        "owned seed/final trajectories and diagnostics survive optimizer, manager and node destruction");
  rclcpp::shutdown();
  return failures == 0 ? 0 : 1;
}
