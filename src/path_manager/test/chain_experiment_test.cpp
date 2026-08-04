// [CHAIN] Stage-1 experiment harness: baseline vs N-segment chained plan,
// headless, on a synthetic mission-scale DEM. Drives SegmentChainPlanner
// directly (no FSM, no RViz, no terrain server) with the REAL production
// parameter set loaded from optimizer_params.yaml, then audits the stitched
// trajectory's EVERY piece junction — internal MINCO joints and chain seams
// alike — for P/V/A continuity. A seam that broke shows up here as a junction
// whose left-piece end state disagrees with the right-piece start state;
// MINCO's own joints sit at solver noise (~1e-12), so one threshold covers
// both.
//
// Run inside the dev container from the workspace root:
//   ./install/path_manager/lib/path_manager/chain_experiment_test \
//       [optimizer_params.yaml] [segments] [zone] [segdiff]
// Optional literals (any order after the first two args):
//   zone     — plants one risk zone straddling the first junction's nominal
//              position, exercising the moat+taper nudge: the junction must
//              move off its equal-time station and seams stay exact.
//   segdiff  — stage-2 per-segment requirement overrides: segment 1 prices
//              time 4x (optimization/weight_time=1000), the last segment
//              lowers its speed ceiling (optimization/max_vel=1.8). The
//              [CHAIN-REPORT] mean-speed column shows the differentiation;
//              the seam audit must stay at solver noise regardless.
// The [CHAIN]/[CHAIN-REPORT] narrative lands in ./logs/runtime/ (LogManager
// is file-only); this binary prints the machine-checkable verdicts to stdout.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "path_manager/segment_chain_planner.h"

namespace {

int failures = 0;

void expect(bool condition, const std::string &label)
{
  std::cout << (condition ? "[OK]   " : "[FAIL] ") << label << '\n';
  if (!condition) ++failures;
}

// Rolling-hills DEM, mission scale (1 u = 100 m): 360 x 300 u at 2 u/cell,
// crests ~1.8 u (180 m). Heights are authored through the SAME forward map
// terrainToWorld uses ([TERRAIN-FRAME] per-axis mirrors), so the planner
// samples exactly the surface written here regardless of the mirror.
grid_map_msgs::msg::GridMap::SharedPtr makeHillsMap()
{
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr double kRes = 2.0;
  constexpr double kLx = 360.0, kLy = 300.0;
  constexpr int kRows = static_cast<int>(kLx / kRes);  // row spans X
  constexpr int kCols = static_cast<int>(kLy / kRes);  // col spans Y
  msg->info.resolution = kRes;
  msg->info.length_x = kLx;
  msg->info.length_y = kLy;
  msg->info.pose.position.x = kLx / 2.0;  // world spans [0, kLx] x [0, kLy]
  msg->info.pose.position.y = kLy / 2.0;
  msg->layers.push_back("elevation");
  msg->data.resize(1);
  auto &layer = msg->data.front();
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index";
  layer.layout.dim[0].size = kCols;
  layer.layout.dim[0].stride = kCols * kRows;
  layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = kRows;
  layer.layout.dim[1].stride = kRows;
  layer.data.assign(static_cast<size_t>(kCols) * kRows, 0.0f);
  const double ox = 0.0, oy = 0.0;
  for (int col = 0; col < kCols; ++col) {
    for (int row = 0; row < kRows; ++row) {
      const double wx = ox + kLx - (row + 0.5) * kRes;
      const double wy = oy + kLy - (col + 0.5) * kRes;
      const double h = 1.0 + 0.8 * std::sin(wx / 22.0) * std::cos(wy / 31.0);
      layer.data[static_cast<size_t>(col) * kRows + row] =
          static_cast<float>(std::max(0.0, h));
    }
  }
  return msg;
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  const std::string params =
      argc > 1 ? argv[1]
               : "src/mmp_path_planning/src/path_manager/config/"
                 "optimizer_params.yaml";
  const int segments = argc > 2 ? std::atoi(argv[2]) : 3;

  bool with_zone = false, with_segdiff = false;
  for (int a = 3; a < argc; ++a) {
    if (std::string(argv[a]) == "zone") with_zone = true;
    if (std::string(argv[a]) == "segdiff") with_segdiff = true;
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params});
  auto node = std::make_shared<rclcpp::Node>("chain_experiment", options);
  node->declare_parameter("drone_id", 0);
  if (with_segdiff) {
    node->declare_parameter(
        "chain/seg1/params",
        std::vector<std::string>{"optimization/weight_time=1000.0"});
    node->declare_parameter(
        "chain/seg" + std::to_string(segments) + "/params",
        std::vector<std::string>{"optimization/max_vel=1.8"});
    std::cout << "segdiff: seg1 weight_time=1000, seg" << segments
              << " max_vel=1.8\n";
  }

  auto pm = std::make_shared<path_manager::PathManager>(node);
  pm->initOptimizer();
  pm->deliverTrajToOptimizer();
  pm->setTerrainData(makeHillsMap());

  // Optional zone on the first junction's nominal station (x=130 for 3
  // segments): reach 5 + taper 30 forces the nudge but leaves the ±40%-span
  // window (±40 u) enough room to clear.
  if (with_zone) {
    path_manager::RiskZone zone;
    zone.center = Eigen::Vector3d(130.0, 150.0, 2.0);
    zone.reach = 5.0;
    zone.peak = 0.8;
    pm->setRiskZonesRuntime({zone});
    std::cout << "risk zone planted at (130, 150), reach 5\n";
  }

  auto log = std::make_unique<swarm_formation::LogManager>(
      "chain_experiment", "./logs/runtime", swarm_formation::LogManager::INFO);
  path_manager::SegmentChainPlanner chain(node, pm, log.get(), segments);

  // 30 km eastbound mission over the hills: start at cruise, goal at 150 m
  // AGL. Start z is absolute (max terrain 1.8 u); goal z rides [GOAL AGL].
  const Eigen::Vector3d start_pos(30.0, 150.0, 3.0);
  const Eigen::Vector3d start_vel(2.0, 0.0, 0.0);
  const Eigen::Vector3d start_acc(0.0, 0.0, 0.0);
  const std::vector<Eigen::Vector3d> goal = {{330.0, 150.0, 1.5}};

  const bool ok = chain.plan(start_pos, start_vel, start_acc, goal,
                             /*start_vel_synthesized=*/true);
  expect(ok, "chained plan returns success");
  if (!ok) {
    rclcpp::shutdown();
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  const poly_traj::Trajectory &chained = pm->traj_.local_traj.traj;
  const poly_traj::Trajectory &baseline = pm->traj_.global_traj.traj;
  expect(chained.getPieceNum() > 0 && baseline.getPieceNum() > 0,
         "both chained and baseline trajectories are stored");

  // Continuity audit across EVERY junction of the stitched trajectory.
  double max_dp = 0.0, max_dv = 0.0, max_da = 0.0;
  int worst_junc = -1;
  for (int j = 1; j < chained.getPieceNum(); ++j) {
    const auto left = chained.getPiece(j - 1);
    const double dur = left.getDuration();
    const double dp = (chained.getJuncPos(j) - left.getPos(dur)).norm();
    const double dv = (chained.getJuncVel(j) - left.getVel(dur)).norm();
    const double da = (chained.getJuncAcc(j) - left.getAcc(dur)).norm();
    if (std::max({dp, dv, da}) > std::max({max_dp, max_dv, max_da}))
      worst_junc = j;
    max_dp = std::max(max_dp, dp);
    max_dv = std::max(max_dv, dv);
    max_da = std::max(max_da, da);
  }
  std::cout << "junctions audited: " << chained.getPieceNum() - 1
            << " (chain seams + internal MINCO joints)\n"
            << "  max |dP| = " << max_dp << " u\n"
            << "  max |dV| = " << max_dv << " u/s\n"
            << "  max |dA| = " << max_da << " u/s^2\n"
            << "  worst junction index: " << worst_junc << '\n';
  expect(max_dp < 1e-6, "position continuous at every junction (< 1e-6 u)");
  expect(max_dv < 1e-6, "velocity continuous at every junction (< 1e-6 u/s)");
  expect(max_da < 1e-6,
         "acceleration continuous at every junction (< 1e-6 u/s^2)");

  // Sanity: the split must not change the flight materially.
  const double bt = baseline.getTotalDuration();
  const double ct = chained.getTotalDuration();
  std::cout << "baseline: " << baseline.getPieceNum() << " pieces, " << bt
            << " s | chained: " << chained.getPieceNum() << " pieces, " << ct
            << " s (" << 100.0 * (ct / bt - 1.0) << "% time)\n";
  expect(std::abs(ct / bt - 1.0) < 0.15,
         "chained flight time within 15% of the baseline");

  // Endpoint fidelity: same start, same goal neighbourhood.
  expect((chained.getJuncPos(0) - start_pos).norm() < 1e-9,
         "chained trajectory starts at the mission start");
  expect((chained.getJuncPos(chained.getPieceNum()) -
          baseline.getJuncPos(baseline.getPieceNum())).norm() < 1e-6,
         "chained and baseline end at the same resolved goal");

  rclcpp::shutdown();
  std::cout << (failures == 0 ? "PASS" : "FAIL") << ": " << failures
            << " failed check(s)\n";
  return failures == 0 ? 0 : 1;
}
