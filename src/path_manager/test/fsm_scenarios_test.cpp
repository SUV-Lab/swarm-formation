// [ENV-CHANGE / ABORT] The PRODUCTION plumbing, driven through ReplanFSM.
//
// Everything else about this feature is pinned one layer down: the manager's
// detection (chain_experiment_test `envchange`) and the consumer's rules
// (exec_abort_test). What neither can see is whether the real callbacks pass
// the right things — whether the terrain path really says TERRAIN_BLOCKED,
// whether a zone update really fires once and only on a change, and whether
// a mission after an abort really leaves WAIT_EXTERNAL_RECOVERY and gets
// published. Those are all decided in ReplanFSM, so this stands one up.
//
// ORDER MATTERS AND IS NOT ARBITRARY. The zone set starts EMPTY (the params
// file ships no risk_zones), so the FIRST non-empty batch is itself a change.
// It has to go in BEFORE a trajectory exists, or it aborts the flight the
// no-op case is supposed to be checked against — the first draft of this
// ordering did exactly that.
//
// Registered regression: use a level, feasible flight so safety-gate refusal
// cannot prevent the callback/ABORT scenarios below from being exercised.

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "path_manager/replan_fsm.h"
#include "mmp_mission_msgs/msg/trajectory_command.hpp"
#include "mmp_mission_msgs/msg/risk_zone_array.hpp"
#include "mmp_mission_msgs/msg/risk_zone_spec.hpp"
#include "mmp_mission_msgs/msg/dynamic_obstacle_array.hpp"
#include "mmp_mission_msgs/msg/dynamic_obstacle_spec.hpp"
#include "mmp_traj_msgs/msg/poly_traj.hpp"
#include "mmp_traj_msgs/msg/trajectory_execution_control.hpp"

using Ctl = mmp_traj_msgs::msg::TrajectoryExecutionControl;
using Traj = mmp_traj_msgs::msg::PolyTraj;

namespace {

int g_failed = 0;
void expect(bool c, const std::string &label) {
  std::cout << "  " << (c ? "[OK]   " : "[FAIL] ") << label << "\n";
  if (!c) ++g_failed;
}

// Flat DEM at `height`, identical geometry every time so a re-ingest does not
// take the geometry-changed branch (which clears dynamic obstacles and
// rebuilds the SDF — a different event from "the ground moved").
grid_map_msgs::msg::GridMap::SharedPtr makeFlatMap(double height) {
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr double kRes = 2.0, kLx = 360.0, kLy = 300.0;
  constexpr int kRows = static_cast<int>(kLx / kRes);
  constexpr int kCols = static_cast<int>(kLy / kRes);
  msg->info.resolution = kRes;
  msg->info.length_x = kLx;
  msg->info.length_y = kLy;
  msg->info.pose.position.x = kLx / 2.0;
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
  layer.data.assign(static_cast<size_t>(kCols) * kRows,
                    static_cast<float>(height));
  return msg;
}

mmp_mission_msgs::msg::RiskZoneArray::SharedPtr makeZones(
    const std::vector<std::array<double, 5>> &zs) {
  auto m = std::make_shared<mmp_mission_msgs::msg::RiskZoneArray>();
  m->replace = true;
  for (const auto &z : zs) {
    mmp_mission_msgs::msg::RiskZoneSpec s;
    s.center.x = z[0]; s.center.y = z[1]; s.center.z = z[2];
    s.reach = z[3]; s.peak = z[4];
    m->zones.push_back(s);
  }
  return m;
}

mmp_mission_msgs::msg::TrajectoryCommand::SharedPtr makeMission(
    int seq, const std::string &mission_id, double gx, double gy, double gz) {
  auto m = std::make_shared<mmp_mission_msgs::msg::TrajectoryCommand>();
  m->drone_id = 0;
  m->sequence = seq;
  m->mission_id = mission_id;
  m->target_position.x = gx;
  m->target_position.y = gy;
  m->target_position.z = gz;
  m->use_initial_speed = true;
  m->initial_speed = 200.0;
  m->start_position.x = 30.0;
  m->start_position.y = 150.0;
  m->start_position.z = 4.0;  // AGL; avoid an unintended ground-launch test
  return m;
}

void spin(rclcpp::Node::SharedPtr n, int ms) {
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(ms)) {
    rclcpp::spin_some(n);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: fsm_scenarios_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);

  // The FSM and PathManager declare ~90 parameters between them in their
  // constructors; declaring any of them here first would throw. Everything
  // therefore rides the params file / NodeOptions, and nothing is set after
  // construction — those reads happen once, in the ctor.
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", argv[1]});
  // drone_id MUST be 0: SEQUENTIAL_START is gated on
  // drone_id_ <= 0 || (drone_id_ >= 1 && have_recv_pre_agent_), and nothing
  // in this repo ever sets have_recv_pre_agent_. With any other id the plan
  // succeeds and is never published.
  options.parameter_overrides({
      rclcpp::Parameter("drone_id", 0),
      rclcpp::Parameter("chain/enable", false),
  });
  auto node = rclcpp::Node::make_shared("fsm_scenarios", options);
  path_manager::ReplanFSM fsm(node);

  auto qos = rclcpp::QoS(5).reliable().transient_local();
  std::vector<Traj> trajs;
  std::vector<Ctl> ctls;
  std::vector<std::vector<double>> terrain_receipts;
  auto terrain_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/planning/terrain_ready", qos,
      [&terrain_receipts](std_msgs::msg::Float64MultiArray::SharedPtr m) {
        terrain_receipts.push_back(m->data);
      });
  auto ts = node->create_subscription<Traj>(
      "/planning/trajectory", qos,
      [&trajs](Traj::SharedPtr m) { trajs.push_back(*m); });
  auto cs = node->create_subscription<Ctl>(
      "/planning/execution_control", qos,
      [&ctls](Ctl::SharedPtr m) { ctls.push_back(*m); });
  spin(node, 300);

  // (1) Ground under the mission.
  fsm.terrainCallback(makeFlatMap(0.0));
  spin(node, 200);
  expect(!terrain_receipts.empty() &&
             terrain_receipts.back() == std::vector<double>({2.,0.,0.,360.,300.}),
         "terrain-ready receipt describes the accepted map");
  const auto receipt_count = terrain_receipts.size();
  auto malformed_map = makeFlatMap(0.0);
  malformed_map->info.resolution = 4.;
  malformed_map->data[0].data.clear();
  fsm.terrainCallback(malformed_map);
  fsm.terrainCallback(nullptr);
  spin(node, 200);
  expect(terrain_receipts.size() == receipt_count,
         "rejected/null terrain does not certify requested geometry as ingested");
  fsm.terrainCallback(std::make_shared<grid_map_msgs::msg::GridMap>());
  spin(node, 200);
  expect(terrain_receipts.size() == receipt_count + 1 &&
             terrain_receipts.back() == std::vector<double>(5,0.),
         "accepted explicit terrain clear publishes the zero receipt");
  fsm.terrainCallback(makeFlatMap(0.0));
  spin(node, 200);
  expect(terrain_receipts.size() == receipt_count + 2 &&
             terrain_receipts.back() == std::vector<double>({2.,0.,0.,360.,300.}),
         "valid terrain after rejection/clear restores the true ready receipt");

  // (2) A zone set, BEFORE any trajectory exists. The set starts empty, so
  // this batch IS a change — installing it now means the no-op check below
  // is about the fingerprint and not about this.
  const std::vector<std::array<double, 5>> zonesA{
      {600.0, 600.0, 3.0, 15.0, 0.9}};   // far off any route
  fsm.loadRiskZonesCallback(makeZones(zonesA));
  spin(node, 200);
  expect(ctls.empty(), "installing zones with nothing flying aborts nothing");

  // (3) Fly.
  fsm.trajectoryCommandCallback(makeMission(1, "A", 330.0, 150.0, 4.0));
  spin(node, 4000);
  expect(!trajs.empty(), "the mission is planned AND published");
  if (trajs.empty()) {
    std::cout << "FAIL: " << ++g_failed << " failed check(s)\n";
    rclcpp::shutdown();
    return 1;
  }
  const uint64_t id1 = trajs.back().trajectory_id;
  std::cout << "  .. trajectory id " << id1 << "\n";

  // (4) The SAME zone set again. Nothing about the flight changed, so
  // nothing may be withdrawn — this is the case the content fingerprint
  // exists for, and it is checked while a trajectory is live.
  const size_t ctls_before = ctls.size();
  fsm.loadRiskZonesCallback(makeZones(zonesA));
  spin(node, 300);
  expect(ctls.size() == ctls_before,
         "republishing the SAME zone set aborts nothing");

  // (5) A CHANGED set. Cannot be re-judged per leg, so the flight is
  // withdrawn — once, naming what is flying, with the policy reason.
  fsm.loadRiskZonesCallback(makeZones({{600.0, 600.0, 3.0, 25.0, 0.9}}));
  spin(node, 400);
  expect(ctls.size() == ctls_before + 1,
         "a CHANGED zone set aborts exactly once");
  if (ctls.size() == ctls_before + 1) {
    const auto &c = ctls.back();
    expect(c.action == Ctl::ACTION_ABORT, "...as an ABORT");
    expect(c.trajectory_id == id1, "...naming the trajectory being flown");
    expect(c.reason == Ctl::REASON_POLICY_UNEVALUATED,
           "...with the POLICY reason, not the obstacle one");
  }

  // (6) A new mission after the abort. Without the recovery branch the FSM
  // sits in WAIT_EXTERNAL_RECOVERY and this plan is never published.
  const size_t trajs_before = trajs.size();
  fsm.trajectoryCommandCallback(makeMission(2, "B", 320.0, 150.0, 4.0));
  spin(node, 4000);
  expect(trajs.size() > trajs_before,
         "a mission after an abort is published — the FSM leaves the "
         "recovery state");
  if (trajs.size() > trajs_before) {
    const uint64_t id2 = trajs.back().trajectory_id;
    std::cout << "  .. trajectory id " << id2 << "\n";
    expect(id2 > id1, "...with a HIGHER id, so the old abort cannot bind it");

    // (7) Terrain rises above the flight. Same geometry, higher ground.
    const size_t ctls_before2 = ctls.size();
    fsm.terrainCallback(makeFlatMap(60.0));
    spin(node, 600);
    expect(ctls.size() == ctls_before2 + 1,
           "a DEM that puts ground through the flight aborts it");
    if (ctls.size() == ctls_before2 + 1) {
      const auto &c = ctls.back();
      expect(c.trajectory_id == id2, "...naming the CURRENT trajectory");
      expect(c.reason == Ctl::REASON_TERRAIN_BLOCKED,
             "...with the TERRAIN reason — this is the path that used to "
             "report every terrain block as an obstacle");
    }
  }

  // (8) An OBSTACLE abort, and the environment revision on the wire. The
  // field used to carry the zone policy generation alone, which an obstacle
  // change does not move — so consecutive aborts reported the same
  // environment. Only a wire assertion catches that: a test of the accessor
  // alone passes with the old value still being published.
  if (!ctls.empty()) {
    const uint64_t gen_before = ctls.back().environment_generation;
    const size_t ctls_before3 = ctls.size();
    // Restore the ground before testing an obstacle at the original height.
    fsm.terrainCallback(makeFlatMap(0.0));
    const size_t before_third = trajs.size();
    fsm.trajectoryCommandCallback(makeMission(3, "C", 300.0, 150.0, 4.0));
    spin(node, 4000);
    expect(trajs.size() > before_third, "obstacle scenario starts with a fresh flight");
    if (trajs.size() > before_third && trajs.back().trajectory_id > 0) {
      const uint64_t id3 = trajs.back().trajectory_id;
      auto obstacles = std::make_shared<mmp_mission_msgs::msg::DynamicObstacleArray>();
      obstacles->replace = true;
      mmp_mission_msgs::msg::DynamicObstacleSpec sp;
      sp.kind = mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_SPHERE;
      sp.center.x = 165.0; sp.center.y = 150.0; sp.center.z = 3.0;
      sp.radius = 12.0;
      obstacles->obstacles.push_back(sp);
      fsm.loadObstaclesCallback(obstacles);
      spin(node, 600);
      expect(ctls.size() == ctls_before3 + 1,
             "an obstacle on the remaining flight aborts it");
      if (ctls.size() == ctls_before3 + 1) {
        const auto &c = ctls.back();
        expect(c.reason == Ctl::REASON_OBSTACLE_BLOCKED,
               "...with the OBSTACLE reason");
        expect(c.trajectory_id == id3, "...naming the current trajectory");
        expect(c.environment_generation > gen_before,
               "...and an environment revision that MOVED — an obstacle "
               "change does not touch the zone policy generation, which is "
               "what this field used to carry");
      }
    }
  }

  // (9) A valid result can wait for its publication tick while a replacement
  // command is refused. The refusal must not discard the valid pending result.
  auto empty_obstacles = std::make_shared<mmp_mission_msgs::msg::DynamicObstacleArray>();
  empty_obstacles->replace = true;
  fsm.loadObstaclesCallback(empty_obstacles);
  fsm.terrainCallback(makeFlatMap(0.0));
  const size_t before_pending = trajs.size();
  fsm.trajectoryCommandCallback(makeMission(4, "D", 330.0, 150.0, 4.0));
  auto rejected = makeMission(5, "invalid-replacement", 330.0, 150.0, 4.0);
  rejected->initial_speed = 400.0;
  fsm.trajectoryCommandCallback(rejected);
  spin(node, 1200);
  expect(trajs.size() == before_pending + 1,
         "refused replacement retains and publishes the prior successful pending result once");

  // (10) Invalidation before the timer publishes must discard the pending
  // calculation too. This path is narrower than aborting an already flown ID.
  const size_t before_withdrawn = trajs.size();
  const size_t controls_before_pending = ctls.size();
  const uint64_t last_published_id = trajs.empty() ? 0 : trajs.back().trajectory_id;
  fsm.trajectoryCommandCallback(makeMission(5, "E", 320.0, 150.0, 4.0));
  fsm.loadRiskZonesCallback(makeZones({{600.0, 600.0, 3.0, 35.0, 0.9}}));
  spin(node, 800);
  expect(trajs.size() == before_withdrawn,
         "a result withdrawn before its publication tick is never emitted");
  expect(ctls.size() == controls_before_pending + 1 &&
             ctls.back().trajectory_id > last_published_id &&
             ctls.back().reason == Ctl::REASON_POLICY_UNEVALUATED,
         "pre-publication withdrawal names the newly allocated ID exactly once");
  const uint64_t withdrawn_id = ctls.empty() ? 0 : ctls.back().trajectory_id;
  fsm.trajectoryCommandCallback(makeMission(6, "F", 330.0, 150.0, 4.0));
  spin(node, 1200);
  expect(trajs.size() == before_withdrawn + 1 &&
             trajs.back().trajectory_id > withdrawn_id,
         "a fresh library result recovers after pending publication was withdrawn");

  // (11) ROS commands must use the same settings validation as the public
  // mission API. Previously the FSM wrote these fields to PathManager first,
  // then sent a request whose override fields were both zero. A refused
  // +infinity clearance poisoned every later otherwise-valid mission.
  const auto same_polynomial = [](const Traj &a, const Traj &b) {
    return a.order == b.order && a.coef_x == b.coef_x &&
           a.coef_y == b.coef_y && a.coef_z == b.coef_z &&
           a.duration == b.duration;
  };
  Traj before_overrides;
  fsm.polyTraj2ROSMsg(before_overrides);
  int override_sequence = 7;
  for (double invalid : {std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    for (bool invalid_length : {true, false}) {
      const std::string mission_id = "override-" + std::to_string(override_sequence);
      auto command = makeMission(override_sequence, mission_id, 330.0, 150.0, 4.0);
      command->length_per_piece = invalid_length ? invalid : 37.0;
      command->obstacle_clearance = invalid_length ? 5.0 : invalid;
      fsm.trajectoryCommandCallback(command);
      Traj refused;
      fsm.polyTraj2ROSMsg(refused);
      expect(refused.trajectory_id == before_overrides.trajectory_id &&
                 refused.start_time == before_overrides.start_time &&
                 same_polynomial(refused, before_overrides),
             "nonfinite ROS override refuses without replacing the active result");

      // The same sequence must remain available after refusal. Negative
      // finite values retain the existing settings, including the OTHER
      // field whose valid positive override was paired with the bad value.
      command = makeMission(override_sequence, mission_id, 330.0, 150.0, 4.0);
      command->length_per_piece = -3.0;
      command->obstacle_clearance = -2.0;
      fsm.trajectoryCommandCallback(command);
      Traj corrected;
      fsm.polyTraj2ROSMsg(corrected);
      expect(corrected.trajectory_id == before_overrides.trajectory_id + 1 &&
                 same_polynomial(corrected, before_overrides),
             "same-sequence corrected ROS command reproduces every coefficient and piece time");
      before_overrides = std::move(corrected);
      ++override_sequence;
    }
  }

  // Positive overrides still apply and persist. The next command deliberately
  // omits them and must produce exactly the same changed polynomial.
  auto positive = makeMission(override_sequence++, "positive-overrides", 330.0, 150.0, 4.0);
  positive->length_per_piece = 12.5;
  positive->obstacle_clearance = 0.45;
  fsm.trajectoryCommandCallback(positive);
  Traj changed;
  fsm.polyTraj2ROSMsg(changed);
  expect(changed.trajectory_id == before_overrides.trajectory_id + 1 &&
             !same_polynomial(changed, before_overrides),
         "valid positive ROS settings reach the library and change the solve");
  fsm.trajectoryCommandCallback(makeMission(
      override_sequence, "retained-overrides", 330.0, 150.0, 4.0));
  Traj retained;
  fsm.polyTraj2ROSMsg(retained);
  expect(retained.trajectory_id == changed.trajectory_id + 1 &&
             same_polynomial(retained, changed),
         "omitted ROS settings retain both previously accepted overrides");

  rclcpp::shutdown();
  if (g_failed == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
  std::cout << "FAIL: " << g_failed << " failed check(s)\n";
  return 1;
}
