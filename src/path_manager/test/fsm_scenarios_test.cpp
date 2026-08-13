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
// Not registered with ctest, for the same reason chain_experiment_test is
// not: it runs real solves.

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "path_manager/replan_fsm.h"
#include "mmp_mission_msgs/msg/trajectory_command.hpp"
#include "mmp_mission_msgs/msg/risk_zone_array.hpp"
#include "mmp_mission_msgs/msg/risk_zone_spec.hpp"
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
  m->initial_speed = 150.0;   // inside the cruise band (122..200 m/s)
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

  // (2) A zone set, BEFORE any trajectory exists. The set starts empty, so
  // this batch IS a change — installing it now means the no-op check below
  // is about the fingerprint and not about this.
  const std::vector<std::array<double, 5>> zonesA{
      {600.0, 600.0, 3.0, 15.0, 0.9}};   // far off any route
  fsm.loadRiskZonesCallback(makeZones(zonesA));
  spin(node, 200);
  expect(ctls.empty(), "installing zones with nothing flying aborts nothing");

  // (3) Fly.
  fsm.trajectoryCommandCallback(makeMission(1, "A", 330.0, 150.0, 1.5));
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
  fsm.trajectoryCommandCallback(makeMission(2, "B", 320.0, 140.0, 1.5));
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

  rclcpp::shutdown();
  if (g_failed == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
  std::cout << "FAIL: " << g_failed << " failed check(s)\n";
  return 1;
}
