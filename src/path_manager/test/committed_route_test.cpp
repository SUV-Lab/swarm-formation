// Exercise the public route-only boundary with real front-end planning.
// No executor, FSM, private hooks, or external mission/map fixtures are needed.
#include "path_manager/path_manager.h"
#include "path_manager/committed_route.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

using path_manager::CommittedRoute;
using path_manager::PathManager;
using path_manager::StartHead;
int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

bool samePoints(const std::vector<Eigen::Vector3d> &a,
                const std::vector<Eigen::Vector3d> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!((a[i] - b[i]).norm() == 0.0)) return false;
  return true;
}

bool sameRoute(const CommittedRoute &a, const CommittedRoute &b)
{
  return samePoints(a.path, b.path) && a.cap == b.cap &&
         a.edge_leg == b.edge_leg && a.epoch == b.epoch &&
         a.front_end_ms == b.front_end_ms;
}

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

poly_traj::Trajectory sentinel(double x, double duration)
{
  poly_traj::CoefficientMat c = poly_traj::CoefficientMat::Zero();
  c.col(5) = Eigen::Vector3d(x, 150.0, 6.0);
  c(0, 4) = 2.0;
  return poly_traj::Trajectory({duration}, {c});
}

void seedExecution(PathManager &manager)
{
  path_manager::PlanningComputation candidate;
  candidate.status = path_manager::PlanResult::success();
  candidate.recordProduct({sentinel(12.0, 4.0), sentinel(10.0, 8.0), 124.0, 123.0});
  check(manager.applyPlanningComputation(
            manager.sealPlanningComputation(std::move(candidate))).applied,
        "execution fixture is adopted through the production boundary");
  auto peer = manager.executionTrajectory();
  peer.drone_id = 8;
  peer.end_time = 128.0;
  manager.replaceSwarmTrajectories({peer});
}

ego_planner::TrajContainer executionSnapshot(const PathManager &manager)
{
  ego_planner::TrajContainer result;
  result.local_traj = manager.executionTrajectory();
  result.global_traj = manager.referenceTrajectory();
  result.swarm_traj = manager.swarmTrajectories();
  result.next_traj_id_ = manager.sealPlanningComputation({}).previous_identity_counter;
  return result;
}

bool sameLocal(const ego_planner::LocalTrajData &a,
               const ego_planner::LocalTrajData &b)
{
  return coefficients(a.traj) == coefficients(b.traj) &&
         a.drone_id == b.drone_id && a.traj_id == b.traj_id &&
         a.duration == b.duration && a.start_time == b.start_time &&
         a.end_time == b.end_time && (a.start_pos - b.start_pos).norm() == 0.0;
}

bool sameExecution(const ego_planner::TrajContainer &a,
                   const ego_planner::TrajContainer &b)
{
  const auto &ga = a.global_traj;
  const auto &gb = b.global_traj;
  if (coefficients(ga.traj) != coefficients(gb.traj) ||
      ga.global_start_time != gb.global_start_time ||
      ga.duration != gb.duration || ga.glb_t_of_lc_tgt != gb.glb_t_of_lc_tgt ||
      ga.last_glb_t_of_lc_tgt != gb.last_glb_t_of_lc_tgt ||
      a.next_traj_id_ != b.next_traj_id_ ||
      !sameLocal(a.local_traj, b.local_traj) ||
      a.swarm_traj.size() != b.swarm_traj.size()) return false;
  for (size_t i = 0; i < a.swarm_traj.size(); ++i)
    if (!sameLocal(a.swarm_traj[i], b.swarm_traj[i])) return false;
  return true;
}

grid_map_msgs::msg::GridMap::SharedPtr flatTerrain()
{
  auto map = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr int rows = 180, cols = 150;
  map->info.resolution = 2.0;
  map->info.length_x = 360.0;
  map->info.length_y = 300.0;
  map->info.pose.position.x = 180.0;
  map->info.pose.position.y = 150.0;
  map->info.pose.orientation.w = 1.0;
  map->layers = {"elevation"};
  map->data.resize(1);
  auto &layer = map->data.front();
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index";
  layer.layout.dim[0].size = cols;
  layer.layout.dim[0].stride = cols * rows;
  layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = rows;
  layer.layout.dim[1].stride = rows;
  layer.data.assign(rows * cols, 2.0f);
  return map;
}

rclcpp::Node::SharedPtr makeNode(const char *config, const std::string &name)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config});
  options.parameter_overrides({
      rclcpp::Parameter("manager/front_end", std::string("fm2")),
      rclcpp::Parameter("manager/length_per_piece", 3.0),
      rclcpp::Parameter("manager/corner_fillet_radius", 0.0),
      rclcpp::Parameter("manager/obstacle_clearance", 0.3),
      rclcpp::Parameter("manager/min_goal_agl", 1.0),
      rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
  auto node = rclcpp::Node::make_shared(name, options);
  node->declare_parameter("drone_id", 7);
  return node;
}

StartHead head()
{
  StartHead result;
  result.src = path_manager::StartStateSource::STATED_VECTOR;
  result.pos_u = Eigen::Vector3d(30.0, 150.0, 6.0);
  result.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
  result.acc_u.setZero();
  return result;
}

std::vector<Eigen::Vector3d> mission()
{
  return {{180.0, 150.0, 4.0}, {330.0, 150.0, 4.0}};
}

bool sameCaptures(const std::vector<PathManager::LegPolicySnapshot> &a,
                  const std::vector<PathManager::LegPolicySnapshot> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].leg != b[i].leg || a[i].search_serial != b[i].search_serial ||
        a[i].policy.epoch != b[i].policy.epoch ||
        a[i].policy.generation != b[i].policy.generation ||
        a[i].policy.valid != b[i].policy.valid ||
        !a[i].policy.zones.empty() || !b[i].policy.zones.empty()) return false;
  }
  return true;
}

void routeOnly(const char *config)
{
  std::optional<CommittedRoute> retained;
  CommittedRoute expected;
  {
    auto node = makeNode(config, "committed_route_api");
    PathManager manager(node);
    manager.initOptimizer();
    manager.deliverTrajToOptimizer();
    manager.setTerrainData(flatTerrain());
    check(manager.setRiskZonesRuntime({}), "fixture clears configured risk zones");
    // There are no pending obstacles or later environment updates. The
    // route API retains existing SDF-flush/ABORT behavior when those exist.
    seedExecution(manager);
    const ego_planner::TrajContainer execution = executionSnapshot(manager);
    const auto stated = head();
    auto waypoints = mission();
    retained = manager.planCommittedRoute(stated, waypoints);
    check(retained.has_value(), "one public call returns a committed route");
    if (!retained) return;
    expected = *retained;
    const auto &route = *retained;
    check(route.path.size() >= 3 && route.cap.size() == route.path.size() &&
              route.edge_leg.size() + 1 == route.path.size(),
          "geometry, altitude caps and edge provenance are returned together");
    check((route.path.front() - stated.pos_u).norm() < 1e-9 &&
              (route.path.back() - Eigen::Vector3d(330.0, 150.0, 6.0)).norm() < 1e-9,
          "head remains absolute and goal AGL is resolved against the supplied map");
    check(samePoints(waypoints, mission()) && stated.pos_u == head().pos_u,
          "route preparation leaves the caller's mission inputs unchanged");
    check(!route.edge_leg.empty() && route.edge_leg.front() == 0 &&
              route.edge_leg.back() == 1 &&
              std::is_sorted(route.edge_leg.begin(), route.edge_leg.end()),
          "both mission legs keep their own edge attribution in flight order");
    const auto first_captures = manager.legPolicySnapshots();
    check(route.epoch != 0 && route.epoch == manager.zonePolicyEpoch() &&
              first_captures.size() == 2 &&
              first_captures[0].leg == 0 && first_captures[1].leg == 1 &&
              first_captures[0].search_serial == 1 &&
              first_captures[1].search_serial == 2 &&
              first_captures[0].policy.epoch == route.epoch &&
              first_captures[1].policy.epoch == route.epoch,
          "the returned epoch identifies the two actual front-end captures");
    check(std::isfinite(route.front_end_ms) && route.front_end_ms > 0.0,
          "route-only result includes measured preparation time");
    check(sameExecution(executionSnapshot(manager), execution) &&
              !manager.lastHeadPolicy().evaluated && manager.lastPieceLeg().empty(),
          "route preparation does not run head optimization policy or adopt a flight");

    const bool legacy = manager.planGlobalTraj(
        stated, waypoints, {}, false, nullptr, nullptr, true);
    check(legacy && samePoints(manager.lastCommittedRoute(), route.path) &&
              manager.lastCommittedCapRef() == route.cap &&
              manager.lastCommittedRouteEdgeLeg() == route.edge_leg,
          "legacy front-end-only entry returns exactly the same route products");
    const uint64_t current_epoch = manager.lastCommittedRouteEpoch();
    check(current_epoch == route.epoch + 1 &&
              current_epoch == manager.zonePolicyEpoch(),
          "a repeated search advances its epoch once, even for identical geometry");

    const auto current_captures = manager.legPolicySnapshots();
    PathManager::RouteProvenance current{&route.edge_leg, current_epoch};
    check(manager.planGlobalTraj(stated, waypoints, {}, false, &route.path,
                                &route.cap, true, &current) &&
              manager.lastCommittedRouteEpoch() == current_epoch &&
              manager.lastCommittedRouteEdgeLeg() == route.edge_leg &&
              sameCaptures(manager.legPolicySnapshots(), current_captures),
          "inherited current-epoch provenance preserves captures without a search");
    PathManager::RouteProvenance stale{&route.edge_leg, route.epoch};
    check(manager.planGlobalTraj(stated, waypoints, {}, false, &route.path,
                                &route.cap, true, &stale) &&
              samePoints(manager.lastCommittedRoute(), route.path) &&
              manager.lastCommittedRouteEdgeLeg().empty() &&
              manager.lastCommittedRouteEpoch() == 0 &&
              manager.legPolicySnapshots().empty() &&
              manager.zonePolicyEpoch() == current_epoch,
          "stale inherited provenance loses attribution while geometry remains usable");
    check(sameExecution(executionSnapshot(manager), execution),
          "legacy route-only compatibility and provenance checks preserve execution storage");

    waypoints.front().x() = 170.0;
    waypoints.back().x() = 320.0;
    const auto changed = manager.planCommittedRoute(stated, waypoints);
    check(changed && changed->epoch == current_epoch + 1 &&
              (changed->path.back() - Eigen::Vector3d(320.0, 150.0, 6.0)).norm() < 1e-9 &&
              sameRoute(*retained, expected),
          "another mission returns a new owned route without rewriting the retained result");
    const auto captures = manager.legPolicySnapshots();
    const uint64_t before_refusal = manager.zonePolicyEpoch();
    auto bad_position = stated;
    bad_position.pos_u.x() = std::numeric_limits<double>::quiet_NaN();
    auto bad_velocity = stated;
    bad_velocity.vel_u.y() = std::numeric_limits<double>::infinity();
    auto bad_acceleration = stated;
    bad_acceleration.acc_u.z() = std::numeric_limits<double>::quiet_NaN();
    const std::vector<StartHead> invalid_heads = {
        StartHead{}, bad_position, bad_velocity, bad_acceleration};
    for (size_t i = 0; i < invalid_heads.size(); ++i) {
      check(!manager.planCommittedRoute(invalid_heads[i], waypoints),
            "invalid initial state " + std::to_string(i) + " returns no route");
      check(manager.lastCommittedRoute().empty() &&
                manager.lastCommittedCapRef().empty() &&
                manager.lastCommittedRouteEdgeLeg().empty() &&
                manager.lastCommittedRouteEpoch() == 0 && manager.lastPieceLeg().empty(),
            "entry refusal clears all products attributed to the refused call");
      check(manager.zonePolicyEpoch() == before_refusal &&
                sameCaptures(manager.legPolicySnapshots(), captures) &&
                sameExecution(executionSnapshot(manager), execution) && sameRoute(*retained, expected),
            "entry refusal preserves existing policy captures, execution and owned results");
    }
    check(!manager.planCommittedRoute(stated, {}) &&
              manager.zonePolicyEpoch() == before_refusal &&
              sameCaptures(manager.legPolicySnapshots(), captures),
          "empty mission refuses before any front-end epoch is created");
    const auto recovered = manager.planCommittedRoute(stated, mission());
    check(recovered && recovered->epoch == before_refusal + 1 &&
              samePoints(recovered->path, retained->path) && recovered->cap == retained->cap &&
              recovered->edge_leg == retained->edge_leg &&
              sameExecution(executionSnapshot(manager), execution),
          "the manager can prepare the original mission again after all refusals");
  }
  check(retained && sameRoute(*retained, expected),
        "returned route, caps, provenance and timing outlive manager and node destruction");
}

void missingBackend(const char *config)
{
  auto node = makeNode(config, "committed_route_no_backend");
  PathManager manager(node);
  manager.setTerrainData(flatTerrain());
  check(manager.setRiskZonesRuntime({}), "backend-free fixture clears configured zones");
  seedExecution(manager);
  const ego_planner::TrajContainer execution = executionSnapshot(manager);
  const auto route = manager.planCommittedRoute(head(), mission());
  check(!manager.isOptimizerInitialized() && route.has_value(),
        "route-only preparation works before an optimizer is initialized");
  if (!route) return;
  check(!manager.planGlobalTraj(head(), mission()) &&
            samePoints(manager.lastCommittedRoute(), route->path) &&
            manager.lastCommittedCapRef() == route->cap &&
            manager.lastCommittedRouteEdgeLeg() == route->edge_leg &&
            manager.lastCommittedRouteEpoch() == route->epoch + 1 &&
            manager.lastHeadPolicy().evaluated && manager.lastPieceLeg().empty(),
        "legacy backend refusal retains this call's successfully committed route products");
  check(sameExecution(executionSnapshot(manager), execution),
        "backend refusal does not replace or renumber an already stored flight");
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: committed_route_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  routeOnly(argv[1]);
  missingBackend(argv[1]);
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures
            << " failed check(s)\n";
  return failures ? 1 : 0;
}
