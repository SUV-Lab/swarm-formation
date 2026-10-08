// Public route-search replacement and admission contracts. Real-backend
// integration uses a separately owned built-in searcher, not canned geometry.
#include <path_manager/route_search_backend.h>
#include <path_manager/builtin_route_search.h>
#include <path_manager/ros_route_search.h>
#include <path_manager/path_manager.h>
#include <path_manager/mission_planner.h>
#include <path_manager/ros_planning_backend.h>
#include <path_planner/sdf/distance_field.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <vector>

namespace pm = path_manager;
namespace {
int failures = 0;
void check(bool ok, const std::string &message) {
  std::cout << (ok ? "[OK] " : "[FAIL] ") << message << std::endl;
  if (!ok) ++failures;
}
bool samePoints(const std::vector<Eigen::Vector3d> &a,
                const std::vector<Eigen::Vector3d> &b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) if ((a[i] - b[i]).norm() != 0.0) return false;
  return true;
}
bool sameRoute(const pm::CommittedRoute &a, const pm::CommittedRoute &b) {
  return samePoints(a.path, b.path) && a.cap == b.cap && a.edge_leg == b.edge_leg;
}
std::vector<double> coefficients(const poly_traj::Trajectory &trajectory) {
  std::vector<double> result;
  for (const auto &piece : trajectory) {
    result.push_back(piece.getDuration()); const auto &m = piece.getCoeffMat();
    result.insert(result.end(), m.data(), m.data() + m.size());
  }
  return result;
}
void admission() {
  pm::RouteSearchProblem p;
  p.identity = {101, 102, 103, 104, 105, 106, 1};
  p.environment.zones.resize(2);
  pm::RouteSearchPolicy policy;
  policy.knowledge = pm::RoutePolicyKnowledge::CANONICAL_DISPOSITIONS;
  policy.identity = p.identity;
  policy.dispositions = {pm::RouteZoneDisposition::HARD_AVOID,
                         pm::RouteZoneDisposition::SOFT_UNAVOIDABLE};
  check(pm::routeSearchPolicyMeasured(p, policy),
        "external canonical dispositions are measured without inventing an FM2 pass");
  auto projected = policy;
  projected.endpoint_exempt = {0, 0}; projected.soft_override = {0, 1};
  check(pm::routeSearchPolicyMeasured(p, projected),
        "pass-zero diagnostics may supply consistent canonical projections");
  projected.endpoint_exempt[0] = 1;
  check(!pm::routeSearchPolicyMeasured(p, projected),
        "pass-zero endpoint flag cannot contradict HARD_AVOID disposition");
  projected = policy; projected.dispositions[0] = pm::RouteZoneDisposition::SOFT_ENDPOINT;
  projected.endpoint_exempt = {0, 0};
  check(!pm::routeSearchPolicyMeasured(p, projected),
        "pass-zero endpoint flag cannot deny SOFT_ENDPOINT disposition");
  projected.endpoint_exempt[0] = 1;
  check(pm::routeSearchPolicyMeasured(p, projected),
        "pass-zero endpoint projection can be supplied without soft diagnostics");
  projected = policy; projected.soft_override = {1, 1};
  check(!pm::routeSearchPolicyMeasured(p, projected),
        "pass-zero soft flag cannot contradict HARD_AVOID disposition");
  projected.soft_override = {0, 0};
  check(!pm::routeSearchPolicyMeasured(p, projected),
        "pass-zero soft flag cannot deny SOFT_UNAVOIDABLE disposition");
  auto unknown = policy; unknown.knowledge = pm::RoutePolicyKnowledge::UNKNOWN;
  check(!pm::routeSearchPolicyMeasured(p, unknown), "UNKNOWN policy cannot certify ordered zone dispositions");
  using IdentityField = uint64_t pm::RouteSearchIdentity::*;
  const std::vector<std::pair<std::string, IdentityField>> fields = {
      {"request", &pm::RouteSearchIdentity::request_id},
      {"context", &pm::RouteSearchIdentity::context_id},
      {"environment revision", &pm::RouteSearchIdentity::environment_revision},
      {"zone generation", &pm::RouteSearchIdentity::zone_generation},
      {"policy epoch", &pm::RouteSearchIdentity::policy_epoch},
      {"search serial", &pm::RouteSearchIdentity::search_serial}};
  for (const auto &field : fields) {
    auto bad = policy; ++(bad.identity.*field.second);
    check(!pm::routeSearchPolicyMeasured(p, bad), "stale " + field.first + " cannot be admitted as current policy");
  }
  auto bad = policy; ++bad.identity.leg;
  check(!pm::routeSearchPolicyMeasured(p, bad), "policy for another leg cannot be admitted");
  bad = policy; bad.dispositions.pop_back();
  check(!pm::routeSearchPolicyMeasured(p, bad), "missing ordered disposition is unmeasured");
  bad = policy; bad.dispositions.push_back(pm::RouteZoneDisposition::HARD_AVOID);
  check(!pm::routeSearchPolicyMeasured(p, bad), "extra ordered disposition is unmeasured");
  bad = policy; bad.dispositions[0] = static_cast<pm::RouteZoneDisposition>(99);
  check(!pm::routeSearchPolicyMeasured(p, bad), "unrecognized disposition enum is unmeasured");
  bad = policy; bad.knowledge = static_cast<pm::RoutePolicyKnowledge>(99);
  check(!pm::routeSearchPolicyMeasured(p, bad), "unrecognized policy knowledge enum is unmeasured");
  for (bool endpoint : {false, true}) {
    bad = policy;
    (endpoint ? bad.endpoint_exempt : bad.soft_override) = {0};
    check(!pm::routeSearchPolicyMeasured(p, bad), "partial raw diagnostic shape is unmeasured");
    bad = policy;
    (endpoint ? bad.endpoint_exempt : bad.soft_override) = {0, 2};
    check(!pm::routeSearchPolicyMeasured(p, bad), "nonboolean raw diagnostic is unmeasured");
  }
  for (int pass : {-1, 4}) {
    bad = policy; bad.legacy_pass = pass;
    check(!pm::routeSearchPolicyMeasured(p, bad), "invalid legacy pass cannot certify a policy");
  }
  auto builtin = policy;
  builtin.legacy_pass = 1; builtin.endpoint_exempt = {1, 0}; builtin.soft_override = {0, 1};
  builtin.dispositions = {pm::RouteZoneDisposition::SOFT_ENDPOINT, pm::RouteZoneDisposition::SOFT_UNAVOIDABLE};
  builtin.suppress_crossing_exempt = true;
  check(pm::routeSearchPolicyMeasured(p, builtin), "complete consistent built-in pass-one diagnostics are admitted");
  bad = builtin; bad.endpoint_exempt.clear();
  check(!pm::routeSearchPolicyMeasured(p, bad), "claimed built-in pass requires endpoint diagnostics");
  bad = builtin; bad.soft_override.clear();
  check(!pm::routeSearchPolicyMeasured(p, bad), "claimed built-in pass requires soft-override diagnostics");
  bad = builtin; bad.dispositions[0] = pm::RouteZoneDisposition::HARD_AVOID;
  check(!pm::routeSearchPolicyMeasured(p, bad), "endpoint disposition cannot contradict built-in diagnostics");
  bad = builtin; bad.suppress_crossing_exempt = false;
  check(!pm::routeSearchPolicyMeasured(p, bad), "pass-one crossing policy cannot contradict its explicit diagnostic claim");
  builtin.legacy_pass = 2; builtin.suppress_crossing_exempt = false;
  builtin.dispositions[1] = pm::RouteZoneDisposition::SOFT_FALLBACK;
  check(pm::routeSearchPolicyMeasured(p, builtin), "pass-two fallback still respects endpoint precedence");
  bad = builtin; bad.dispositions[1] = pm::RouteZoneDisposition::SOFT_UNAVOIDABLE;
  check(!pm::routeSearchPolicyMeasured(p, bad), "pass-two fallback cannot claim an incompatible disposition");
  builtin.legacy_pass = 3; builtin.dispositions[1] = pm::RouteZoneDisposition::SOFT_UNAVOIDABLE;
  check(pm::routeSearchPolicyMeasured(p, builtin), "pass-three ordered diagnostics retain unavoidable-soft disposition");

  pm::RouteSearchResult geometry;
  geometry.status = pm::RouteSearchStatus::SUCCESS;
  geometry.points = {Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6)};
  check(pm::routeSearchGeometryUsable(geometry), "finite successful geometry is usable independently of UNKNOWN policy");
  auto invalid = geometry; invalid.status = pm::RouteSearchStatus::NO_ROUTE;
  check(!pm::routeSearchGeometryUsable(invalid), "failure status cannot publish leftover geometry");
  invalid = geometry; invalid.points.resize(1);
  check(!pm::routeSearchGeometryUsable(invalid), "successful one-point geometry is refused");
  invalid = geometry; invalid.points.clear();
  check(!pm::routeSearchGeometryUsable(invalid), "successful empty geometry is refused");
  for (double value : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    invalid = geometry; invalid.points.back().z() = value;
    check(!pm::routeSearchGeometryUsable(invalid), "nonfinite geometry is refused before route postprocessing");
  }
  const auto id1 = pm::nextRouteSearchIdentity(), id2 = pm::nextRouteSearchIdentity();
  check(id1 != 0 && id2 > id1, "actual invocation identities are nonzero and advance");
}

class FlatDistanceField final : public path_planner::sdf::IDistanceField {
 public:
  float getDistance(const Eigen::Vector3d &p) const override {
    ++queries; return static_cast<float>(p.z());
  }
  bool getDistanceAndGradient(const Eigen::Vector3d &p, float *distance,
                              Eigen::Vector3d *gradient) const override {
    if (distance) *distance = getDistance(p);
    if (gradient) *gradient = Eigen::Vector3d::UnitZ();
    return true;
  }
  bool hasData() const override { return true; }
  mutable size_t queries{0};
};

void astarIntegration() {
  // Deliberately tiny grid: this exercises real A* pool allocation and route
  // generation without ever selecting A* on the 2804 x 4119 R2 terrain.
  FlatDistanceField field;
  pm::RouteSearchProblem problem;
  problem.start = {4, 16, 4}; problem.goal = {28, 16, 4};
  problem.identity = {pm::nextRouteSearchIdentity(), pm::nextRouteSearchIdentity(), 1, 1, 1, 1, 0};
  problem.environment.distance_field = &field;
  problem.environment.lower = {0, 0, 0}; problem.environment.upper = {32, 32, 12};
  problem.environment.fine_grid_shape = {33, 33, 13};
  problem.environment.voxel_xy = 1; problem.environment.voxel_z = 1;
  problem.environment.terrain_cell_u = 1;
  problem.environment.elevation = [](double, double) { return 0.0f; };
  problem.environment.elevation_gradient = [](double, double, float *z, float *x, float *y) {
    if (z) *z = 0;
    if (x) *x = 0;
    if (y) *y = 0;
    return true;
  };
  problem.environment.zones.push_back({Eigen::Vector3d(16, 3, 4), 2.0, 1.0, 2.0});
  problem.constraints.flight_path_angle_max_rad = 0.5;
  pm::BuiltinRouteSearchConfig config;
  config.algorithm = pm::BuiltinRouteSearchConfig::Algorithm::ASTAR;
  config.astar.step_size = 1.0;
  auto backend = pm::makeBuiltinRouteSearchBackend(config);
  auto result = backend->search(problem);
  check(result.algorithm == "astar" && pm::routeSearchGeometryUsable(result) && field.queries > 0,
        "public built-in factory selects and executes actual A* on a small SDF grid");
  check(result.policy.legacy_pass == 0 && result.policy.knowledge == pm::RoutePolicyKnowledge::UNKNOWN &&
            !pm::routeSearchPolicyMeasured(problem, result.policy),
        "zone-bearing A* truthfully returns UNKNOWN policy instead of inventing an FM2 pass");
  check(pm::routeSearchGeometryUsable(result) && (result.points.front() - problem.start).norm() == 0.0 &&
            (result.points.back() - problem.goal).norm() == 0.0,
        "A* UNKNOWN policy retains successful finite endpoint geometry");
  const auto retained = result.points;
  problem.environment.zones.clear(); problem.identity.request_id = pm::nextRouteSearchIdentity();
  ++problem.identity.zone_generation; ++problem.identity.search_serial;
  result = backend->search(problem);
  check(pm::routeSearchGeometryUsable(result) && result.policy.legacy_pass == 0 &&
            result.policy.dispositions.empty() && pm::routeSearchPolicyMeasured(problem, result.policy),
        "A* empty-zone result certifies only the empty canonical policy");
  problem.environment.fine_grid_shape = {41, 33, 13};
  problem.environment.upper.x() = 40; problem.goal.x() = 36;
  problem.identity.request_id = pm::nextRouteSearchIdentity();
  result = backend->search(problem);
  check(pm::routeSearchGeometryUsable(result) && result.algorithm == "astar" &&
            (result.points.back() - problem.goal).norm() == 0.0,
        "reused actual A* backend resizes its tiny pool for changed grid geometry");
  backend.reset();
  check(retained.size() >= 2 && retained.front() == Eigen::Vector3d(4, 16, 4) &&
            retained.back() == Eigen::Vector3d(28, 16, 4),
        "returned A* points remain owned after later searches and backend destruction");
}

struct Observation {
  pm::RouteSearchIdentity identity;
  Eigen::Vector3d start, goal;
  size_t zone_count;
  const mmp_vehicle_dynamics::VehicleModel *model;
  bool has_distance_field, has_elevation;
};
class RealWrapper final : public pm::RouteSearchBackend {
 public:
  enum class Mode { NORMAL, UNKNOWN_ALL, UNKNOWN_FIRST, UNKNOWN_SECOND,
                    STALE_REQUEST, STALE_CONTEXT, BAD_DISPOSITIONS, BAD_RAW,
                    FAIL_SECOND, NONFINITE_SECOND, SHORT_SECOND };
  explicit RealWrapper(pm::BuiltinRouteSearchConfig config)
      : delegate_(pm::makeBuiltinRouteSearchBackend(config)) {}
  pm::RouteSearchResult search(const pm::RouteSearchProblem &p) override {
    observations.push_back({p.identity, p.start, p.goal, p.environment.zones.size(),
                            p.model.get(), p.environment.distance_field != nullptr,
                            static_cast<bool>(p.environment.elevation)});
    auto result = delegate_->search(p); // This is an actual independent FM2 search.
    if (result.status != pm::RouteSearchStatus::SUCCESS) return result;
    switch (mode) {
      case Mode::NORMAL: break;
      case Mode::UNKNOWN_ALL: result.policy.knowledge = pm::RoutePolicyKnowledge::UNKNOWN; break;
      case Mode::UNKNOWN_FIRST: if (p.identity.leg == 0) result.policy.knowledge = pm::RoutePolicyKnowledge::UNKNOWN; break;
      case Mode::UNKNOWN_SECOND: if (p.identity.leg == 1) result.policy.knowledge = pm::RoutePolicyKnowledge::UNKNOWN; break;
      case Mode::STALE_REQUEST: ++result.policy.identity.request_id; break;
      case Mode::STALE_CONTEXT: ++result.policy.identity.context_id; break;
      case Mode::BAD_DISPOSITIONS: result.policy.dispositions.clear(); break;
      case Mode::BAD_RAW: result.policy.endpoint_exempt = {0, 0, 0}; break;
      case Mode::FAIL_SECOND: if (p.identity.leg == 1) result.status = pm::RouteSearchStatus::NO_ROUTE; break;
      case Mode::NONFINITE_SECOND: if (p.identity.leg == 1) result.points.back().z() = std::numeric_limits<double>::quiet_NaN(); break;
      case Mode::SHORT_SECOND: if (p.identity.leg == 1) result.points.resize(1); break;
    }
    last_result = result; return result;
  }
  Mode mode{Mode::NORMAL};
  std::vector<Observation> observations; // Values only; no borrowed providers retained.
  pm::RouteSearchResult last_result;
 private:
  std::shared_ptr<pm::RouteSearchBackend> delegate_;
};

grid_map_msgs::msg::GridMap::SharedPtr flatTerrain() {
  auto map = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr int rows = 180, cols = 150;
  map->info.resolution = 2.0; map->info.length_x = 360.0; map->info.length_y = 300.0;
  map->info.pose.position.x = 180.0; map->info.pose.position.y = 150.0;
  map->info.pose.orientation.w = 1.0; map->layers = {"elevation"}; map->data.resize(1);
  auto &layer = map->data.front(); layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index"; layer.layout.dim[0].size = cols;
  layer.layout.dim[0].stride = cols * rows; layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = rows; layer.layout.dim[1].stride = rows;
  layer.data.assign(rows * cols, 2.0f); return map;
}
rclcpp::Node::SharedPtr nodeFor(const char *config, const std::string &name) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config});
  options.parameter_overrides({rclcpp::Parameter("manager/front_end", std::string("fm2")),
      rclcpp::Parameter("chain/enable", true), rclcpp::Parameter("chain/segments", 3),
      rclcpp::Parameter("chain/phase/enable", false), rclcpp::Parameter("chain/author_from_route", true),
      rclcpp::Parameter("chain/parallel", true), rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
  auto node = rclcpp::Node::make_shared(name, options);
  node->declare_parameter("drone_id", 7);
  return node;
}
pm::StartHead head() {
  pm::StartHead h; h.src = pm::StartStateSource::STATED_VECTOR;
  h.pos_u = Eigen::Vector3d(30, 150, 6); h.vel_u = Eigen::Vector3d(2, 0, 0); return h;
}
std::vector<Eigen::Vector3d> waypoints() { return {{180, 150, 4}, {330, 150, 4}}; }
void routeIntegration(const char *config) {
  std::optional<pm::CommittedRoute> retained;
  pm::CommittedRoute expected;
  pm::RouteSearchResult retained_raw;
  std::vector<Eigen::Vector3d> raw_points;
  std::weak_ptr<pm::RouteSearchBackend> backend_lifetime;
  std::shared_ptr<RealWrapper> reusable;
  uint64_t first_context = 0, first_request = 0;
  {
    auto node = nodeFor(config, "route_backend_manager");
    pm::PathManager manager(node); manager.initOptimizer();
    manager.setTerrainData(flatTerrain());
    check(manager.setRiskZonesRuntime({{Eigen::Vector3d(180, 40, 8), 4.0, 1.0}}),
          "zone-bearing fixture installs a distant nonintersecting policy volume");
    const auto baseline = manager.planCommittedRoute(head(), waypoints());
    check(baseline.has_value(), "default backend plans the real two-leg fixture");
    if (!baseline) return;
    const auto default_policies = manager.legPolicySnapshots();
    check(default_policies.size() == 2 && default_policies[0].policy.valid && default_policies[1].policy.valid,
          "built-in baseline captures measured policy for both legs");
    reusable = std::make_shared<RealWrapper>(pm::readRosBuiltinRouteSearchConfig(*node));
    backend_lifetime = reusable; manager.setRouteSearchBackend(reusable);
    retained = manager.planCommittedRoute(head(), waypoints());
    check(retained.has_value() && sameRoute(*baseline, *retained),
          "a separately owned real-backend wrapper reproduces default geometry, caps and edge provenance exactly");
    if (!retained) return;
    expected = *retained;
    check(reusable->observations.size() == 2, "replacement receives one actual search per mission leg");
    if (reusable->observations.size() >= 2) {
      const auto &a = reusable->observations[0], &z = reusable->observations[1];
      first_context = a.identity.context_id; first_request = a.identity.request_id;
      check(a.identity.request_id != 0 && z.identity.request_id != a.identity.request_id &&
                a.identity.context_id != 0 && a.identity.context_id == z.identity.context_id &&
                a.identity.policy_epoch == retained->epoch && z.identity.policy_epoch == retained->epoch &&
                a.identity.search_serial == 1 && z.identity.search_serial == 2 &&
                a.identity.leg == 0 && z.identity.leg == 1,
            "actual leg identities carry unique invocation, stable owner, current epoch and ordered serials");
      check(a.has_distance_field && a.has_elevation && a.zone_count == 1 &&
                a.model == manager.vehicleModel().get() && z.model == a.model &&
                a.identity.environment_revision == manager.environmentRevision() &&
                a.identity.zone_generation == manager.zonePolicyGeneration(),
            "replacement receives the real terrain/SDF/ordered zones and the same physical model");
    }
    auto captures = manager.legPolicySnapshots();
    check(captures.size() == 2 && captures[0].policy.valid && captures[1].policy.valid &&
              captures[0].policy.zones.size() == 1 && captures[1].policy.zones.size() == 1,
          "measured replacement metadata becomes complete per-leg policy captures");
    retained_raw = reusable->last_result; raw_points = retained_raw.points;
    const std::vector<std::pair<RealWrapper::Mode, std::string>> unmeasured = {
        {RealWrapper::Mode::UNKNOWN_ALL, "unknown"}, {RealWrapper::Mode::UNKNOWN_FIRST, "first-leg unknown"},
        {RealWrapper::Mode::UNKNOWN_SECOND, "second-leg unknown"}, {RealWrapper::Mode::STALE_REQUEST, "stale invocation"},
        {RealWrapper::Mode::STALE_CONTEXT, "wrong owner"}, {RealWrapper::Mode::BAD_DISPOSITIONS, "missing dispositions"},
        {RealWrapper::Mode::BAD_RAW, "malformed raw diagnostics"}};
    for (const auto &entry : unmeasured) {
      reusable->mode = entry.first;
      const auto result = manager.planCommittedRoute(head(), waypoints());
      check(result.has_value() && sameRoute(*baseline, *result), entry.second + " policy keeps real successful geometry");
      check(manager.legPolicySnapshots().empty(), entry.second + " policy leaves no partial or stale measured captures");
    }
    const std::vector<std::pair<RealWrapper::Mode, std::string>> invalid_geometry = {
        {RealWrapper::Mode::FAIL_SECOND, "second-leg failure"},
        {RealWrapper::Mode::NONFINITE_SECOND, "second-leg nonfinite result"},
        {RealWrapper::Mode::SHORT_SECOND, "second-leg short result"}};
    for (const auto &entry : invalid_geometry) {
      reusable->mode = entry.first;
      const auto result = manager.planCommittedRoute(head(), waypoints());
      check(!result && manager.lastCommittedRoute().empty() && manager.lastCommittedCapRef().empty() &&
                manager.lastCommittedRouteEdgeLeg().empty() && manager.lastCommittedRouteEpoch() == 0,
            entry.second + " returns no partial committed route or stale cache");
    }
    reusable->mode = RealWrapper::Mode::NORMAL;
    const auto recovered = manager.planCommittedRoute(head(), waypoints());
    check(recovered.has_value() && sameRoute(*baseline, *recovered) && manager.legPolicySnapshots().size() == 2,
          "valid measured search recovers after unknown policy and geometry failures");
    check(sameRoute(*retained, expected) && samePoints(retained_raw.points, raw_points),
          "later backend calls cannot mutate retained route and raw result storage");
    manager.setRouteSearchBackend(nullptr);
    const auto calls = reusable->observations.size();
    const auto restored = manager.planCommittedRoute(head(), waypoints());
    check(restored.has_value() && sameRoute(*baseline, *restored) && reusable->observations.size() == calls,
          "null restores default routing and stops calling the replacement");
  }
  // Reuse an actual backend only after its old borrowed environment is gone.
  {
    auto node = nodeFor(config, "route_backend_second_owner");
    pm::PathManager manager(node); manager.initOptimizer(); manager.setTerrainData(flatTerrain());
    manager.setRiskZonesRuntime({}); manager.setRouteSearchBackend(reusable);
    const auto calls = reusable->observations.size();
    const auto route = manager.planCommittedRoute(head(), waypoints());
    check(route.has_value() && reusable->observations.size() >= calls + 2,
          "owned replacement can bind a new manager after the original environment is destroyed");
    if (reusable->observations.size() > calls) {
      const auto &identity = reusable->observations[calls].identity;
      check(identity.context_id != first_context && identity.request_id != first_request,
            "backend reuse across managers receives a distinct context and invocation identity");
    }
    reusable.reset();
    check(!backend_lifetime.expired(), "manager owns the replacement after caller ownership is released");
  }
  check(backend_lifetime.expired(), "manager destruction releases the replacement without a cycle");
  check(retained.has_value() && sameRoute(*retained, expected) && samePoints(retained_raw.points, raw_points),
        "returned route and backend result own storage after all managers/backends die");
}
void facadeIntegration(const char *config) {
  auto node = nodeFor(config, "route_backend_facade");
  auto backend = pm::makeRosPlanningBackend(node); auto *ros = backend.get();
  pm::MissionPlanner planner({}, {}, std::move(backend));
  ros->setTerrainData(flatTerrain()); planner.setRiskZones({});
  pm::MissionPlanRequest request; request.head = head();
  request.waypoints = {Eigen::Vector3d(330, 150, 4)};
  request.tail = ego_planner::TailBoundary::pinned(request.head.vel_u, Eigen::Vector3d::Zero());
  const auto baseline = planner.plan(request);
  check(baseline.hasTrajectory() && baseline.evaluation.evaluated, "default facade fixture generates and evaluates a whole mission");
  if (!baseline.hasTrajectory()) return;
  auto replacement = std::make_shared<RealWrapper>(pm::readRosBuiltinRouteSearchConfig(*node));
  planner.setRouteSearchBackend(replacement);
  const auto result = planner.plan(request);
  check(result.hasTrajectory() && !replacement->observations.empty(), "whole-mission API actually invokes the external real backend");
  check(result.hasTrajectory() && coefficients(result.trajectory) == coefficients(baseline.trajectory) &&
            coefficients(result.reference_trajectory) == coefficients(baseline.reference_trajectory),
        "replacement route backend preserves exact final and reference polynomial coefficients/durations");
  bool phases_equal = result.phases.size() == baseline.phases.size();
  for (size_t i = 0; phases_equal && i < result.phases.size(); ++i)
    phases_equal = result.phases[i].t_end == baseline.phases[i].t_end && result.phases[i].kind == baseline.phases[i].kind &&
                   result.phases[i].name == baseline.phases[i].name;
  check(phases_equal && result.evaluation.evaluated == baseline.evaluation.evaluated &&
            result.evaluation.clean == baseline.evaluation.clean && result.evaluation.util_peak == baseline.evaluation.util_peak &&
            result.evaluation.viol_pct == baseline.evaluation.viol_pct &&
            result.evaluation.policy_measurable == baseline.evaluation.policy_measurable,
        "replacement preserves phase boundaries and whole-flight evaluation");
}
} // namespace
int main(int argc, char **argv) {
  if (argc != 2) { std::cerr << "usage: route_search_backend_test <optimizer_params.yaml>\n"; return 2; }
  rclcpp::init(argc, argv);
  try { admission(); astarIntegration(); routeIntegration(argv[1]); facadeIntegration(argv[1]); }
  catch (const std::exception &e) { check(false, std::string("unexpected exception: ") + e.what()); }
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
