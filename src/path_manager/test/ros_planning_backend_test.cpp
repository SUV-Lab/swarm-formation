#include "path_manager/mission_planner.h"
#include "path_manager/ros_planning_backend.h"
#include "path_manager/ros_terrain_grid.h"
#include "path_manager/path_manager.h"

#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace pm = path_manager;
namespace {
int failures = 0;
void check(bool ok, const std::string &message)
{
  std::cout << (ok ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!ok) ++failures;
}
template<class Exception, class F> bool throws(F f)
{
  try { f(); } catch (const Exception &) { return true; }
  return false;
}
bool bits(float a, float b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
pm::TerrainGrid terrain()
{
  pm::TerrainGrid value;
  value.rows = 3; value.cols = 2; value.resolution = 0.3;
  value.center_x = -2.7; value.center_y = 9.125;
  value.length_x = 0.9123; value.length_y = 0.6234;
  value.elevation = {-1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 99.0f};
  const uint32_t payload = 0x7fc12345;
  std::memcpy(&value.elevation[1], &payload, sizeof(payload));
  return value;
}
pm::TerrainData sampler(const grid_map_msgs::msg::GridMap &message)
{
  pm::TerrainData result;
  const auto &layer = message.data[1];
  result.cols = layer.layout.dim[0].size; result.rows = layer.layout.dim[1].size;
  result.resolution = message.info.resolution;
  result.length_x = message.info.length_x; result.length_y = message.info.length_y;
  result.origin_x = message.info.pose.position.x - message.info.length_x / 2.0;
  result.origin_y = message.info.pose.position.y - message.info.length_y / 2.0;
  result.elevation = layer.data; result.valid = true; result.generation = 1;
  return result;
}
void terrainConversion()
{
  const auto input = terrain();
  auto message = pm::encodeTerrainGrid(input);
  check(message && message->info.length_x == input.length_x &&
            message->info.length_x != input.rows * input.resolution &&
            message->info.pose.position.x == input.center_x,
        "encoder preserves explicit non-normalized geometry and original center");
  // The old ingest ignores these metadata fields. Conversion must not correct
  // them, reorder the raw buffer, or accidentally select the first layer.
  message->layers.insert(message->layers.begin(), "other");
  message->data.insert(message->data.begin(), std_msgs::msg::Float32MultiArray{});
  message->data[1].layout.data_offset = 5;
  message->data[1].layout.dim[0].stride = 123;
  message->data[1].layout.dim[1].stride = 456;
  message->outer_start_index = 1; message->inner_start_index = 2;
  auto decoded = pm::decodeTerrainGrid(message);
  check(decoded.update == pm::TerrainGridUpdate::REPLACE && decoded.terrain &&
            decoded.terrain->elevation.size() == input.elevation.size() &&
            std::memcmp(decoded.terrain->elevation.data(), input.elevation.data(),
                        input.elevation.size() * sizeof(float)) == 0,
        "decoder retains rectangular column-major raw float/NaN bits and extra cells");
  auto roundtrip = pm::encodeTerrainGrid(*decoded.terrain);
  roundtrip->layers.insert(roundtrip->layers.begin(), "other");
  roundtrip->data.insert(roundtrip->data.begin(), std_msgs::msg::Float32MultiArray{});
  const auto before = sampler(*message), after = sampler(*roundtrip);
  bool identical = before.origin_x == after.origin_x && before.origin_y == after.origin_y;
  for (int col = 0; col < before.cols; ++col) for (int row = 0; row < before.rows; ++row) {
    const auto center = before.terrainToWorld(col, row, 0.0f);
    identical &= center == after.terrainToWorld(col, row, 0.0f);
  }
  for (int ix = -1; ix <= 8; ++ix) for (int iy = -1; iy <= 8; ++iy) {
    const double x = before.origin_x + ix * before.length_x / 8.0;
    const double y = before.origin_y + iy * before.length_y / 8.0;
    identical &= bits(before.getElevation(x, y), after.getElevation(x, y));
    float h1 = 0, x1 = 0, y1 = 0, h2 = 0, x2 = 0, y2 = 0;
    const bool a = before.getElevationAndGrad(x, y, &h1, &x1, &y1);
    const bool b = after.getElevationAndGrad(x, y, &h2, &x2, &y2);
    identical &= a == b && bits(h1, h2) && bits(x1, x2) && bits(y1, y2);
  }
  check(identical, "legacy cell centers, boundaries, bilinear heights and gradients remain bit-identical");
  check(pm::decodeTerrainGrid(nullptr).update == pm::TerrainGridUpdate::NO_CHANGE &&
            pm::decodeTerrainGrid(std::make_shared<grid_map_msgs::msg::GridMap>()).update == pm::TerrainGridUpdate::CLEAR,
        "null no-change is distinct from explicit empty-world clear");
  auto malformed = std::make_shared<grid_map_msgs::msg::GridMap>(*message);
  malformed->data[1].data.resize(5);
  const auto rejected = pm::decodeTerrainGrid(malformed);
  check(rejected.update == pm::TerrainGridUpdate::REJECTED && !rejected.terrain && !rejected.detail.empty(),
        "malformed nonempty message is rejection rather than terrain clear");
  malformed = std::make_shared<grid_map_msgs::msg::GridMap>(*message);
  malformed->layers[1] = "wrong";
  check(pm::decodeTerrainGrid(malformed).update == pm::TerrainGridUpdate::REJECTED,
        "missing elevation layer is explicitly rejected");
}
pm::RosPlanningOptions options(const char *config, const std::string &name)
{
  pm::RosPlanningOptions result; result.node_name = name;
  result.node_options.arguments({"--ros-args", "--params-file", config});
  result.node_options.append_parameter_override("chain/enable", false);
  return result;
}
void valueSettings(const char *config)
{
  pm::PlanSettings settings;
  settings.parameters = {{"drone_id", int64_t{41}}, {"chain/segments", int64_t{5}},
    {"test/boolean", true}, {"test/integer", int64_t{17}}, {"test/double", 1.125},
    {"test/text", std::string("typed")}, {"test/bytes", std::vector<uint8_t>{0, 255}},
    {"test/bools", std::vector<bool>{true, false}}, {"test/integers", std::vector<int64_t>{1, 2}},
    {"test/doubles", std::vector<double>{1.25, 2.5}}, {"test/strings", std::vector<std::string>{"a", "b"}}};
  auto runtime = options(config, "value_settings");
  runtime.node_options.append_parameter_override("drone_id", 31);
  runtime.node_options.append_parameter_override("chain/segments", 4);
  auto backend = pm::makeRosPlanningBackend(runtime); auto *ros = backend.get();
  check(throws<std::logic_error>([&] { ros->plan({}); }), "uninitialized ROS backend refuses planning");
  pm::MissionPlanner planner(settings, {}, std::move(backend));
  const auto node = ros->node();
  check(node->get_parameter("drone_id").as_int() == 41 && ros->configuredSegments() == 5,
        "explicit typed settings override NodeOptions and YAML before original declarations");
  check(!node->has_parameter("test/integer"), "unconsumed overrides are not eagerly declared");
  bool types = true;
  for (const auto &entry : settings.parameters) {
    const auto expected = std::visit([&](const auto &value) { return rclcpp::Parameter(entry.first, value); }, entry.second);
    if (!node->has_parameter(entry.first)) node->declare_parameter(entry.first, expected.get_type());
    types &= node->get_parameter(entry.first).get_parameter_value() == expected.get_parameter_value();
  }
  check(types, "every supported scalar/array type reaches ROS without coercion");
  check(throws<std::logic_error>([&] { ros->initialize({}, {}); }), "ROS backend rejects repeated initialization");
  settings.parameters["drone_id"] = 41.0;
  check(throws<std::exception>([&] {
    pm::MissionPlanner invalid(settings, {}, pm::makeRosPlanningBackend(options(config, "invalid_type")));
  }), "strict existing int declaration rejects double instead of silently coercing");
  auto eager = options(config, "invalid_eager");
  eager.node_options.automatically_declare_parameters_from_overrides(true);
  check(throws<std::invalid_argument>([&] { pm::MissionPlanner invalid({}, {}, pm::makeRosPlanningBackend(eager)); }),
        "eager automatic declarations are refused before legacy duplicate declarations");
}
void boundNodeAndEnvironment(const char *config)
{
  auto runtime = options(config, "bound_settings");
  runtime.node_options.append_parameter_override("drone_id", 12);
  runtime.node_options.append_parameter_override("risk_zones", std::vector<double>{-2.7, 9.125, 3.0, 0.1, 0.8});
  auto node = std::make_shared<rclcpp::Node>(runtime.node_name, runtime.node_options);
  node->declare_parameter("existing_value", 19.0);
  auto backend = pm::makeRosPlanningBackend(node); auto *ros = backend.get();
  pm::MissionPlanner planner({}, {}, std::move(backend));
  auto manager = ros->executionManager();
  check(node->get_parameter("existing_value").as_double() == 19.0 &&
            node->get_parameter("drone_id").as_int() == 12 && manager->riskZones().size() == 1,
        "empty bound-node settings/environment preserve declared values, pending overrides and zones");
  node->set_parameter(rclcpp::Parameter("existing_value", 23.0)); planner.plan({});
  check(node->get_parameter("existing_value").as_double() == 23.0,
        "subsequent plans do not reapply startup settings over live node values");
  auto before_node = std::make_shared<rclcpp::Node>("bound_rejection");
  pm::PlanSettings conflict; conflict.parameters["drone_id"] = int64_t{17};
  check(throws<std::invalid_argument>([&] {
    pm::MissionPlanner invalid(conflict, {}, pm::makeRosPlanningBackend(before_node));
  }) && !before_node->has_parameter("drone_id") && !before_node->has_parameter("manager/min_goal_agl"),
        "bound-node scalar settings rejection occurs before legacy parameter declarations");
  pm::PlanningEnvironment invalid_env; invalid_env.terrain = terrain(); invalid_env.terrain->rows = 0;
  auto terrain_node = std::make_shared<rclcpp::Node>("initial_terrain_rejection");
  check(throws<std::invalid_argument>([&] {
    pm::MissionPlanner invalid({}, invalid_env, pm::makeRosPlanningBackend(terrain_node));
  }) && !terrain_node->has_parameter("drone_id"), "invalid initial terrain is refused before manager construction");
  std::string error;
  auto grid = terrain(); grid.elevation.assign(7, 2.0f);
  check(planner.setTerrain(grid, &error) && error.empty() && manager->hasTerrainData(), "owned terrain reaches real manager ingest");
  double original = 0; manager->terrainElevation(grid.center_x, grid.center_y, &original);
  const auto generation = manager->environmentRevision();
  auto invalid = grid; invalid.elevation.clear();
  double retained = 0;
  check(!planner.setTerrain(invalid, &error) && !error.empty() &&
            manager->terrainElevation(grid.center_x, grid.center_y, &retained) && retained == original &&
            manager->environmentRevision() == generation,
        "rejected value terrain preserves accepted data and environment revision");
  grid.elevation.assign(7, 4.0f);
  check(planner.setTerrain(grid, &error) && error.empty() &&
            manager->terrainElevation(grid.center_x, grid.center_y, &retained) && retained == 4.0 &&
            manager->environmentRevision() > generation,
        "same-size value terrain refresh uses the existing generation/cache invalidation");
  planner.clearTerrain(); check(!manager->hasTerrainData(), "explicit value clear removes terrain through existing world-none path");

  pm::PlanningEnvironment environment; environment.terrain = grid;
  environment.risk_zones = std::vector<pm::RiskZone>{{Eigen::Vector3d(grid.center_x, grid.center_y, 3.0), 0.1, 0.8}};
  environment.dynamic_obstacles = std::vector<pm::DynamicObstacle>{
      pm::SphereObstacle{Eigen::Vector3d(grid.center_x, grid.center_y, 1.0), 0.1, "sphere"},
      pm::BoxObstacle{Eigen::Vector3d(grid.center_x, grid.center_y, 1.0), Eigen::Vector3d(0.1, 0.2, 0.3), "box"}};
  auto patch_backend = pm::makeRosPlanningBackend(options(config, "initial_patch")); auto *patch_ros = patch_backend.get();
  pm::MissionPlanner patched({}, environment, std::move(patch_backend));
  check(patch_ros->executionManager()->hasTerrainData() &&
            patch_ros->executionManager()->riskZones().size() == 1 &&
            patch_ros->executionManager()->riskZones()[0].center.z() == 7.0 &&
            patch_ros->executionManager()->numDynamicObstacles() == 2,
        "initial terrain then AGL zones and obstacles initialize successfully in order");
  pm::PlanningEnvironment pending_environment;
  pending_environment.dynamic_obstacles = environment.dynamic_obstacles;
  auto pending_backend = pm::makeRosPlanningBackend(options(config, "pending_patch"));
  auto *pending_ros = pending_backend.get();
  pm::MissionPlanner pending({}, pending_environment, std::move(pending_backend));
  check(pending_ros->executionManager()->numDynamicObstacles() == 0 &&
            pending.addDynamicSphere(Eigen::Vector3d(grid.center_x, grid.center_y, 1.0), 0.1) == -2,
        "no-map pending obstacles are accepted as -2 rather than failed initialization");
  patched.clearDynamicObstacles(); patched.setRiskZones({});
  check(patch_ros->executionManager()->riskZones().empty(), "explicit empty zone update clears active zones");
  pm::PlanningEnvironment empty_fields; empty_fields.risk_zones = std::vector<pm::RiskZone>{};
  empty_fields.dynamic_obstacles = std::vector<pm::DynamicObstacle>{};
  auto explicit_runtime = options(config, "explicit_empty_patch");
  explicit_runtime.node_options.append_parameter_override("risk_zones", std::vector<double>{-2.7, 9.125, 3.0, 0.1, 0.8});
  auto explicit_backend = pm::makeRosPlanningBackend(explicit_runtime); auto *explicit_ros = explicit_backend.get();
  pm::MissionPlanner explicit_empty({}, empty_fields, std::move(explicit_backend));
  check(explicit_ros->executionManager()->riskZones().empty(), "present-empty initial zones override nonempty YAML/NodeOptions values");
}
void contextOwnership(const char *config)
{
  auto uninitialized = std::make_shared<rclcpp::Context>();
  auto runtime = options(config, "uninitialized_context"); runtime.node_options.context(uninitialized);
  check(throws<std::invalid_argument>([&] { pm::MissionPlanner planner({}, {}, pm::makeRosPlanningBackend(runtime)); }) &&
            !uninitialized->is_valid() && rclcpp::ok(), "backend refuses uninitialized context without implicit global init/shutdown");
  auto context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
  runtime = options(config, "owned_context"); runtime.node_options.context(context);
  {
    pm::MissionPlanner planner({}, {}, pm::makeRosPlanningBackend(runtime));
    check(planner.plan({}).status.reason == pm::PlanReason::INITIAL_STATE_UNSPECIFIED,
          "explicit non-global context runs the existing refusal pipeline");
  }
  check(context->is_valid() && rclcpp::ok(), "backend destruction leaves caller-owned ROS contexts initialized");
  context->shutdown("test completed");
}
class UnusedRouteBackend final : public pm::RouteSearchBackend {
 public:
  pm::RouteSearchResult search(const pm::RouteSearchProblem &) override { return {}; }
};
void injectedSettings(const char *config)
{
  auto model = std::make_shared<mmp_vehicle_dynamics::FixedWingModel>();
  auto route = std::make_shared<UnusedRouteBackend>();
  std::weak_ptr<const mmp_vehicle_dynamics::VehicleModel> model_lifetime = model;
  std::weak_ptr<pm::RouteSearchBackend> route_lifetime = route;
  const auto *identity = model.get();
  {
    pm::PlanSettings settings; settings.vehicle_model = model; settings.route_search_backend = route;
    auto runtime = options(config, "initial_injection");
    auto node = std::make_shared<rclcpp::Node>(runtime.node_name, runtime.node_options);
    auto backend = pm::makeRosPlanningBackend(node); auto *ros = backend.get();
    pm::MissionPlanner planner(std::move(settings), {}, std::move(backend));
    model.reset(); route.reset();
    auto manager = ros->executionManager(); manager->initOptimizer();
    auto worker = manager->makeConfiguredOptimizer();
    check(!model_lifetime.expired() && !route_lifetime.expired() &&
              manager->vehicleModel().get() == identity && worker->vehicleModel().get() == identity,
          "bound-node initial injection handles survive caller release and share one model with workers");
  }
  check(model_lifetime.expired() && route_lifetime.expired(), "planner/backend destruction releases initial injection handles");
  mmp_vehicle_dynamics::Parameters parameters; parameters.mass_kg = -1.0;
  pm::PlanSettings invalid; invalid.vehicle_model = std::make_shared<mmp_vehicle_dynamics::FixedWingModel>(parameters);
  auto node = std::make_shared<rclcpp::Node>("initial_model_rejection");
  check(throws<std::invalid_argument>([&] {
    pm::MissionPlanner planner(invalid, {}, pm::makeRosPlanningBackend(node));
  }) && !node->has_parameter("drone_id"), "invalid initial model is refused before bound-node declarations");
}
}  // namespace
int main(int argc, char **argv)
{
  if (argc < 2) return 2;
  rclcpp::init(argc, argv);
  terrainConversion(); valueSettings(argv[1]); boundNodeAndEnvironment(argv[1]);
  contextOwnership(argv[1]); injectedSettings(argv[1]);
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
