#include <path_manager/path_manager.h>
#include <risk_model/model.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <optional>
#include <thread>

namespace {
using Models = mmp_mission_msgs::msg::RiskModelArray;
using Source = mmp_mission_msgs::msg::RiskModelSpec;
using Grid = grid_map_msgs::msg::GridMap;
int checks = 0, failures = 0;
void check(bool ok, const char *name) {
  ++checks;
  if (!ok) ++failures;
  std::cout << (ok ? "[OK] " : "[FAIL] ") << name << std::endl;
}
bool waitFor(rclcpp::executors::SingleThreadedExecutor &executor,
             const std::function<bool()> &ready) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  do {
    executor.spin_some();
    if (ready()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while (std::chrono::steady_clock::now() < end);
  return ready();
}
const std::vector<float> &layer(const Grid &grid, const std::string &name) {
  const auto it = std::find(grid.layers.begin(), grid.layers.end(), name);
  if (it == grid.layers.end()) throw std::runtime_error("missing layer " + name);
  return grid.data.at(static_cast<std::size_t>(it - grid.layers.begin())).data;
}
bool allNan(const Grid &grid, const std::string &name) {
  const auto &values = layer(grid, name);
  return !values.empty() && std::all_of(values.begin(), values.end(),
                                      [](float value) { return std::isnan(value); });
}
Grid::SharedPtr terrain(float height) {
  auto grid = std::make_shared<Grid>();
  constexpr int n = 40;
  grid->header.frame_id = "map";
  grid->info.resolution = 1;
  grid->info.length_x = n;
  grid->info.length_y = n;
  grid->info.pose.orientation.w = 1;
  grid->layers = {"elevation"};
  grid->data.resize(1);
  auto &a = grid->data.front();
  a.layout.dim.resize(2);
  a.layout.dim[0].label = "column_index";
  a.layout.dim[0].size = n;
  a.layout.dim[0].stride = n * n;
  a.layout.dim[1].label = "row_index";
  a.layout.dim[1].size = n;
  a.layout.dim[1].stride = n;
  a.data.assign(n * n, height);
  return grid;
}
Models models() {
  Models m;
  m.schema_version = 2;
  m.replace = true;
  m.unit_xy_m = 100; m.unit_z_m = 50;
  m.rcs_a = .3172; m.rcs_b = .1784; m.rcs_c = 1.003;
  m.sigma_ref_m2 = 1; m.fallback_v0_mps = 1; m.fallback_v1_mps = 5;
  m.height_width_m = 5; m.radius_width_m = 5;
  m.center_rho_width = .001; m.plateau_rho_width = .01;
  m.visibility_width_m = 5; m.clearance_m = 10; m.grounded = true;
  m.reference_length_configured = true; m.reference_length_m = 100;
  for (int i = 0; i != 2; ++i) {
    Source s;
    s.source_id = i == 0 ? "teaching_A" : "teaching_B";
    s.center.x = i == 0 ? -2 : 2;
    s.center.z = 0;
    s.height_mode = Source::MSL;
    s.c1 = 1; s.c2 = risk_model::calibratedC2(1500, 1);
    s.r_max_m = 1000; s.h_max_m = 1000; s.r_min_m = 0; s.h_min_m = 0;
    s.p0 = .9; s.rho_min = 0; s.exponent = 2;
    s.policy_reach_frame = .5;  // Much smaller than the physical damage extent.
    m.sources.push_back(s);
  }
  return m;
}
}  // namespace

int main(int argc, char **argv) {
  const bool demo = argc > 1 && std::string(argv[1]) == "--demo";
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.append_parameter_override("manager/risk_model_mode", std::string("default"));
  options.append_parameter_override("manager/risk_mask_viz_mode", std::string("heatmap"));
  options.append_parameter_override("manager/risk_probability_heatmap_max_dim", demo ? 96 : 24);
  options.append_parameter_override("optimization/dynamics_unit_xy_m", 100.0);
  options.append_parameter_override("optimization/dynamics_unit_z_m", 50.0);
  auto node = std::make_shared<rclcpp::Node>("probability_heatmap_ros_test", options);
  node->declare_parameter("drone_id", 0);
  path_manager::PathManager manager(node, true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  std::optional<Grid> map, visibility;
  std::string info;
  int received = 0;
  auto sub = node->create_subscription<Grid>("/viz/risk_probability_heatmap", qos,
      [&](Grid::ConstSharedPtr m) { map = *m; ++received; });
  auto old = node->create_subscription<Grid>("/viz/risk_heatmap", qos,
      [&](Grid::ConstSharedPtr m) { visibility = *m; });
  auto metadata = node->create_subscription<std_msgs::msg::String>("/viz/risk_probability_heatmap_info", qos,
      [&](std_msgs::msg::String::ConstSharedPtr m) { info = m->data; });
  check(waitFor(executor, [&] { return map && visibility && !info.empty(); }),
        "late subscribers receive initial map and metadata");
  check(map && allNan(*map, "probability"), "no accepted environment clears rather than showing zero");
  std::string reason;
  manager.setTerrainData(terrain(0));
  const int before = received;
  check(manager.setRiskModelsRuntime(models(), &reason), "synthetic teaching environment accepted");
  check(waitFor(executor, [&] { return received > before && map && map->info.length_x > 2 &&
      info.find("sigma") != std::string::npos; }), "new environment publishes probability slice and metadata");
  if (!map || map->info.length_x <= 2) { rclcpp::shutdown(); return 1; }
  check(visibility && allNan(*visibility, "elevation"), "auto mode clears visibility layer to avoid mixed legends");
  check(map->info.length_x >= 24 && map->info.length_y >= 20,
        "physical damage extents are not clipped to policy radius");
  const auto snapshot = manager.riskProbabilitySnapshot();
  const auto &values = layer(*map, "probability");
  const auto &heights = layer(*map, "evaluation_height_m");
  const auto &validity = layer(*map, "validity");
  const int rows = static_cast<int>(map->data.front().layout.dim.at(1).size);
  const int cols = static_cast<int>(map->data.front().layout.dim.at(0).size);
  bool equal = true;
  std::size_t finite_count = 0;
  for (int col = 0; col < cols; ++col) for (int row = 0; row < rows; ++row) {
    const auto index = static_cast<std::size_t>(col) * rows + row;
    if (!std::isfinite(values.at(index))) continue;
    ++finite_count;
    const double x = map->info.pose.position.x + map->info.length_x / 2 - (row + .5) * map->info.resolution;
    const double y = map->info.pose.position.y + map->info.length_y / 2 - (col + .5) * map->info.resolution;
    const auto exact = risk_model::evaluateCombined(snapshot->sources,
        {x * snapshot->scale.xy_m, y * snapshot->scale.xy_m, heights.at(index)}, {},
        snapshot->settings, risk_model::FormulaMode::Exact, risk_model::RcsMode::Reference, snapshot->terrain.get());
    equal = equal && std::abs(values.at(index) - exact.risk.value) < 1e-6 &&
        heights.at(index) == 100 && validity.at(index) == 1;
  }
  check(equal && finite_count > 20, "published cells equal shared Exact Reference OR at SI AGL height");
  if (demo) {
    std::cout << "DEMO_READY: fixed synthetic environment; no trajectory planning executed" << std::endl;
    executor.spin();
    rclcpp::shutdown();
    return failures ? 1 : 0;
  }
  std::optional<Grid> late_map;
  auto late = node->create_subscription<Grid>("/viz/risk_probability_heatmap", qos,
      [&](Grid::ConstSharedPtr m) { late_map = *m; });
  check(waitFor(executor, [&] { return late_map.has_value(); }) &&
        layer(*late_map, "probability") == values, "late-joining RViz receives full current probability grid");
  auto invalid = models(); invalid.sources.front().p0 = -1;
  check(!manager.setRiskModelsRuntime(invalid, &reason) && manager.riskProbabilitySnapshot() == snapshot,
        "rejected model preserves the accepted visualization environment");
  auto empty = models(); empty.sources.clear();
  const int clear_before = received;
  check(manager.setRiskModelsRuntime(empty, &reason), "explicit empty environment accepted");
  check(waitFor(executor, [&] { return received > clear_before && map && allNan(*map, "probability"); }),
        "Clear replaces latched heatmap with empty geometry");
  auto agl = models(); for (auto &s : agl.sources) s.height_mode = Source::AGL;
  check(manager.setRiskModelsRuntime(agl, &reason), "AGL environment accepted with terrain");
  waitFor(executor, [&] { return map && map->info.length_x > 2; });
  const int terrain_before = received;
  manager.setTerrainData(std::make_shared<Grid>());
  check(!manager.riskProbabilitySnapshot(), "terrain removal invalidates accepted AGL world");
  check(waitFor(executor, [&] { return received > terrain_before && map && allNan(*map, "probability"); }),
        "invalidated terrain clears stale probability display");
  std::cout << checks << " checks, " << failures << " failures" << std::endl;
  rclcpp::shutdown();
  return failures ? 1 : 0;
}
