#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "path_manager/path_manager.h"

namespace {

int failures = 0;

void expect(bool condition, const std::string &label)
{
  std::cout << (condition ? "[OK]   " : "[FAIL] ") << label << '\n';
  if (!condition) ++failures;
}

grid_map_msgs::msg::GridMap::SharedPtr makeRidgeMap()
{
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr int kCols = 20;
  constexpr int kRows = 20;
  msg->info.resolution = 1.0;
  msg->info.length_x = 20.0;
  msg->info.length_y = 20.0;
  msg->info.pose.position.x = 0.0;
  msg->info.pose.position.y = 0.0;
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
  layer.data.assign(kCols * kRows, 0.0f);

  // terrainToWorld(row=6) -> x=3.5. A north-south ridge at x=3.5
  // therefore blocks eastbound LOS from the source at x=0.5.
  for (int col = 0; col < kCols; ++col) {
    layer.data[col * kRows + 6] = 4.0f;
  }
  return msg;
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // This fixture deliberately exercises the opt-in AGL authoring mode; the
  // production default is absolute z for backward-compatible scenarios.
  options.append_parameter_override("manager/risk_zone_agl", true);
  options.append_parameter_override("manager/risk_mask_viz_mode",
                                    std::string("volume"));
  auto node = std::make_shared<rclcpp::Node>("terrain_risk_mask_test",
                                             options);
  // Launch defaults to drone_id=1; risk-field publication and masking must
  // not depend on a drone_0 instance being present.
  node->declare_parameter("drone_id", 1);

  visualization_msgs::msg::Marker last_floor;
  bool got_floor = false;
  bool got_wire = false;
  auto marker_sub = node->create_subscription<visualization_msgs::msg::Marker>(
      "/viz/risk_field", rclcpp::QoS(128).reliable().transient_local(),
      [&](visualization_msgs::msg::Marker::SharedPtr marker) {
        if (marker->ns == "effective_risk_floor" && marker->id == 0 &&
            marker->type == visualization_msgs::msg::Marker::TRIANGLE_LIST) {
          last_floor = *marker;
          got_floor = true;
        }
        if (marker->ns == "effective_risk_volume" && marker->id == 0 &&
            marker->type == visualization_msgs::msg::Marker::LINE_LIST)
          got_wire = true;
      });

  path_manager::PathManager manager(node);
  path_manager::RiskZone zone;
  zone.center = Eigen::Vector3d(0.5, 0.5, 2.0);  // flat ground: AGL == abs
  zone.reach = 10.0;
  zone.peak = 0.8;
  // Second zone ON the ridge crest with a 0.5-unit mast. With AGL grounding
  // its emitter is 4.0 + 0.5 = 4.5, which sees over its own crest: the
  // east-side low query below is visible. Left ungrounded (z=0.5, below the
  // crest) the same query sits deep in shadow (ceiling z≈3.5) — the
  // assertion fails, so it pins the grounding, not just the viewshed.
  path_manager::RiskZone ridge_zone;
  ridge_zone.center = Eigen::Vector3d(3.5, 0.5, 0.5);
  ridge_zone.reach = 10.0;
  ridge_zone.peak = 0.8;
  manager.setRiskZonesRuntime({zone, ridge_zone});

  // Without a DEM the callback must preserve the legacy ideal field.
  expect(manager.getRiskVisibility(0, Eigen::Vector3d(6.5, 0.5, 2.0)) > 0.999,
         "no DEM -> full visibility fallback");
  const double risk_horizontal_half = manager.getEffectiveRisk(
      0, Eigen::Vector3d(5.5, 0.5, 2.0));
  const double risk_vertical_half = manager.getEffectiveRisk(
      0, Eigen::Vector3d(0.5, 0.5, 3.75));  // Rv=0.35*10, dz=Rv/2
  expect(std::abs(risk_horizontal_half - 0.2) < 1e-6 &&
             std::abs(risk_vertical_half - risk_horizontal_half) < 1e-6,
         "horizontal/vertical q=0.5 points share the ellipsoid moat value");
  expect(manager.getEffectiveRisk(0, Eigen::Vector3d(0.5, 0.5, 5.6)) == 0.0,
         "risk is zero above the vertical ellipsoid support");

  manager.setTerrainData(makeRidgeMap());
  const Eigen::Vector3d behind_low(6.5, 0.5, 2.0);
  const double ceiling = manager.getRiskShadowCeiling(0, behind_low);
  expect(std::isfinite(ceiling) && ceiling > 4.5,
         "near ridge raises the far-side shadow ceiling");
  expect(manager.getRiskVisibility(0, behind_low) < 0.01,
         "aircraft below horizon is terrain-shadowed");
  expect(manager.getRiskVisibility(0, Eigen::Vector3d(6.5, 0.5, 8.0)) > 0.99,
         "aircraft above horizon regains risk visibility");
  expect(manager.getRiskVisibility(0, Eigen::Vector3d(-6.5, 0.5, 2.0)) > 0.99,
         "unblocked azimuth remains visible");
  expect(manager.getRiskVisibility(0, zone.center) > 0.999,
         "risk source is visible at zero range");
  expect(manager.getRiskVisibility(1, Eigen::Vector3d(6.5, 0.5, 1.0)) > 0.9,
         "AGL-grounded ridge-top emitter sees past its own crest");

  // Verify that the same field reaches RViz for the launch-default nonzero
  // drone id. The floor mesh should rise to the horizon behind the ridge;
  // unlike a 2-D clipped carpet, it preserves the fact that risk reappears
  // above the terrain shadow.
  for (int i = 0; i < 30; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  expect(got_floor && !last_floor.points.empty(),
         "effective-risk TRIANGLE_LIST floor is published for RViz");
  expect(got_wire, "clipped ellipsoid wire volume is published for RViz");
  bool has_west_cell = false;
  bool has_far_east_cell = false;
  bool far_east_floor_is_raised = false;
  for (const auto &p : last_floor.points) {
    if (p.x < -4.5) has_west_cell = true;
    if (p.x > 4.0) {
      has_far_east_cell = true;
      if (p.z > 4.5) far_east_floor_is_raised = true;
    }
  }
  expect(has_west_cell && has_far_east_cell && far_east_floor_is_raised,
         "RViz volume raises its far-side floor to the terrain horizon");

  // === Draped heatmap channel (mode "heatmap", the launch default) ===
  // A second manager runs the heatmap mode; the fixture above stays on
  // "volume" so both marker paths remain pinned.
  grid_map_msgs::msg::GridMap heatmap;
  bool got_heatmap = false;
  auto heatmap_sub = node->create_subscription<grid_map_msgs::msg::GridMap>(
      "/viz/risk_heatmap", rclcpp::QoS(16).reliable().transient_local(),
      [&](grid_map_msgs::msg::GridMap::SharedPtr m) {
        // Non-heatmap managers latch a 1x1 NaN stub; keep real maps only.
        // Same-publisher reliable delivery is ordered, so the last kept map
        // is the post-DEM one.
        if (!m->data.empty() && m->data[0].data.size() > 1) {
          heatmap = *m;
          got_heatmap = true;
        }
      });

  rclcpp::NodeOptions hm_options;
  hm_options.append_parameter_override("manager/risk_zone_agl", true);
  hm_options.append_parameter_override("manager/risk_mask_viz_mode",
                                       std::string("heatmap"));
  auto hm_node = std::make_shared<rclcpp::Node>("terrain_risk_heatmap_test",
                                                hm_options);
  hm_node->declare_parameter("drone_id", 1);
  path_manager::PathManager hm_manager(hm_node);
  hm_manager.setRiskZonesRuntime({zone, ridge_zone});
  hm_manager.setTerrainData(makeRidgeMap());
  for (int i = 0; i < 100; ++i) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(hm_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (got_heatmap && i > 20) break;  // extra spins drain queued updates
  }
  expect(got_heatmap, "draped risk heatmap is published");
  if (got_heatmap) {
    const auto &info = heatmap.info;
    const int rows = static_cast<int>(heatmap.data[0].layout.dim[1].size);
    const int cols = static_cast<int>(heatmap.data[0].layout.dim[0].size);
    const double res = info.resolution;
    const double ox = info.pose.position.x - 0.5 * info.length_x;
    const double oy = info.pose.position.y - 0.5 * info.length_y;
    // Inverse of the [TERRAIN-FRAME] cell mapping used by the publisher.
    auto cellAt = [&](double wx, double wy, size_t layer) -> float {
      const int row = static_cast<int>(
          std::lround((ox + info.length_x - wx) / res - 0.5));
      const int col = static_cast<int>(
          std::lround((oy + info.length_y - wy) / res - 0.5));
      if (row < 0 || row >= rows || col < 0 || col >= cols)
        return std::numeric_limits<float>::quiet_NaN();
      return heatmap.data[layer].data[static_cast<size_t>(col) * rows + row];
    };
    auto rgbAt = [&](double wx, double wy, int *r, int *g, int *b) {
      const float packed = cellAt(wx, wy, 1);
      uint32_t rgb = 0;
      std::memcpy(&rgb, &packed, sizeof(rgb));
      *r = (rgb >> 16) & 0xFF;
      *g = (rgb >> 8) & 0xFF;
      *b = rgb & 0xFF;
    };
    expect(std::isnan(cellAt(ox + info.length_x - 0.5 * res,
                             oy + info.length_y - 0.5 * res, 0)),
           "heatmap corner outside every footprint is transparent");
    expect(std::isfinite(cellAt(-6.5, 0.5, 0)),
           "west heatmap cell is painted");
    int r = 0, g = 0, b = 0;
    rgbAt(-6.5, 0.5, &r, &g, &b);
    expect(r > g && r > b, "west cell (detectable at ground level) is red");
    // East of the ridge only the crest-top emitter sees the column, and only
    // above its ellipsoid lower shell (~1.16 u AGL) -> mid-ramp (green-ish).
    rgbAt(6.5, 0.5, &r, &g, &b);
    expect(g > r, "east cell behind the ridge shows a raised (cooler) floor");
  }
  (void)heatmap_sub;
  (void)marker_sub;

  rclcpp::shutdown();
  std::cout << (failures == 0 ? "PASS" : "FAIL") << ": " << failures
            << " failed check(s)\n";
  return failures == 0 ? 0 : 1;
}
