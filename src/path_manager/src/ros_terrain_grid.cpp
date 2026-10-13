#include "path_manager/ros_terrain_grid.h"
#include "terrain_grid_validation.h"

#include <algorithm>
#include <utility>

namespace path_manager {
namespace {
TerrainGrid geometryFromMessage(const grid_map_msgs::msg::GridMap &message,
                                size_t index)
{
  TerrainGrid terrain;
  const auto &layer = message.data[index];
  terrain.cols = layer.layout.dim[0].size;
  terrain.rows = layer.layout.dim[1].size;
  terrain.resolution = message.info.resolution;
  terrain.center_x = message.info.pose.position.x;
  terrain.center_y = message.info.pose.position.y;
  terrain.length_x = message.info.length_x;
  terrain.length_y = message.info.length_y;
  return terrain;
}
}

TerrainGridInspection inspectTerrainGrid(
    const grid_map_msgs::msg::GridMap::SharedPtr &message)
{
  TerrainGridInspection result;
  if (!message) return result;
  if (message->layers.empty() && message->data.empty()) {
    result.update = TerrainGridUpdate::CLEAR;
    return result;
  }
  const auto reject = [&](const char *detail) {
    result.update = TerrainGridUpdate::REJECTED;
    result.detail = detail;
    return result;
  };
  const auto found = std::find(message->layers.begin(), message->layers.end(), "elevation");
  if (found == message->layers.end()) return reject("elevation layer is missing");
  const size_t index = static_cast<size_t>(found - message->layers.begin());
  if (index >= message->data.size()) return reject("elevation payload is missing");
  const auto &layer = message->data[index];
  if (layer.layout.dim.size() < 2) return reject("elevation layout has fewer than two dimensions");
  const auto terrain = geometryFromMessage(*message, index);
  if (!detail::validateTerrainGridMetadata(terrain, layer.data.size(), &result.detail)) {
    result.update = TerrainGridUpdate::REJECTED;
    return result;
  }
  result.update = TerrainGridUpdate::REPLACE;
  result.elevation_layer = index;
  return result;
}

TerrainGridDecode decodeTerrainGrid(
    const grid_map_msgs::msg::GridMap::SharedPtr &message)
{
  const auto inspection = inspectTerrainGrid(message);
  TerrainGridDecode result;
  result.update = inspection.update;
  result.detail = inspection.detail;
  if (inspection.update == TerrainGridUpdate::REPLACE) {
    auto terrain = geometryFromMessage(*message, inspection.elevation_layer);
    terrain.elevation = message->data[inspection.elevation_layer].data;
    result.terrain = std::move(terrain);
  }
  return result;
}

grid_map_msgs::msg::GridMap::SharedPtr encodeTerrainGrid(
    TerrainGrid terrain, std::string *error)
{
  if (!validateTerrainGrid(terrain, error)) return nullptr;
  auto message = std::make_shared<grid_map_msgs::msg::GridMap>();
  message->info.resolution = terrain.resolution;
  message->info.length_x = terrain.length_x;
  message->info.length_y = terrain.length_y;
  message->info.pose.position.x = terrain.center_x;
  message->info.pose.position.y = terrain.center_y;
  message->info.pose.orientation.w = 1.0;
  message->layers = {"elevation"};
  message->data.resize(1);
  auto &layer = message->data.front();
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index";
  layer.layout.dim[0].size = terrain.cols;
  layer.layout.dim[0].stride = terrain.rows * terrain.cols;
  layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = terrain.rows;
  layer.layout.dim[1].stride = terrain.rows;
  layer.data = std::move(terrain.elevation);
  return message;
}
}  // namespace path_manager
