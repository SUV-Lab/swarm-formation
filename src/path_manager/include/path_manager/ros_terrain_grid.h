#pragma once

#include <optional>
#include <string>
#include <grid_map_msgs/msg/grid_map.hpp>
#include "path_manager/planning_environment.h"

namespace path_manager {

enum class TerrainGridUpdate { NO_CHANGE, CLEAR, REPLACE, REJECTED };
struct TerrainGridDecode {
  TerrainGridUpdate update{TerrainGridUpdate::NO_CHANGE};
  std::optional<TerrainGrid> terrain;
  std::string detail;
};

// Compatibility conversion for value-input callers. A null pointer means no
// update; both empty layers/data mean clear. Invalid nonempty input is rejected.
// Copy the fields used by PathManager, retaining raw float order and extra
// payload; ignored GridMap offsets/strides/ring indices remain ignored.
// The raw ROS callback still calls PathManager directly and keeps its original
// validation order and side effects, including pre-rejection SDF pitch updates.
TerrainGridDecode decodeTerrainGrid(
    const grid_map_msgs::msg::GridMap::SharedPtr &message);

// Returns null on representation failure and sets error. On success clears
// error. Geometry and float cells are copied without sampling or normalization.
grid_map_msgs::msg::GridMap::SharedPtr encodeTerrainGrid(
    TerrainGrid terrain, std::string *error = nullptr);
}  // namespace path_manager
