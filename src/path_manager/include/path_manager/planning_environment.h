#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>
#include <Eigen/Core>

#include "path_manager/risk_zone.h"

namespace path_manager {

// Original planner frame: geometry and elevation are in planner units (u).
// ROW spans world X and COL spans world Y, both in decreasing world direction:
// x = center_x + length_x/2 - (row + 0.5)*resolution,
// y = center_y + length_y/2 - (col + 0.5)*resolution.
// elevation[col * rows + row] is the column-major float cell-center sample.
// Keep explicit lengths/center and NaN payloads. No transpose, ring-buffer
// adjustment, resampling or unit conversion is performed at this boundary.
// TerrainData remains the single authority for the existing mirrored sampler.
struct TerrainGrid {
  uint32_t rows{0}, cols{0};
  double resolution{0.0};
  double center_x{0.0}, center_y{0.0};
  double length_x{0.0}, length_y{0.0};
  std::vector<float> elevation;
};

// Representation checks only. Nonfinite cell heights are retained. Extra
// payload cells are permitted and retained, matching existing GridMap ingest.
// Does not check rows*resolution == length_x or cols*resolution == length_y;
// callers must supply mutually consistent grid geometry.
// Returns a synchronous error and clears it on success.
bool validateTerrainGrid(const TerrainGrid &terrain, std::string *error = nullptr);

struct SphereObstacle {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
  std::string model;
};
struct BoxObstacle {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  std::string model;
};
using DynamicObstacle = std::variant<SphereObstacle, BoxObstacle>;

// Initial environment patch. An absent field makes no setter call. Present
// empty zones/obstacles deliberately clear using the existing API semantics.
// Apply terrain, zones, then obstacle clear/add in vector order. Terrain removal
// is explicit clearTerrain(). This is owned input, not a frozen live snapshot.
struct PlanningEnvironment {
  std::optional<TerrainGrid> terrain;
  std::optional<std::vector<RiskZone>> risk_zones;
  std::optional<std::vector<DynamicObstacle>> dynamic_obstacles;
};
}  // namespace path_manager
