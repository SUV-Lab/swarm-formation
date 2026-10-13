#pragma once

#include "path_manager/planning_environment.h"
#include <cstddef>

namespace path_manager::detail {
// Internal adapter helper: validate geometry and the actual payload size
// before copying a large ROS raster or changing any live environment state.
// The geometry object's elevation vector is deliberately not consulted.
bool validateTerrainGridMetadata(const TerrainGrid &geometry,
                                 size_t elevation_cells, std::string *error);
}
