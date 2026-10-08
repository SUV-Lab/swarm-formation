#include "path_manager/planning_environment.h"

#include <cmath>
#include <limits>

namespace path_manager {
bool validateTerrainGrid(const TerrainGrid &terrain, std::string *error)
{
  const auto reject = [&](const char *detail) {
    if (error) *error = detail;
    return false;
  };
  if (!std::isfinite(terrain.resolution) || !(terrain.resolution > 0.0))
    return reject("terrain resolution must be positive and finite");
  if (!std::isfinite(terrain.center_x) || !std::isfinite(terrain.center_y) ||
      !std::isfinite(terrain.length_x) || !std::isfinite(terrain.length_y) ||
      !(terrain.length_x > 0.0) || !(terrain.length_y > 0.0))
    return reject("terrain center must be finite and lengths positive and finite");
  if (!std::isfinite(terrain.center_x - terrain.length_x / 2.0) ||
      !std::isfinite(terrain.center_y - terrain.length_y / 2.0) ||
      !std::isfinite(terrain.center_x + terrain.length_x / 2.0) ||
      !std::isfinite(terrain.center_y + terrain.length_y / 2.0))
    return reject("terrain bounds are not representable");
  // TerrainData indexes cells with int; validate before narrowing dimensions
  // or forming the GridMap's uint32 stride. No multiplication may overflow.
  const uint64_t cells = static_cast<uint64_t>(terrain.rows) * terrain.cols;
  if (terrain.rows == 0 || terrain.cols == 0 ||
      cells > static_cast<uint64_t>(std::numeric_limits<int>::max()))
    return reject("terrain dimensions exceed the current sampler's index range");
  if (cells > terrain.elevation.size())
    return reject("terrain elevation payload is shorter than rows * cols");
  if (error) error->clear();
  return true;
}
}  // namespace path_manager
