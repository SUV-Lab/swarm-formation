#ifndef PATH_MANAGER_RISK_ZONE_H
#define PATH_MANAGER_RISK_ZONE_H

#include <Eigen/Core>

namespace path_manager
{
  // Terrain-occluded risk zone. The authored reach is the horizontal
  // reach; PathManager derives a vertical reach from risk_vertical_ratio_ and
  // supplies the resulting ellipsoid consistently to the front/back ends.
  struct RiskZone {
    Eigen::Vector3d center; // frame units (1 unit = 100 m)
    double reach;   // horizontal reach in FRAME UNITS (typ. 90 = 9 km)
    double peak;    // dimensionless in (0, 1]; values >1 are field-capped
  };
}  // namespace path_manager

#endif  // PATH_MANAGER_RISK_ZONE_H
