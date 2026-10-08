#ifndef PATH_MANAGER_MISSION_GEOMETRY_H
#define PATH_MANAGER_MISSION_GEOMETRY_H

#include <cstddef>
#include <functional>
#include <vector>

#include "path_manager/risk_zone.h"

namespace path_manager {

// An empty callback means no terrain is loaded. A present callback describes
// the current terrain, including invalid/off-DEM samples. It is borrowed only
// during each synchronous call; the provider must remain stable meanwhile.
// Neither result retains a callback or claims to snapshot the environment.
using ElevationQuery = std::function<float(double, double)>;

struct WaypointAltitudeChange {
  size_t index{0};
  double original_z{0.0};
  double terrain_base{0.0};
  double agl{0.0};
  double absolute_z{0.0};
};

struct ResolvedMissionWaypoints {
  std::vector<Eigen::Vector3d> waypoints;
  // Only changes larger than the existing 1e-9 diagnostic threshold appear.
  std::vector<WaypointAltitudeChange> altitude_changes;
};

// Command waypoint z is AGL when terrain exists. A final junction goal is
// already absolute; interior waypoints retain AGL semantics. With no terrain,
// all waypoint values are retained verbatim.
ResolvedMissionWaypoints resolveMissionWaypoints(
    std::vector<Eigen::Vector3d> waypoints, bool junction_goal,
    double min_goal_agl, const ElevationQuery &elevation = {});

struct MissionBoundsConfig {
  double ceiling_headroom{20.0};
  double risk_vertical_ratio{0.35};
};

struct MissionBoundsResult {
  Eigen::Vector3d lower{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upper{Eigen::Vector3d::Zero()};
  std::vector<char> zone_in;
  double bound_margin_xy{10.0};
  double max_included_reach{0.0};
  double max_terrain_z{0.0};
  double max_zone_top{0.0};
  size_t included_zone_count{0};
};

// Input waypoints are already resolved to absolute planner coordinates.
// Preserve the current bounds algorithm: finish the zone fixpoint before
// expanding waypoint margins, without another fixpoint pass. Upper z uses
// max(sampled terrain, start.z) + headroom; waypoint z affects only lower z.
// Zone tops are diagnostic only. No SDF, search-policy state or ROS work is
// performed here; callers retain environment preparation and publication.
MissionBoundsResult computeMissionBounds(
    const Eigen::Vector3d &start,
    const std::vector<Eigen::Vector3d> &waypoints,
    const std::vector<RiskZone> &zones, const MissionBoundsConfig &config,
    const ElevationQuery &elevation = {});

}  // namespace path_manager

#endif  // PATH_MANAGER_MISSION_GEOMETRY_H
