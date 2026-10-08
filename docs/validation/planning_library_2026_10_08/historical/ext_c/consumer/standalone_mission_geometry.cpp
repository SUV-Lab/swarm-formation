#include "path_manager/mission_geometry.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace path_manager {

ResolvedMissionWaypoints resolveMissionWaypoints(
    std::vector<Eigen::Vector3d> waypoints, bool junction_goal,
    double min_goal_agl, const ElevationQuery &elevation) {
  ResolvedMissionWaypoints resolved;
  resolved.waypoints = std::move(waypoints);
  if (!elevation) return resolved;

  for (size_t wi = 0; wi < resolved.waypoints.size(); ++wi) {
    auto &wp = resolved.waypoints[wi];
    if (junction_goal && wi + 1 == resolved.waypoints.size()) continue;
    const double agl = std::max(wp.z(), min_goal_agl);
    const float elev = elevation(wp.x(), wp.y());
    const double base = (elev > -1e9f) ? static_cast<double>(elev) : 0.0;
    const double z_abs = base + agl;
    if (std::abs(z_abs - wp.z()) > 1e-9)
      resolved.altitude_changes.push_back({wi, wp.z(), base, agl, z_abs});
    wp.z() = z_abs;
  }
  return resolved;
}

MissionBoundsResult computeMissionBounds(
    const Eigen::Vector3d &start,
    const std::vector<Eigen::Vector3d> &waypoints,
    const std::vector<RiskZone> &zones, const MissionBoundsConfig &config,
    const ElevationQuery &elevation) {
  MissionBoundsResult bounds;
  std::vector<Eigen::Vector3d> all_points;
  all_points.push_back(start);
  for (const auto &wp : waypoints) all_points.push_back(wp);
  bounds.lower = start;
  bounds.upper = start;
  const double bound_margin_z_below = 5.0;
  const auto expand_points = [&](double margin_xy) {
    for (const auto &pt : all_points) {
      bounds.lower.x() = std::min(bounds.lower.x(), pt.x() - margin_xy);
      bounds.lower.y() = std::min(bounds.lower.y(), pt.y() - margin_xy);
      bounds.lower.z() = std::min(bounds.lower.z(), pt.z() - bound_margin_z_below);
      bounds.upper.x() = std::max(bounds.upper.x(), pt.x() + margin_xy);
      bounds.upper.y() = std::max(bounds.upper.y(), pt.y() + margin_xy);
    }
  };
  expand_points(bounds.bound_margin_xy);

  bounds.zone_in.assign(zones.size(), 0);
  bool grew = true;
  while (grew) {
    grew = false;
    for (size_t zi = 0; zi < zones.size(); ++zi) {
      if (bounds.zone_in[zi]) continue;
      const auto &zone = zones[zi];
      const double r = zone.reach + 5.0;
      const double dx = std::max({bounds.lower.x() - zone.center.x(), 0.0,
                                  zone.center.x() - bounds.upper.x()});
      const double dy = std::max({bounds.lower.y() - zone.center.y(), 0.0,
                                  zone.center.y() - bounds.upper.y()});
      if (std::hypot(dx, dy) > r) continue;
      bounds.zone_in[zi] = 1;
      grew = true;
      bounds.max_included_reach = std::max(bounds.max_included_reach, zone.reach);
      bounds.lower.x() = std::min(bounds.lower.x(), zone.center.x() - r);
      bounds.lower.y() = std::min(bounds.lower.y(), zone.center.y() - r);
      bounds.upper.x() = std::max(bounds.upper.x(), zone.center.x() + r);
      bounds.upper.y() = std::max(bounds.upper.y(), zone.center.y() + r);
    }
  }
  if (bounds.max_included_reach > 0.0) {
    bounds.bound_margin_xy = std::max(bounds.bound_margin_xy,
                                     bounds.max_included_reach + 5.0);
    expand_points(bounds.bound_margin_xy);
  }

  if (elevation) {
    for (size_t i = 0; i < all_points.size() - 1; ++i) {
      Eigen::Vector3d dir = all_points[i + 1] - all_points[i];
      double dist = dir.head<2>().norm();
      int n_samples = std::max(2, static_cast<int>(dist / 2.0));
      for (int s = 0; s <= n_samples; ++s) {
        double t = static_cast<double>(s) / n_samples;
        Eigen::Vector3d p = all_points[i] + t * dir;
        for (double offset : {-bounds.bound_margin_xy, 0.0, bounds.bound_margin_xy}) {
          float elev = elevation(p.x() + offset, p.y());
          if (elev > -1e10)
            bounds.max_terrain_z = std::max(bounds.max_terrain_z, static_cast<double>(elev));
          elev = elevation(p.x(), p.y() + offset);
          if (elev > -1e10)
            bounds.max_terrain_z = std::max(bounds.max_terrain_z, static_cast<double>(elev));
        }
      }
    }
  }
  bounds.upper.z() =
      std::max(bounds.max_terrain_z, bounds.upper.z()) + config.ceiling_headroom;
  for (size_t zi = 0; zi < zones.size(); ++zi) {
    if (!bounds.zone_in[zi]) continue;
    const double top = zones[zi].center.z() +
                       zones[zi].reach * config.risk_vertical_ratio;
    if (top > bounds.max_zone_top) bounds.max_zone_top = top;
    ++bounds.included_zone_count;
  }
  return bounds;
}

}  // namespace path_manager
