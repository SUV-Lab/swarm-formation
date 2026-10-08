#ifndef PATH_MANAGER_MISSION_ROUTE_SEARCH_H
#define PATH_MANAGER_MISSION_ROUTE_SEARCH_H

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>
#include <Eigen/Core>
#include "path_manager/route_postprocessing.h"

namespace path_manager {

struct MissionRouteSearchRequest {
  Eigen::Vector3d start{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> absolute_waypoints;
  double step_size{1.0};
  int drone_id{-1};
  bool takeoff_start{false};
};

struct RouteLegSearchRequest {
  size_t leg{0};
  Eigen::Vector3d start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  double step_size{1.0};
  int drone_id{-1};
  bool is_takeoff_leg{false};
};

enum class MissionRouteSearchStatus { SUCCESS, NO_WAYPOINTS, LEG_FAILED };

struct MissionRouteSearchResult {
  MissionRouteSearchStatus status{MissionRouteSearchStatus::NO_WAYPOINTS};
  TaggedRoute route;
  size_t attempted_legs{0};
  std::optional<size_t> failed_leg;
};

using RouteLegSearch =
    std::function<std::vector<Eigen::Vector3d>(const RouteLegSearchRequest &)>;
using LegSearchObserver =
    std::function<void(const RouteLegSearchRequest &, size_t point_count)>;

// Search each mission leg in order and return owned geometry/provenance.
// `search` is required for a nonempty mission. Both callbacks are borrowed
// synchronously and never retained. The observer runs after every search,
// including failure, before accepting its geometry or starting the next leg.
// A live adapter can capture that leg's policy while the searcher still owns
// its buffers. No epoch, environment, ROS publication or execution state is
// created here. A failed leg yields no partial route.
MissionRouteSearchResult searchMissionRoute(
    const MissionRouteSearchRequest &request,
    const RouteLegSearch &search,
    const LegSearchObserver &observer = {});

}  // namespace path_manager

#endif  // PATH_MANAGER_MISSION_ROUTE_SEARCH_H
