#include "path_manager/mission_route_search.h"
#include <utility>

namespace path_manager {

MissionRouteSearchResult searchMissionRoute(
    const MissionRouteSearchRequest &request,
    const RouteLegSearch &search,
    const LegSearchObserver &observer)
{
  MissionRouteSearchResult result;
  if (request.absolute_waypoints.empty()) return result;

  TaggedRoute route;
  route.points.push_back(request.start);
  for (size_t seg = 0; seg < request.absolute_waypoints.size(); ++seg) {
    const RouteLegSearchRequest leg{
        seg, seg == 0 ? request.start : request.absolute_waypoints[seg - 1],
        request.absolute_waypoints[seg], request.step_size, request.drone_id,
        seg == 0 && request.takeoff_start};
    const auto path = search(leg);
    ++result.attempted_legs;
    if (observer) observer(leg, path.size());
    if (path.size() < 2) {
      result.status = MissionRouteSearchStatus::LEG_FAILED;
      result.failed_leg = seg;
      return result;
    }
    // The next leg's first point is its shared junction. Preserve the
    // established unconditional skip, then the same 3-D duplicate gate.
    for (size_t i = seg == 0 ? 0 : 1; i < path.size(); ++i) {
      if (!route.points.empty() && (route.points.back() - path[i]).norm() < 1e-3)
        continue;
      route.points.push_back(path[i]);
      route.edge_leg.push_back(seg);
    }
  }
  result.status = MissionRouteSearchStatus::SUCCESS;
  result.route = std::move(route);
  return result;
}

}  // namespace path_manager
