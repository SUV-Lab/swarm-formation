#include "path_manager/mission_route_search.h"

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace path_manager;
using Points = std::vector<Eigen::Vector3d>;
int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

bool samePoints(const Points &a, const Points &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!((a[i] - b[i]).norm() == 0.0)) return false;
  return true;
}

struct ScriptedSearch {
  std::vector<Points> paths;
  std::vector<RouteLegSearchRequest> requests;
  std::vector<std::string> *events;
  Points operator()(const RouteLegSearchRequest &request)
  {
    requests.push_back(request);
    events->push_back("search" + std::to_string(request.leg));
    return paths.at(request.leg);
  }
};

struct Observer {
  std::vector<std::pair<size_t, size_t>> observations;
  std::vector<std::string> *events;
  void operator()(const RouteLegSearchRequest &request, size_t count)
  {
    observations.emplace_back(request.leg, count);
    events->push_back("observe" + std::to_string(request.leg));
  }
};

void sequentialOwnership()
{
  MissionRouteSearchResult retained;
  const Points expected{{0, 0, 0}, {5, 0, 0}, {10, 0, 0}, {10, 5, 0},
                        {10, 10, 0}, {15, 10, 0}, {20, 10, 0}};
  {
    MissionRouteSearchRequest request;
    request.start = Eigen::Vector3d::Zero();
    request.absolute_waypoints = {{10, 0, 0}, {10, 10, 0}, {20, 10, 0}};
    request.step_size = 0.75;
    request.drone_id = 4;
    request.takeoff_start = true;
    const auto original = request.absolute_waypoints;
    std::vector<std::string> events;
    RouteLegSearch search = ScriptedSearch{
        {{{0, 0, 0}, {0, 0, 0.0005}, {5, 0, 0}, {10, 0, 0}},
         {{999, 999, 999}, {10, 0.0005, 0}, {10, 5, 0}, {10, 10, 0}},
         {{10, 10, 0}, {15, 10, 0}, {20, 10, 0}}}, {}, &events};
    LegSearchObserver observer = Observer{{}, &events};
    retained = searchMissionRoute(request, search, observer);
    check(retained.status == MissionRouteSearchStatus::SUCCESS &&
              retained.attempted_legs == 3 && !retained.failed_leg &&
              samePoints(retained.route.points, expected) &&
              retained.route.edge_leg == std::vector<size_t>({0, 0, 1, 1, 2, 2}),
          "all legs form one owned route with duplicate removal and exact edge provenance");
    check(events == std::vector<std::string>({"search0", "observe0", "search1",
                                             "observe1", "search2", "observe2"}),
          "each observer runs while its leg still owns the backend, before the next search");
    const auto &calls = search.target<ScriptedSearch>()->requests;
    const auto &observed = observer.target<Observer>()->observations;
    bool forwarded = calls.size() == 3;
    for (size_t i = 0; forwarded && i < calls.size(); ++i) {
      const Eigen::Vector3d start = i == 0 ? request.start : original[i - 1];
      forwarded = calls[i].leg == i && calls[i].start == start &&
          calls[i].goal == original[i] && calls[i].step_size == 0.75 &&
          calls[i].drone_id == 4 && calls[i].is_takeoff_leg == (i == 0);
    }
    check(forwarded && observed == std::vector<std::pair<size_t, size_t>>({{0, 4}, {1, 4}, {2, 3}}),
          "original callback objects receive explicit leg inputs and pre-filter point counts");
    check(samePoints(request.absolute_waypoints, original),
          "waypoint inputs retain their absolute values");
    request.absolute_waypoints.clear();
    search.target<ScriptedSearch>()->paths.clear();
    check(samePoints(retained.route.points, expected),
          "editing caller and backend scratch storage cannot rewrite the result");
  }
  check(samePoints(retained.route.points, expected) &&
            retained.route.edge_leg == std::vector<size_t>({0, 0, 1, 1, 2, 2}),
        "route geometry and leg provenance survive all inputs and callbacks");
}

void refusalsAndBoundaryPolicy()
{
  MissionRouteSearchRequest request;
  request.absolute_waypoints = {{1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  std::vector<std::string> events;
  RouteLegSearch search = ScriptedSearch{
      {{{0, 0, 0}, {1, 0, 0}}, {{1, 0, 0}}, {{2, 0, 0}, {3, 0, 0}}}, {}, &events};
  std::vector<size_t> policy_legs;
  LegSearchObserver observer = [&](const RouteLegSearchRequest &leg, size_t count) {
    events.push_back("observe" + std::to_string(leg.leg));
    if (count < 2) policy_legs.clear();
    else policy_legs.push_back(leg.leg);
  };
  const auto failed = searchMissionRoute(request, search, observer);
  check(failed.status == MissionRouteSearchStatus::LEG_FAILED &&
            failed.failed_leg == 1 && failed.attempted_legs == 2 &&
            failed.route.points.empty() && failed.route.edge_leg.empty() &&
            policy_legs.empty() && events == std::vector<std::string>({
                "search0", "observe0", "search1", "observe1"}),
        "a failed leg is observed once, clears the adapter's captures, and returns no prefix");
  bool no_takeoff = true;
  for (const auto &leg : search.target<ScriptedSearch>()->requests)
    no_takeoff = no_takeoff && !leg.is_takeoff_leg;
  check(no_takeoff, "a junction or handoff mission never receives first-leg takeoff relief");

  events.clear();
  search.target<ScriptedSearch>()->paths.front().clear();
  const auto first_failed = searchMissionRoute(request, search, observer);
  check(first_failed.status == MissionRouteSearchStatus::LEG_FAILED &&
            first_failed.failed_leg == 0 && first_failed.attempted_legs == 1 &&
            first_failed.route.points.empty() &&
            events == std::vector<std::string>({"search0", "observe0"}),
        "an empty first-leg result obeys the same failure and observer contract");
  const auto empty = searchMissionRoute({}, {}, observer);
  check(empty.status == MissionRouteSearchStatus::NO_WAYPOINTS &&
            empty.attempted_legs == 0 && !empty.failed_leg &&
            empty.route.points.empty() &&
            events == std::vector<std::string>({"search0", "observe0"}),
        "an empty mission makes no backend or observer calls");

  MissionRouteSearchRequest vertical;
  vertical.absolute_waypoints = {{0, 0, 1}};
  const auto separated = searchMissionRoute(vertical, [](const RouteLegSearchRequest &) {
    return Points{{0, 0, 0}, {0, 0, 0.0005}, {0, 0, 0.001}, {0, 0, 1}};
  });
  check(samePoints(separated.route.points, {{0, 0, 0}, {0, 0, 0.001}, {0, 0, 1}}) &&
            separated.route.edge_leg == std::vector<size_t>({0, 0}),
        "the duplicate gate uses full 3-D distance and its original strict threshold");
  vertical.absolute_waypoints = {{0, 0, 0}};
  const auto collapsed = searchMissionRoute(vertical, [](const RouteLegSearchRequest &) {
    return Points{{0, 0, 0}, {0, 0, 0}};
  });
  check(collapsed.status == MissionRouteSearchStatus::SUCCESS &&
            collapsed.route.points.size() == 1 && collapsed.route.edge_leg.empty(),
        "postprocessing retains responsibility for rejecting an entirely collapsed route");
}

void callbackLifetime()
{
  MissionRouteSearchResult retained;
  std::weak_ptr<int> lifetime;
  {
    auto state = std::make_shared<int>(0);
    lifetime = state;
    RouteLegSearch search = [state](const RouteLegSearchRequest &leg) {
      ++*state;
      return Points{leg.start, leg.goal};
    };
    LegSearchObserver observer = [state](const RouteLegSearchRequest &, size_t) { ++*state; };
    MissionRouteSearchRequest request;
    request.absolute_waypoints = {{2, 0, 0}};
    retained = searchMissionRoute(request, search, observer);
    check(*state == 2, "search and observer run synchronously before return");
  }
  check(lifetime.expired() && samePoints(retained.route.points, {{0, 0, 0}, {2, 0, 0}}),
        "owned route retains neither the search provider nor its observer");
}
}  // namespace

int main()
{
  sequentialOwnership();
  refusalsAndBoundaryPolicy();
  callbackLifetime();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
