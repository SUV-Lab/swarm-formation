// Nothing in route_search_backend.h includes this header.
#pragma once
#include "path_manager/route_search_backend.h"

namespace path_planner { namespace search { class PathSearcher; } }

namespace swarm_formation { class LogManager; }

namespace path_manager {

struct Fm2RouteSearchConfig {
  int coarse_k{4};
  // Keep the old signed parsed type until applying the legacy setter cast.
  int max_cells{8000000};
  bool star{true};
  double altitude_weight{2.0};
  double altitude_scale_up{10.0};
  double altitude_scale_down{2.0};
};

struct AstarRouteSearchConfig {
  double step_size{1.0};
  double smha_weight{2.0};
};

struct BuiltinRouteSearchConfig {
  enum class Algorithm { FM2, ASTAR };
  Algorithm algorithm{Algorithm::FM2};
  Fm2RouteSearchConfig fm2;
  AstarRouteSearchConfig astar;
  // These define the built-in cost/shortcut/policy implementation. An
  // external backend may expose entirely different configuration types.
  double risk_weight{1.0};
  double risk_barrier{100.0};
  double risk_goal_taper_radius{30.0};
  double roughness_weight{0.0};
  double roughness_slope0{0.20};
  bool lexicographic_zone_avoidance{true};
  bool bypass_shortcut{false};
};

// Concrete legacy implementation. The public constructor owns its searcher.
// The borrowed-searcher overload is for the ROS adapter's canonical query
// object, whose owner must outlive this backend. Providers are used only in
// synchronous calls; that adapter separately owns its live query bindings.
class BuiltinRouteSearchBackend final : public RouteSearchBackend {
 public:
  explicit BuiltinRouteSearchBackend(BuiltinRouteSearchConfig config = {},
      std::shared_ptr<swarm_formation::LogManager> logger = nullptr,
      path_planner::search::PathSearcher *borrowed_searcher = nullptr);
  ~BuiltinRouteSearchBackend() override;
  RouteSearchResult search(const RouteSearchProblem &problem) override;
  // Adapter hook: configure canonical queries before the first search, and
  // allocate the A* pool only if this implementation will generate the route.
  bool prepare(const RouteSearchProblem &problem, bool prepare_search);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::shared_ptr<RouteSearchBackend> makeBuiltinRouteSearchBackend(
    BuiltinRouteSearchConfig config = {},
    std::shared_ptr<swarm_formation::LogManager> logger = nullptr);

}  // namespace path_manager
