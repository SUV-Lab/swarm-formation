#include "path_manager/builtin_route_search.h"
#include "path_planner/dyn_a_star.h"

#include <cmath>
#include <utility>

namespace path_manager {

struct BuiltinRouteSearchBackend::Impl {
  BuiltinRouteSearchConfig config;
  std::shared_ptr<swarm_formation::LogManager> logger;
  std::unique_ptr<path_planner::search::PathSearcher> owned;
  path_planner::search::PathSearcher *searcher;
  std::vector<path_planner::search::RiskZoneLite> zones;
  bool pool_initialized{false};
  Eigen::Vector3i pool_size{Eigen::Vector3i::Zero()};
  uint64_t prepared_context{0};
  bool search_ready{false};

  Impl(BuiltinRouteSearchConfig options,
       std::shared_ptr<swarm_formation::LogManager> log,
       path_planner::search::PathSearcher *borrowed)
      : config(std::move(options)), logger(std::move(log)),
        owned(borrowed ? nullptr : new path_planner::search::PathSearcher),
        searcher(borrowed ? borrowed : owned.get()) {}

  void releaseOwnedBindings() {
    if (!owned) return;  // PM separately owns its canonical live bindings.
    searcher->setSDF(nullptr, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);
    searcher->setTerrainHeightmap({});
    searcher->setTerrainHeightGrad({});
    searcher->setRiskVisibility({});
    searcher->setRiskZones(nullptr);
    search_ready = false;
  }
};

BuiltinRouteSearchBackend::BuiltinRouteSearchBackend(
    BuiltinRouteSearchConfig config,
    std::shared_ptr<swarm_formation::LogManager> logger,
    path_planner::search::PathSearcher *borrowed_searcher)
    : impl_(new Impl(std::move(config), std::move(logger), borrowed_searcher)) {}

BuiltinRouteSearchBackend::~BuiltinRouteSearchBackend() = default;

bool BuiltinRouteSearchBackend::prepare(const RouteSearchProblem &problem,
                                       bool prepare_search) {
  auto &state = *impl_;
  auto &searcher = *state.searcher;
  const auto &environment = problem.environment;
  const auto &constraints = problem.constraints;
  const auto &config = state.config;
  state.search_ready = false;
  state.zones.clear();
  state.zones.reserve(environment.zones.size());
  for (const auto &zone : environment.zones)
    state.zones.push_back({zone.center, zone.reach, zone.peak, zone.vertical_reach});
  searcher.setRiskZones(state.zones.empty() ? nullptr : &state.zones);
  searcher.setRiskVisibility(environment.risk_visibility);
  searcher.setLogManager(state.logger);
  searcher.setSDF(environment.distance_field, environment.lower,
                 environment.upper - environment.lower,
                 environment.voxel_xy, environment.voxel_z);
  searcher.setObstacleMargin(constraints.obstacle_clearance);
  searcher.setZoneAvoidLexico(config.lexicographic_zone_avoidance);
  searcher.setGroundHeight(constraints.ground_height);
  searcher.setVirtualCeilHeight(constraints.virtual_ceiling);
  searcher.setTerrainHeightmap(environment.elevation, environment.terrain_cell_u);
  searcher.setRiskAlpha(config.risk_weight);
  searcher.setRiskBarrier(config.risk_barrier);
  searcher.setRiskGoalTaperRadius(config.risk_goal_taper_radius);
  searcher.setGeodesicSlopeTanMax(std::tan(constraints.flight_path_angle_max_rad));
  searcher.setSmhaW(config.astar.smha_weight);
  searcher.setFrontEnd(config.algorithm == BuiltinRouteSearchConfig::Algorithm::FM2
      ? path_planner::search::PathSearcher::FrontEnd::FM2
      : path_planner::search::PathSearcher::FrontEnd::ASTAR);
  searcher.setFm2CoarseK(config.fm2.coarse_k);
  searcher.setFm2MaxCells(static_cast<size_t>(config.fm2.max_cells));
  searcher.setFm2Star(config.fm2.star);
  searcher.setFm2AltPenalty(config.fm2.altitude_weight, config.fm2.altitude_scale_up,
                           config.fm2.altitude_scale_down);
  searcher.setTerrainHeightGrad(environment.elevation_gradient);
  searcher.setRoughness(config.roughness_weight, config.roughness_slope0);
  searcher.setDynObstacleMargin(constraints.dynamic_obstacle_margin);
  searcher.setBypassShortcut(config.bypass_shortcut);
  if (prepare_search && config.algorithm == BuiltinRouteSearchConfig::Algorithm::ASTAR) {
    const auto &desired = environment.fine_grid_shape;
    if (desired.minCoeff() <= 0) {
      if (state.logger) state.logger->errorf("SDF shape not available, cannot size A* pool");
      return false;
    }
    if (!state.pool_initialized) {
      state.pool_size = desired;
      searcher.initGridMap(state.pool_size);
      state.pool_initialized = true;
      if (state.logger) state.logger->infof("A* pool allocated: (%d,%d,%d)",
          desired.x(), desired.y(), desired.z());
    } else if (desired != state.pool_size) {
      state.pool_size = desired;
      searcher.resizePool(state.pool_size);
      if (state.logger) state.logger->infof("A* pool resized: (%d,%d,%d)",
          desired.x(), desired.y(), desired.z());
    }
  }
  state.prepared_context = problem.identity.context_id;
  state.search_ready = prepare_search;
  return true;
}

RouteSearchResult BuiltinRouteSearchBackend::search(const RouteSearchProblem &problem) {
  auto &state = *impl_;
  // Owned standalone backends clear borrowed providers before returning.
  struct Cleanup { Impl &state; ~Cleanup() { state.releaseOwnedBindings(); } } cleanup{state};
  RouteSearchResult result;
  result.algorithm = state.config.algorithm == BuiltinRouteSearchConfig::Algorithm::FM2
      ? "fm2" : "astar";
  result.policy.identity = problem.identity;
  if (!state.search_ready || problem.identity.context_id == 0 ||
      state.prepared_context != problem.identity.context_id) {
    if (!prepare(problem, true)) {
      result.status = RouteSearchStatus::INVALID_REQUEST;
      result.detail = "search grid is unavailable";
      return result;
    }
  }
  result.points = state.searcher->astarSearchAndGetSimplePath(
      state.config.astar.step_size, problem.start, problem.goal,
      problem.drone_id, problem.is_takeoff_leg);
  result.status = result.points.size() >= 2 ? RouteSearchStatus::SUCCESS
                                         : RouteSearchStatus::NO_ROUTE;
  auto &policy = result.policy;
  // A* does not execute the FM2 three-pass, even if a borrowed searcher
  // contains buffers from an earlier FM2 use.
  policy.legacy_pass = state.config.algorithm == BuiltinRouteSearchConfig::Algorithm::FM2
      ? state.searcher->zoneAvoidPass() : 0;
  policy.endpoint_exempt = state.searcher->zoneNoBarrier();
  policy.soft_override = state.searcher->zoneSoftOverride();
  const size_t count = problem.environment.zones.size();
  if (count == 0 || (policy.legacy_pass != 0 &&
      policy.endpoint_exempt.size() == count && policy.soft_override.size() == count)) {
    policy.knowledge = RoutePolicyKnowledge::CANONICAL_DISPOSITIONS;
    policy.suppress_crossing_exempt = policy.legacy_pass == 1;
    for (size_t i = 0; i < count; ++i)
      policy.dispositions.push_back(policy.endpoint_exempt[i]
          ? RouteZoneDisposition::SOFT_ENDPOINT
          : policy.legacy_pass == 2 ? RouteZoneDisposition::SOFT_FALLBACK
          : policy.soft_override[i] ? RouteZoneDisposition::SOFT_UNAVOIDABLE
                                    : RouteZoneDisposition::HARD_AVOID);
    // Stale legacy buffers have no meaning for an empty zone set.
    if (count == 0) { policy.endpoint_exempt.clear(); policy.soft_override.clear(); }
  }
  return result;
}

std::shared_ptr<RouteSearchBackend> makeBuiltinRouteSearchBackend(
    BuiltinRouteSearchConfig config,
    std::shared_ptr<swarm_formation::LogManager> logger) {
  return std::make_shared<BuiltinRouteSearchBackend>(std::move(config), std::move(logger));
}

}  // namespace path_manager
