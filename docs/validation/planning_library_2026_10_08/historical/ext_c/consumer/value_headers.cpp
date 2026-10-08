#include <mmp_vehicle_dynamics/vehicle_model.hpp>
// This translation unit must compile with only the installed manager and
// optimizer include roots, Eigen, and the compiler's standard include paths.
#include <path_manager/mission_plan.h>
#include <path_manager/mission_planner.h>
#include <path_manager/plan_settings.h>
#include <path_manager/planning_environment.h>
#include <path_manager/mission_planning_backend.h>
#include <path_manager/committed_route.h>
#include <path_manager/flight_assembly.h>
#include <path_manager/direct_trajectory_solver.h>
#include <path_manager/route_slice_solver.h>
#include <path_manager/route_slice_batch.h>
#include <path_manager/mission_geometry.h>
#include <path_manager/risk_zone.h>
#include <path_manager/route_postprocessing.h>
#include <path_optimizer/route_cost_profile.h>
#include <path_optimizer/optimizer_environment.h>
#include <path_optimizer/optimizer_request.h>
#include <path_manager/mission_route_search.h>
#include <path_manager/route_search_backend.h>
#include <path_manager/builtin_route_search.h>

#include <type_traits>

static_assert(std::is_copy_constructible<path_manager::MissionPlanRequest>::value,
              "mission request should be a value type");
static_assert(std::is_copy_constructible<path_manager::MissionPlanResult>::value,
              "mission result should own copyable trajectory/evaluation values");
static_assert(std::is_copy_constructible<path_manager::CommittedRoute>::value,
              "committed route should be a value type");
static_assert(std::is_copy_constructible<path_manager::AssembledFlight>::value,
              "assembled flight should be a value type");

static_assert(std::is_copy_constructible<path_manager::DirectTrajectoryRequest>::value, "direct input is a value");
static_assert(std::is_copy_constructible<path_manager::DirectTrajectoryResult>::value, "direct result is a value");
static_assert(std::is_copy_constructible<path_manager::RouteSliceRequest>::value, "slice input is a value");
static_assert(std::is_copy_constructible<path_manager::RouteSliceResult>::value, "slice result is a value");
static_assert(std::is_copy_constructible<path_manager::RouteSliceBatchResult>::value, "batch result is a value");

static_assert(std::is_copy_constructible<ego_planner::PieceAltitudeCapResult>::value, "altitude caps own copyable data");
static_assert(std::is_copy_constructible<ego_planner::PieceRoughnessResult>::value, "roughness owns copyable data");

static_assert(std::is_copy_constructible<ego_planner::OptimizerSolveResult>::value, "optimizer result owns its output");
static_assert(std::is_copy_constructible<path_manager::MissionRouteSearchResult>::value, "mission route owns its output");
static_assert(std::is_copy_constructible<path_manager::RouteSearchResult>::value, "replacement backend owns its output");

int main()
{
  ego_planner::OptimizerEnvironment environment;
  ego_planner::OptimizerRuntimeSettings runtime_settings;
  ego_planner::OptimizerSolveRequest solve_request;
  ego_planner::OptimizerSolveResult solve_result;
  path_manager::MissionRouteSearchRequest route_request;
  path_manager::MissionRouteSearchResult route_result;
  ego_planner::PieceAltitudeCapConfig cap_config;
  ego_planner::PieceAltitudeCapResult cap_result;
  ego_planner::PieceRoughnessConfig rough_config;
  ego_planner::PieceRoughnessResult rough_result;
  path_manager::ResolvedMissionWaypoints resolved_waypoints;
  path_manager::MissionBoundsConfig bounds_config;
  path_manager::MissionBoundsResult bounds_result;
  path_manager::RiskZone risk_zone{};
  path_manager::DirectTrajectoryRequest direct_request;
  path_manager::DirectTrajectoryResult direct_result;
  path_manager::RouteSliceRequest slice_request;
  path_manager::RouteSliceResult slice_result;
  path_manager::RouteSliceBatchResult batch_result;
  path_manager::MissionPlanRequest request;
  request.head.src = path_manager::StartStateSource::STATED_VECTOR;
  request.head.pos_u = Eigen::Vector3d(30.0, 150.0, 4.0);
  request.head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
  request.tail = ego_planner::TailBoundary::pinned(
      request.head.vel_u, Eigen::Vector3d::Zero());
  request.waypoints = {Eigen::Vector3d(330.0, 150.0, 4.0)};

  path_manager::CommittedRoute route;
  route.path = {request.head.pos_u, request.waypoints.front()};
  route.cap = {4.0, 4.0};
  route.edge_leg = {0};
  path_manager::MissionPlanResult result;
  path_manager::AssembledFlight assembled;
  path_manager::PrefixJoinResult join;
  return request.tail.prescribe_vel && route.path.size() == 2 &&
                 !result.hasTrajectory() && assembled.phases.empty() && !join.joined
             ? 0 : 1;
}
