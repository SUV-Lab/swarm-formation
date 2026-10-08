#pragma once

#include <Eigen/Core>
#include <mmp_vehicle_dynamics/vehicle_model.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace path_planner { namespace sdf { class IDistanceField; } }

namespace path_manager {

// Existing planner coordinates/units. No ROS messages or SI conversion here.
struct RouteSearchIdentity {
  // Unique per actual invocation, including across different planner objects.
  // Equal per-manager generation/epoch numbers alone cannot reject a result
  // accidentally replayed by a backend reused with a different planner.
  uint64_t request_id{0};
  uint64_t context_id{0};
  uint64_t environment_revision{0};
  uint64_t zone_generation{0};
  uint64_t policy_epoch{0};
  uint64_t search_serial{0};
  size_t leg{0};
};

struct RouteSearchZone {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double reach{0.0};
  double peak{0.0};
  double vertical_reach{0.0};
};

// Shared physical query constraints, not algorithm tuning. FM2/A* options,
// objective weights and discretization choices belong to concrete backends.
struct RouteQueryConstraints {
  double obstacle_clearance{0.5};
  double ground_height{-1.0};
  double virtual_ceiling{-1.0};
  double dynamic_obstacle_margin{0.0};
  // Resolved physical cone: explicit model limit, otherwise the ROS adapter
  // reads the legacy live setting. No backend-specific discretization here.
  double flight_path_angle_max_rad{0.0};
};

// A live binding during one synchronous call. The values/functions belong
// to the problem; the distance field and callback providers are borrowed.
// Implementations must not retain references to this problem or invoke its
// providers after returning. Owned callback objects may be copied as working
// state; they do not extend the lifetime of providers captured by raw pointer.
struct RouteSearchEnvironment {
  const path_planner::sdf::IDistanceField *distance_field{nullptr};
  Eigen::Vector3d lower{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upper{Eigen::Vector3d::Zero()};
  double voxel_xy{1.0};
  double voxel_z{1.0};
  Eigen::Vector3i fine_grid_shape{Eigen::Vector3i::Zero()};
  double terrain_cell_u{0.0};
  std::vector<RouteSearchZone> zones;
  std::function<float(double, double)> elevation;
  std::function<bool(double, double, float *, float *, float *)> elevation_gradient;
  std::function<double(size_t, const Eigen::Vector3d &)> risk_visibility;
};

struct RouteSearchProblem {
  Eigen::Vector3d start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  int drone_id{-1};
  bool is_takeoff_leg{false};
  RouteSearchIdentity identity;
  RouteQueryConstraints constraints;
  RouteSearchEnvironment environment;
  mmp_vehicle_dynamics::VehicleModel::Ptr model;
};

enum class RouteZoneDisposition {
  HARD_AVOID,
  SOFT_UNAVOIDABLE,
  SOFT_ENDPOINT,
  SOFT_FALLBACK
};

enum class RoutePolicyKnowledge {
  UNKNOWN,
  // The implementation supplies dispositions under MMP's existing canonical
  // zone-volume semantics. This does not assert that it ran the FM2 3-pass.
  CANONICAL_DISPOSITIONS
};

struct RouteSearchPolicy {
  RoutePolicyKnowledge knowledge{RoutePolicyKnowledge::UNKNOWN};
  RouteSearchIdentity identity;
  // One entry for each environment.zones entry in exactly the same order.
  std::vector<RouteZoneDisposition> dispositions;
  // Legacy diagnostics: 0 = unavailable/off, 1..3 = built-in FM2 passes.
  // An external implementation must not invent a FM2 pass it did not run.
  int legacy_pass{0};
  // With a legacy pass these are the unchanged raw FM2 diagnostics, whose
  // endpoint/fallback precedence is checked against dispositions. With pass
  // zero they may be omitted; if supplied, they must be exact boolean
  // projections of SOFT_ENDPOINT / SOFT_UNAVOIDABLE respectively.
  std::vector<char> endpoint_exempt;
  std::vector<char> soft_override;
  // The existing optimizer crossing-exemption switch, explicitly supplied
  // independently of the diagnostic pass number. False when unmeasured.
  bool suppress_crossing_exempt{false};
};

enum class RouteSearchStatus { SUCCESS, NO_ROUTE, INVALID_REQUEST, BACKEND_FAILURE };

struct RouteSearchResult {
  RouteSearchStatus status{RouteSearchStatus::NO_ROUTE};
  std::vector<Eigen::Vector3d> points;
  RouteSearchPolicy policy;
  std::string algorithm;
  std::string detail;
};

// The caller owns command/state/epoch/ABORT policy. The backend generates one
// leg and returns owned geometry and whatever policy it can actually attest.
// Each instance is used serially; a call returns after all of its work ends.
// Unknown policy stays unknown and uses the existing unmeasured-policy gates.
class RouteSearchBackend {
 public:
  virtual ~RouteSearchBackend() = default;
  virtual RouteSearchResult search(const RouteSearchProblem &problem) = 0;
};

// Structural admission only; UNKNOWN remains unknown. Geometry and metadata
// are checked separately so legacy unmeasured-policy gates retain authority.
bool routeSearchGeometryUsable(const RouteSearchResult &result);
bool routeSearchPolicyMeasured(const RouteSearchProblem &problem,
                               const RouteSearchPolicy &policy);
uint64_t nextRouteSearchIdentity();

}  // namespace path_manager
