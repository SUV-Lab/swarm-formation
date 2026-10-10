#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include "path_optimizer/poly_traj_utils.hpp"
#include "path_manager/risk_zone.h"

namespace path_manager::visualization {
// All views are borrowed for one synchronous preparation call. Builders retain
// neither references nor callbacks. Returned messages own all geometry/data.
// Structural violations throw std::invalid_argument; provider exceptions propagate.
// No partial message is returned. Allocation failures follow standard C++ rules.
// Trajectories must have positive finite piece durations, finite total duration
// and coefficients. Empty trajectories keep the original clear/no-tube behavior.
// Finite marker scales and sphere mesh heights retain legacy values, including
// zero (hidden geometry); these display settings do not constrain planning.
// Callbacks required by a nonempty branch must be callable. Risk values and
// visibility must be finite in [0,1]; shadow horizons may be nonfinite (the
// existing full-visibility fallback). Terrain NaN/-inf mean unavailable terrain.
// Provider functions must be observational and use the same canonical field as
// planning. They are called synchronously and are not retained by the result.
using Stamp = std::function<builtin_interfaces::msg::Time()>;
struct TerrainView {
  bool valid{false};
  double resolution{0.0};
  std::function<float(double, double)> elevation;
  // Preserve the profile's existing validity and gradient-sampler semantics.
  std::function<bool(double, double, float *, float *, float *)> elevation_and_gradient;
};
struct RiskView {
  const std::vector<RiskZone> &zones;
  TerrainView terrain;
  std::function<double(size_t, const Eigen::Vector3d &)> value;
  std::function<double(size_t, const Eigen::Vector3d &)> visibility;
  std::function<double(size_t, const Eigen::Vector3d &)> shadow_ceiling;
};
// Projection of settings owned by PathManager. No duplicate parameter reads or
// alternate defaults: adapters fill these from the existing setting authority.
// This zero-initialized aggregate is NOT a usable numerical default profile.
// Nonempty risk outputs require vertical_ratio > 0, finite softness/threshold.
// Profiles additionally use finite max_velocity/obstacle_clearance; heatmaps
// need heatmap_max_dim > 0 and finite heatmap settings; fields need finite
// step/slice_offset/agl/volume_alpha/heatmap_offset. step <= 0 retains automatic
// sampling. Terrain callbacks are required only when terrain.valid is true,
// with positive finite terrain.resolution. makeTrajectoryRisk uses value only;
// profiles use value/shadow/elevation_and_gradient; heatmaps use shadow/elevation;
// fields use value/visibility/shadow/elevation. Unknown modes retain the legacy
// diagnostic-slice behavior. A clock callback is required for stamped messages.
struct RiskSettings {
  double vertical_ratio{}, softness{}, threshold{}, obstacle_clearance{};
  bool grounded{};
  double max_velocity{}, step{}, slice_offset{}, agl{}, volume_alpha{};
  int contours{};
  std::string mode;
  double heatmap_agl_max{}, heatmap_offset{}, heatmap_preview_lift{};
  int heatmap_max_dim{};
  bool heatmap_safe_transparent{};
};
struct Obstacle {
  int id{};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()}, size{Eigen::Vector3d::Zero()};
  bool is_box{};
  double yaw{};
  std::string mesh_resource;
  Eigen::Vector3d mesh_native_size{Eigen::Vector3d::Ones()};
  double mesh_scale{1.0};
};
template<class Message> struct Prepared {
  Message message;
  // Existing diagnostic text, emitted by the caller at its original event.
  std::string diagnostic;
};
enum class TubeChannel { Optimized, Reference, FrontEnd };

nav_msgs::msg::Path makeFrontEndPath(const std::vector<Eigen::Vector3d> &route,
                                    const builtin_interfaces::msg::Time &stamp);
std::optional<visualization_msgs::msg::Marker> makeTube(
    const std::vector<Eigen::Vector3d> &points, int drone_id, double scale,
    const std::string &ns_prefix, float r, float g, float b, float a,
    const Stamp &stamp);
std::optional<visualization_msgs::msg::Marker> makeTrajectoryTube(
    const poly_traj::Trajectory &trajectory, int drone_id, double scale,
    const std::string &ns_prefix, float r, float g, float b, float a,
    const Stamp &stamp);
visualization_msgs::msg::MarkerArray makeChain(
    const std::vector<poly_traj::Trajectory> &runs,
    const std::vector<Eigen::Vector3d> &junctions,
    const poly_traj::Trajectory *terminal, double scale, const Stamp &stamp);
Prepared<visualization_msgs::msg::MarkerArray> makePipeline(
    const std::vector<Eigen::Vector3d> &shortcut_route,
    const std::vector<Eigen::Vector3d> &inner_points, const Stamp &stamp);
Prepared<visualization_msgs::msg::Marker> makeTrajectoryRisk(
    const poly_traj::Trajectory &trajectory, const RiskView &risk,
    const Stamp &stamp);
Prepared<std_msgs::msg::Float64MultiArray> makeRiskProfile(
    const poly_traj::Trajectory &trajectory, const RiskView &risk,
    const RiskSettings &settings);
Prepared<grid_map_msgs::msg::GridMap> makeRiskHeatmap(
    const RiskView &risk, const RiskSettings &settings, const Stamp &stamp);
visualization_msgs::msg::MarkerArray makeRiskField(
    const RiskView &risk, const RiskSettings &settings, const Stamp &stamp);
visualization_msgs::msg::MarkerArray makeDynamicObstacles(
    const std::vector<Obstacle> &obstacles, double sphere_mesh_height,
    const Stamp &stamp);
visualization_msgs::msg::Marker makeTrajectoryClear();
}  // namespace path_manager::visualization
