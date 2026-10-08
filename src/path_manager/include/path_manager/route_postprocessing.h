#ifndef PATH_MANAGER_ROUTE_POSTPROCESSING_H
#define PATH_MANAGER_ROUTE_POSTPROCESSING_H

#include <cstddef>
#include <functional>
#include <vector>

#include "path_manager/mission_geometry.h"

namespace path_planner {
namespace sdf {
class IDistanceField;
}
}

namespace path_manager {

struct TaggedRoute {
  std::vector<Eigen::Vector3d> points;
  // One entry per edge, when provenance is known.
  std::vector<size_t> edge_leg;
};

using CornerAcceptance =
    std::function<bool(const Eigen::Vector3d &sample,
                       const Eigen::Vector3d &original_corner)>;

// Round route corners with the existing sampled quadratic Bezier fillets.
// The callback is borrowed synchronously and sees only internal arc samples,
// in candidate order; an empty callback accepts them. It owns the caller's
// terrain/zone predicate, including its distinct NaN comparison semantics.
// Inactive radius or fewer than three points returns the input unchanged.
// An active fillet requires one leg tag per edge and throws invalid_argument
// on a mismatch; it must not invent attribution at a leg junction.
TaggedRoute roundRouteCorners(TaggedRoute route, double radius,
                              const CornerAcceptance &sample_ok);

struct RouteProfileConfig {
  // Parameter defaults; the PathManager adapter supplies its current values.
  double length_per_piece{3.0};
  double obstacle_clearance{0.3};
};

struct RouteDenoiseStats {
  bool applied{false};
  size_t adjusted{0};
  double min_before{0.0};
  double max_before{0.0};
  double min_after{0.0};
  double max_after{0.0};
};

struct RouteProfileResult {
  std::vector<Eigen::Vector3d> filtered_route;
  std::vector<Eigen::Vector3d> path;
  std::vector<double> cap_ref;
  std::vector<size_t> edge_leg;
  // Snapshot before denoising; upward clamps may raise cap_ref above this.
  double raw_max_z{-1e9};
  RouteDenoiseStats denoise;
  bool input_tags_match{false};
  double max_segment_length{1.0};
};

// Smooth z from an immutable original-height window, preserve the committed
// cap as max(raw, filtered), then subdivide long 3D edges. Inputs use absolute
// planner coordinates. A missing elevation callback skips terrain/water
// clamping; a present callback's invalid samples use the mission water floor.
// The optional distance provider is queried only while hasData() is true;
// denoising accepts finite distance >= clearance, retaining raw z otherwise.
// Providers must remain alive/stable during this synchronous call; no callback
// or provider is retained. Results own all geometry and diagnostic values.
// Malformed tags drop output provenance but still produce geometry. Empty
// input gives an empty result; single-point/degenerate results are left for
// the caller's existing route-length acceptance policy.
RouteProfileResult prepareRouteProfile(
    TaggedRoute route, const Eigen::Vector3d &start_pos,
    const std::vector<Eigen::Vector3d> &waypoints,
    const RouteProfileConfig &config, const ElevationQuery &elevation = {},
    const path_planner::sdf::IDistanceField *distance = nullptr);

}  // namespace path_manager

#endif  // PATH_MANAGER_ROUTE_POSTPROCESSING_H
