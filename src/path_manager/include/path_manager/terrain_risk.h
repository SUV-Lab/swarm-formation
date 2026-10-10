#pragma once

#include <Eigen/Core>
#include <vector>

#include "path_manager/risk_zone.h"
#include "path_manager/terrain_data.h"

namespace path_manager::terrain_risk {

// The caller supplies its validated settings in the existing planning frame
// (1 unit = 100 m). These are calculation inputs, not another defaults store.
// Softness must be finite and positive, as the existing ROS adapter validates.
struct Settings {
  bool enabled;
  bool grounded;
  double radial_step;  // <= 0 selects the borrowed DEM resolution
  double softness;    // vertical sigmoid width
  double obstacle_clearance;
};

// Owned polar horizon table. It remains valid after the builder's inputs are
// destroyed. Shadow samples use the original float storage and angular/radial
// layout. Treat a built mask as immutable while readers use it.
struct Mask {
  Eigen::Vector3d source{Eigen::Vector3d::Zero()};
  double max_range{0.0};
  double radial_step{1.0};
  int radial_count{0};
  int angular_count{0};
  std::vector<float> shadow_ceiling;  // [angle * radial_count + radius]
  bool valid{false};
};

struct BuildResult {
  Mask mask;
  float source_ground{0.0f};
  bool source_at_or_below_ground{false};
};

// Borrows zone and terrain synchronously, returning one owned mask. Disabled,
// unavailable terrain or nonpositive reach yields an invalid (ideal) mask.
// Geometry/settings must satisfy the same validated input contract as the
// planner; this extraction deliberately introduces no new sampling policy.
// Optional owned storage reuses its sample-vector capacity. Previous samples
// and metadata are replaced; passing std::move(previous_mask) avoids allocating
// another horizon buffer when the environment is rebuilt.
BuildResult buildMask(const RiskZone &zone, const TerrainData &terrain,
                      const Settings &settings, Mask reusable_storage = {});

// Query functions borrow the current immutable mask/terrain; they keep no
// cache or references beyond the call. A null or invalid mask returns the
// existing ideal visibility fallback: ceiling -infinity, visibility 1, grad 0.
// Elevation sampling retains TerrainData's existing per-thread memo.
double shadowCeiling(const Mask *mask, bool enabled,
                     const Eigen::Vector3d &pos);
double groundedZ(const TerrainData &terrain, const Settings &settings,
                 const Eigen::Vector3d &pos, double *dzeff_dz = nullptr);
double visibilityValue(const Mask *mask, const TerrainData &terrain,
                       const Settings &settings, const Eigen::Vector3d &pos);
double visibility(const Mask *mask, const TerrainData &terrain,
                  const Settings &settings, const Eigen::Vector3d &pos,
                  Eigen::Vector3d *grad = nullptr);

}  // namespace path_manager::terrain_risk
