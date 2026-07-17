// SDFManager: CPU ESDF (Felzenszwalb-Huttenlocher + OpenMP) with flat
// binary save/load.
//
// Two-layer architecture:
//   static layer:   one signed ESDF over the whole grid, built once from
//                   terrain occupancy via Felzenszwalb 1D sweep x3, or
//                   loaded from a cached .esdf file.
//   dynamic layer:  zero or more "patches", each a small dense ESDF slice
//                   covering one user-added obstacle's AABB + influence
//                   radius. Patches are independent; remove = O(1) erase.
//
// Query semantics:
//   getDistance(p) = min(static_distance(p),
//                        min over patches that contain p of patch_distance(p))
// Adding obstacles can only *decrease* the distance, never increase it.
// The static layer is never modified by addObstacle/removeObstacle, so a
// pre-built terrain ESDF stays intact across runs.

#ifndef PATH_PLANNER_SDF_MANAGER_H_
#define PATH_PLANNER_SDF_MANAGER_H_

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>

#include "path_planner/sdf/distance_field.h"

namespace path_planner {
namespace sdf {

struct SDFManagerImpl;  // pimpl

// Geometric primitive used by addObstacle().
enum class PrimitiveKind { kCube, kCylinder, kSphere };

struct PrimitiveSpec {
  PrimitiveKind kind = PrimitiveKind::kCube;
  // World-frame center of the primitive.
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  // Full extents (not half-extents):
  //   kCube     : (x_size, y_size, z_size)
  //   kCylinder : (diameter_x, diameter_y, height_z)  (axis = z)
  //   kSphere   : (diameter, diameter, diameter)
  Eigen::Vector3d size = Eigen::Vector3d::Ones();
  // Heading about +Z in radians (kCube/kCylinder; ignored for kSphere).
  // The patch rasterizer tests occupancy in the rotated frame, so the
  // resulting patch SDF is exact for the ORIENTED primitive — collision
  // follows the displayed hull instead of an axis-aligned stand-in.
  double yaw = 0.0;
};

class SDFManager : public IDistanceField {
 public:
  SDFManager();
  ~SDFManager() override;

  SDFManager(const SDFManager&) = delete;
  SDFManager& operator=(const SDFManager&) = delete;

  // Isotropic voxels (all axes = voxel_size, frame units).
  bool initialize(double voxel_size);
  // ANISOTROPIC voxels: the terrain frame compresses z (1 z-unit >> 1 xy
  // metre-equivalent), so a cubic voxel is wastefully fine in xy (the DEM
  // itself is coarser) while quantizing altitude in ~half-cell steps that
  // dwarf real terrain features. A thin-z voxel fixes the vertical
  // quantization at unchanged memory.
  bool initialize(double voxel_xy, double voxel_z);

  // occupancy layout: ((x * ny) + y) * nz + z (numpy C-order).
  // 0 = free, nonzero = occupied.
  bool buildFromVoxels(const uint8_t* occupancy,
                       int nx, int ny, int nz,
                       const Eigen::Vector3d& origin);

  // Grid known to contain NO occupied voxels: identical result to
  // buildFromVoxels(all-zero) but without materializing the occupancy array —
  // the boxes-only design always builds empty (obstacles arrive as patches),
  // yet the zeroed vector alone cost ~5.8 GB transient on a 30 m corridor.
  bool buildEmpty(int nx, int ny, int nz, const Eigen::Vector3d& origin);

  bool saveToFile(const std::string& path) const;
  bool loadFromFile(const std::string& path,
                    const Eigen::Vector3d& bbox_lo,
                    const Eigen::Vector3d& bbox_hi);

  // Returns signed distance in FRAME UNITS (1 unit = 100 m): min(static, dynamic).
  // +inf if outside map or unobserved by any layer.
  uint64_t revision() const override { return revision_; }

  float getDistance(const Eigen::Vector3d& pos) const override;
  // Distance to the dynamic patch layer only (+inf outside patch AABBs).
  float getDynamicDistance(const Eigen::Vector3d& pos) const override;

  // Distance + gradient. Gradient is taken from whichever layer (static or
  // a dynamic patch) produces the minimum at pos; this is the sub-gradient
  // of the min and is what downstream gradient-based optimizers expect.
  bool getDistanceAndGradient(const Eigen::Vector3d& pos,
                              float* distance,
                              Eigen::Vector3d* gradient) const override;

  // Bulk-read protocol (see IDistanceField): the guard holds the dynamic-
  // patch lock in shared mode once; the *Bulk queries then skip per-call
  // locking. Used by the FM2 speed-map build (per-cell queries over the
  // whole grid — per-call locking there cost ~60 s of pure rwlock traffic).
  std::unique_ptr<BulkReadGuard> bulkReadGuard() const override;
  float getDistanceBulk(const Eigen::Vector3d& pos) const override;
  float getDynamicDistanceBulk(const Eigen::Vector3d& pos) const override;

  // ----- dynamic obstacle layer -----
  //
  // Patches extend influenceRadius() frame units beyond the primitive AABB so
  // gradient is well defined out to that radius. Outside the patch AABB the
  // patch's contribution is +inf (i.e. only the static layer matters there).

  // Default 2.0 m, matches the typical clearance reach of our optimizer.
  void   setInfluenceRadius(double r);
  double influenceRadius() const;

  // Returns patch id (>= 0) on success, -1 on failure
  // (e.g. primitive entirely outside the grid, or static layer missing).
  int  addObstacle(const PrimitiveSpec& spec);

  // Remove a previously added patch by id. No-op for an unknown/freed id.
  void removeObstacle(int patch_id);

  // Drop all dynamic patches. Static layer untouched.
  void clearObstacles();

  // Number of currently active (not removed) patches.
  size_t numActiveObstacles() const;

  bool isInitialized() const;
  bool hasData() const override;
  double voxelSize() const;            // x-axis size (legacy callers)
  Eigen::Vector3d voxelSizes() const;  // per-axis (vx, vy, vz)
  size_t numAllocatedBlocks() const;

  // Grid extent in cells along x/y/z. Zero if no data.
  Eigen::Vector3i shape() const;
  // World-frame origin (lower corner of voxel (0,0,0)).
  Eigen::Vector3d origin() const;

 private:
  std::unique_ptr<SDFManagerImpl> impl_;
  uint64_t revision_ = 0;
};

}  // namespace sdf
}  // namespace path_planner

#endif  // PATH_PLANNER_SDF_MANAGER_H_
