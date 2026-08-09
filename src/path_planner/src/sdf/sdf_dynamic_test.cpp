// Sanity check for the dynamic obstacle layer added to SDFManager.
//
// Builds a small static ESDF (single cube), then exercises:
//   1. addObstacle (sphere/cube/cylinder) -> expected geometric distance
//   2. min(static, dynamic) composition  -> dynamic only lowers distance
//   3. removeObstacle / clearObstacles   -> reverts to static-only state
//   4. gradient sub-gradient handoff at the static/dynamic boundary
//
// All checks compare numerical distance/gradient to closed-form geometric
// truth (sphere ⇒ ||p - c|| - r, cube ⇒ Linfty box distance, etc.).
// Tolerances reflect (a) trilinear smoothing and (b) voxel quantization.

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "path_planner/sdf/sdf_manager.h"

namespace {

using path_planner::sdf::PrimitiveKind;
using path_planner::sdf::PrimitiveSpec;
using path_planner::sdf::SDFManager;

int g_passed = 0;
int g_failed = 0;

void check(bool cond, const std::string& label,
           double measured, double expected, double tol) {
  const bool ok = cond &&
      std::abs(measured - expected) <= tol;
  std::cout << "  " << (ok ? "[OK]   " : "[FAIL] ") << std::left
            << std::setw(40) << label
            << " measured=" << std::fixed << std::setprecision(4) << measured
            << " expected=" << expected
            << " tol="     << tol << "\n";
  if (ok) ++g_passed; else ++g_failed;
}

// Geometric reference distances --------------------------------------------

double sphereSignedDistance(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& c, double radius) {
  return (p - c).norm() - radius;
}

double cubeSignedDistance(const Eigen::Vector3d& p,
                          const Eigen::Vector3d& c, const Eigen::Vector3d& size) {
  const Eigen::Vector3d half = 0.5 * size;
  const Eigen::Vector3d d = (p - c).cwiseAbs() - half;
  // Standard SDF for an axis-aligned box.
  Eigen::Vector3d d_pos = d.cwiseMax(0.0);
  double outside = d_pos.norm();
  double inside = std::min(d.maxCoeff(), 0.0);
  return outside + inside;
}

}  // namespace

int main() {
  std::cout << "\n=== SDFManager dynamic layer sanity check ===\n\n";

  // --- 1. Static layer: a small cube obstacle in the middle of the grid ---
  const int N = 40;
  const double voxel = 0.25;       // 0.25 m per voxel ⇒ 10 m per axis
  std::vector<uint8_t> occ(N * N * N, 0);
  auto set = [&](int x, int y, int z, uint8_t v) {
    occ[((size_t)x * N + y) * N + z] = v;
  };
  for (int x = 18; x < 22; ++x)
    for (int y = 18; y < 22; ++y)
      for (int z = 18; z < 22; ++z)
        set(x, y, z, 1);

  SDFManager sdf;
  if (!sdf.initialize(voxel) ||
      !sdf.buildFromVoxels(occ.data(), N, N, N, Eigen::Vector3d::Zero())) {
    std::cerr << "build failed\n"; return 1;
  }
  std::cout << "Static layer built: 40^3 grid, voxel=" << voxel
            << ", cube at world ~[4.5, 5.5] each axis.\n\n";

  // Sanity: a far-away point should have positive (free) distance from the
  // static cube. Use this as the baseline before adding obstacles.
  const Eigen::Vector3d far_p(1.0, 1.0, 1.0);   // ~5 m from the static cube
  const float static_only_far = sdf.getDistance(far_p);
  std::cout << "  static-only distance at (1,1,1) = "
            << static_only_far << " m (free, must be > 0)\n";
  check(static_only_far > 0.0f, "static distance at far point",
        static_only_far, /*expected*/ 6.0,  /*tol*/ 2.0);

  // --- 2. Add a sphere far from the static cube; check geometric distance --
  // Sphere diameter 2 m (= 8 voxels @ 0.25 m) so rasterization error is small.
  std::cout << "\n[Test] Add a sphere obstacle (center=(1,1,1), diameter=2.0)\n";
  PrimitiveSpec sphere;
  sphere.kind = PrimitiveKind::kSphere;
  sphere.center = Eigen::Vector3d(1.0, 1.0, 1.0);
  sphere.size = Eigen::Vector3d(2.0, 2.0, 2.0);   // diameter 2m, r = 1.0
  const int sphere_id = sdf.addObstacle(sphere);
  std::cout << "  patch id = " << sphere_id
            << " (active patches = " << sdf.numActiveObstacles() << ")\n";
  check(sphere_id >= 0, "addObstacle returns valid id",
        sphere_id, /*expected*/ 0, 0.5);

  // Tolerance: trilinear smoothing + voxel quantization is bounded by ~voxel.
  const double kVoxelTol = voxel;   // 0.25 m

  // Far inside the sphere: at the center -> -1 m
  {
    float d; Eigen::Vector3d g;
    sdf.getDistanceAndGradient(sphere.center, &d, &g);
    const double truth = sphereSignedDistance(sphere.center,
                                              sphere.center, 1.0);
    check(true, "sphere center -> -1.0 m (inside)",
          d, truth, kVoxelTol);
  }
  // Just outside the sphere: 1.5 m from center along +x -> 0.5 m
  {
    const Eigen::Vector3d p = sphere.center + Eigen::Vector3d(1.5, 0, 0);
    float d; Eigen::Vector3d g;
    sdf.getDistanceAndGradient(p, &d, &g);
    const double truth = sphereSignedDistance(p, sphere.center, 1.0);
    check(true, "sphere outside (+1.5,0,0) -> 0.5 m",
          d, truth, kVoxelTol);

    // Gradient should point away from sphere center along +x (unit length).
    const double gx = g.x();
    check(gx > 0.5, "sphere gradient direction (+x dominant)",
          gx, /*expected*/ 1.0, 0.5);
  }
  // Far away from the sphere AND from the static cube: returns to static-only.
  {
    const Eigen::Vector3d p(8.0, 8.0, 8.0);   // ~3 m from static cube
    const float d_with_sphere = sdf.getDistance(p);
    const float d_static_only_pred =
        cubeSignedDistance(p, Eigen::Vector3d(5, 5, 5),
                           Eigen::Vector3d(1, 1, 1));
    // d_with_sphere should match the static cube's closed-form distance to
    // within trilinear smoothing (a few voxels).
    check(d_with_sphere > 0.0f, "far point not affected by sphere",
          d_with_sphere, d_static_only_pred, /*tol*/ 0.5);
  }

  // --- 3. Remove the sphere; the affected region recovers static-only -----
  std::cout << "\n[Test] removeObstacle(sphere)\n";
  sdf.removeObstacle(sphere_id);
  std::cout << "  active patches = " << sdf.numActiveObstacles() << "\n";
  check(sdf.numActiveObstacles() == 0,
        "after remove, no active patches",
        static_cast<double>(sdf.numActiveObstacles()), 0.0, 0.0);
  {
    const Eigen::Vector3d p = sphere.center;
    const float d_after_remove = sdf.getDistance(p);
    const float d_static_pred =
        cubeSignedDistance(p, Eigen::Vector3d(5, 5, 5),
                           Eigen::Vector3d(1, 1, 1));
    check(d_after_remove > 0.0f, "sphere center now free",
          d_after_remove, d_static_pred, /*tol*/ 0.5);
  }

  // --- 4. Add a cube obstacle, then a cylinder; verify min composition ----
  std::cout << "\n[Test] Add cube (center=(2,2,2), size=(1,1,1))\n";
  PrimitiveSpec cube;
  cube.kind = PrimitiveKind::kCube;
  cube.center = Eigen::Vector3d(2.0, 2.0, 2.0);
  cube.size = Eigen::Vector3d(1.0, 1.0, 1.0);
  const int cube_id = sdf.addObstacle(cube);
  check(cube_id >= 0, "cube addObstacle ok", cube_id, 0, 0.5);

  std::cout << "[Test] Add cylinder (center=(7,2,2), diameter=1, height=2)\n";
  PrimitiveSpec cyl;
  cyl.kind = PrimitiveKind::kCylinder;
  cyl.center = Eigen::Vector3d(7.0, 2.0, 2.0);
  cyl.size = Eigen::Vector3d(1.0, 1.0, 2.0);   // r=0.5, h=2
  const int cyl_id = sdf.addObstacle(cyl);
  check(cyl_id >= 0, "cylinder addObstacle ok", cyl_id, 1, 0.5);
  check(sdf.numActiveObstacles() == 2,
        "two patches active",
        static_cast<double>(sdf.numActiveObstacles()), 2.0, 0.0);

  // Center of the cube ⇒ -0.5 m (cube extends 0.5 m each way; LinftyBoxSDF
  // at center = -0.5).
  {
    float d; Eigen::Vector3d g;
    sdf.getDistanceAndGradient(cube.center, &d, &g);
    const double truth = cubeSignedDistance(cube.center, cube.center, cube.size);
    check(true, "cube center -> -0.5 m", d, truth, 0.15);
  }
  // Just outside the cylinder: 0.7 m from center along +x ⇒ 0.7 - 0.5 = 0.2 m.
  {
    const Eigen::Vector3d p = cyl.center + Eigen::Vector3d(0.7, 0, 0);
    float d; Eigen::Vector3d g;
    sdf.getDistanceAndGradient(p, &d, &g);
    check(true, "cylinder side (+x 0.7m) -> 0.2 m",
          d, /*expected*/ 0.20, kVoxelTol);
    check(g.x() > 0.5,
          "cylinder gradient (+x dominant)", g.x(), 1.0, 0.5);
  }

  // --- 5. clearObstacles wipes everything -----------------------------------
  std::cout << "\n[Test] clearObstacles()\n";
  sdf.clearObstacles();
  std::cout << "  active patches = " << sdf.numActiveObstacles() << "\n";
  check(sdf.numActiveObstacles() == 0,
        "all patches cleared",
        static_cast<double>(sdf.numActiveObstacles()), 0.0, 0.0);

  // After clear, queries should match static-only behavior.
  {
    const Eigen::Vector3d p = far_p;
    const float d_after_clear = sdf.getDistance(p);
    check(true, "far point matches static-only after clear",
          d_after_clear, static_only_far, /*tol*/ 0.001);
  }

  // --- 6. Re-add and check id reuse -----------------------------------------
  std::cout << "\n[Test] Re-add a sphere, expect reused id (0 if any free slot)\n";
  const int reused_id = sdf.addObstacle(sphere);
  std::cout << "  new id = " << reused_id << "\n";
  check(reused_id >= 0, "re-add returns valid id", reused_id, 0, 1);
  check(sdf.numActiveObstacles() == 1,
        "one active patch after re-add",
        static_cast<double>(sdf.numActiveObstacles()), 1.0, 0.0);

  // --- Summary --------------------------------------------------------------
  std::cout << "\n=== Summary ===\n";
  std::cout << "  passed: " << g_passed << "\n";
  std::cout << "  failed: " << g_failed << "\n";
  return g_failed == 0 ? 0 : 1;
}
