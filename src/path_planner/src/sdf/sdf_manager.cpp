// SDFManager: CPU-only signed Euclidean distance field using the
// Felzenszwalb-Huttenlocher separable transform with OpenMP parallelization,
// and a flat binary save/load format. No CUDA/NVBlox dependencies.

#include "path_planner/sdf/sdf_manager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace path_planner {
namespace sdf {

namespace {

// One-dimensional squared-distance transform (Felzenszwalb-Huttenlocher)
// with per-axis sample spacing h, so ANISOTROPIC voxels compose correctly
// across the three sweeps (values handed between sweeps are physical squared
// distances; parabola vertices/intersections live at physical q*h).
// f_get(q) returns the seed at grid index q (0 for obstacle, +inf otherwise);
// f_set(q, val) stores the squared distance.
inline void fillEDT1D(const std::function<double(int)>& f_get,
                      const std::function<void(int, double)>& f_set,
                      int start, int end, double h) {
  const int len = end - start + 1;
  // Use start-indexed arrays (matches the original Felzenszwalb-Huttenlocher
  // formulation from main/GridMap::fillESDF).
  std::vector<int> v(len + start);
  std::vector<double> z(len + start + 1);

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; ++q) {
    ++k;
    double s;
    const double qh = q * h;
    do {
      --k;
      const double vh = v[k] * h;
      double num = (f_get(q) + qh * qh) -
                   (f_get(v[k]) + vh * vh);
      double den = 2.0 * (qh - vh);
      s = num / den;
      // k > start guard: with sub-unit spacing h the intersection of a
      // finite parabola against an INF-seeded one can overflow to -inf,
      // which compares <= z[start] (-DBL_MAX) and underflows k right out
      // of the array (the isotropic h=1 code only survived because
      // den >= 2 kept -MAX/den > -MAX). Exiting at k = start with
      // s <= z[start] still yields the correct field: the dominated
      // vertex is skipped by the z-scan below.
    } while (k > start && s <= z[k]);
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;
  for (int q = start; q <= end; ++q) {
    while (z[k + 1] < q * h) ++k;
    double dq = (q - v[k]) * h;
    f_set(q, dq * dq + f_get(v[k]));
  }
}

}  // namespace

// Analytic dynamic obstacle: the primitive spec itself plus its inflated
// world AABB (primitive AABB + influence radius). No per-patch voxel grid —
// distance and gradient are evaluated in CLOSED FORM (primitiveSignedDistance),
// so box faces are exact instead of aliased to the global voxel: with 229 m
// xy voxels a 50-160 m box rasterized to 0-1 cells (±115 m geometry error,
// masked only by the 1 km front-end berth). Value and gradient come from one
// function — the cost/gradient-consistency rule the trilinear layer obeyed.
struct DynamicPatch {
  bool active = false;
  PrimitiveSpec spec;
  // Influence AABB in world coords; outside it the patch contributes nothing
  // (same "meaningful only within influenceRadius" semantic as the old grid).
  Eigen::Vector3d aabb_lo = Eigen::Vector3d::Zero();
  Eigen::Vector3d aabb_hi = Eigen::Vector3d::Zero();
};

struct SDFManagerImpl {
  Eigen::Vector3d voxel = Eigen::Vector3d::Zero();  // per-axis (vx, vy, vz)
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  int nx = 0, ny = 0, nz = 0;

  // Static-layer signed distance in meters. +inf for out-of-map queries.
  // Flat layout: ((x * ny) + y) * nz + z.
  std::vector<float> distance_cache;

  // Fully-free static layer (the common case now that terrain lives in the
  // 2.5D heightmap): no grid is allocated — queries return the scalar
  // static_free_dist instead of sampling. On a 300 km world the old
  // assign(N, kLargeFree) held 3.6 GB of a single constant.
  bool static_empty = false;
  float static_free_dist = std::numeric_limits<float>::infinity();

  bool initialized = false;
  bool has_data = false;

  // Dynamic obstacle layer.
  std::vector<DynamicPatch> patches;
  double influence_radius_m = 2.0;

  inline size_t flatIdx(int xi, int yi, int zi) const {
    return ((size_t(xi) * ny) + yi) * nz + zi;
  }

  inline Eigen::Vector3d worldToVoxelF(const Eigen::Vector3d& p) const {
    return (p - origin).cwiseQuotient(voxel);
  }
};

SDFManager::SDFManager() : impl_(std::make_unique<SDFManagerImpl>()) {}
SDFManager::~SDFManager() = default;

bool SDFManager::initialize(double voxel_size) {
  return initialize(voxel_size, voxel_size);
}

bool SDFManager::initialize(double voxel_xy, double voxel_z) {
  if (voxel_xy <= 0.0 || voxel_z <= 0.0) {
    std::cerr << "[SDFManager] invalid voxel sizes " << voxel_xy << ", "
              << voxel_z << "\n";
    return false;
  }
  impl_->voxel = Eigen::Vector3d(voxel_xy, voxel_xy, voxel_z);
  impl_->initialized = true;
  impl_->has_data = false;
  impl_->distance_cache.clear();
  return true;
}

bool SDFManager::buildFromVoxels(const uint8_t* occupancy,
                                  int nx, int ny, int nz,
                                  const Eigen::Vector3d& origin) {
  ++revision_;
  if (!impl_->initialized) {
    std::cerr << "[SDFManager] buildFromVoxels: not initialized\n";
    return false;
  }
  if (!occupancy || nx <= 0 || ny <= 0 || nz <= 0) {
    std::cerr << "[SDFManager] buildFromVoxels: invalid input\n";
    return false;
  }

  impl_->origin = origin;
  impl_->nx = nx;
  impl_->ny = ny;
  impl_->nz = nz;

  const size_t N = size_t(nx) * ny * nz;
  const Eigen::Vector3d res = impl_->voxel;
  // Large-but-finite seed: far above any real squared distance on the grid,
  // yet safe against overflow inside the parabola intersection (DBL_MAX
  // seeds divided by a sub-unit den overflow to -inf; see fillEDT1D).
  const double INF = 1e30;

  auto idx = [nx_ = size_t(nx), ny_ = size_t(ny), nz_ = size_t(nz)]
             (int x, int y, int z) {
    return ((size_t(x) * ny_) + y) * nz_ + z;
  };

  // Special case: no obstacles at all. fillEDT1D with all-INF seeds produces
  // NaN (inf - inf in the parabola intersection). Fill the whole cache with
  // a large finite free-distance value and return.
  bool any_occupied = false;
  for (size_t i = 0; i < N; ++i) {
    if (occupancy[i] != 0) { any_occupied = true; break; }
  }
  if (!any_occupied) {
    const float kLargeFree = static_cast<float>(
        std::max({nx * res.x(), ny * res.y(), nz * res.z()}));
    // Allocation-free: keep only the scalar. Queries short-circuit on
    // static_empty (see getDistance / getDistanceAndGradient).
    impl_->distance_cache.clear();
    impl_->distance_cache.shrink_to_fit();
    impl_->static_empty = true;
    impl_->static_free_dist = kLargeFree;
    impl_->has_data = true;
    std::cerr << "[SDFManager] built (no obstacles): shape=(" << nx << ","
              << ny << "," << nz << ") voxel=(" << res.x() << "," << res.y()
              << "," << res.z() << ") static layer EMPTY (0 bytes),"
              << " free_distance=" << kLargeFree << "\n";
    return true;
  }
  impl_->static_empty = false;

  // Positive DT on the occupied set (obstacles = 0, free = +inf).
  std::vector<double> tmp1(N), tmp2(N);
  std::vector<double> d_pos(N);

#ifdef _OPENMP
  const bool use_par = (N > 10000);
#else
  const bool use_par = false;
#endif

  // Sweep Z
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) {
      fillEDT1D(
          [&](int z) { return occupancy[idx(x, y, z)] != 0 ? 0.0 : INF; },
          [&](int z, double v) { tmp1[idx(x, y, z)] = v; },
          0, nz - 1, res.z());
    }
  }
  // Sweep Y
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int y) { return tmp1[idx(x, y, z)]; },
          [&](int y, double v) { tmp2[idx(x, y, z)] = v; },
          0, ny - 1, res.y());
    }
  }
  // Sweep X (final, take sqrt in meters)
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int y = 0; y < ny; ++y) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int x) { return tmp2[idx(x, y, z)]; },
          [&](int x, double v) {
            d_pos[idx(x, y, z)] = std::sqrt(v);
          },
          0, nx - 1, res.x());
    }
  }

  // Negative DT on the complement (free voxels become obstacles).
  std::vector<double> d_neg(N);
#pragma omp parallel for schedule(static) if(use_par)
  for (size_t i = 0; i < N; ++i) tmp1[i] = 0.0;
#pragma omp parallel for schedule(static) if(use_par)
  for (size_t i = 0; i < N; ++i) tmp2[i] = 0.0;

#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) {
      fillEDT1D(
          [&](int z) { return occupancy[idx(x, y, z)] == 0 ? 0.0 : INF; },
          [&](int z, double v) { tmp1[idx(x, y, z)] = v; },
          0, nz - 1, res.z());
    }
  }
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int y) { return tmp1[idx(x, y, z)]; },
          [&](int y, double v) { tmp2[idx(x, y, z)] = v; },
          0, ny - 1, res.y());
    }
  }
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int y = 0; y < ny; ++y) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int x) { return tmp2[idx(x, y, z)]; },
          [&](int x, double v) {
            d_neg[idx(x, y, z)] = std::sqrt(v);
          },
          0, nx - 1, res.x());
    }
  }

  // Signed distance: positive outside obstacles, negative inside.
  // For a free voxel: d_pos > 0, d_neg = 0 -> signed = +d_pos.
  // For an obstacle:  d_pos = 0, d_neg > 0 -> signed = -d_neg.
  impl_->distance_cache.assign(N, std::numeric_limits<float>::infinity());
#pragma omp parallel for schedule(static) if(use_par)
  for (size_t i = 0; i < N; ++i) {
    double signed_d = (occupancy[i] != 0) ? -d_neg[i] : d_pos[i];
    impl_->distance_cache[i] = static_cast<float>(signed_d);
  }

  impl_->has_data = true;
  std::cerr << "[SDFManager] built: shape=(" << nx << "," << ny << "," << nz
            << ") voxel=(" << res.x() << "," << res.y() << "," << res.z()
            << ") voxels=" << N << "\n";
  return true;
}

namespace {

// Single-file binary format: magic + header + flat float distance array.
// v1: uint32 magic, uint32 version=1, double voxel_size (isotropic),
//     double origin_xyz[3], int32 nx,ny,nz
// v2: uint32 magic, uint32 version=2, double voxel_x,y,z (ANISOTROPIC),
//     double origin_xyz[3], int32 nx,ny,nz
// v1 files load as isotropic (vx=vy=vz); saving always writes v2.
struct EsdfHeaderV1 {
  uint32_t magic = 0x4D455344;  // "MESD"
  uint32_t version = 1;
  double voxel_size = 0.0;
  double origin_x = 0.0, origin_y = 0.0, origin_z = 0.0;
  int32_t nx = 0, ny = 0, nz = 0;
};
struct EsdfHeaderV2 {
  uint32_t magic = 0x4D455344;  // "MESD"
  uint32_t version = 2;
  double voxel_x = 0.0, voxel_y = 0.0, voxel_z = 0.0;
  double origin_x = 0.0, origin_y = 0.0, origin_z = 0.0;
  int32_t nx = 0, ny = 0, nz = 0;
};

}  // namespace

bool SDFManager::saveToFile(const std::string& path) const {
  if (!impl_->initialized || !impl_->has_data) {
    std::cerr << "[SDFManager] saveToFile: no data\n";
    return false;
  }
  if (impl_->static_empty) {
    // Nothing worth persisting: the layer is one scalar. Refusing here also
    // prevents recreating multi-GB constant-value cache files.
    std::cerr << "[SDFManager] saveToFile: static layer empty — nothing to cache\n";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    std::cerr << "[SDFManager] saveToFile: cannot open " << path << "\n";
    return false;
  }

  EsdfHeaderV2 h;
  h.voxel_x = impl_->voxel.x();
  h.voxel_y = impl_->voxel.y();
  h.voxel_z = impl_->voxel.z();
  h.origin_x = impl_->origin.x();
  h.origin_y = impl_->origin.y();
  h.origin_z = impl_->origin.z();
  h.nx = impl_->nx;
  h.ny = impl_->ny;
  h.nz = impl_->nz;
  f.write(reinterpret_cast<const char*>(&h), sizeof(h));

  const size_t N = size_t(h.nx) * h.ny * h.nz;
  f.write(reinterpret_cast<const char*>(impl_->distance_cache.data()),
          N * sizeof(float));
  if (!f) {
    std::cerr << "[SDFManager] saveToFile: write failed\n";
    return false;
  }

  std::cerr << "[SDFManager] saved " << path << " shape=(" << h.nx << ","
            << h.ny << "," << h.nz << ") voxel=(" << h.voxel_x << ","
            << h.voxel_y << "," << h.voxel_z << ")"
            << " bytes=" << (sizeof(h) + N * sizeof(float)) << "\n";
  return true;
}

bool SDFManager::loadFromFile(const std::string& path,
                               const Eigen::Vector3d& /*bbox_lo*/,
                               const Eigen::Vector3d& /*bbox_hi*/) {
  ++revision_;
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    std::cerr << "[SDFManager] loadFromFile: cannot open " << path << "\n";
    return false;
  }

  uint32_t magic = 0, version = 0;
  f.read(reinterpret_cast<char*>(&magic), sizeof(magic));
  f.read(reinterpret_cast<char*>(&version), sizeof(version));
  if (!f || magic != 0x4D455344 || (version != 1 && version != 2)) {
    std::cerr << "[SDFManager] loadFromFile: bad header (magic=" << std::hex
              << magic << std::dec << ", version=" << version << ")\n";
    return false;
  }
  Eigen::Vector3d voxel, origin;
  int32_t nx_h = 0, ny_h = 0, nz_h = 0;
  f.seekg(0);
  if (version == 1) {
    EsdfHeaderV1 h;
    f.read(reinterpret_cast<char*>(&h), sizeof(h));
    voxel = Eigen::Vector3d(h.voxel_size, h.voxel_size, h.voxel_size);
    origin = Eigen::Vector3d(h.origin_x, h.origin_y, h.origin_z);
    nx_h = h.nx; ny_h = h.ny; nz_h = h.nz;
  } else {
    EsdfHeaderV2 h;
    f.read(reinterpret_cast<char*>(&h), sizeof(h));
    voxel = Eigen::Vector3d(h.voxel_x, h.voxel_y, h.voxel_z);
    origin = Eigen::Vector3d(h.origin_x, h.origin_y, h.origin_z);
    nx_h = h.nx; ny_h = h.ny; nz_h = h.nz;
  }
  if (!f || nx_h <= 0 || ny_h <= 0 || nz_h <= 0 || voxel.minCoeff() <= 0.0) {
    std::cerr << "[SDFManager] loadFromFile: invalid dims/voxel\n";
    return false;
  }

  // A cache built with different voxel sizes than the caller requested is
  // still self-consistent — adopt its sizes (callers compare via voxelSizes()
  // and rebuild if they need something else).
  if (!impl_->initialized || (impl_->voxel - voxel).cwiseAbs().maxCoeff() > 1e-9) {
    initialize(voxel.x(), voxel.z());
    impl_->voxel = voxel;
  }

  impl_->origin = origin;
  impl_->nx = nx_h;
  impl_->ny = ny_h;
  impl_->nz = nz_h;

  const size_t N = size_t(nx_h) * ny_h * nz_h;
  impl_->distance_cache.assign(N, std::numeric_limits<float>::infinity());
  f.read(reinterpret_cast<char*>(impl_->distance_cache.data()),
         N * sizeof(float));
  if (!f) {
    std::cerr << "[SDFManager] loadFromFile: read failed (expected "
              << (N * sizeof(float)) << " bytes)\n";
    return false;
  }

  impl_->static_empty = false;  // a real grid is resident now
  impl_->has_data = true;
  std::cerr << "[SDFManager] loaded " << path << " (v" << version
            << ") shape=(" << nx_h << "," << ny_h << "," << nz_h
            << ") voxel=(" << voxel.x() << "," << voxel.y() << ","
            << voxel.z() << ")\n";
  return true;
}

// Voxel (i,j,k) sample is located at voxel CENTER in world coords:
//   p_center(i,j,k) = origin + (i + 0.5, j + 0.5, k + 0.5) * voxel_size
// Trilinear interpolation blends the 8 surrounding centers. Falling back to
// nearest-voxel lookup produced a piecewise-constant distance field with zero
// interior gradient, which prevented L-BFGS from climbing out of obstacles.
namespace {
inline int clampIdx(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}


// Trilinear-sample a dense distance grid `cache` of shape (nx,ny,nz) at
// voxel-fractional coordinates vf (already shifted by -0.5 for cell centers).
// Returns +inf if the floor index is fully outside the grid by more than 1.
// On success: writes distance and (optionally) world-frame gradient.
template <bool WantGrad>
inline bool sampleTrilinear(const float* cache, int nx, int ny, int nz,
                            const Eigen::Vector3d& voxel,
                            const Eigen::Vector3d& vf,
                            float* out_d,
                            Eigen::Vector3d* out_grad) {
  const int xi0 = int(std::floor(vf.x()));
  const int yi0 = int(std::floor(vf.y()));
  const int zi0 = int(std::floor(vf.z()));
  if (xi0 < -1 || yi0 < -1 || zi0 < -1 ||
      xi0 >= nx || yi0 >= ny || zi0 >= nz) {
    return false;
  }
  const double tx = vf.x() - xi0;
  const double ty = vf.y() - yi0;
  const double tz = vf.z() - zi0;
  const int xi1 = clampIdx(xi0 + 1, 0, nx - 1);
  const int yi1 = clampIdx(yi0 + 1, 0, ny - 1);
  const int zi1 = clampIdx(zi0 + 1, 0, nz - 1);
  const int xi0c = clampIdx(xi0, 0, nx - 1);
  const int yi0c = clampIdx(yi0, 0, ny - 1);
  const int zi0c = clampIdx(zi0, 0, nz - 1);

  auto At = [&](int x, int y, int z) -> double {
    return cache[((size_t(x) * ny) + y) * nz + z];
  };
  const double c000 = At(xi0c, yi0c, zi0c);
  const double c100 = At(xi1,  yi0c, zi0c);
  const double c010 = At(xi0c, yi1,  zi0c);
  const double c110 = At(xi1,  yi1,  zi0c);
  const double c001 = At(xi0c, yi0c, zi1 );
  const double c101 = At(xi1,  yi0c, zi1 );
  const double c011 = At(xi0c, yi1,  zi1 );
  const double c111 = At(xi1,  yi1,  zi1 );

  const double c00 = c000 * (1.0 - tx) + c100 * tx;
  const double c10 = c010 * (1.0 - tx) + c110 * tx;
  const double c01 = c001 * (1.0 - tx) + c101 * tx;
  const double c11 = c011 * (1.0 - tx) + c111 * tx;
  const double c0  = c00  * (1.0 - ty) + c10  * ty;
  const double c1  = c01  * (1.0 - ty) + c11  * ty;
  const double d   = c0   * (1.0 - tz) + c1  * tz;
  if (out_d) *out_d = static_cast<float>(d);

  if (WantGrad && out_grad) {
    const double dc00_dtx = c100 - c000;
    const double dc10_dtx = c110 - c010;
    const double dc01_dtx = c101 - c001;
    const double dc11_dtx = c111 - c011;
    const double dc0_dtx  = dc00_dtx * (1.0 - ty) + dc10_dtx * ty;
    const double dc1_dtx  = dc01_dtx * (1.0 - ty) + dc11_dtx * ty;
    const double dD_dtx   = dc0_dtx * (1.0 - tz) + dc1_dtx * tz;
    const double dc0_dty  = c10 - c00;
    const double dc1_dty  = c11 - c01;
    const double dD_dty   = dc0_dty * (1.0 - tz) + dc1_dty * tz;
    const double dD_dtz   = c1 - c0;
    (*out_grad) << dD_dtx / voxel.x(), dD_dty / voxel.y(), dD_dtz / voxel.z();
  }
  return true;
}

// Sample a dynamic patch at world position p. The patch occupies voxel-index
// (Dynamic patches are analytic now; see primitiveSignedDistance and the new
// samplePatch below primitiveAabb.)

// Voxel test for a primitive (in world frame).
// Exact signed distance (and its analytic gradient) of a primitive, in the
// WORLD frame. Negative inside. The gradient is the true derivative of the
// returned value (unit outward normal; a subgradient on the measure-zero
// edge/corner/axis sets) — value and gradient always form a consistent pair.
// kCylinder assumes a CIRCULAR cross-section (all in-tree cylinders are;
// an elliptical exact SDF has no closed form — the mean radius is used).
inline float primitiveSignedDistance(const PrimitiveSpec& s,
                                     const Eigen::Vector3d& p,
                                     Eigen::Vector3d* out_grad) {
  Eigen::Vector3d d = p - s.center;
  const bool rotated = (s.yaw != 0.0 && s.kind != PrimitiveKind::kSphere);
  double cy = 1.0, sy = 0.0;
  if (rotated) {
    cy = std::cos(s.yaw);
    sy = std::sin(s.yaw);
    const double bx =  cy * d.x() + sy * d.y();
    const double by = -sy * d.x() + cy * d.y();
    d.x() = bx;
    d.y() = by;
  }

  double dist = 0.0;
  Eigen::Vector3d g = Eigen::Vector3d::Zero();  // gradient in the local frame
  switch (s.kind) {
    case PrimitiveKind::kCube: {
      const Eigen::Vector3d h = 0.5 * s.size;
      const Eigen::Vector3d q = d.cwiseAbs() - h;
      const Eigen::Vector3d qpos = q.cwiseMax(0.0);
      const double outside = qpos.norm();
      if (outside > 1e-12) {
        dist = outside;
        g = qpos / outside;
      } else {
        // Inside: distance to (and normal of) the nearest face.
        int ax = 0;
        dist = q.maxCoeff(&ax);
        g[ax] = 1.0;
      }
      // Undo the |d| folding: the gradient picks up sign(d) per axis.
      for (int i = 0; i < 3; ++i) g[i] *= (d[i] >= 0.0 ? 1.0 : -1.0);
      break;
    }
    case PrimitiveKind::kCylinder: {
      const double r  = 0.25 * (s.size.x() + s.size.y());  // = radius when circular
      const double hz = 0.5 * s.size.z();
      const double rho = std::hypot(d.x(), d.y());
      const double qr = rho - r;
      const double qz = std::abs(d.z()) - hz;
      const double or_ = std::max(qr, 0.0), oz = std::max(qz, 0.0);
      const double outside = std::hypot(or_, oz);
      double gr, gz;  // gradient in (radial, |z|) space
      if (outside > 1e-12) {
        dist = outside;
        gr = or_ / outside;
        gz = oz / outside;
      } else {
        if (qr > qz) { dist = qr; gr = 1.0; gz = 0.0; }
        else         { dist = qz; gr = 0.0; gz = 1.0; }
      }
      const double inv_rho = (rho > 1e-12) ? 1.0 / rho : 0.0;
      g = Eigen::Vector3d(gr * d.x() * inv_rho,
                          gr * d.y() * inv_rho,
                          gz * (d.z() >= 0.0 ? 1.0 : -1.0));
      break;
    }
    case PrimitiveKind::kSphere: {
      const double r = 0.5 * s.size.x();   // uniform diameter
      const double n = d.norm();
      dist = n - r;
      g = (n > 1e-12) ? Eigen::Vector3d(d / n) : Eigen::Vector3d(0, 0, 1);
      break;
    }
  }

  if (out_grad) {
    if (rotated) {  // rotate the local gradient back by +yaw
      out_grad->x() = cy * g.x() - sy * g.y();
      out_grad->y() = sy * g.x() + cy * g.y();
      out_grad->z() = g.z();
    } else {
      *out_grad = g;
    }
  }
  return static_cast<float>(dist);
}

// AABB of the primitive's bounding box in world coords. For yawed primitives
// this is the AABB of the ROTATED footprint (conservative for cylinders).
inline void primitiveAabb(const PrimitiveSpec& s,
                          Eigen::Vector3d* lo, Eigen::Vector3d* hi) {
  Eigen::Vector3d half = 0.5 * s.size;
  if (s.yaw != 0.0 && s.kind != PrimitiveKind::kSphere) {
    const double c = std::abs(std::cos(s.yaw)), sn = std::abs(std::sin(s.yaw));
    const double hx = c * half.x() + sn * half.y();
    const double hy = sn * half.x() + c * half.y();
    half.x() = hx;
    half.y() = hy;
  }
  *lo = s.center - half;
  *hi = s.center + half;
}

// Evaluate a dynamic patch at world point p: closed-form signed distance and
// gradient of the primitive itself. Contributes nothing outside the
// influence AABB (early-out; keeps getDynamicDistance's "only meaningful
// within influenceRadius" semantic).
inline bool samplePatch(const DynamicPatch& patch,
                        const Eigen::Vector3d& p,
                        float* out_d,
                        Eigen::Vector3d* out_grad) {
  if (!patch.active) return false;
  if ((p.array() < patch.aabb_lo.array()).any() ||
      (p.array() > patch.aabb_hi.array()).any()) {
    return false;
  }
  *out_d = primitiveSignedDistance(patch.spec, p, out_grad);
  return true;
}
}  // namespace

float SDFManager::getDistance(const Eigen::Vector3d& pos) const {
  if (!impl_->initialized || !impl_->has_data) {
    return std::numeric_limits<float>::infinity();
  }
  const Eigen::Vector3d vf = impl_->worldToVoxelF(pos) -
                             Eigen::Vector3d(0.5, 0.5, 0.5);

  // Static layer. Empty (all-free) layer holds no grid — use the scalar so
  // the front-end's "non-finite = outside map = blocked" rule never fires
  // in free space.
  float best = std::numeric_limits<float>::infinity();
  if (impl_->static_empty) {
    best = impl_->static_free_dist;
  } else {
    float d_static;
    if (sampleTrilinear<false>(impl_->distance_cache.data(),
                               impl_->nx, impl_->ny, impl_->nz,
                               impl_->voxel, vf, &d_static, nullptr)) {
      best = d_static;
    }
  }

  // Dynamic layer: min over patches that contain pos.
  for (const auto& patch : impl_->patches) {
    float d_p;
    if (samplePatch(patch, pos, &d_p, nullptr)) {
      if (d_p < best) best = d_p;
    }
  }
  return best;
}

float SDFManager::getDynamicDistance(const Eigen::Vector3d& pos) const {
  // Patch-only distance (no static terrain). +inf outside every patch AABB,
  // so it is only meaningful within influenceRadius() of an obstacle — which
  // is exactly the range a stand-off margin needs.
  float best = std::numeric_limits<float>::infinity();
  if (!impl_->initialized) return best;
  for (const auto& patch : impl_->patches) {
    float d_p;
    if (samplePatch(patch, pos, &d_p, nullptr)) {
      if (d_p < best) best = d_p;
    }
  }
  return best;
}

bool SDFManager::getDistanceAndGradient(const Eigen::Vector3d& pos,
                                         float* distance,
                                         Eigen::Vector3d* gradient) const {
  if (distance) *distance = std::numeric_limits<float>::infinity();
  if (gradient) gradient->setZero();

  if (!impl_->initialized || !impl_->has_data) return false;

  const Eigen::Vector3d vf = impl_->worldToVoxelF(pos) -
                             Eigen::Vector3d(0.5, 0.5, 0.5);

  // Take the layer that produces the minimum signed distance. Its gradient
  // is the sub-gradient of min(...) and is what L-BFGS expects.
  float best = std::numeric_limits<float>::infinity();
  Eigen::Vector3d best_grad = Eigen::Vector3d::Zero();
  bool any = false;

  if (impl_->static_empty) {
    // All-free static layer: scalar distance, zero gradient (no grid).
    best = impl_->static_free_dist;
    best_grad = Eigen::Vector3d::Zero();
    any = true;
  } else {
    float d_static;
    Eigen::Vector3d g_static;
    if (sampleTrilinear<true>(impl_->distance_cache.data(),
                              impl_->nx, impl_->ny, impl_->nz,
                              impl_->voxel, vf, &d_static, &g_static)) {
      best = d_static;
      best_grad = g_static;
      any = true;
    }
  }

  for (const auto& patch : impl_->patches) {
    float d_p;
    Eigen::Vector3d g_p;
    if (samplePatch(patch, pos, &d_p, &g_p)) {
      if (!any || d_p < best) {
        best = d_p;
        best_grad = g_p;
        any = true;
      }
    }
  }

  if (!any) return false;
  if (distance) *distance = best;
  if (gradient) *gradient = best_grad;
  return true;
}

bool SDFManager::isInitialized() const { return impl_->initialized; }
bool SDFManager::hasData() const { return impl_->has_data; }
double SDFManager::voxelSize() const { return impl_->voxel.x(); }
Eigen::Vector3d SDFManager::voxelSizes() const { return impl_->voxel; }

size_t SDFManager::numAllocatedBlocks() const {
  // Retained for API compatibility. No block concept here; report voxel count.
  return impl_->distance_cache.size();
}

Eigen::Vector3i SDFManager::shape() const {
  return Eigen::Vector3i(impl_->nx, impl_->ny, impl_->nz);
}

Eigen::Vector3d SDFManager::origin() const {
  return impl_->origin;
}

// ------------------ dynamic obstacle layer ------------------

void SDFManager::setInfluenceRadius(double r) {
  impl_->influence_radius_m = (r > 0.0) ? r : 0.0;
}

double SDFManager::influenceRadius() const {
  return impl_->influence_radius_m;
}

int SDFManager::addObstacle(const PrimitiveSpec& spec) {
  ++revision_;
  if (!impl_->initialized || !impl_->has_data) {
    std::cerr << "[SDFManager] addObstacle: static layer not built\n";
    return -1;
  }

  // Analytic patch: store the primitive itself plus its influence AABB.
  // No rasterization, no local EDT — queries evaluate the closed-form
  // signed distance, so geometry is exact regardless of voxel size (the
  // grid-based patch aliased sub-voxel boxes by up to half a cell).
  DynamicPatch patch;
  patch.spec = spec;
  Eigen::Vector3d wlo, whi;
  primitiveAabb(spec, &wlo, &whi);
  const Eigen::Vector3d inflate =
      Eigen::Vector3d::Constant(impl_->influence_radius_m);
  patch.aabb_lo = wlo - inflate;
  patch.aabb_hi = whi + inflate;
  patch.active = true;

  // Reuse a freed slot if any to keep ids dense.
  for (size_t i = 0; i < impl_->patches.size(); ++i) {
    if (!impl_->patches[i].active) {
      impl_->patches[i] = std::move(patch);
      return static_cast<int>(i);
    }
  }
  impl_->patches.push_back(std::move(patch));
  return static_cast<int>(impl_->patches.size() - 1);
}

void SDFManager::removeObstacle(int patch_id) {
  ++revision_;
  if (patch_id < 0 ||
      static_cast<size_t>(patch_id) >= impl_->patches.size()) return;
  impl_->patches[patch_id].active = false;
}

void SDFManager::clearObstacles() {
  ++revision_;
  impl_->patches.clear();
}

size_t SDFManager::numActiveObstacles() const {
  size_t n = 0;
  for (const auto& p : impl_->patches) {
    if (p.active) ++n;
  }
  return n;
}

}  // namespace sdf
}  // namespace path_planner
