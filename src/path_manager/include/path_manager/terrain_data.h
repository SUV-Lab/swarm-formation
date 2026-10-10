#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace path_manager {
  // Terrain elevation data extracted from GridMap
  // [TERRAIN-FRAME] grid_map core convention (what the RViz plugin renders):
  //   wx = origin_x + length_x - (row + 0.5) * res   (matrix ROW spans X, mirrored)
  //   wy = origin_y + length_y - (col + 0.5) * res   (matrix COL spans Y, mirrored)
  // The historical form here ("X-mirror then -90° rotation about the centre")
  // composes to wx = (Lx+Ly)/2 - (row+.5)res, wy = (Lx+Ly)/2 - (col+.5)res —
  // identical to the above ONLY when length_x == length_y. All maps preceding
  // full_map were square, so the planner agreed with the rendered mesh by
  // luck; on full_map (2991x4478) the two frames diverged by
  // (Ly-Lx)/2 = 1858.75 u per
  // axis (verified: the DEM peak rendered at (6263.75, 7443.75) while the
  // planner placed it at (8122.50, 5585.00)). Direct per-axis mirrors below.
  struct TerrainData {
    std::vector<float> elevation;  // Column-major elevation data
    int cols = 0;
    int rows = 0;
    double resolution = 0.0;
    double origin_x = 0.0;       // Terrain grid origin X
    double origin_y = 0.0;       // Terrain grid origin Y
    double length_x = 0.0;       // Terrain total length X
    double length_y = 0.0;       // Terrain total length Y
    bool valid = false;
    // Bumped on every setTerrainData ingest. The ELEV-MEMO keys on this, NOT
    // the elevation buffer address: a same-geometry corridor re-crop copies
    // new values into the SAME std::vector buffer (libstdc++ reuses it when
    // the size fits capacity), so a data()-pointer key would hit and serve
    // the previous crop's stale sample.
    uint64_t generation = 0;

    // Convert world (planning) coordinate to terrain grid index and query elevation
    float getElevation(double world_x, double world_y) const {
      if (!valid) return -std::numeric_limits<float>::infinity();

      // [TERRAIN-FRAME] inverse: pure per-axis mirror (row <-> X, col <-> Y).
      double terrain_world_x = origin_x + (origin_y + length_y - world_y);
      double terrain_world_y = origin_y + (origin_x + length_x - world_x);

      // World coord → fractional grid coordinate (same fx/fy convention
      // as the RViz panel's getElevationAt) then BILINEAR sample.
      // CELL-CENTER convention (the -0.5): elevation[col*rows+row] is the
      // height AT the cell centre — that is where terrainToWorld places it
      // and where the grid_map_rviz_plugin renders its mesh vertex. Without
      // the shift the bilinear nodes sit on cell corners, displacing the
      // whole sampled surface by +half a cell (~114 m at 2.288-unit DEM
      // cells) in BOTH world axes relative to the rendered terrain: every
      // consumer (FM2 occupancy, optimizer terrain term, panel) shared the
      // shift, so layers agreed with each other but not with the mesh.
      // WHY bilinear (was nearest-cell): the SDF the optimizer plans against is
      // voxelised from THIS function; nearest-cell made it a piecewise-constant,
      // ~cell-coarse (~230 m DEM cell) terrain, so a trajectory that only grazed
      // that blocky surface appeared to overlap the finer bilinear terrain
      // the altitude/clearance panels display. Interpolating here makes the
      // optimizer's terrain match the panels' terrain, so grazes stop reading as
      // pass-throughs. NaN water is blended with sea level; at the geometric
      // map boundary the outer stencil sample is clamped to the nearest DEM
      // cell so a cropped land edge does not become a fabricated downhill.
      constexpr float kInv = -std::numeric_limits<float>::infinity();
      const double fx = (terrain_world_x - origin_x) / resolution - 0.5;
      const double fy = (terrain_world_y - origin_y) / resolution - 0.5;
      // The raster represents the physical rectangle extending half a cell
      // beyond its outer cell centres. Queries outside that rectangle have no
      // terrain; queries within its half-cell edge use replicated-border
      // bilinear interpolation.
      if (fx < -0.5 || fx > static_cast<double>(cols) - 0.5 ||
          fy < -0.5 || fy > static_cast<double>(rows) - 0.5) {
        return kInv;
      }
      const int col0 = static_cast<int>(std::floor(fx));
      const int row0 = static_cast<int>(std::floor(fy));

      auto at = [&](int c, int r) -> float {
        const int cc = std::clamp(c, 0, cols - 1);
        const int rr = std::clamp(r, 0, rows - 1);
        const int index = cc * rows + rr;  // Column-major
        if (index < 0 || index >= static_cast<int>(elevation.size())) return kInv;
        const float e = elevation[index];
        return std::isnan(e) ? kInv : e;
      };

      const float e00 = at(col0, row0),     e10 = at(col0 + 1, row0);
      const float e01 = at(col0, row0 + 1), e11 = at(col0 + 1, row0 + 1);
      // Blend NaN water corners as SEA LEVEL 0 — exactly like the RViz panel's
      // sampler — so the surface is C0-continuous across coastlines. Pure
      // water (all four invalid) stays -inf, preserving "water = no terrain"
      // semantics for the front end and SDF construction.
      const bool n00 = (e00 == kInv), n10 = (e10 == kInv);
      const bool n01 = (e01 == kInv), n11 = (e11 == kInv);
      if (n00 && n10 && n01 && n11) return kInv;
      const double f00 = n00 ? 0.0 : e00;
      const double f10 = n10 ? 0.0 : e10;
      const double f01 = n01 ? 0.0 : e01;
      const double f11 = n11 ? 0.0 : e11;
      const double tx = fx - col0, ty = fy - row0;
      return static_cast<float>(
          (1.0 - tx) * (1.0 - ty) * f00 + tx * (1.0 - ty) * f10 +
          (1.0 - tx) * ty         * f01 + tx * ty         * f11);
    }

    // Bilinear elevation + the ANALYTIC gradient of that same 0-blended
    // surface, in WORLD (planning) coordinates. The optimizer's terrain term
    // needs cost and gradient from the SAME surface: a smoothed central-diff
    // slope estimate disagrees with the bilinear cost near DEM-cell edges,
    // and on steep (cliff) cells the mismatch kills the L-BFGS line search
    // (-1008 rounding error at near-optimal points). Analytic-per-patch
    // gradients are the same smoothness class the well-validated SDF term had
    // (trilinear value + its exact gradient). Returns false over pure water /
    // outside the DEM (no terrain there).
    bool getElevationAndGrad(double world_x, double world_y,
                             float *h, float *dhdx, float *dhdy) const {
      // [ELEV-MEMO] 1-entry memo. The optimizer evaluates the SAME constraint
      // point consecutively for the terrain term + every risk zone's grounded
      // clamp (5 zones -> up to 6 identical (x,y) queries back-to-back);
      // measured 88% of the per-iteration cost was zone evaluation, most of
      // it these repeats. Same inputs return the STORED outputs verbatim, so
      // results are bit-identical by construction. Keyed on the terrain
      // GENERATION (bumped every ingest): a same-geometry re-crop copies new
      // values into the same buffer, so a data()-pointer key would wrongly
      // hit and serve a stale sample. thread_local: the optimizer runs on one
      // thread; other threads just get their own slot.
      struct ElevMemo {
        uint64_t identity, gen;
        bool valid;
        double x, y;
        float h, gx, gy;
        bool ok;
      };
      static thread_local ElevMemo memo{
          0, std::numeric_limits<uint64_t>::max(), false,
          0.0, 0.0, 0.f, 0.f, 0.f, false};
      if (memo.identity == cache_identity_.value && memo.gen == generation &&
          memo.valid == valid && memo.x == world_x && memo.y == world_y) {
        *h = memo.h; *dhdx = memo.gx; *dhdy = memo.gy;
        return memo.ok;
      }
      float th = 0.f, tgx = 0.f, tgy = 0.f;
      const bool ok = getElevationAndGradUncached(world_x, world_y, &th, &tgx, &tgy);
      memo = ElevMemo{cache_identity_.value, generation, valid,
                      world_x, world_y, th, tgx, tgy, ok};
      *h = th; *dhdx = tgx; *dhdy = tgy;
      return ok;
    }

    bool getElevationAndGradUncached(double world_x, double world_y,
                                     float *h, float *dhdx, float *dhdy) const {
      if (!valid) return false;
      // [TERRAIN-FRAME] world -> terrain-grid coords (same transform as
      // getElevation): twx = oy + Ly - wy (col axis), twy = ox + Lx - wx (row
      // axis). Axis pairing (col<->Y, row<->X, both mirrored) is unchanged
      // from the old rotate+mirror form, so the chain rule below still holds.
      const double twx = origin_x + (origin_y + length_y - world_y);
      const double twy = origin_y + (origin_x + length_x - world_x);
      // Cell-centre convention (-0.5), same as getElevation — cost and
      // gradient must come from the SAME surface.
      const double fx = (twx - origin_x) / resolution - 0.5;
      const double fy = (twy - origin_y) / resolution - 0.5;
      if (fx < -0.5 || fx > static_cast<double>(cols) - 0.5 ||
          fy < -0.5 || fy > static_cast<double>(rows) - 0.5) {
        return false;
      }
      const int col0 = static_cast<int>(std::floor(fx));
      const int row0 = static_cast<int>(std::floor(fy));
      constexpr float kInv = -std::numeric_limits<float>::infinity();
      auto at = [&](int c, int r) -> float {
        const int cc = std::clamp(c, 0, cols - 1);
        const int rr = std::clamp(r, 0, rows - 1);
        const int index = cc * rows + rr;
        if (index < 0 || index >= static_cast<int>(elevation.size())) return kInv;
        const float e = elevation[index];
        return std::isnan(e) ? kInv : e;
      };
      const float e00 = at(col0, row0),     e10 = at(col0 + 1, row0);
      const float e01 = at(col0, row0 + 1), e11 = at(col0 + 1, row0 + 1);
      const bool n00 = (e00 == kInv), n10 = (e10 == kInv);
      const bool n01 = (e01 == kInv), n11 = (e11 == kInv);
      // [WATER-FLOOR] all-water patch: report SEA LEVEL (h=0, flat) instead
      // of "no terrain". Off-DEM queries were rejected by the physical-bound
      // check above. Mixed coastal patches already blend NaN corners as 0, so
      // this is the same surface extended continuously over open water. The
      // front-end getElevation accessor retains its "water = no terrain"
      // behavior; this gradient accessor gives the optimizer a sea-level
      // safety floor.
      if (n00 && n10 && n01 && n11) {
        *h = 0.0f;
        *dhdx = 0.0f;
        *dhdy = 0.0f;
        return true;
      }
      const double f00 = n00 ? 0.0 : e00;
      const double f10 = n10 ? 0.0 : e10;
      const double f01 = n01 ? 0.0 : e01;
      const double f11 = n11 ? 0.0 : e11;
      const double tx = fx - col0, ty = fy - row0;
      *h = static_cast<float>(
          (1.0 - tx) * (1.0 - ty) * f00 + tx * (1.0 - ty) * f10 +
          (1.0 - tx) * ty         * f01 + tx * ty         * f11);
      // Patch-analytic slopes in terrain-grid axes, then chain rule through
      // the transform above (dtwx/dwy = -1, dtwy/dwx = -1, rest 0):
      const double dHdtwx = ((1.0 - ty) * (f10 - f00) + ty * (f11 - f01)) / resolution;
      const double dHdtwy = ((1.0 - tx) * (f01 - f00) + tx * (f11 - f10)) / resolution;
      *dhdx = static_cast<float>(-dHdtwy);
      *dhdy = static_cast<float>(-dHdtwx);
      return true;
    }

    // Convert terrain grid cell to world (planning) coordinate (for obstacle_points_)
    Eigen::Vector3d terrainToWorld(int col, int row, float elev) const {
      // [TERRAIN-FRAME] forward map, exactly grid_map core's getPosition():
      const double wx = origin_x + length_x - (row + 0.5) * resolution;
      const double wy = origin_y + length_y - (col + 0.5) * resolution;
      return Eigen::Vector3d(wx, wy, static_cast<double>(elev));
    }

  private:
    // The memo is shared by all TerrainData objects on a thread. A local
    // generation (or an address that an allocator may reuse) is not an
    // instance identity. Copies, moves and assignments receive a fresh key;
    // public terrain/environment generation counters retain their meaning.
    struct CacheIdentity {
      static uint64_t next() {
        static std::atomic<uint64_t> sequence{0};
        return sequence.fetch_add(1, std::memory_order_relaxed) + 1;
      }
      uint64_t value{next()};
      CacheIdentity() = default;
      CacheIdentity(const CacheIdentity &) : CacheIdentity() {}
      CacheIdentity(CacheIdentity &&other) noexcept : CacheIdentity() {
        other.value = next();
      }
      CacheIdentity &operator=(const CacheIdentity &) {
        value = next();
        return *this;
      }
      CacheIdentity &operator=(CacheIdentity &&other) noexcept {
        value = next();
        other.value = next();
        return *this;
      }
    } cache_identity_;
  };

}  // namespace path_manager
