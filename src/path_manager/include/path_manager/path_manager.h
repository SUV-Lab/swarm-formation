#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "path_planner/sdf/sdf_manager.h"
#include "path_planner/dyn_a_star.h"
#include "path_optimizer/poly_traj_optimizer.h"
#include "path_optimizer/plan_container.hpp"
#include "../../common/log_manager.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <map>
#include <string>
#include <chrono>
#include <random>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sys/resource.h>
#include <sys/time.h>
#include <fstream>

using namespace ego_planner;

namespace path_manager
{
  // Terrain-occluded risk zone. The authored reach is the horizontal
  // reach; PathManager derives a vertical reach from risk_vertical_ratio_ and
  // supplies the resulting ellipsoid consistently to the front/back ends.
  struct RiskZone {
    Eigen::Vector3d center; // frame units (1 unit = 100 m)
    double reach;   // horizontal reach in FRAME UNITS (typ. 90 = 9 km)
    double peak;    // dimensionless in (0, 1]; values >1 are field-capped
  };

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
      struct ElevMemo { uint64_t gen; double x, y; float h, gx, gy; bool ok; };
      static thread_local ElevMemo memo{
          std::numeric_limits<uint64_t>::max(), 0.0, 0.0, 0.f, 0.f, 0.f, false};
      if (memo.gen == generation && memo.x == world_x && memo.y == world_y) {
        *h = memo.h; *dhdx = memo.gx; *dhdy = memo.gy;
        return memo.ok;
      }
      float th = 0.f, tgx = 0.f, tgy = 0.f;
      const bool ok = getElevationAndGradUncached(world_x, world_y, &th, &tgx, &tgy);
      memo = ElevMemo{generation, world_x, world_y, th, tgx, tgy, ok};
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
  };

  class PathManager
  {
  public:
    PathManager(rclcpp::Node::SharedPtr node);

    void initOptimizer(bool force_reinit = false);
    bool isOptimizerInitialized() const { return is_optimizer_initialized_ && poly_traj_opt_ != nullptr; }
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void deliverTrajToOptimizer(void) {
        if (isOptimizerInitialized()) {
            poly_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
        }
    };
    double getSwarmClearance(void) {
        return isOptimizerInitialized() ? poly_traj_opt_->getSwarmClearance() : 0.0; 
    }
    void setFormationToOptimizer(const std::vector<Eigen::Vector3d>& formation_positions, int formation_size) {
        if (!isOptimizerInitialized()) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot set formation: optimizer not initialized!");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "Setting formation with %zu positions to optimizer", formation_positions.size());
        poly_traj_opt_->setFormation(formation_positions, formation_size);
    }

    TrajContainer traj_;


    // Set formation information for path planning
    void setFormationInfo(int drone_id, const std::string& formation_type,
                         const std::vector<Eigen::Vector3d>& formation_pattern);

    void setLengthPerPiece(double val) { length_per_piece_ = val; }
    void setObstacleClearance(double val) { obstacle_clearance_ = val; }

    // Emergency stop: generate hovering trajectory at current position
    bool EmergencyStop(const Eigen::Vector3d& stop_pos);

    // Terrain data interface
    void setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr &msg);
    bool hasTerrainData() const { return terrain_data_.valid; }
    // Terrain elevation at (x, y) in frame units; false over water / off-DEM
    // / no terrain yet. Used by the FSM's [START AGL] seed conversion.
    bool terrainElevation(double x, double y, double *elev_out) const {
        if (!terrain_data_.valid) return false;
        const float e = terrain_data_.getElevation(x, y);
        if (e <= -1e9f) return false;
        *elev_out = static_cast<double>(e);
        return true;
    }
    // AGL floor shared by [GOAL AGL] and [START AGL].
    double minGoalAgl() const { return min_goal_agl_; }

    // [VEL-ALIGN] The FSM synthesizes a default start velocity along the
    // first-leg CHORD before any route exists; when flagged here,
    // planGlobalTraj re-aims that velocity onto the front-end route's actual
    // initial direction (manager/align_start_vel_to_route). Explicitly
    // commanded / trajectory-derived velocities are never re-aimed.
    void setStartVelSynthesized(bool s) { start_vel_synthesized_ = s; }

    // Dynamic obstacle interface (RViz-driven). Patches are layered on top of
    // the static terrain ESDF; the next plan picks them up via min(static,dyn).
    // Returns patch id (>= 0) on success, -1 if SDF not built yet or out of map.
    int  addDynamicSphere(const Eigen::Vector3d& center, double radius,
                          const std::string& model = "");
    // Axis-aligned box: collision (SDF kCube) == visual mesh bbox. size = full extents.
    int  addDynamicBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                       const std::string& model = "");
    void clearDynamicObstacles();
    size_t numDynamicObstacles() const { return sdf_manager_.numActiveObstacles(); }

    // Runtime risk-zone reset (called when ObstacleScenarioPanel / mission
    // authority publishes a new RiskZoneArray). Atomically replaces the
    // internal zone list. A* / optimizer are re-bound on next planGlobalTraj.
    // Thread/timing: callers must ensure this is invoked on the same
    // callback group as trajectory commands (handled in ReplanFSM).
    void setRiskZonesRuntime(const std::vector<RiskZone>& zones);
    size_t numRiskZones() const { return risk_zones_.size(); }
    // Read-only inspection hooks for metrics/tests. These expose the same
    // precomputed field consumed by FM2/A* and MINCO; no second LOS model.
    double getRiskVisibility(size_t zone_index,
                             const Eigen::Vector3d &pos) const {
      return riskVisibilityValue(zone_index, pos);
    }
    double getRiskShadowCeiling(size_t zone_index,
                                const Eigen::Vector3d &pos) const {
      return riskShadowCeiling(zone_index, pos);
    }
    double getEffectiveRisk(size_t zone_index,
                            const Eigen::Vector3d &pos) const {
      return riskZoneValue(zone_index, pos);
    }

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<RiskZone> risk_zones_;
    // Searcher-facing copy of risk_zones_. PathSearcher::setRiskZones stores
    // a RAW POINTER to this vector, so it must outlive the plan call — a
    // function-local here left the searcher holding a dangling pointer
    // between plans (latent: nothing dereferences it today, but any future
    // between-plan getRiskCost query would be a use-after-free).
    std::vector<path_planner::search::RiskZoneLite> astar_risks_;
    // Zones exactly as authored (yaml param / runtime topic). risk_zones_ is
    // DERIVED from this list: with risk_zone_agl_ on, center.z is height
    // ABOVE the DEM at (x,y) — a source-height offset on the terrain — and the
    // effective absolute z is re-derived whenever the DEM (re)arrives, so
    // zone-before-terrain ordering does not change the result.
    std::vector<RiskZone> risk_zones_raw_;
    bool risk_zone_agl_{false};
    void refreshEffectiveRiskZones();
    double risk_weight_;
    double risk_barrier_{100.0};   // front-end finite "hard wall" inside zones
    // Rv / Rh for the compact ellipsoidal risk envelope. Keeping this
    // independent of the LOS mask separates geometric risk coverage from
    // terrain visibility.
    double risk_vertical_ratio_{0.35};
    double riskEllipsoidRadius(const RiskZone &zone,
                               const Eigen::Vector3d &pos) const;
    double riskZoneValue(size_t zone_index,
                         const Eigen::Vector3d &pos) const;
    // Terrain-masked risk field (radial horizon / viewshed). For each source
    // and azimuth, shadow_ceiling stores the highest terrain LOS line implied
    // by all nearer DEM samples. A query is visible above that ceiling and
    // shadowed below it. The field is O(1) to query in FM2's hot loop.
    struct TerrainRiskMask {
      Eigen::Vector3d source{Eigen::Vector3d::Zero()};
      double max_range{0.0};
      double radial_step{1.0};
      int radial_count{0};
      int angular_count{0};
      std::vector<float> shadow_ceiling;  // [angle * radial_count + radius]
      bool valid{false};
    };
    std::vector<TerrainRiskMask> terrain_risk_masks_;
    bool risk_terrain_mask_enable_{true};
    double risk_mask_radial_step_{0.0};  // <=0: DEM resolution
    double risk_mask_softness_{0.10};    // vertical sigmoid width (frame units)
    bool risk_grounded_{true};           // visibility clamped at terrain+band
                                         // ("no terrain-occluded state below the band")
    double risk_mask_viz_step_{0.0};     // <=0: auto per zone/DEM
    double risk_mask_viz_slice_offset_{0.0};
    double risk_mask_viz_threshold_{0.50};
    // Visibility-boundary heatmap (mode "heatmap"): color ramp saturates at this
    // AGL (frame units; 2.0 = 200 m), grid capped at max_dim on the longer
    // side of the zone-union AABB.
    double risk_heatmap_agl_max_{2.0};
    int risk_heatmap_max_dim_{768};
    // Drape height above the DEM. Must clear the RENDERED terrain mesh, which
    // deviates from the bilinear DEM sample by up to ~half a cell on slopes
    // (76 m cells on big_terrain) — 0.05 u sank into hillsides.
    double risk_heatmap_offset_{0.30};
    // [PREVIEW] Extra render lift applied ONLY while no DEM has arrived, so
    // the ideal-field drape floats above the rendered terrain instead of
    // being buried inside it (frame units; 20 = 2 km, above any terrain).
    double risk_heatmap_preview_lift_{20.0};
    // Cells whose visibility boundary is at/above agl_max ("low risk unless you
    // climb") paint GREEN by default; true leaves them transparent instead
    // so the terrain imagery dominates. Occluded cells (never visible)
    // always keep their explicit blue.
    bool risk_heatmap_safe_transparent_{false};
    // RViz modes: "volume" (default risk-boundary mesh + clipped wire shell),
    // "fixed_agl" (terrain-following diagnostic), or "fixed_msl" (planar
    // diagnostic). The old drape bool remains only as a compatibility hint.
    std::string risk_mask_viz_mode_{"heatmap"};
    int risk_mask_viz_contours_{5};
    double risk_mask_viz_volume_alpha_{0.24};
    double risk_mask_viz_agl_{0.15};     // draped eval height above ground
    void rebuildTerrainRiskMasks();
    double riskShadowCeiling(size_t zone_index,
                             const Eigen::Vector3d &pos) const;
    double riskVisibilityValue(size_t zone_index,
                               const Eigen::Vector3d &pos) const;
    double riskVisibility(size_t zone_index, const Eigen::Vector3d &pos,
                          Eigen::Vector3d *grad) const;
    // [RISK-GROUNDED] z used by the visibility sigmoid (manager/risk_grounded,
    // default ON): smooth-max(z, terrain + band) so the
    // field has no legal terrain-occluded state below the clearance band.
    // dzeff_dz (opt)
    // receives ∂z_eff/∂z for the analytic z-gradient — one C1 surface.
    double riskGroundedZ(const Eigen::Vector3d &pos, double *dzeff_dz) const;
    void publishEffectiveRiskField();
    void publishRiskHeatmap();
    // [TRAJ-RISK] the one view where color == cost: the planned trajectory
    // painted by the risk density the aircraft ACTUALLY experiences at its 3D
    // position (OR-combined terrain-masked moats). Green (0) -> yellow -> red.
    // The draped heatmap shows the ground-level visibility boundary, which is NOT
    // the flown risk — this line is.
    void publishTrajRisk(const poly_traj::Trajectory &traj);
    // [RISK-PROFILE] Versioned altitude-panel channel. Payload (v4):
    //   [4.0, N,
    //    s, x, y, z, combined_risk,
    //    lower_0, floor_0, top_0, ..., lower_N-1, floor_N-1, top_N-1,
    //    ...]
    // with one fixed-width record (5 + 3N) per dense trajectory sample.
    // Geometry is in planning-frame units; combined_risk is the exact
    // OR-combination used by /viz/traj_risk. Per zone: lower_i is the
    // geometric ellipsoid lower bound, floor_i the grounded-visibility
    // detection floor, top_i the geometric top — the triplet lets the panel
    // distinguish a shadow-safe pocket between two zones from uncovered sky.
    // NaN triplet means that zone has no visible vertical interval at (x,y).
    bool riskProfileVisibleInterval(size_t zone_index, double x, double y,
                                    double *lower_z,
                                    double *floor_z, double *top_z) const;
    void publishRiskProfile(const poly_traj::Trajectory &traj);
    double risk_smha_w_{2.0};
    std::string front_end_str_{"fm2"};
    int fm2_coarse_k_{4};
    int fm2_max_cells_{8000000};  // FMM grid cell-count cap (tunable)
    bool fm2_star_{true};
    double fm2_alt_penalty_{2.0};  // wave slowdown above mission altitude
    double fm2_alt_zscale_{10.0};  // up-side ramp (gentle: climbs allowed)
    double fm2_alt_zscale_dn_{2.0};// down-side ramp (stiff: no diving)
    // [ROUGH] H2 terrain-roughness routing: additive slope cost in the FE
    // shared field (FM2 speed + A* edges + shortcut acceptance), own currency
    // (NOT risk_weight-scaled — keeps zone sensitivity intact). 0 = off.
    double fm2_rough_weight_{0.0};
    double fm2_rough_slope0_{0.20};
    double dyn_obstacle_margin_{3.0};  // berth around dynamic obstacles
    double opt_obstacle_clearance_{0.7};  // optimizer penalty onset (< front-end margin)
    double weight_altitude_{1000.0};      // optimizer z-cap weight above mission band
    double alt_cap_headroom_{5.0};        // z-cap slack above the geodesic max; must fit sparse-piece quintic swell (~ FM2 coarse-cell band tolerance)
    double alt_floor_headroom_{0.5};      // z-floor slack below min(start,goal) z; stops min-jerk sags bouncing off the water/terrain clearance
    double min_goal_agl_{1.0};            // waypoints get z >= terrain elevation + this (frame z units); kills underground goals from fixed-z mission sources
    // [VEL-ALIGN] see setStartVelSynthesized().
    bool start_vel_synthesized_{false};
    bool align_start_vel_to_route_{true};
    // [ZONE-AVOID] lexicographic zone policy (see dyn_a_star.h).
    bool zone_avoid_lexico_{true};
    double corner_fillet_radius_{0.0};    // legacy geometric fallback; 0 = off
    bool astar_bypass_shortcut_{false};
    // Max z of the front-end route BEFORE the z-denoise filter (per plan, set
    // in planFrontEnd). The altitude cap must reference the COMMITTED profile,
    // not the filtered one — the moving average planes crest maxima (~34 m
    // observed) and an under-referenced cap grinds against the terrain band.
    double fe_raw_max_z_{-1e9};
    Eigen::Vector3d map_lower_bound_;
    Eigen::Vector3d map_upper_bound_;
    std::vector<LocalTrajData> swarm_traj_;
    double max_vel_;
    double max_acc_;
    double length_per_piece_ = 2.0;
    double obstacle_clearance_ = 1.0;
    // Virtual ground / ceiling planes. If set ≥ 0, these altitudes are
    // voxelised as solid obstacle layers when the SDF is built, so the
    // optimizer naturally treats "flying below ground" or "above ceiling"
    // as collision (with a smooth lateral gradient). A value < 0 (e.g.
    // the default -0.1) disables that plane. Without these, L-BFGS can
    // drift the trajectory below the start altitude to dodge obstacles,
    // which is non-physical.
    double ground_height_ = -0.1;    // frame units (absolute world z; 1 u = 100 m)
    double virtual_ceil_height_ = -0.1;  // frame units (absolute world z)
    TerrainData terrain_data_;

    // ESDF map for SDF-based RRT* queries (phase 3).
    // Built from terrain inside planGlobalTraj.
    path_planner::sdf::SDFManager sdf_manager_;
    double sdf_voxel_size_ = 1.0;  // frame units
    bool sdf_voxel_size_auto_ = false;   // param was <=0: track the DEM cell on map change
    double sdf_voxel_z_{0.0};             // vertical voxel size; <=0 -> isotropic (= sdf_voxel_size_)
    // A* search step size (frame units between neighboring path nodes). Kept
    // independent of sdf_voxel_size_ so we can coarsen A* path density
    // without touching SDF resolution. Must be a multiple of voxel size
    // for the grid-aligned neighbor set to make sense.
    double astar_step_size_ = 1.0;  // frame units

    // 3D A* front-end. Uses ESDF for collision, risk_zones_ for soft cost.
    path_planner::search::PathSearcher searcher_;
    bool astar_initialized_ = false;
    Eigen::Vector3i astar_pool_size_ = Eigen::Vector3i(120, 120, 40);

    // Precomputed-ESDF paths (both optional, via yaml).
    //   load: if set and file present, skip voxelization on first plan.
    //   save: if set, write the freshly built ESDF after first build.
    // When either is set, the ESDF covers the full loaded terrain (not the
    // per-mission bbox) so the cached map is reusable across missions.
    // Boxes-only SDF grid over the current terrain bbox. Built on the first
    // terrain message and REBUILT when the map geometry changes (world switch
    // / corridor re-crop) — setTerrainData resets this latch and clears the
    // dynamic-obstacle patches, which were grounded on the old DEM.
    bool sdf_built_ = false;

    // Full-terrain bbox used when save/load is active. Computed once from
    // terrain_data_ metadata on first use.
    bool terrain_bbox_computed_ = false;
    Eigen::Vector3d terrain_bbox_lo_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d terrain_bbox_hi_ = Eigen::Vector3d::Zero();
    bool computeTerrainBBox(Eigen::Vector3d* lo, Eigen::Vector3d* hi);

    // Rebuilds sdf_manager_ from current terrain + obstacles, covering the
    // bounding box given in world coords. Returns true on success.
    bool buildSDFForBounds(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi);

    // Stage 1 (front-end): A*/FM2 search + corner-adaptive densification.
    // Produces the route (full_route) and densified path (clean_path) the
    // optimizer consumes, plus cap_ref: the COMMITTED z per clean_path vertex
    // (pre-z-denoise profile elementwise-maxed with the safety-clamped one,
    // interpolated through the same subdivision) — the arc-varying altitude
    // cap's reference. SDF must already be built. Returns true on success.
    bool planFrontEnd(const Eigen::Vector3d &start_pos,
                      const std::vector<Eigen::Vector3d> &waypoints,
                      std::vector<Eigen::Vector3d> &full_route,
                      std::vector<Eigen::Vector3d> &clean_path,
                      std::vector<double> &cap_ref);

    // Stage 2 (trajectory optimization): MINCO initial trajectory + L-BFGS.
    // Takes the front-end path; sets traj_ global/local. Returns true on success.
    bool optimizeStage(std::vector<Eigen::Vector3d> &clean_path,
                       const std::vector<Eigen::Vector3d> &full_route,
                       const Eigen::Vector3d &start_pos,
                       const Eigen::Vector3d &start_vel,
                       const Eigen::Vector3d &start_acc,
                       const std::vector<Eigen::Vector3d> &waypoints,
                       const std::vector<double> &cap_ref);

    ego_planner::PolyTrajOptimizer::Ptr poly_traj_opt_;
    bool is_optimizer_initialized_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr front_end_path_pub_;
    // Terrain-masked risk visualization: ONE latched MarkerArray carries the
    // whole picture (DELETEALL + every zone's markers), so a late joiner gets
    // all of it or none of it. See publishEffectiveRiskField.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_field_pub_;
    // Draped visibility-boundary heatmap (GridMap, rendered by a second
    // grid_map_rviz_plugin display; see publishRiskHeatmap).
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr risk_heatmap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_risk_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr risk_profile_pub_;
    // Dynamic obstacle visualization (one MarkerArray republished on every add/clear).
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dyn_obstacle_pub_;
    // Terrain ESDF cache status string (RViz panel reads this).
    // Only drone_0's PathManager owns this publisher to avoid duplicate writes.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terrain_status_pub_;
    void publishTerrainStatus(const std::string &msg);
    // Tracks live patch ids so clearObstacles + visualization stay in sync.
    std::vector<int> dyn_patch_ids_;
    std::vector<Eigen::Vector3d> dyn_patch_centers_;
    std::vector<Eigen::Vector3d> dyn_patch_sizes_;   // full extents [m]: box=(sx,sy,sz), sphere=(2r,2r,2r)
    std::vector<uint8_t> dyn_patch_is_box_;          // 1=box (collision==visual), 0=sphere
    std::vector<std::string> dyn_patch_models_;      // visual mesh catalog key per patch
    std::vector<double> dyn_patch_yaws_;             // per-spawn yaw [rad]. NOT visual-only: oriented
                                                     // boxes go into the SDF (samplePatch), so yaw
                                                     // changes the FM2 route. Seed via manager/dyn_yaw_seed.
    std::mt19937 yaw_rng_;   // spawn-orientation RNG; seeded from manager/dyn_yaw_seed (fixed=reproducible, <0=random)

    // Dynamic obstacles requested before the SDF exists (e.g. no ESDF cache on a
    // fresh run) are deferred here, then added once the SDF is built.
    struct PendingObstacle {
      bool is_box; Eigen::Vector3d center; Eigen::Vector3d size; double radius; std::string model;
    };
    std::vector<PendingObstacle> pending_obstacles_;
    void flushPendingObstacles();
    // Visual mesh catalog: model name -> mesh resource + rendered native size [m]
    // (convention: mesh base at z=0, XY centered). Collision is independent (SDF).
    struct ObstacleMeshInfo {
        std::string resource;
        Eigen::Vector3d native_size;
        double viz_scale{1.0};  // per-model mesh magnification (visual only)
    };
    std::map<std::string, ObstacleMeshInfo> mesh_catalog_;
    const ObstacleMeshInfo& meshFor(const std::string& model) const;
    // /viz/dynamic_obstacles rendering. obstacle_mesh_resource_ = default ("building").
    std::string obstacle_mesh_resource_;
    double obstacle_mesh_height_{60.0};   // fixed building height [m] (spheres only)
    double obstacle_viz_scale_{1.0};      // mesh-only magnification (see decl site)
    bool obstacle_ground_snap_{true};     // base obstacles on terrain/sea surface
    // Grounded center for a dynamic obstacle: base at max(terrain, sea level)
    // under (x, y), center half_height above it. Falls back to the given
    // center when snapping is disabled.
    Eigen::Vector3d groundedCenter(const Eigen::Vector3d& center, double half_height) const;
    void publishDynamicObstacles();

    std::shared_ptr<swarm_formation::LogManager> log_manager_;
    bool enable_debug_logs_;

    // Formation information for path adjustment
    std::string current_formation_type_;
    std::vector<Eigen::Vector3d> current_formation_pattern_;

  };

} // namespace path_manager

#endif // PATH_MANAGER_H
