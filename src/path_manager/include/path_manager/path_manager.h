#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "path_planner/sdf/sdf_manager.h"
#include "path_planner/dyn_a_star.h"
#include "path_optimizer/poly_traj_optimizer.h"
#include "path_optimizer/plan_container.hpp"
#include "../../common/log_manager.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <string>
#include <chrono>
#include <random>
#include "path_manager/msg/poly_traj.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/resource.h>
#include <sys/time.h>
#include <fstream>

using namespace ego_planner;

namespace path_manager
{
  enum class ObstacleShape {
    CIRCLE,
    RECTANGLE
  };

  struct Obstacle {
    Eigen::Vector3d center;       // base (bottom) position, xy at the axis
    ObstacleShape shape;
    double param1;  // Circle: radius, Rectangle: width (x-extent)
    double param2;  // Circle: unused, Rectangle: length (y-extent)
    double z_extent;  // vertical height above center.z; 0 = "infinite column" (back-compat)

    Obstacle() : center(0, 0, 0), shape(ObstacleShape::CIRCLE), param1(-1.0), param2(0.0), z_extent(0.0) {}
    Obstacle(const Eigen::Vector3d& c) : center(c), shape(ObstacleShape::CIRCLE), param1(-1.0), param2(0.0), z_extent(0.0) {}
    Obstacle(const Eigen::Vector3d& c, double radius) : center(c), shape(ObstacleShape::CIRCLE), param1(radius), param2(0.0), z_extent(0.0) {}
    Obstacle(const Eigen::Vector3d& c, double radius, double height, bool /*circle_with_height*/) : center(c), shape(ObstacleShape::CIRCLE), param1(radius), param2(0.0), z_extent(height) {}
    Obstacle(const Eigen::Vector3d& c, double width, double length) : center(c), shape(ObstacleShape::RECTANGLE), param1(width), param2(length), z_extent(0.0) {}
    Obstacle(const Eigen::Vector3d& c, double width, double length, double height) : center(c), shape(ObstacleShape::RECTANGLE), param1(width), param2(length), z_extent(height) {}
  };

  // V3 risk zone: quadratic moat with compact support.
  // moat(x) = peak * (1 - ||x - center|| / reach)^2 for d < reach, else 0.
  struct RiskZone {
    Eigen::Vector3d center;
    double reach;   // meters; risk is exactly zero outside this ball
    double peak;    // dimensionless in (0, 1]
  };

  // Terrain elevation data extracted from GridMap
  // Coordinate transform: terrain_publisher uses a different axis convention.
  // terrain → world: X-mirror, then -90° rotation around center.
  // world → terrain: +90° rotation, then X-mirror (inverse).
  struct TerrainData {
    std::vector<float> elevation;  // Column-major elevation data
    int cols = 0;
    int rows = 0;
    double resolution = 0.0;
    double origin_x = 0.0;       // Terrain grid origin X
    double origin_y = 0.0;       // Terrain grid origin Y
    double length_x = 0.0;       // Terrain total length X
    double length_y = 0.0;       // Terrain total length Y
    double center_x = 0.0;       // Terrain center X
    double center_y = 0.0;       // Terrain center Y
    bool valid = false;

    // Convert world (planning) coordinate to terrain grid index and query elevation
    float getElevation(double world_x, double world_y) const {
      if (!valid) return -std::numeric_limits<float>::infinity();

      // Inverse of: terrain → X-mirror → -90° rotate → world
      // Step 1: +90° rotation around terrain center
      double rel_x = world_x - center_x;
      double rel_y = world_y - center_y;
      double rot_x = center_x + rel_y;   // +90°: x' = cy + rel_y
      double rot_y = center_y - rel_x;   // +90°: y' = cy - rel_x

      // Step 2: Undo X-mirror
      double terrain_world_x = 2.0 * center_x - rot_x;
      double terrain_world_y = rot_y;

      // Step 3: World coord → fractional grid coordinate (same fx/fy convention
      // as the RViz panel's getElevationAt) then BILINEAR sample.
      // WHY bilinear (was nearest-cell): the SDF the optimizer plans against is
      // voxelised from THIS function; nearest-cell made it a piecewise-constant,
      // ~cell-coarse (~230 m DEM cell) terrain, so a trajectory that only grazed
      // that blocky surface looked like it PENETRATED the finer bilinear terrain
      // the altitude/clearance panels display. Interpolating here makes the
      // optimizer's terrain match the panels' terrain, so grazes stop reading as
      // pass-throughs. Water/edge (any NaN or out-of-range corner) keeps the old
      // nearest-cell + invalid rule so the "water = no terrain" voxelisation is
      // unchanged.
      constexpr float kInv = -std::numeric_limits<float>::infinity();
      const double fx = (terrain_world_x - origin_x) / resolution;
      const double fy = (terrain_world_y - origin_y) / resolution;
      const int col0 = static_cast<int>(std::floor(fx));
      const int row0 = static_cast<int>(std::floor(fy));

      auto at = [&](int c, int r) -> float {
        if (c < 0 || c >= cols || r < 0 || r >= rows) return kInv;
        const int index = c * rows + r;  // Column-major
        if (index < 0 || index >= static_cast<int>(elevation.size())) return kInv;
        const float e = elevation[index];
        return std::isnan(e) ? kInv : e;
      };

      const float e00 = at(col0, row0),     e10 = at(col0 + 1, row0);
      const float e01 = at(col0, row0 + 1), e11 = at(col0 + 1, row0 + 1);
      // Blend invalid corners (water NaN / map edge) as SEA LEVEL 0 — exactly
      // like the RViz panel's sampler — so the surface is C0-continuous across
      // coastlines. The first version fell back to the nearest CELL value
      // there, which JUMPS by the full land height as a query crosses a cell
      // boundary at the coast; the optimizer's terrain term (weight 1e4) fed
      // on that discontinuity and killed the line search (-1005/-1008) on
      // island-dotted maps. Pure water (all four invalid) stays -inf so
      // "water = no terrain" semantics are unchanged for the F-build and the
      // terrain term alike.
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
    // gradients are the same smoothness class the battle-tested SDF term had
    // (trilinear value + its exact gradient). Returns false over pure water /
    // outside the DEM (no terrain there).
    bool getElevationAndGrad(double world_x, double world_y,
                             float *h, float *dhdx, float *dhdy) const {
      if (!valid) return false;
      // world -> terrain-grid coords (same transform as getElevation):
      //   twx = (cx + cy) - wy,  twy = (cx + cy) - wx
      const double rel_x = world_x - center_x;
      const double rel_y = world_y - center_y;
      const double rot_x = center_x + rel_y;
      const double rot_y = center_y - rel_x;
      const double twx = 2.0 * center_x - rot_x;
      const double twy = rot_y;
      const double fx = (twx - origin_x) / resolution;
      const double fy = (twy - origin_y) / resolution;
      const int col0 = static_cast<int>(std::floor(fx));
      const int row0 = static_cast<int>(std::floor(fy));
      constexpr float kInv = -std::numeric_limits<float>::infinity();
      auto at = [&](int c, int r) -> float {
        if (c < 0 || c >= cols || r < 0 || r >= rows) return kInv;
        const int index = c * rows + r;
        if (index < 0 || index >= static_cast<int>(elevation.size())) return kInv;
        const float e = elevation[index];
        return std::isnan(e) ? kInv : e;
      };
      const float e00 = at(col0, row0),     e10 = at(col0 + 1, row0);
      const float e01 = at(col0, row0 + 1), e11 = at(col0 + 1, row0 + 1);
      const bool n00 = (e00 == kInv), n10 = (e10 == kInv);
      const bool n01 = (e01 == kInv), n11 = (e11 == kInv);
      if (n00 && n10 && n01 && n11) return false;
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
      // Grid index → terrain world coord
      double tw_x = origin_x + (col + 0.5) * resolution;
      double tw_y = origin_y + (row + 0.5) * resolution;

      // X-mirror
      double fx = 2.0 * center_x - tw_x;
      double fy = tw_y;

      // -90° rotation around center
      double rel_x = fx - center_x;
      double rel_y = fy - center_y;
      double wx = center_x - rel_y;
      double wy = center_y + rel_x;

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

    void updateRobotState(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& local_target_pt);
    bool isMapReady(const Eigen::Vector3d& start_pos) const;

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

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<Eigen::Vector3d> simple_path_;
    std::vector<Obstacle> obstacle_centers_;
    // Yaml obstacles are applied once as SDF dynamic patches (not baked into
    // the terrain ESDF / its cache file) — see planGlobalTraj.
    bool static_obstacles_applied_{false};
    std::vector<RiskZone> risk_zones_;
    double risk_weight_;
    double risk_barrier_{100.0};   // front-end finite "hard wall" inside zones
    double risk_smha_w_{2.0};
    std::string front_end_str_{"fm2"};
    int fm2_coarse_k_{4};
    int fm2_max_cells_{8000000};  // FMM grid cell-count cap (tunable)
    bool fm2_star_{true};
    double fm2_alt_penalty_{2.0};  // wave slowdown above mission altitude
    double fm2_alt_zscale_{10.0};  // up-side ramp (gentle: climbs allowed)
    double fm2_alt_zscale_dn_{2.0};// down-side ramp (stiff: no diving)
    double dyn_obstacle_margin_{3.0};  // berth around dynamic obstacles
    double opt_obstacle_clearance_{0.7};  // optimizer penalty onset (< front-end margin)
    double weight_altitude_{1000.0};      // optimizer z-cap weight above mission band
    double alt_cap_headroom_{5.0};        // z-cap slack above the geodesic max; must fit sparse-piece quintic swell (~ FM2 coarse-cell band tolerance)
    double alt_floor_headroom_{0.5};      // z-floor slack below min(start,goal) z; stops min-jerk sags bouncing off the water/terrain clearance
    double min_goal_agl_{1.0};            // waypoints get z >= terrain elevation + this (frame z units); kills underground goals from fixed-z mission sources
    double corner_fillet_radius_{0.0};    // legacy geometric fallback; 0 = off
    uint64_t esdf_viz_revision_{~0ull};   // last SDF revision published as cubes
    double esdf_viz_step_{4.0};           // ESDF occupancy-viz sample step [m]; coarse = cheap
    bool   esdf_viz_enable_{true};        // publish the ESDF occupancy overlay at all
    bool astar_bypass_shortcut_{false};
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
    double ground_height_ = -0.1;    // m (absolute world z)
    double virtual_ceil_height_ = -0.1;  // m (absolute world z)
    TerrainData terrain_data_;

    // ESDF map for SDF-based RRT* queries (phase 3).
    // Built from terrain + obstacle_centers_ inside planGlobalTraj.
    path_planner::sdf::SDFManager sdf_manager_;
    double sdf_voxel_size_ = 1.0;  // m
    double sdf_voxel_z_{0.0};             // vertical voxel size; <=0 -> isotropic (= sdf_voxel_size_)
    // A* search step size (meters between neighboring path nodes). Kept
    // independent of sdf_voxel_size_ so we can coarsen A* path density
    // without touching SDF resolution. Must be a multiple of voxel size
    // for the grid-aligned neighbor set to make sense.
    double astar_step_size_ = 1.0;  // m

    // 3D A* front-end. Uses ESDF for collision, risk_zones_ for soft cost.
    path_planner::search::PathSearcher searcher_;
    bool astar_initialized_ = false;
    Eigen::Vector3i astar_pool_size_ = Eigen::Vector3i(120, 120, 40);

    // Precomputed-ESDF paths (both optional, via yaml).
    //   load: if set and file present, skip voxelization on first plan.
    //   save: if set, write the freshly built ESDF after first build.
    // When either is set, the ESDF covers the full loaded terrain (not the
    // per-mission bbox) so the cached map is reusable across missions.
    // Boxes-only SDF grid built once over the full-terrain bbox. Guard, not
    // cache: a rebuild would drop every dynamic-obstacle patch (they clip to
    // this grid). The old <world>.esdf disk cache members are gone with the
    // cache mechanism itself.
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
    // optimizer consumes. SDF must already be built. Returns true on success.
    bool planFrontEnd(const Eigen::Vector3d &start_pos,
                      const std::vector<Eigen::Vector3d> &waypoints,
                      std::vector<Eigen::Vector3d> &full_route,
                      std::vector<Eigen::Vector3d> &clean_path);

    // Stage 2 (trajectory optimization): MINCO initial trajectory + L-BFGS.
    // Takes the front-end path; sets traj_ global/local. Returns true on success.
    bool optimizeStage(std::vector<Eigen::Vector3d> &clean_path,
                       const std::vector<Eigen::Vector3d> &full_route,
                       const Eigen::Vector3d &start_pos,
                       const Eigen::Vector3d &start_vel,
                       const Eigen::Vector3d &start_acc,
                       const std::vector<Eigen::Vector3d> &waypoints);

    ego_planner::PolyTrajOptimizer::Ptr poly_traj_opt_;
    bool is_optimizer_initialized_;
    Eigen::Vector3d current_start_pt_, current_target_pt_;
    bool has_valid_state_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simple_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr search_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shorten_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr esdf_occ_pub_;
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
    struct ObstacleMeshInfo { std::string resource; Eigen::Vector3d native_size; };
    std::map<std::string, ObstacleMeshInfo> mesh_catalog_;
    const ObstacleMeshInfo& meshFor(const std::string& model) const;
    // /viz/dynamic_obstacles rendering. obstacle_mesh_resource_ = default ("building").
    std::string obstacle_mesh_resource_;
    double obstacle_mesh_height_{60.0};   // fixed building height [m] (spheres only)
    void publishDynamicObstacles();

    std::shared_ptr<swarm_formation::LogManager> log_manager_;
    bool enable_debug_logs_;

    // Formation information for path adjustment
    std::string current_formation_type_;
    std::vector<Eigen::Vector3d> current_formation_pattern_;

  };

} // namespace path_manager

#endif // PATH_MANAGER_H