#include "path_manager/path_manager.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <malloc.h>

namespace path_manager
{

    PathManager::PathManager(rclcpp::Node::SharedPtr node)
        : node_(node),
          max_vel_(-1.0),
          max_acc_(-1.0),
          is_optimizer_initialized_(false),
          current_formation_type_("")
    {
        log_manager_ = std::make_shared<swarm_formation::LogManager>(
            node_->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);
        
        enable_debug_logs_ = false;
        if (node_->has_parameter("enable_debug_logs")) {
            node_->get_parameter("enable_debug_logs", enable_debug_logs_);
        }
            
        int drone_id;
        node_->get_parameter("drone_id", drone_id);
        traj_.local_traj.drone_id = drone_id;

        node_->declare_parameter("manager/max_vel", -1.0);
        node_->declare_parameter("manager/max_acc", -1.0);
        // [ENVELOPE] Handoff contract speed ceiling in m/s (ADR-0002 §속도
        // 상한 분리). 0 = follow the planner's own cap (optimization/
        // max_vel). Declared HERE or a yaml value silently never applies
        // (review find: the validator polled an undeclared name and always
        // used the fallback).
        node_->declare_parameter("planning/handoff_max_vel_mps", 0.0);
        node_->declare_parameter("manager/length_per_piece", 3.0);
        node_->declare_parameter("manager/risk_weight", 1.0);
        node_->declare_parameter("manager/risk_barrier", 100.0);
        // [GNRON] endpoint moat taper radius (frame units; <=0 off): fade a
        // must-enter zone's moat to zero near the contained endpoint so the
        // approach to the mission point is not priced against it.
        node_->declare_parameter("manager/risk_goal_taper_radius", 30.0);
        node_->declare_parameter("manager/risk_terrain_mask_enable", true);
        node_->declare_parameter("manager/risk_zone_agl", false);
        node_->declare_parameter("manager/risk_vertical_ratio", 0.35);
        node_->declare_parameter("manager/risk_mask_radial_step", 0.0);
        node_->declare_parameter("manager/risk_mask_softness", 0.10);
        // [RISK-GROUNDED] visibility evaluated at smoothmax(z, terrain+band):
        // no legal terrain-occluded state below the clearance band. Root fix
        // for the below-horizon terrain-clearance violation (2026-07-20
        // complex-field stress case).
        node_->declare_parameter("manager/risk_grounded", true);
        node_->declare_parameter("manager/risk_mask_viz_step", 0.0);
        node_->declare_parameter("manager/risk_mask_viz_slice_offset", 0.0);
        node_->declare_parameter("manager/risk_mask_viz_threshold", 0.50);
        node_->declare_parameter("manager/risk_mask_viz_mode",
                                 std::string("heatmap"));
        node_->declare_parameter("manager/risk_heatmap_agl_max", 2.0);
        node_->declare_parameter("manager/risk_heatmap_max_dim", 768);
        node_->declare_parameter("manager/risk_heatmap_offset", 0.30);
        node_->declare_parameter("manager/risk_heatmap_preview_lift", 20.0);
        node_->declare_parameter("manager/risk_heatmap_safe_transparent",
                                 false);
        node_->declare_parameter("manager/risk_mask_viz_contours", 5);
        node_->declare_parameter("manager/risk_mask_viz_volume_alpha", 0.24);
        node_->declare_parameter("manager/risk_mask_viz_agl", 0.15);
        node_->declare_parameter("manager/risk_smha_w", 2.0);
        node_->declare_parameter("manager/front_end", std::string("fm2"));
        node_->declare_parameter("manager/fm2_coarse_k", 4);
        node_->declare_parameter("manager/fm2_max_cells", 8000000);
        node_->declare_parameter("manager/fm2_star", true);
        node_->declare_parameter("manager/fm2_alt_penalty", 2.0);
        node_->declare_parameter("manager/fm2_alt_zscale", 10.0);
        node_->declare_parameter("manager/fm2_alt_zscale_down", 2.0);
        // [ROUGH] H2 terrain-roughness routing (0 = off/legacy field).
        node_->declare_parameter("manager/fm2_rough_weight", 0.0);
        node_->declare_parameter("manager/fm2_rough_slope0", 0.20);
        node_->declare_parameter("manager/astar_bypass_shortcut", false);
        node_->declare_parameter("manager/astar_step_size", 1.0);
        node_->declare_parameter("manager/dyn_yaw_seed", 42);  // fixed = reproducible obstacle orientations; <0 = randomize each run
        node_->declare_parameter("manager/sdf_voxel_size", 1.0);
        node_->declare_parameter("manager/sdf_voxel_z", 0.0);
        node_->declare_parameter("manager/ground_height", -0.1);
        node_->declare_parameter("manager/virtual_ceil_height", -0.1);
        node_->declare_parameter("manager/map_ceiling_headroom", 20.0);
        node_->declare_parameter("manager/obstacle_clearance", 0.3);
        node_->declare_parameter("manager/dyn_obstacle_margin", 3.0);
        node_->declare_parameter("optimization/obstacle_clearance", 0.7);
        node_->declare_parameter("optimization/weight_altitude", 1000.0);
        node_->declare_parameter("optimization/alt_cap_headroom", 5.0);
        node_->declare_parameter("optimization/alt_floor_headroom", 0.5);
        node_->declare_parameter("manager/min_goal_agl", 1.0);
        node_->declare_parameter("manager/align_start_vel_to_route", true);
        node_->declare_parameter("manager/zone_avoid_lexicographic", true);
        node_->declare_parameter("manager/corner_fillet_radius", 0.0);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/length_per_piece", length_per_piece_);
        node_->get_parameter("manager/risk_weight", risk_weight_);
        node_->get_parameter("manager/risk_barrier", risk_barrier_);
        node_->get_parameter("manager/risk_goal_taper_radius",
                             risk_goal_taper_radius_);
        node_->get_parameter("manager/risk_terrain_mask_enable",
                             risk_terrain_mask_enable_);
        node_->get_parameter("manager/risk_zone_agl", risk_zone_agl_);
        node_->get_parameter("manager/risk_vertical_ratio",
                             risk_vertical_ratio_);
        risk_vertical_ratio_ = std::max(0.01, risk_vertical_ratio_);
        node_->get_parameter("manager/risk_mask_radial_step",
                             risk_mask_radial_step_);
        node_->get_parameter("manager/risk_mask_softness",
                             risk_mask_softness_);
        node_->get_parameter("manager/risk_grounded", risk_grounded_);
        node_->get_parameter("manager/risk_mask_viz_step",
                             risk_mask_viz_step_);
        node_->get_parameter("manager/risk_mask_viz_slice_offset",
                             risk_mask_viz_slice_offset_);
        node_->get_parameter("manager/risk_mask_viz_threshold",
                             risk_mask_viz_threshold_);
        node_->get_parameter("manager/risk_mask_viz_mode",
                             risk_mask_viz_mode_);
        node_->get_parameter("manager/risk_heatmap_agl_max",
                             risk_heatmap_agl_max_);
        node_->get_parameter("manager/risk_heatmap_max_dim",
                             risk_heatmap_max_dim_);
        risk_heatmap_agl_max_ = std::max(0.05, risk_heatmap_agl_max_);
        risk_heatmap_max_dim_ = std::clamp(risk_heatmap_max_dim_, 64, 4096);
        node_->get_parameter("manager/risk_heatmap_offset",
                             risk_heatmap_offset_);
        node_->get_parameter("manager/risk_heatmap_preview_lift",
                             risk_heatmap_preview_lift_);
        risk_heatmap_offset_ = std::clamp(risk_heatmap_offset_, 0.0, 5.0);
        node_->get_parameter("manager/risk_heatmap_safe_transparent",
                             risk_heatmap_safe_transparent_);
        node_->get_parameter("manager/risk_mask_viz_contours",
                             risk_mask_viz_contours_);
        node_->get_parameter("manager/risk_mask_viz_volume_alpha",
                             risk_mask_viz_volume_alpha_);
        node_->get_parameter("manager/risk_mask_viz_agl",
                             risk_mask_viz_agl_);
        risk_mask_softness_ = std::max(1e-4, risk_mask_softness_);
        risk_mask_viz_threshold_ =
            std::clamp(risk_mask_viz_threshold_, 0.0, 1.0);
        risk_mask_viz_contours_ = std::clamp(risk_mask_viz_contours_, 1, 9);
        risk_mask_viz_volume_alpha_ =
            std::clamp(risk_mask_viz_volume_alpha_, 0.02, 0.80);
        if (risk_mask_viz_mode_ != "heatmap" &&
            risk_mask_viz_mode_ != "heatmap3d" &&
            risk_mask_viz_mode_ != "volume" &&
            risk_mask_viz_mode_ != "fixed_agl" &&
            risk_mask_viz_mode_ != "fixed_msl") {
            RCLCPP_WARN(node_->get_logger(),
                        "Unknown risk_mask_viz_mode '%s'; using heatmap",
                        risk_mask_viz_mode_.c_str());
            risk_mask_viz_mode_ = "heatmap";
        }
        node_->get_parameter("manager/risk_smha_w", risk_smha_w_);
        node_->get_parameter("manager/front_end", front_end_str_);
        node_->get_parameter("manager/fm2_coarse_k", fm2_coarse_k_);
        node_->get_parameter("manager/fm2_max_cells", fm2_max_cells_);
        node_->get_parameter("manager/fm2_star", fm2_star_);
        node_->get_parameter("manager/fm2_alt_penalty", fm2_alt_penalty_);
        node_->get_parameter("manager/fm2_alt_zscale", fm2_alt_zscale_);
        node_->get_parameter("manager/fm2_alt_zscale_down", fm2_alt_zscale_dn_);
        node_->get_parameter("manager/fm2_rough_weight", fm2_rough_weight_);
        node_->get_parameter("manager/fm2_rough_slope0", fm2_rough_slope0_);
        node_->get_parameter("manager/astar_bypass_shortcut", astar_bypass_shortcut_);
        node_->get_parameter("manager/astar_step_size", astar_step_size_);
        node_->get_parameter("manager/sdf_voxel_size", sdf_voxel_size_);
        node_->get_parameter("manager/sdf_voxel_z", sdf_voxel_z_);
        // sdf_voxel_size <= 0 resolves to the DEM cell size once terrain
        // arrives (setTerrainData) — finer only burns memory, coarser loses
        // data, and the right value differs per world.
        if (sdf_voxel_z_ <= 0.0) sdf_voxel_z_ = 0.1;  // 10 m real (frame /100)
        node_->get_parameter("manager/ground_height", ground_height_);
        node_->get_parameter("manager/virtual_ceil_height", virtual_ceil_height_);
        node_->get_parameter("manager/map_ceiling_headroom", map_ceiling_headroom_);
        // A non-positive headroom would put the ceiling at the terrain peak
        // itself, leaving the wave no room to cross it. Fail back to the
        // shipped value rather than plan in a box the route cannot fit.
        if (!(map_ceiling_headroom_ > 0.0)) {
            log_manager_->warnf(
                "[BBOX] manager/map_ceiling_headroom %.2f is not positive — "
                "using 20.0", map_ceiling_headroom_);
            map_ceiling_headroom_ = 20.0;
        }
        node_->get_parameter("manager/obstacle_clearance", obstacle_clearance_);
        node_->get_parameter("manager/dyn_obstacle_margin", dyn_obstacle_margin_);
        node_->get_parameter("optimization/obstacle_clearance", opt_obstacle_clearance_);
        node_->get_parameter("optimization/weight_altitude", weight_altitude_);
        node_->get_parameter("optimization/alt_cap_headroom", alt_cap_headroom_);
        node_->get_parameter("optimization/alt_floor_headroom", alt_floor_headroom_);
        node_->get_parameter("manager/min_goal_agl", min_goal_agl_);
        node_->get_parameter("manager/align_start_vel_to_route",
                             align_start_vel_to_route_);
        node_->get_parameter("manager/zone_avoid_lexicographic",
                             zone_avoid_lexico_);
        node_->get_parameter("manager/corner_fillet_radius", corner_fillet_radius_);
        // Dynamic-obstacle spawn-orientation RNG seed. FIXED by default so the
        // SAME mission spawns the SAME oriented obstacles -> reproducible plans.
        // (The box yaw is NOT "visual only": oriented boxes change the SDF and
        // hence the FM2 route, so a random seed made identical missions plan
        // differently each run.) Set < 0 to randomize per run for robustness tests.
        {
          // int64, not int. The drawn seed is a uint32 printed with %u and
          // the log tells the reader to pin it — but read back into `int`,
          // every value above INT32_MAX wrapped NEGATIVE and fell into the
          // randomize branch below. Half of all "reproducible" seeds
          // reproduced nothing, silently, and the log said FIXED nowhere.
          int64_t yaw_seed = 42;
          node_->get_parameter("manager/dyn_yaw_seed", yaw_seed);
          std::mt19937::result_type applied;
          if (yaw_seed >= 0) {
            if (yaw_seed > 0xFFFFFFFFLL)
              log_manager_->warnf(
                  "dyn_yaw_seed %ld is outside the 32-bit seed range — "
                  "truncated, so this value does NOT replay a logged run",
                  static_cast<long>(yaw_seed));
            applied = static_cast<std::mt19937::result_type>(
                yaw_seed & 0xFFFFFFFFLL);
            log_manager_->infof("dyn_yaw_seed: FIXED %u", applied);
          } else {
            // Random mode still logs the drawn seed so ANY run is
            // reproducible after the fact: pin manager/dyn_yaw_seed to the
            // logged value and the obstacle layout replays exactly.
            applied = std::random_device{}();
            log_manager_->infof(
                "dyn_yaw_seed: RANDOM -> %u (pin manager/dyn_yaw_seed to this "
                "to reproduce)", applied);
          }
          yaw_rng_.seed(applied);
        }
        // Patches must extend at least as far as the dynamic berth, or the
        // distance query reads +inf before the margin is reached.
        if (dyn_obstacle_margin_ > sdf_manager_.influenceRadius())
            sdf_manager_.setInfluenceRadius(dyn_obstacle_margin_);

        // Building-mesh rendering of dynamic obstacles (see publishDynamicObstacles).
        node_->declare_parameter("obstacle_mesh_resource",
                                 std::string("package://mmp_viz_assets/meshes/building.dae"));
        node_->declare_parameter("obstacle_mesh_height", 60.0);
        // VISUAL-ONLY mesh magnification. Since the scenario-yaml unit fix,
        // obstacles render at true physical size — a 160 m ship is sub-pixel
        // at peninsula zoom, so scenario loads looked like "ships vanished".
        // Scales the MESH about the obstacle center only; the SDF primitive,
        // the invisible geometry companion marker and the altitude-panel
        // overlay all keep the exact collision size.
        node_->declare_parameter("obstacle_viz_scale", 1.0);
        // Rest every dynamic obstacle's BASE on the surface under it (terrain
        // elevation on land, sea level over water), ignoring the yaml z.
        // Scenario files carried center-z values that buried boxes (building:
        // center 0.35 with sz 1.6 -> 45 m underground, collision included).
        node_->declare_parameter("manager/obstacle_ground_snap", true);
        // Half-width (frame units) of the FE-floor swath published alongside
        // the underfoot floor: max floor within this lateral radius. Shows
        // the constraints the route DODGED (a clean dodge leaves its cause
        // beside the path, invisible to an underfoot profile).
        node_->get_parameter("obstacle_mesh_resource", obstacle_mesh_resource_);
        node_->get_parameter("obstacle_mesh_height", obstacle_mesh_height_);
        node_->get_parameter("obstacle_viz_scale", obstacle_viz_scale_);
        node_->get_parameter("manager/obstacle_ground_snap", obstacle_ground_snap_);
        if (obstacle_viz_scale_ < 1.0) obstacle_viz_scale_ = 1.0;

        // Visual mesh catalog: model name -> mesh resource + rendered native size [m]
        // (mesh base at z=0, XY centered). Add a model = drop a .dae in
        // mmp_viz_assets/meshes/ + one line here (measure size with trimesh).
        mesh_catalog_["building"] = { obstacle_mesh_resource_,
                                      Eigen::Vector3d(16.374, 13.358, 17.345) };
        mesh_catalog_["car"]      = { "package://mmp_viz_assets/meshes/car.dae",
                                      Eigen::Vector3d(17.679, 10.093, 4.620) };
        mesh_catalog_["ship"]     = { "package://mmp_viz_assets/meshes/simple_ship.dae",
                                      Eigen::Vector3d(9.972, 42.275, 10.234) };

        // Per-model override of obstacle_viz_scale (<= 0 inherits the global).
        for (auto & kv : mesh_catalog_) {
            const std::string pname = "obstacle_viz_scale_" + kv.first;
            node_->declare_parameter(pname, -1.0);
            double v = -1.0;
            node_->get_parameter(pname, v);
            kv.second.viz_scale = (v > 0.0) ? v : obstacle_viz_scale_;
        }

        // ESDF cache resolution:
        //   manager/world  : map name (default "small_island"); RViz
        //                    MapSelector overrides.
        // The old <world>.esdf disk cache is GONE: with terrain in the 2.5D
        // heightmap the static SDF is empty, so caching wrote ~3.6 GB of
        // constant free-space per world — and loading a PRE-heightmap cache
        // silently resurrected voxel terrain (double-count vs the heightmap
        // term). The boxes-only grid now builds instantly when terrain arrives.
        node_->declare_parameter("manager/world", std::string("small_island"));
        std::string world_name;
        node_->get_parameter("manager/world", world_name);
        log_manager_->infof("world='%s' (no ESDF disk cache; boxes-only SDF builds on demand)",
                            world_name.c_str());

        // Parse risk zones: [cx, cy, cz, sensing_range, max_risk_level, ...]
        node_->declare_parameter("risk_zones", std::vector<double>{});
        std::vector<double> tz_params;
        node_->get_parameter("risk_zones", tz_params);
        log_manager_->infof("Risk zone params size: %zu", tz_params.size());
        if (tz_params.size() >= 5 && tz_params.size() % 5 == 0) {
            for (size_t ti = 0; ti < tz_params.size(); ti += 5) {
                RiskZone tz;
                tz.center = Eigen::Vector3d(tz_params[ti], tz_params[ti+1], tz_params[ti+2]);
                tz.reach = tz_params[ti+3];
                tz.peak = tz_params[ti+4];
                risk_zones_raw_.push_back(tz);
                log_manager_->infof("  RiskZone #%zu: center=(%.1f,%.1f,%.1f) range=%.1f risk=%.1f",
                    risk_zones_raw_.size()-1, tz.center.x(), tz.center.y(), tz.center.z(),
                    tz.reach, tz.peak);
            }
            log_manager_->infof("Loaded %zu risk zones (risk_weight=%.1f)", risk_zones_raw_.size(), risk_weight_);
        } else if (tz_params.empty()) {
            log_manager_->infof("No risk zones configured");
        } else {
            log_manager_->warnf("Invalid risk_zones param size: %zu (must be multiple of 5)", tz_params.size());
        }

        // Latched so a late-joining altitude panel still gets the last
        // front-end route (transient_local pub serves volatile subs fine).
        front_end_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/planning/front_end_path",
            rclcpp::QoS(1).reliable().transient_local());
        // Trajectory tube channels (see publishTrajTube in the header). Same
        // latched QoS as the /planning sources they mirror.
        const auto tube_qos = rclcpp::QoS(1).reliable().transient_local();
        opt_traj_tube_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/opt_trajectory", tube_qos);
        global_traj_tube_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/global_trajectory", tube_qos);
        front_end_tube_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/front_end_path", tube_qos);
        // Tube diameter; parameterized because the right size depends on the
        // map scale being viewed (5.0 read as a 500 m sausage nationwide).
        node_->declare_parameter("path_scale", 1.5);
        node_->get_parameter("path_scale", path_scale_);
        // TRANSIENT_LOCAL so RViz, joining late, still gets the latest set.
        rclcpp::QoS dyn_qos(1);
        dyn_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        dyn_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        dyn_obstacle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/viz/dynamic_obstacles", dyn_qos);
        // The launch default is drone_id=1 and single-agent runs do not
        // necessarily create drone_0, so risk visualization cannot use the
        // terrain-status publisher's drone_0-only ownership rule. Multiple
        // planners publish an identical id/namespace set and are harmless.
        //
        // Depth 1: the field is ONE MarkerArray per refresh, so the latched
        // sample is the complete picture. The former per-marker burst needed a
        // deep queue (7 markers x N zones) and still truncated silently past
        // ~18 zones, leaving late joiners with a half-drawn field.
        risk_field_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/viz/risk_field", rclcpp::QoS(1).reliable().transient_local());
        // [CHAIN-VIZ] per-segment chain view — same one-latched-array
        // contract as the risk field.
        chain_segments_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/viz/chain_segments", rclcpp::QoS(1).reliable().transient_local());
        // [DEBUG-PIPELINE] debug-mode only (see the member comment). Off by
        // default: the esdf overlay taught us that always-on debug output
        // with no watcher is pure cost — here the publisher itself is only
        // created in debug mode, so `ros2 topic list` tells the truth.
        node_->declare_parameter("manager/debug_pipeline_viz", false);
        node_->get_parameter("manager/debug_pipeline_viz", debug_pipeline_viz_);
        if (debug_pipeline_viz_) {
            debug_pipeline_pub_ =
                node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                    "/debug/pipeline",
                    rclcpp::QoS(1).reliable().transient_local());
            RCLCPP_INFO(node_->get_logger(),
                        "[DEBUG-PIPELINE] /debug/pipeline enabled "
                        "(shortcut_points + inner_points per plan)");
        }
        // One latched map; a second grid_map_rviz_plugin display drapes it
        // over the terrain (same message-layout contract as /terrain/grid_map).
        rclcpp::QoS heatmap_qos(1);
        heatmap_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        heatmap_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        // [TRAJ-RISK] latched: published once per plan; a late-joining RViz
        // must still see the colored line.
        traj_risk_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/traj_risk", rclcpp::QoS(1).reliable().transient_local());
        // [RISK-PROFILE] altitude-panel dome/roof channels, latched per plan.
        risk_profile_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/viz/risk_profile", rclcpp::QoS(1).reliable().transient_local());
        risk_heatmap_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(
            "/viz/risk_heatmap", heatmap_qos);

        // Terrain may arrive after construction. This first call publishes an
        // ideal circular fallback; setTerrainData() replaces it with the DEM-
        // masked field as soon as the heightmap is ready.
        refreshEffectiveRiskZones();
        rebuildTerrainRiskMasks();
    }

    void PathManager::initOptimizer(bool force_reinit)
    {
        if (is_optimizer_initialized_ && poly_traj_opt_ && !force_reinit)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Optimizer already initialized for drone %d", traj_.local_traj.drone_id);
            return;
        }

        if (force_reinit && is_optimizer_initialized_) {
            RCLCPP_INFO(node_->get_logger(), "Force reinitializing optimizer for drone %d", traj_.local_traj.drone_id);
        }

        // Reset state in case of partial initialization
        is_optimizer_initialized_ = false;
        poly_traj_opt_.reset();

        try {
            RCLCPP_INFO(node_->get_logger(), "Initializing optimizer for drone %d...", traj_.local_traj.drone_id);
            poly_traj_opt_ = makeConfiguredOptimizer();
            // Only mark as initialized after all steps succeed
            is_optimizer_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Optimizer initialized successfully for drone %d", traj_.local_traj.drone_id);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during optimizer initialization: %s", e.what());
            poly_traj_opt_.reset();  // Reset to nullptr on failure
            is_optimizer_initialized_ = false;
            throw;  // Re-throw the exception
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown exception during optimizer initialization");
            poly_traj_opt_.reset();
            is_optimizer_initialized_ = false;
            throw;
        }
    }

    std::unique_ptr<ego_planner::PolyTrajOptimizer>
    PathManager::makeConfiguredOptimizer()
    {
        {
            // Check prerequisites
            if (!node_) {
                throw std::runtime_error("Node is null");
            }

            auto poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();

            // Set LogManager for unified logging
            poly_traj_opt_->setLogManager(log_manager_);

            // Set parameters first to ensure node_ is initialized
            poly_traj_opt_->setParam(node_);
            poly_traj_opt_->setDroneId(traj_.local_traj.drone_id);

            // Four optimization/* values are consumed by PathManager, not by
            // setParam — re-read them on every (forced) re-init, or a
            // per-segment override ([CHAIN] chain/seg<i>/params) is a silent
            // no-op, and alt_cap_headroom DESYNCHRONIZES from the optimizer's
            // own copy (alt_cap_headroom_opt_), which setParam does re-read:
            // the scalar cap and the arc-varying envelope must share one
            // headroom (see the note at its declaration in setParam).
            node_->get_parameter("optimization/obstacle_clearance",
                                 opt_obstacle_clearance_);
            node_->get_parameter("optimization/weight_altitude",
                                 weight_altitude_);
            node_->get_parameter("optimization/alt_cap_headroom",
                                 alt_cap_headroom_);
            node_->get_parameter("optimization/alt_floor_headroom",
                                 alt_floor_headroom_);

            // Wire SDF-based obstacle avoidance into the optimizer.
            poly_traj_opt_->setSDFManager(&sdf_manager_);
            // Strictly below the front-end margin: a margin-respecting path then
            // carries ZERO obstacle cost, so its 10000-weighted gradient cannot
            // pin the geometry to the polyline and smoothness can round corners
            // inside the (margin - clearance) buffer.
            poly_traj_opt_->setObstacleClearance(
                std::min(opt_obstacle_clearance_, obstacle_clearance_));
            {
                // [REJECT] ancestor safety net (default ON): an audit-failed
                // (terrain/obstacle-overlapping) trajectory is discarded, not
                // published. optimization/collision_reject: false restores the
                // old logs-only behavior.
                bool reject_on = true;
                if (!node_->has_parameter("optimization/collision_reject"))
                    node_->declare_parameter("optimization/collision_reject", true);
                node_->get_parameter("optimization/collision_reject", reject_on);
                poly_traj_opt_->setCollisionReject(reject_on);

                // [CONV-REJECT] non-convergence gate: an L-BFGS failure exit
                // whose envelope violations exceed the threshold is discarded
                // like a collision (healthy plans <1%; the r3 goal-in-zone
                // deadlock 18.4%).
                bool env_reject_on = true;
                double env_viol_max = 0.25;
                double env_viol_hard = 0.30;
                if (!node_->has_parameter("optimization/audit_envelope_reject"))
                    node_->declare_parameter(
                        "optimization/audit_envelope_reject", true);
                if (!node_->has_parameter(
                        "optimization/audit_envelope_violation_max"))
                    node_->declare_parameter(
                        "optimization/audit_envelope_violation_max", 0.25);
                if (!node_->has_parameter(
                        "optimization/audit_envelope_violation_hard_max"))
                    node_->declare_parameter(
                        "optimization/audit_envelope_violation_hard_max", 0.30);
                node_->get_parameter("optimization/audit_envelope_reject",
                                     env_reject_on);
                node_->get_parameter(
                    "optimization/audit_envelope_violation_max", env_viol_max);
                node_->get_parameter(
                    "optimization/audit_envelope_violation_hard_max",
                    env_viol_hard);
                poly_traj_opt_->setEnvelopeReject(env_reject_on, env_viol_max,
                                                  env_viol_hard);

                // [GNRON] endpoint moat taper — same radius as the front-end
                // (wired in initSearcher) so both stages price one field.
                poly_traj_opt_->setRiskGoalTaperRadius(risk_goal_taper_radius_);

                // [LBFGS-TUNE] solver knobs for no-rebuild parameter sweeps.
                // Defaults reproduce the hardcoded values exactly.
                {
                    auto dp = [&](const char *n, auto v) {
                        if (!node_->has_parameter(n)) node_->declare_parameter(n, v);
                    };
                    dp("optimization/lbfgs_mem_size", 64);
                    dp("optimization/lbfgs_g_epsilon", 0.1);
                    dp("optimization/lbfgs_past", 3);
                    dp("optimization/lbfgs_delta", 1.0e-6);
                    dp("optimization/lbfgs_max_linesearch", 0);
                    dp("optimization/lbfgs_f_dec", 0.0);
                    dp("optimization/lbfgs_s_curv", 0.0);
                    int mem = 64, past = 3, maxls = 0;
                    double geps = 0.1, delta = 1.0e-6, fdec = 0.0, scurv = 0.0;
                    node_->get_parameter("optimization/lbfgs_mem_size", mem);
                    node_->get_parameter("optimization/lbfgs_g_epsilon", geps);
                    node_->get_parameter("optimization/lbfgs_past", past);
                    node_->get_parameter("optimization/lbfgs_delta", delta);
                    node_->get_parameter("optimization/lbfgs_max_linesearch", maxls);
                    node_->get_parameter("optimization/lbfgs_f_dec", fdec);
                    node_->get_parameter("optimization/lbfgs_s_curv", scurv);
                    poly_traj_opt_->setLbfgsParams(mem, geps, past, delta,
                                                   maxls, fdec, scurv);
                }
            }
            poly_traj_opt_->setGroundHeight(ground_height_);
            poly_traj_opt_->setVirtualCeilHeight(virtual_ceil_height_);
            // 2.5D terrain heightmap: the optimizer queries the DEM elevation
            // directly (exact z) instead of the coarse voxelised SDF, so terrain
            // clearance matches the panel/DEM. terrain_data_ outlives the optimizer.
            poly_traj_opt_->setTerrainHeightmap(
                [this](double x, double y) -> float {
                    return terrain_data_.valid
                             ? terrain_data_.getElevation(x, y)
                             : -std::numeric_limits<float>::infinity();
                },
                // DEM cell in frame units — scales the SWATH-FLOOR sampling
                // pitch to the actual grid (corridor 30 m vs
                // full_map 250 m).
                terrain_data_.valid ? terrain_data_.resolution : 0.0);
            // Value + analytic slope of the same surface, for the terrain term
            // and alt-cap gate: cost and gradient must agree exactly (see
            // TerrainData::getElevationAndGrad).
            poly_traj_opt_->setTerrainHeightGrad(
                [this](double x, double y, float *h, float *gx, float *gy) -> bool {
                    return terrain_data_.getElevationAndGrad(x, y, h, gx, gy);
                });

            // Pass risk zones to optimizer for trajectory fine-tuning (2nd stage)
            if (!risk_zones_.empty()) {
                std::vector<ego_planner::RiskZone> opt_zones;
                for (const auto &tz : risk_zones_) {
                    ego_planner::RiskZone oz;
                    oz.center = tz.center;
                    oz.reach = tz.reach;
                    oz.peak = tz.peak;
                    oz.vertical_reach = tz.reach * risk_vertical_ratio_;
                    opt_zones.push_back(oz);
                }
                poly_traj_opt_->setRiskZones(opt_zones);
                log_manager_->infof("Passed %zu risk zones to optimizer", opt_zones.size());
            }
            poly_traj_opt_->setRiskVisibility(
                [this](size_t zi, const Eigen::Vector3d &p,
                       Eigen::Vector3d *grad) -> double {
                    return riskVisibility(zi, p, grad);
                });
            // [OCCLUSION-CAP] raw LOS horizon for the visibility-aware cap: same
            // zone indexing as the visibility callback above.
            poly_traj_opt_->setRiskShadowCeiling(
                [this](size_t zi, const Eigen::Vector3d &p) -> double {
                    return riskShadowCeiling(zi, p);
                });

            // NOTE: the local unique_ptr deliberately shadows the member of
            // the same name so this configuration body — moved verbatim from
            // initOptimizer — reads unchanged. initOptimizer assigns the
            // returned instance to the member; [CHAIN-PAR] workers keep
            // their own.
            return poly_traj_opt_;
        }
    }

    double PathManager::effectiveHandoffMaxMps() const
{
    const auto *dynp = dynamicsParams();
    const double model_max =
        dynp ? dynp->speed_max_mps : std::numeric_limits<double>::infinity();
    double um_xy = 100.0;
    if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
        node_->get_parameter("optimization/dynamics_unit_xy_m", um_xy);
    double explicit_cap = 0.0;
    if (node_->has_parameter("planning/handoff_max_vel_mps"))
        node_->get_parameter("planning/handoff_max_vel_mps", explicit_cap);
    double planning_cap = 0.0;
    if (node_->has_parameter("optimization/max_vel")) {
        double mv = 0.0;
        node_->get_parameter("optimization/max_vel", mv);
        planning_cap = mv * um_xy;
    }
    const double cap_mps =
        explicit_cap > 1e-9
            ? explicit_cap
            : (planning_cap > 1e-9 ? planning_cap : model_max);
    return std::min(model_max, cap_mps);
}

std::string PathManager::stateEnvelopeProblem(
        const Eigen::Vector3d &vel_units) const
    {
        // Finiteness is judged BEFORE the model-off early return (review
        // find): a NaN state is unplannable whether or not the dynamics
        // model is loaded, and "model off" must not certify it.
        if (!vel_units.allFinite()) return "non-finite velocity";
        if (!poly_traj_opt_ || !poly_traj_opt_->dynamicsEnabled()) return {};
        const auto unit = [&](const char *n, double def) {
            return node_->has_parameter(n) ? node_->get_parameter(n).as_double()
                                           : def;
        };
        const double um_xy = unit("optimization/dynamics_unit_xy_m", 100.0);
        const double um_z = unit("optimization/dynamics_unit_z_m", 100.0);
        const double vh = std::hypot(vel_units.x() * um_xy,
                                     vel_units.y() * um_xy);
        const double vz = vel_units.z() * um_z;
        const double vm = std::hypot(vh, vz);
        const auto &dyn = poly_traj_opt_->dynamicsParams();
        const double floor_mps =
            poly_traj_opt_->dynamicsMinSpeedFloorUnits() * um_xy;
        char buf[160];
        // [SPEED-BOUNDARY] see path_manager.h. Commanding exactly the floor
        // must pass whatever the direction; the unit round trip puts some
        // directions 2.8e-14 m/s under it — (1,1,0) and r5's own first-leg
        // aim (-18.3, -199.2, 0) among them.
        if (belowSpeedBoundary(vm, floor_mps)) {
            snprintf(buf, sizeof buf,
                     "speed %.1f m/s below the margin-backed cruise floor "
                     "%.1f m/s", vm, floor_mps);
            return buf;
        }
        // THREE ceilings, three meanings — conflated once, separated on
        // review find:
        //   dynamics_speed_max      the physics model's validity ceiling
        //   optimization/max_vel    the planner's frame cap (cubic soft
        //                           cost in the solver; contracts author
        //                           at it) — a PLANNING preference
        //   planning/handoff_max_vel_mps
        //                           the ENTRY/HANDOFF contract ceiling
        //                           this validator enforces. Default 0 =
        //                           follow the frame cap, which keeps the
        //                           gate consistent with what the solver
        //                           will accept (a 200-230 m/s state must
        //                           not pass the entrance and then fail in
        //                           the solve). Setting it explicitly
        //                           decouples the handoff contract from
        //                           planning preference — whether an
        //                           above-cap state should instead
        //                           classify as TRANSITION_REQUIRED is a
        //                           contract-2 decision, made there.
        // Which ceiling binds decides both the number AND the reason string:
        //   explicit handoff cap   planning/handoff_max_vel_mps > 0
        //   default planning cap   handoff unset -> the optimizer's own
        //                          optimization/max_vel. POLICY alignment,
        //                          not hard enforcement: in the solver that
        //                          value is a cubic SOFT cost, so following
        //                          it keeps the handoff contract consistent
        //                          with planning preference without
        //                          claiming the solve guarantees it. (The
        //                          manager copy is a separate parameter
        //                          and may drift — not used here.)
        //   model maximum          dynamics speed_max is the tightest
        const double vmax_mps = effectiveHandoffMaxMps();
        double explicit_cap = 0.0;
        if (node_->has_parameter("planning/handoff_max_vel_mps"))
            node_->get_parameter("planning/handoff_max_vel_mps",
                                 explicit_cap);
        // [SPEED-BOUNDARY] INCLUSIVE at the cap, same rule as the floor.
        // Eight of the ten shipped missions state exactly
        // optimization/max_vel * unit = 200.0 m/s, and a strict > put every
        // one of them on the last bits of the direction normalization.
        if (aboveSpeedBoundary(vm, vmax_mps)) {
            const char *which =
                vmax_mps >= dyn.speed_max_mps - 1e-9
                    ? "model maximum"
                    : (explicit_cap > 1e-9 ? "explicit handoff cap"
                                           : "default planning cap");
            snprintf(buf, sizeof buf,
                     "speed %.1f m/s above the effective maximum %.1f m/s "
                     "(%s)", vm, vmax_mps, which);
            return buf;
        }
        const double gamma = std::atan2(std::abs(vz), vh);
        if (gamma > dyn.flight_path_angle_max_rad) {
            snprintf(buf, sizeof buf,
                     "flight path angle %.1f deg outside the +/-%.1f deg "
                     "validity cone",
                     gamma * 180.0 / M_PI,
                     dyn.flight_path_angle_max_rad * 180.0 / M_PI);
            return buf;
        }
        return {};
    }

    std::string PathManager::pvaEnvelopeProblem(
        const Eigen::Vector3d &pos_units, const Eigen::Vector3d &vel_units,
        const Eigen::Vector3d &acc_units) const
    {
        // Same ordering doctrine as stateEnvelopeProblem: every component of
        // the state is checked for finiteness FIRST, so a disabled dynamics
        // model cannot certify a NaN input by short-circuiting.
        if (!pos_units.allFinite()) return "non-finite position";
        if (!acc_units.allFinite()) return "non-finite acceleration";
        const std::string vel_problem = stateEnvelopeProblem(vel_units);
        if (!vel_problem.empty()) return vel_problem;
        if (!poly_traj_opt_ || !poly_traj_opt_->dynamicsEnabled()) return {};
        const auto unit = [&](const char *n, double def) {
            return node_->has_parameter(n) ? node_->get_parameter(n).as_double()
                                           : def;
        };
        const double um_xy = unit("optimization/dynamics_unit_xy_m", 100.0);
        const double um_z = unit("optimization/dynamics_unit_z_m", 100.0);
        const Eigen::Vector3d S(um_xy, um_xy, um_z);
        const auto &dyn = poly_traj_opt_->dynamicsParams();
        const auto ev = mmp_vehicle_dynamics::evaluateInverseDynamics(
            dyn, S.cwiseProduct(pos_units), S.cwiseProduct(vel_units),
            S.cwiseProduct(acc_units));
        char buf[160];
        if (!ev.valid) {
            return "inverse dynamics undefined for this state — cannot be "
                   "certified";
        }
        if (!mmp_vehicle_dynamics::isWithinEnvelope(dyn, ev)) {
            mmp_vehicle_dynamics::EnvelopeLimit lim;
            const double u =
                mmp_vehicle_dynamics::envelopeUtilization(dyn, ev, &lim);
            snprintf(buf, sizeof buf,
                     "flying this exact state demands %.0f%% of the %s limit",
                     100.0 * u, mmp_vehicle_dynamics::envelopeLimitName(lim));
            return buf;
        }
        return {};
    }

    bool PathManager::planGlobalTraj(const StartHead &head,
                                     const std::vector<Eigen::Vector3d> &waypoints,
                                     const ego_planner::TailBoundary &tail,
                                     bool junction_goal,
                                     const std::vector<Eigen::Vector3d> *route_override,
                                     const std::vector<double> *cap_ref_override,
                                     bool front_end_only,
                                     const RouteProvenance *provenance)
    {
        const Eigen::Vector3d &start_pos = head.pos_u;
        const Eigen::Vector3d &start_vel = head.vel_u;
        const Eigen::Vector3d &start_acc = head.acc_u;
        // Same fail-closed entry as SegmentChainPlanner::plan: a
        // default-constructed head is UNSPECIFIED, and planning from a state
        // nobody described is what this contract exists to stop.
        last_head_policy_ = HeadPolicy{};
        // [LEG-POLICY] Provenance describes THIS call's products. Every
        // refusal below returns before they are written, so leaving them
        // standing would have the accessors describe the last plan that
        // SUCCEEDED while the caller is handling a failure — and the audit
        // reads them by size against a trajectory that is no longer the one
        // they came from. Cleared here, before the first return, rather than
        // at each of them.
        //
        // leg_policies_ is deliberately NOT cleared with them. It belongs to
        // the front-end epoch, not to a call: the inherited-route branch runs
        // no search and relies on the capture surviving from the run that
        // committed the route it is re-solving.
        last_route_edge_leg_.clear();
        last_route_epoch_ = 0;
        last_piece_leg_.clear();
        // Same argument, same call: the committed route and its cap are
        // products of THIS plan. Their two production readers run only after
        // a success, but "nobody reads it wrong today" is not the property —
        // "it cannot be read wrong" is.
        last_clean_path_.clear();
        last_cap_ref_.clear();
        if (head.src == StartStateSource::UNSPECIFIED) {
            log_manager_->errorf(
                "[HEAD-POLICY] planGlobalTraj called with an UNSPECIFIED "
                "start-state source — refusing");
            return false;
        }
        if (!start_pos.allFinite() || !start_vel.allFinite() ||
            !start_acc.allFinite()) {
            log_manager_->errorf(
                "[HEAD-POLICY] planGlobalTraj called with a non-finite start "
                "state — refusing");
            return false;
        }
        log_manager_->infof("Planning global trajectory with %zu waypoints", waypoints.size());
        auto t_total_start = std::chrono::steady_clock::now();

        if (waypoints.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "planGlobalTraj: No waypoints provided!");
            return false;
        }

        // Waypoint z is HEIGHT ABOVE TERRAIN (AGL), not absolute: mission
        // sources cannot know the DEM, so an absolute z routinely ended up
        // inside a hill (SDF probe < 0 -> infeasible pinned tail -> -1005
        // with the obstacle cost frozen). z_abs = elevation(x,y) + max(z,
        // min_goal_agl). Over water / outside the DEM the elevation is 0-ish
        // by construction (getElevation invalid), so AGL == ASL there and
        // legacy over-water missions behave identically.
        std::vector<Eigen::Vector3d> wps = waypoints;
        if (terrain_data_.valid) {
            for (size_t wi = 0; wi < wps.size(); ++wi) {
                auto &wp = wps[wi];
                // [CHAIN] A junction goal's z is ABSOLUTE — sampled from a
                // trajectory the baseline plan actually flies — not AGL.
                // Re-adding the terrain elevation here would lift the goal
                // off the junction contract and break the chained head/tail
                // match at the seam. Interior waypoints (if any) keep their
                // AGL semantics: they are genuine mission points.
                if (junction_goal && wi + 1 == wps.size()) continue;
                const double agl = std::max(wp.z(), min_goal_agl_);
                const float elev = terrain_data_.getElevation(wp.x(), wp.y());
                const double base =
                    (elev > -1e9f) ? static_cast<double>(elev) : 0.0;
                const double z_abs = base + agl;
                if (std::abs(z_abs - wp.z()) > 1e-9) {
                    log_manager_->infof(
                        "[GOAL AGL] waypoint (%.1f, %.1f) z=%.2f AGL "
                        "-> absolute %.2f (terrain %.2f + agl %.2f)",
                        wp.x(), wp.y(), wp.z(), z_abs, base, agl);
                }
                wp.z() = z_abs;
            }
        }

        // Diagnostic guard: warn if the mission start or any waypoint falls
        // OUTSIDE the terrain bbox. Off-DEM points are treated as sea-level (0)
        // water by getElevation — legitimate for over-water legs. But after a
        // corridor/map publisher failure the TRANSIENT_LOCAL topic can still
        // serve a STALE map whose bbox no longer covers the mission, which
        // would silently plan terrain-less. We WARN (not refuse) so genuine
        // over-water missions still plan, but a stale-map swap is visible.
        if (terrain_data_.valid) {
            Eigen::Vector3d tlo, thi;
            if (computeTerrainBBox(&tlo, &thi)) {
                auto oob = [&](const Eigen::Vector3d &p) {
                    return p.x() < tlo.x() || p.x() > thi.x() ||
                           p.y() < tlo.y() || p.y() > thi.y();
                };
                int n_oob = oob(start_pos) ? 1 : 0;
                for (const auto &wp : wps) if (oob(wp)) ++n_oob;
                if (n_oob > 0) {
                    log_manager_->warnf(
                        "[TERRAIN-BOUNDS] %d of %zu mission point(s) fall "
                        "OUTSIDE terrain bbox [%.0f,%.0f]-[%.0f,%.0f]; those "
                        "legs use sea-level (off-DEM). If unexpected, the served "
                        "map may be STALE (publisher failure).",
                        n_oob, wps.size() + 1, tlo.x(), tlo.y(), thi.x(), thi.y());
                }
            }
        }

        // === STEP 1: Build waypoint sequence ===
        // Build segment list: start -> wp1 -> wp2 -> ... -> wpN
        std::vector<Eigen::Vector3d> all_points;
        all_points.push_back(start_pos);
        for (const auto& wp : wps) {
            all_points.push_back(wp);
        }

        // Compute map bounds from waypoints
        map_lower_bound_ = start_pos;
        map_upper_bound_ = start_pos;

        // Extend bounds to include all waypoints with margin
        double bound_margin_xy = 10.0;
        // Z margin below the lowest waypoint. Without this the map bottom
        // sits exactly on the start altitude, meaning obstacles anchored at
        // that altitude touch the map floor — their SDF gradient looks
        // asymmetric (no voxels below, voxels above), which tricks L-BFGS
        // into attempting a vertical-only escape that smoothness then blocks.
        const double bound_margin_z_below = 5.0;

        auto expand_points = [&](double margin_xy) {
            for (const auto& pt : all_points) {
                map_lower_bound_.x() = std::min(map_lower_bound_.x(), pt.x() - margin_xy);
                map_lower_bound_.y() = std::min(map_lower_bound_.y(), pt.y() - margin_xy);
                map_lower_bound_.z() = std::min(map_lower_bound_.z(), pt.z() - bound_margin_z_below);
                map_upper_bound_.x() = std::max(map_upper_bound_.x(), pt.x() + margin_xy);
                map_upper_bound_.y() = std::max(map_upper_bound_.y(), pt.y() + margin_xy);
            }
        };
        expand_points(bound_margin_xy);

        // Include only risk zones whose moat can actually TOUCH the mission
        // area. The old code covered EVERY configured zone and inflated the
        // margin to the largest reach: the default scenario layout ~3000 u
        // from a k3 corridor mission blew its FM2 grid from 1.3M to 1.4G
        // cells (0.55 s -> 18 s eikonal) and drove k2's grid build into
        // outright failure (grid=0x0x0 -> silent front-end fallback) — for
        // zones the route could never meet. Fixpoint so zone-zone chains
        // still work: a zone that touches an included zone's footprint can
        // shape the detour around it.
        std::vector<char> zone_in(risk_zones_.size(), 0);
        double max_included_reach = 0.0;
        bool grew = true;
        while (grew) {
            grew = false;
            for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
                if (zone_in[zi]) continue;
                const auto &tz = risk_zones_[zi];
                const double r = tz.reach + 5.0;
                const double dx = std::max({map_lower_bound_.x() - tz.center.x(),
                                            0.0,
                                            tz.center.x() - map_upper_bound_.x()});
                const double dy = std::max({map_lower_bound_.y() - tz.center.y(),
                                            0.0,
                                            tz.center.y() - map_upper_bound_.y()});
                if (std::hypot(dx, dy) > r) continue;
                zone_in[zi] = 1;
                grew = true;
                max_included_reach = std::max(max_included_reach, tz.reach);
                map_lower_bound_.x() = std::min(map_lower_bound_.x(), tz.center.x() - r);
                map_lower_bound_.y() = std::min(map_lower_bound_.y(), tz.center.y() - r);
                map_upper_bound_.x() = std::max(map_upper_bound_.x(), tz.center.x() + r);
                map_upper_bound_.y() = std::max(map_upper_bound_.y(), tz.center.y() + r);
            }
        }
        if (max_included_reach > 0.0) {
            // Routing berth around zones that DO matter (old semantics, but
            // scoped to included zones instead of the global maximum).
            bound_margin_xy = std::max(bound_margin_xy, max_included_reach + 5.0);
            expand_points(bound_margin_xy);
        }
        {
            const size_t n_in = std::count(zone_in.begin(), zone_in.end(), 1);
            if (n_in < risk_zones_.size()) {
                log_manager_->infof(
                    "[BBOX] %zu/%zu risk zones excluded from map bounds "
                    "(moat cannot reach the mission area)",
                    risk_zones_.size() - n_in, risk_zones_.size());
            }
        }

        // Z bounds: sample terrain along route to find max elevation
        double max_terrain_z = 0.0;
        if (terrain_data_.valid) {
            for (size_t i = 0; i < all_points.size() - 1; ++i) {
                Eigen::Vector3d dir = all_points[i+1] - all_points[i];
                double dist = dir.head<2>().norm();
                int n_samples = std::max(2, (int)(dist / 2.0));
                for (int s = 0; s <= n_samples; ++s) {
                    double t = (double)s / n_samples;
                    Eigen::Vector3d p = all_points[i] + t * dir;
                    // Also sample laterally
                    for (double offset : {-bound_margin_xy, 0.0, bound_margin_xy}) {
                        float elev = terrain_data_.getElevation(p.x() + offset, p.y());
                        if (elev > -1e10) max_terrain_z = std::max(max_terrain_z, (double)elev);
                        elev = terrain_data_.getElevation(p.x(), p.y() + offset);
                        if (elev > -1e10) max_terrain_z = std::max(max_terrain_z, (double)elev);
                    }
                }
            }
        }
        // Upper bound: max(terrain peak, flight altitude) + headroom. The max()
        // keeps the flight altitude inside the box even over low terrain.
        // The headroom sets the FM2 grid's z extent and so its cell count
        // (see manager/map_ceiling_headroom).
        //
        // RISK ZONE TOPS ARE NOT IN THIS MAX. Scope of that claim, since an
        // earlier version of this comment overreached twice: what is
        // MEASURED is that covering them bought nothing and cost a lot on
        // r6/r7. It is NOT a general law — if a zone top sits below an
        // altitude the platform can actually reach, overflying it may well
        // be a legitimate route, and nothing here decides that. What is
        // definitely wrong is the reason first given (that the box steers
        // the route); see below.
        //
        // Measured with the shipped risk_vertical_ratio and the zone sets
        // the r6/r7 scenarios actually load (8 zones each), sweeping the
        // ceiling over a 4x range:
        //     r6  ceiling 21.47 / 35.47 / 53.82  ->  geodesic [1.99, 6.60]
        //     r7  ceiling 13.95 / 27.95 / 46.64  ->  geodesic [3.01, 5.56]
        // The geodesic does not move at all. Covering the tops only inflated
        // the z grid (r6 128 -> 189 cells) and the plan (4922 -> 7110 ms,
        // +44%). The box is a PERMISSIVE BOUND, not an attractor: the wave
        // goes where the speed field sends it, and raising the lid over
        // airspace nothing wants to use just buys cells to fill.
        //
        // The zonewall regression does fail when tops are covered, and that
        // failure is real — but it is stage-specific and its mechanism is
        // not "the wave climbs to the ceiling". In the same run, ceiling
        // 157.51 produced a LOWER climb (47.33) than ceiling 112.68 did
        // (81.55), and a third stage at ceiling 111.18 climbed only 23.93
        // and planned CLEAN. That fixture forces risk_vertical_ratio to 3.0
        // (8.6x shipped) with LOS masking off specifically to seal an
        // over-the-top escape, so raising the lid dissolves its premise;
        // what it demonstrates is that the fixture depends on the lid, not
        // that production does.
        //
        // What the failing stages DO show is worth keeping in view: the
        // rejects there name maximum thrust and bank angle, not altitude.
        // The geodesic is shear-limited to the model's flight-path VALIDITY
        // cone (tan 30 deg), while the thrust-sustainable climb grade is
        // ~7 deg — a number segment_chain_planner already computes from the
        // same shared model and never tells the front end. A route the front
        // end may draw at 30 deg and the back end can only fly at 7 is the
        // actual gap; the bounding box is not where it lives.
        map_upper_bound_.z() =
            std::max(max_terrain_z, map_upper_bound_.z()) + map_ceiling_headroom_;

        // zone_top is REPORTED but not applied: the sweep above showed the
        // geodesic ignores the lid, so this number exists to make "is a zone
        // above the ceiling?" answerable from a log rather than only by
        // patching the planner.
        double max_zone_top = 0.0;
        int n_zone_top = 0;
        for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
            if (!zone_in[zi]) continue;
            const double top = risk_zones_[zi].center.z() +
                               risk_zones_[zi].reach * risk_vertical_ratio_;
            if (top > max_zone_top) max_zone_top = top;
            ++n_zone_top;
        }
        log_manager_->infof(
            "[BBOX] headroom=%.2f terrain_peak=%.2f zone_top=%.2f (%d zones, "
            "not covered) -> ceiling %.2f",
            map_ceiling_headroom_, max_terrain_z, max_zone_top, n_zone_top,
            map_upper_bound_.z());
        log_manager_->infof("Map bounds: lower=(%.2f,%.2f,%.2f), upper=(%.2f,%.2f,%.2f)",
            map_lower_bound_.x(), map_lower_bound_.y(), map_lower_bound_.z(),
            map_upper_bound_.x(), map_upper_bound_.y(), map_upper_bound_.z());

        // Build the boxes-only SDF grid ONCE over the full-terrain bbox (or
        // the mission bbox when no terrain exists). The static layer is empty
        // — terrain lives in the 2.5D heightmap — so this is instant and
        // allocation-free. Building once matters beyond speed: dynamic
        // obstacle patches clip against this grid, so a per-plan rebuild
        // would drop every box spawned so far. The old <world>.esdf disk
        // cache (load/save + resolution guards) is gone: it persisted 3.6 GB
        // of constant free space per world, and a pre-heightmap cache
        // silently resurrected voxel terrain (double-count vs the heightmap
        // term).
        if (!sdf_built_) {
            Eigen::Vector3d sdf_lo = map_lower_bound_;
            Eigen::Vector3d sdf_hi = map_upper_bound_;
            Eigen::Vector3d tlo, thi;
            const bool full_world =
                terrain_data_.valid && computeTerrainBBox(&tlo, &thi);
            if (full_world) {
                sdf_lo = tlo;  // world-wide grid: valid for every later mission
                sdf_hi = thi;
            }
            if (!buildSDFForBounds(sdf_lo, sdf_hi)) {
                RCLCPP_ERROR(node_->get_logger(), "SDF build failed");
                return false;
            }
            // A mission-bbox grid (no terrain yet) stays rebuildable so a
            // later mission with terrain gets the world-wide grid.
            if (full_world) sdf_built_ = true;
        }


        // The SDF is now ready — add any obstacles deferred before it existed
        // (no ESDF cache on a fresh run), so they make it into this first plan.
        flushPendingObstacles();

        // SDF sanity probe at start/goal.
        {
            float d_start = sdf_manager_.getDistance(start_pos);
            float d_goal  = sdf_manager_.getDistance(wps.back());
            log_manager_->infof("SDF probe: start=%.3f m, goal=%.3f m (margin=%.2f)",
                                d_start, d_goal, obstacle_clearance_);
        }


        // === STEP 2~3: front-end search + densification ===
        std::vector<Eigen::Vector3d> full_route, clean_path;
        std::vector<double> cap_ref;
        std::vector<size_t> edge_leg;
        if (route_override && route_override->size() >= 2) {
            // [CHAIN] Inherited committed route (see the header comment):
            // the optimizer re-solves this exact geometry; no re-litigation
            // of the route homotopy, the altitude band/cap tables inherit
            // the baseline's committed profile over the span.
            clean_path = *route_override;
            full_route = clean_path;
            if (cap_ref_override) cap_ref = *cap_ref_override;
            fe_raw_max_z_ = -1e9;
            for (const auto &p : clean_path)
                fe_raw_max_z_ = std::max(fe_raw_max_z_, p.z());
            log_manager_->infof(
                "[CHAIN] inherited committed route: %zu vertices, cap_ref "
                "%zu, committed max z %.2f (front-end search skipped)",
                clean_path.size(), cap_ref.size(), fe_raw_max_z_);
            // [LEG-POLICY] This branch runs NO search, so it can mint no
            // tags, and it never runs the epoch reset at the top of
            // planFrontEnd — leg_policies_ here still describes whichever
            // plan last searched. The geometry came from the caller, so its
            // provenance comes from the caller too, and it is only usable if
            // the caller's epoch is still the current one: that is the proof
            // that no front end has run in between and that leg_policies_
            // still describes the legs these tags name. Anything else drops
            // both, rather than read this plan's route against another
            // plan's legs.
            const bool prov_ok =
                provenance != nullptr && provenance->edge_leg != nullptr &&
                provenance->epoch != 0 &&
                provenance->epoch == zone_policy_epoch_ &&
                provenance->edge_leg->size() + 1 == clean_path.size();
            if (prov_ok) {
                edge_leg = *provenance->edge_leg;
            } else {
                if (provenance != nullptr && provenance->edge_leg != nullptr)
                    log_manager_->warnf(
                        "[LEG-POLICY] inherited route provenance refused "
                        "(epoch %lu vs %lu, %zu tags for %zu vertices) — "
                        "this plan has no leg attribution",
                        (unsigned long)provenance->epoch,
                        (unsigned long)zone_policy_epoch_,
                        provenance->edge_leg->size(), clean_path.size());
                leg_policies_.clear();
            }
        } else if (!planFrontEnd(start_pos, wps, full_route, clean_path,
                                 cap_ref, edge_leg,
                                 allowsTakeoffRelief(head.src))) {
            return false;
        }
        // [CHAIN] retain the committed products for the chain planner to
        // slice — copied BEFORE the optimizer's midpoint/lead-in insertions
        // mutate clean_path.
        last_clean_path_ = clean_path;
        last_cap_ref_ = cap_ref;
        // [LEG-POLICY] Tags and epoch travel together or not at all: a tag
        // set without the epoch that minted it cannot be checked for
        // staleness, and an epoch without tags indexes nothing.
        // Only the positive case assigns: anything else leaves them as the
        // entry reset above left them, which is empty. A second clear here
        // would be a branch no test can kill.
        if (edge_leg.size() + 1 == clean_path.size() && !clean_path.empty()) {
            last_route_edge_leg_ = edge_leg;
            last_route_epoch_ = zone_policy_epoch_;
        }
        if (front_end_only) {
            // [CHAIN-PAR] route-based contract authoring needs only the
            // committed front-end products retained above.
            log_manager_->infof("[CHAIN-PAR] front-end only: %zu vertices "
                                "committed, optimization skipped",
                                clean_path.size());
            return true;
        }

        // [HEAD-POLICY] One implementation, in start_state.h. This block
        // and its twin in SegmentChainPlanner::planOverRoute were the same
        // two rules written twice, and they had already drifted — the
        // numeric-equality rule at the speed boundaries reached this copy
        // and not the other. The DECISION now has one owner; the log tags
        // stay here because the two callers report differently.
        //
        // The head arrives with its own provenance; nothing here derives it.
        const double v_floor = poly_traj_opt_
            ? poly_traj_opt_->dynamicsMinSpeedFloorUnits() : 0.0;
        double um_xy_head = 100.0;
        if (node_->has_parameter("optimization/dynamics_unit_xy_m"))
            node_->get_parameter("optimization/dynamics_unit_xy_m",
                                 um_xy_head);
        const double head_eps_u =
            um_xy_head > 1e-9 ? kSpeedBoundaryEpsMps / um_xy_head : 0.0;
        const HeadPolicy hp = applyHeadPolicy(
            head, clean_path, v_floor, align_start_vel_to_route_,
            head_eps_u);
        last_head_policy_ = hp;
        Eigen::Vector3d start_vel_eff = hp.vel_u;
        if (hp.reaimed) {
            log_manager_->infof(
                "[VEL-ALIGN] synthesized start vel re-aimed to the route's "
                "initial direction: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f) "
                "u/s",
                start_vel.x(), start_vel.y(), start_vel.z(),
                start_vel_eff.x(), start_vel_eff.y(), start_vel_eff.z());
        }
        if (hp.floored) {
            log_manager_->warnf(
                "[STALL-FLOOR] start speed %.3f u/s is below the platform "
                "margin-backed cruise floor %.3f u/s (%.0f m/s) — planning "
                "from the floor along (%.2f, %.2f, %.2f); the launch phase "
                "is outside the planner's envelope",
                start_vel.norm(), hp.floor_u, hp.floor_u * 100.0,
                start_vel_eff.normalized().x(),
                start_vel_eff.normalized().y(),
                start_vel_eff.normalized().z());
        }
        if (hp.floor_declined) {
            log_manager_->infof(
                "[HEAD-POLICY] head speed %.3f u/s is below the "
                "margin-backed cruise floor %.3f u/s and was KEPT — source "
                "%s may not be rewritten",
                start_vel_eff.norm(), hp.floor_u, sourceName(head.src));
        }

        // === STEP 4~5: trajectory optimization (MINCO + L-BFGS) ===
        // [LEG-POLICY] last_route_edge_leg_, not the local edge_leg: the
        // inherited-route branch leaves the local empty and the member is the
        // one place that has already been size-checked against clean_path.
        bool opt_ok = optimizeStage(clean_path, full_route,
                                    start_pos, start_vel_eff, start_acc, wps,
                                    cap_ref, tail, last_route_edge_leg_);

        auto t_total_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] === TOTAL planGlobalTraj: %.1f ms ===",
            std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count());

        // Second half of the anti-fragmentation pair (see setTerrainData):
        // the FM2 fields are re-allocated INSIDE the plan, so the post-swap
        // trim can't see their old epochs — trim again now that the plan's
        // transient allocations are dead. ~ms on a GB heap vs a 0.5-3 s plan.
        malloc_trim(0);

        return opt_ok;
    }

bool PathManager::planFrontEnd(const Eigen::Vector3d &start_pos,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               std::vector<Eigen::Vector3d> &full_route,
                               std::vector<Eigen::Vector3d> &clean_path,
                               std::vector<double> &cap_ref,
                               std::vector<size_t> &edge_leg,
                               bool takeoff_start)
{
    edge_leg.clear();
    // [S13] policy epoch: dispositions are per-search state, so every
    // front-end run invalidates outstanding zone-policy snapshots; the
    // recorded generation says WHICH zone data this search saw.
    ++zone_policy_epoch_;
    zone_policy_epoch_generation_ = zone_policy_generation_;
    zone_policy_epoch_searches_ = 0;
    leg_policies_.clear();

        // Segment list: start -> wp1 -> ... -> wpN
        std::vector<Eigen::Vector3d> all_points;
        all_points.push_back(start_pos);
        for (const auto& wp : waypoints) {
            all_points.push_back(wp);
        }

        // === STEP 2: 3D A* search + visibility-thinning simple_path ===
        // Bind SDF + risk zones to the A* front-end. A* collision check uses
        // the ESDF (distance < obstacle_clearance_ == blocked), and risk
        // cost is added to the A* g-score per visited cell.
        // Member, not a local: the searcher keeps a raw pointer to this
        // vector past the end of this call (see astar_risks_ in the header).
        astar_risks_.clear();
        astar_risks_.reserve(risk_zones_.size());
        for (const auto &tz : risk_zones_) {
            astar_risks_.push_back(
                {tz.center, tz.reach, tz.peak,
                 tz.reach * risk_vertical_ratio_});
        }
        Eigen::Vector3d map_size = map_upper_bound_ - map_lower_bound_;
        searcher_.setLogManager(log_manager_);
        searcher_.setSDF(&sdf_manager_, map_lower_bound_, map_size,
                         sdf_voxel_size_, sdf_voxel_z_);
        const std::vector<path_planner::search::RiskZoneLite> *astar_tz_ptr =
            astar_risks_.empty() ? nullptr : &astar_risks_;
        searcher_.setRiskZones(astar_tz_ptr);
        searcher_.setRiskVisibility(
            [this](size_t zi, const Eigen::Vector3d &p) -> double {
                return riskVisibilityValue(zi, p);
            });
        log_manager_->infof("[PM DBG] setRiskZones: %zu zones (ptr=%p) weight=%.3f",
            astar_risks_.size(), (const void*)astar_tz_ptr, risk_weight_);
        // Re-bind the optimizer with the SAME zone set every plan, symmetric
        // with the searcher_ re-bind above. initOptimizer() only snapshots the
        // set active at startup, so without this per-plan push a runtime zone
        // update (setRiskZonesRuntime) reaches the front-end and the metrics
        // panel but never MINCO — which then smooths trajectories into freshly
        // added zones (and keeps dodging removed ones). An empty list
        // intentionally clears stale zones.
        if (poly_traj_opt_) {
            std::vector<ego_planner::RiskZone> opt_zones;
            opt_zones.reserve(risk_zones_.size());
            for (const auto &tz : risk_zones_) {
                ego_planner::RiskZone oz;
                oz.center = tz.center;
                oz.reach = tz.reach;
                oz.peak = tz.peak;
                oz.vertical_reach = tz.reach * risk_vertical_ratio_;
                opt_zones.push_back(oz);
            }
            poly_traj_opt_->setRiskZones(opt_zones);
            poly_traj_opt_->setRiskVisibility(
                [this](size_t zi, const Eigen::Vector3d &p,
                       Eigen::Vector3d *grad) -> double {
                    return riskVisibility(zi, p, grad);
                });
            poly_traj_opt_->setRiskShadowCeiling(
                [this](size_t zi, const Eigen::Vector3d &p) -> double {
                    return riskShadowCeiling(zi, p);
                });
        }
        // A* must see obstacles so the simple_path it returns is already an
        // avoidance path. Feeding that into MINCO makes the initial inner
        // points sit OUTSIDE the obstacle, and L-BFGS only has to smooth the
        // detour — no saddle problem. If A* is blinded (search_ignores=true)
        // the optimiser gets a straight line through the obstacle centre and
        // the cylinder's rotational symmetry pins it at a zero-gradient
        // saddle, which matches what main-branch would also suffer under the
        // same debug configuration.
        searcher_.setObstacleMargin(obstacle_clearance_);
        searcher_.setZoneAvoidLexico(zone_avoid_lexico_);
        searcher_.setGroundHeight(ground_height_);
        searcher_.setVirtualCeilHeight(virtual_ceil_height_);
        // 2.5D terrain heightmap for the front end (FM2 speed map / A* / shortcut):
        // exact terrain z, so the route no longer cuts through hills the coarse
        // voxel SDF under-saw. Same source as the optimizer's terrain term.
        searcher_.setTerrainHeightmap(
            [this](double x, double y) -> float {
                return terrain_data_.valid
                         ? terrain_data_.getElevation(x, y)
                         : -std::numeric_limits<float>::infinity();
            },
            // DEM cell in frame units: the searcher scales its chord-sampling
            // pitch to half a cell (corridor 30 m crops need ~0.15 u, not the
            // legacy 0.5 u sized for the 250 m full_map grid).
            terrain_data_.valid ? terrain_data_.resolution : 0.0);
        searcher_.setRiskAlpha(risk_weight_);
        searcher_.setRiskBarrier(risk_barrier_);
        searcher_.setRiskGoalTaperRadius(risk_goal_taper_radius_);
        // Fixed-wing slope cap for the geodesic extraction, from the SHARED
        // dynamics model's flight-path-angle limit (single authority — the
        // same number the optimizer's envelope terms enforce).
        {
            double fpa_deg = 30.0;
            if (!node_->has_parameter(
                    "optimization/dynamics_flight_path_max_deg"))
                node_->declare_parameter(
                    "optimization/dynamics_flight_path_max_deg", 30.0);
            node_->get_parameter("optimization/dynamics_flight_path_max_deg",
                                 fpa_deg);
            searcher_.setGeodesicSlopeTanMax(
                std::tan(fpa_deg * M_PI / 180.0));
        }
        searcher_.setSmhaW(risk_smha_w_);
        searcher_.setFrontEnd(front_end_str_ == "fm2"
            ? path_planner::search::PathSearcher::FrontEnd::FM2
            : path_planner::search::PathSearcher::FrontEnd::ASTAR);
        searcher_.setFm2CoarseK(fm2_coarse_k_);
        searcher_.setFm2MaxCells(static_cast<size_t>(fm2_max_cells_));
        searcher_.setFm2Star(fm2_star_);
        searcher_.setFm2AltPenalty(fm2_alt_penalty_, fm2_alt_zscale_, fm2_alt_zscale_dn_);
        // [ROUGH] slope field for the roughness pseudo-moat (same TerrainData
        // surface as the heightmap above; thread_local memo, OMP-safe).
        searcher_.setTerrainHeightGrad(
            [this](double x, double y, float *h, float *gx, float *gy) -> bool {
                return terrain_data_.getElevationAndGrad(x, y, h, gx, gy);
            });
        searcher_.setRoughness(fm2_rough_weight_, fm2_rough_slope0_);
        searcher_.setDynObstacleMargin(dyn_obstacle_margin_);
        searcher_.setBypassShortcut(astar_bypass_shortcut_);

        // A* fine pool is only used by the A* front-end. FM2 runs on its
        // own coarse grid (fm2_F_, fm2_T_) and never touches pool_, so
        // skip the ~2 GB / 655 ms allocation in FM2 mode.
        if (front_end_str_ != "fm2") {
            Eigen::Vector3i sdf_shape = sdf_manager_.shape();
            if (sdf_shape.minCoeff() <= 0) {
                log_manager_->errorf("SDF shape not available, cannot size A* pool");
                return false;
            }
            Eigen::Vector3i desired = sdf_shape;
            if (!astar_initialized_) {
                astar_pool_size_ = desired;
                searcher_.initGridMap(astar_pool_size_);
                astar_initialized_ = true;
                log_manager_->infof("A* pool allocated: (%d,%d,%d)",
                    astar_pool_size_.x(), astar_pool_size_.y(), astar_pool_size_.z());
            } else if (desired != astar_pool_size_) {
                astar_pool_size_ = desired;
                searcher_.resizePool(astar_pool_size_);
                log_manager_->infof("A* pool resized: (%d,%d,%d)",
                    astar_pool_size_.x(), astar_pool_size_.y(), astar_pool_size_.z());
            }
        }

        auto t_astar_start = std::chrono::steady_clock::now();
        full_route.clear();
        full_route.push_back(start_pos);
        // [LEG-POLICY] route_leg[i] owns the edge full_route[i] -> [i+1].
        // Recorded as the geometry is built, not reconstructed later: after
        // the fillets and the subdivision below, nearest-leg projection would
        // pick the wrong leg wherever two legs run close or the route doubles
        // back on itself.
        std::vector<size_t> route_leg;
        // [LEG-POLICY] Latched once any leg fails to yield a snapshot; see
        // the capture block below.
        bool leg_capture_voided = false;
        for (size_t seg = 0; seg < all_points.size() - 1; ++seg)
        {
            std::vector<Eigen::Vector3d> seg_path =
                searcher_.astarSearchAndGetSimplePath(
                    astar_step_size_, all_points[seg], all_points[seg + 1],
                    traj_.local_traj.drone_id,
                    // BOTH conditions. seg == 0 is the route's first leg;
                    // takeoff_start is whether this call begins where the
                    // AIRCRAFT is. planGlobalTraj is re-entered for chain
                    // junctions and transition handoffs, and each of those
                    // has its own leg 0 — seg == 0 alone handed them an
                    // allowance meant for a mission start.
                    /*is_takeoff_leg=*/seg == 0 && takeoff_start);
            ++zone_policy_epoch_searches_;

            log_manager_->infof("A* segment %zu: simple_path_size=%zu",
                seg, seg_path.size());

            if (seg_path.size() < 2)
            {
                RCLCPP_ERROR(node_->get_logger(),
                    "A* failed for segment %zu: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                    seg, all_points[seg].x(), all_points[seg].y(), all_points[seg].z(),
                    all_points[seg+1].x(), all_points[seg+1].y(), all_points[seg+1].z());
                // [LEG-POLICY] A leg that produced no path produced no policy
                // either. Everything captured so far described a route that is
                // now not going to exist, and the legs after this one will
                // never run — so the EPOCH is void, not merely incomplete. A
                // surviving prefix would be a set of snapshots for a plan that
                // failed, sitting behind an accessor whose only size check is
                // against a trajectory from some other plan.
                leg_policies_.clear();
                return false;
            }

            // [LEG-POLICY] Captured only now — AFTER this leg is known to have
            // produced a path, and still while its search owns the searcher
            // buffers (the next leg overwrites them, which is exactly why the
            // plan-wide snapshot refuses a multi-leg epoch). Capturing before
            // the success check recorded a policy for a search that failed.
            {
                LegPolicySnapshot lp;
                if (captureLegPolicySnapshot(seg, zone_policy_epoch_searches_,
                                             &lp) && !leg_capture_voided) {
                    leg_policies_.push_back(lp);
                } else if (!leg_capture_voided) {
                    // A hole is not a set. Note the LATCH: without it the
                    // legs after the failure would append and leave a TAIL —
                    // snapshots whose leg numbers no longer line up with
                    // anything, and which a per-leg reader would index by
                    // position. Voided means voided for the epoch, not until
                    // the next success.
                    //
                    // The PLAN is not failed here: a configuration where no
                    // capture is possible at all (the 3-pass off, with zones
                    // present) is a supported one, and it is the audit's job
                    // to refuse what it cannot attribute — see wpzonepass0.
                    leg_policies_.clear();
                    leg_capture_voided = true;
                    log_manager_->warnf(
                        "[LEG-POLICY] leg %zu could not be captured — the "
                        "epoch is void, not partial", seg);
                }
            }

            for (size_t i = (seg == 0 ? 0 : 1); i < seg_path.size(); ++i)
            {
                if (!full_route.empty() &&
                    (full_route.back() - seg_path[i]).norm() < 1e-3) {
                    continue;
                }
                full_route.push_back(seg_path[i]);
                route_leg.push_back(seg);   // the edge this push just created
            }
        }

        // === Corner fillets: replace each polyline kink with a circular arc ===
        // The back-end has no minimum-speed/curvature constraint (quadrotor
        // heritage): given a kinked polyline it just brakes at the kink, so
        // the optimized geometry stays angular no matter the accel limit. A
        // fixed-radius fillet bakes the coordinated-turn shape into the input
        // instead. Arcs that would clip terrain (SDF below clearance) or
        // enter a risk zone the corner itself avoids shrink R and retry.
        if (corner_fillet_radius_ > 1e-6 && full_route.size() >= 3) {
            auto sample_ok = [&](const Eigen::Vector3d &p,
                                 const Eigen::Vector3d &corner) {
                if (sdf_manager_.hasData() &&
                    sdf_manager_.getDistance(p) < obstacle_clearance_) return false;
                for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
                    const auto &tz = risk_zones_[zi];
                    const bool corner_in =
                        riskEllipsoidRadius(tz, corner) < 1.0 &&
                        riskVisibilityValue(zi, corner) > 0.5;
                    const bool p_in =
                        riskEllipsoidRadius(tz, p) < 1.0 &&
                        riskVisibilityValue(zi, p) > 0.5;
                    if (p_in && !corner_in) return false;  // arc dips INTO a zone
                }
                return true;
            };

            std::vector<Eigen::Vector3d> rounded;
            std::vector<size_t> rounded_leg;
            rounded.reserve(full_route.size() * 4);
            rounded_leg.reserve(full_route.size() * 4);
            rounded.push_back(full_route.front());
            // Every push below adds exactly one edge, so one tag per push
            // keeps rounded_leg.size() + 1 == rounded.size() by construction.
            for (size_t n = 1; n + 1 < full_route.size(); ++n) {
                const Eigen::Vector3d &A = full_route[n - 1];
                const Eigen::Vector3d &B = full_route[n];
                const Eigen::Vector3d &C = full_route[n + 1];
                const size_t leg_in = route_leg[n - 1];    // edge A->B
                const size_t leg_out = route_leg[n];       // edge B->C
                const double d1 = (A - B).norm(), d2 = (C - B).norm();
                if (d1 < 1e-6 || d2 < 1e-6) {
                    rounded.push_back(B); rounded_leg.push_back(leg_in); continue;
                }
                const Eigen::Vector3d u = (A - B) / d1, w = (C - B) / d2;
                const double cosphi = std::clamp(u.dot(w), -1.0, 1.0);
                const double phi = std::acos(cosphi);   // interior angle at B
                if (phi > M_PI - 0.05) {                // ~straight
                    rounded.push_back(B); rounded_leg.push_back(leg_in); continue;
                }

                bool placed = false;
                for (double R = corner_fillet_radius_; R > 1.0; R *= 0.5) {
                    double t = R / std::tan(0.5 * phi);
                    const double t_cap = 0.45 * std::min(d1, d2);
                    double R_eff = R;
                    if (t > t_cap) { t = t_cap; R_eff = t * std::tan(0.5 * phi); }
                    const Eigen::Vector3d P1 = B + u * t, P2 = B + w * t;
                    // Quadratic Bezier P1->B->P2 approximates the arc and is
                    // tangent to both segments; sample every ~3 units.
                    int N = std::max(3, (int)std::ceil((P1 - P2).norm() / 3.0));
                    // [LEG-POLICY] B is a leg junction when the two edges it
                    // joins came from different searches. The fillet deletes B
                    // and replaces it with an arc that belongs to neither leg
                    // alone, so force an EVEN sample count: a = 0.5 is then a
                    // real vertex, the arc apex stands in for the junction B
                    // used to be, and the two halves become separate pieces.
                    // Without it one piece would straddle two policies and the
                    // audit would have to pick one of them arbitrarily.
                    const bool junction = (leg_in != leg_out);
                    if (junction && (N % 2) != 0) ++N;
                    std::vector<Eigen::Vector3d> arc;
                    bool ok = true;
                    for (int k = 1; k < N; ++k) {
                        const double a = (double)k / N, b = 1.0 - a;
                        const Eigen::Vector3d p = b * b * P1 + 2 * a * b * B + a * a * P2;
                        if (!sample_ok(p, B)) { ok = false; break; }
                        arc.push_back(p);
                    }
                    if (!ok) continue;            // shrink R, retry
                    rounded.push_back(P1);        // lies on A->B
                    rounded_leg.push_back(leg_in);
                    for (int k = 1; k < N; ++k) {
                        rounded.push_back(arc[k - 1]);
                        // The edge ARRIVING at the apex (k == N/2) is still
                        // the incoming leg's; everything past it is outgoing.
                        rounded_leg.push_back(
                            (junction && k > N / 2) ? leg_out : leg_in);
                    }
                    rounded.push_back(P2);        // lies on B->C
                    rounded_leg.push_back(leg_out);
                    placed = true;
                    (void)R_eff;
                    break;
                }
                if (!placed) {                   // keep the kink
                    rounded.push_back(B); rounded_leg.push_back(leg_in);
                }
            }
            rounded.push_back(full_route.back());
            rounded_leg.push_back(route_leg.back());
            log_manager_->infof("corner fillet R=%.1f: %zu -> %zu pts",
                                corner_fillet_radius_, full_route.size(),
                                rounded.size());
            full_route.swap(rounded);
            route_leg.swap(rounded_leg);
        }

        auto t_rrt_end = std::chrono::steady_clock::now();
        log_manager_->infof("A* route: %zu waypoints (%.1f ms)",
            full_route.size(),
            std::chrono::duration<double, std::milli>(t_rrt_end - t_astar_start).count());
        // DEBUG: annotate each A* waypoint with the SAME risk field every
        // consumer uses (dyn_a_star.h getRiskNorm and poly_traj_optimizer
        // RiskGradCostP share it verbatim): terrain-masked ellipsoidal
        // quadratic moat m_i = visibility_i*peak*(1-q_i)^2,
        // OR-composed risk
        // = 1 - prod_i (1 - m_i), in [0,1]. (Previously this logged a Gaussian *
        // risk_weight — and later a 3D-sphere distance — neither matched the
        // planner and made edge passes look far riskier than they are.)
        // Full per-waypoint dump only for small routes: at FM2 k=1 the raw
        // route is 30k+ points and 30k formatted log lines cost ~10 s/plan.
        const size_t kRiskDumpMax = 200;
        double dbg_risk_max = 0.0, dbg_risk_sum = 0.0;
        for (size_t ri = 0; ri < full_route.size(); ++ri) {
            const auto &p = full_route[ri];
            double survival = 1.0;
            std::string per_zone;
            const bool dump = full_route.size() <= kRiskDumpMax;
            for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
                const auto &tz = risk_zones_[zi];
                const double q = riskEllipsoidRadius(tz, p);
                double moat = 0.0;
                double visibility = 0.0;
                if (q < 1.0) {
                    const double u = 1.0 - q;
                    visibility = riskVisibilityValue(zi, p);
                    moat = tz.peak * u * u * visibility;
                }
                survival *= (1.0 - std::min(moat, 1.0 - 1e-3));
                if (dump) {
                    char buf[64];
                    std::snprintf(buf, sizeof(buf),
                                  " tz%zu(q=%.2f,v=%.2f,m=%.3f)",
                                  zi, q, visibility, moat);
                    per_zone += buf;
                }
            }
            const double r = 1.0 - survival;
            dbg_risk_max = std::max(dbg_risk_max, r);
            dbg_risk_sum += r;
            if (dump) {
                log_manager_->infof("  A*[%zu]: (%.2f, %.2f, %.2f) risk=%.3f%s",
                    ri, p.x(), p.y(), p.z(), r, per_zone.c_str());
            }
        }
        if (full_route.size() > kRiskDumpMax) {
            log_manager_->infof(
                "  A* route risk: %zu wp, max=%.3f mean=%.4f (per-wp dump skipped)",
                full_route.size(), dbg_risk_max,
                dbg_risk_sum / std::max<size_t>(1, full_route.size()));
        }

        // Front-end route: nav_msgs/Path for programmatic consumers plus the
        // tube marker render (/viz/front_end_path).
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        path_msg.header.frame_id = "map";
        for (const auto &point : full_route) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        front_end_path_pub_->publish(path_msg);
        // ns matches the retired bridge node ("opt_path_drone_<id>" — the
        // shipped RViz configs filter on it).
        publishTubeMarker(front_end_tube_pub_, full_route, "opt_path_drone_",
                          0.12f, 0.39f, 1.0f, 0.7f);

        // === STEP 2.5: z-denoise of the shortcut vertices ===
        // The FM2 grid has only a few dozen z-cells over the whole map, so
        // the geodesic's z carries quantization noise on the order of a full
        // cell — comparable to the mission altitude band itself. The long
        // min-jerk pieces (50-100 s) RING through those jittery pins,
        // amplifying tens of metres of noise into slow z-waves that bounce
        // off the clearance floor (the jerk term is T^5 scale-starved at
        // mission scale and cannot iron them out — see
        // optimization/weight_smoothness). Smooth z ONLY — xy is real
        // geometry (obstacle/risk avoidance). Terrain-forced climbs survive
        // via the elevation clamp; a smoothed vertex that would lose
        // obstacle clearance keeps its original z. Endpoints stay fixed.
        // Cap reference = the PRE-denoise COMMITTED profile. The altitude
        // cap's contract is "no ballooning above the FRONT-END'S COMMITTED
        // profile"; the z-denoise below is a numerical filter (FM2 grid
        // sawtooth), not a commitment change — yet its moving average planes
        // the profile's defining features (crest max: observed 11.90 -> 11.56,
        // i.e. the cap reference lost 34 m). Snapshot the raw z PER VERTEX
        // before the filter; after it, take the elementwise max with the
        // filtered value (the filter's terrain/water CLAMPS may legitimately
        // RAISE z — that lift is part of the commitment too). This vector
        // becomes the arc-varying cap's reference through the subdivision
        // below; its max feeds the scalar fallback.
        std::vector<double> raw_z;
        raw_z.reserve(full_route.size());
        fe_raw_max_z_ = -1e9;
        for (const auto &p : full_route) {
            raw_z.push_back(p.z());
            fe_raw_max_z_ = std::max(fe_raw_max_z_, p.z());
        }

        if (full_route.size() >= 3) {
            const size_t n = full_route.size();
            std::vector<double> zs(n);
            for (size_t i = 0; i < n; ++i) zs[i] = full_route[i].z();
            double z_lo0 = zs[0], z_hi0 = zs[0];
            for (double z : zs) { z_lo0 = std::min(z_lo0, z); z_hi0 = std::max(z_hi0, z); }

            size_t adjusted = 0;
            // Water-pin floor = the MISSION's own minimum altitude (not a
            // clearance multiple). The old 0.5*obstacle_clearance floor was
            // tuned when clearance was 0.2 (= 10 m, "so 10 m-cruise missions
            // are not distorted upward"); the ridge-graze retune to 0.38
            // silently raised it to 19 m and every low-altitude over-water
            // mission grew a mid-route hump to 19 m between its 10 m
            // endpoints. Anchoring to min(start, goal/waypoints) restores the
            // intent for ANY altitude and matches the system-wide rule
            // ("nothing ever requires diving below the mission altitude" —
            // FM2 stiff down-side, optimizer alt floor): over water, a pin
            // below the mission minimum is grid noise by definition. The 2 m
            // absolute guard only defends against underwater start inputs.
            double mission_min_z = start_pos.z();
            for (const auto &wp : waypoints)
                mission_min_z = std::min(mission_min_z, wp.z());
            const double water_pin_floor = std::max(0.02, mission_min_z);
            const int W = 2;  // +-2 vertex moving average
            for (size_t i = 1; i + 1 < n; ++i) {
                double acc = 0.0;
                int cnt = 0;
                for (int k = -W; k <= W; ++k) {
                    const long j = static_cast<long>(i) + k;
                    if (j < 0 || j >= static_cast<long>(n)) continue;
                    acc += zs[j];
                    ++cnt;
                }
                double z_new = acc / cnt;
                if (terrain_data_.valid) {
                    const float elev = terrain_data_.getElevation(
                        full_route[i].x(), full_route[i].y());
                    // Land: hold terrain + full clearance. Water (invalid
                    // elev): the mission-min floor (see water_pin_floor
                    // above) — keeps the geodesic's z-noise pins (observed
                    // -0.40, conflicting with the ground plane) above water WITHOUT
                    // distorting low-altitude cruises upward.
                    if (elev > -1e9f) {
                        z_new = std::max(z_new,
                                         static_cast<double>(elev) + obstacle_clearance_);
                    } else {
                        z_new = std::max(z_new, water_pin_floor);
                    }
                }
                const Eigen::Vector3d cand(full_route[i].x(), full_route[i].y(), z_new);
                if (sdf_manager_.hasData()) {
                    const float d = sdf_manager_.getDistance(cand);
                    if (!(std::isfinite(d) && d >= obstacle_clearance_)) continue;  // keep raw z
                }
                if (std::abs(z_new - zs[i]) > 1e-9) ++adjusted;
                full_route[i].z() = z_new;
            }
            double z_lo1 = full_route[0].z(), z_hi1 = z_lo1;
            for (const auto &p : full_route) {
                z_lo1 = std::min(z_lo1, p.z());
                z_hi1 = std::max(z_hi1, p.z());
            }
            log_manager_->infof(
                "[Z-DENOISE] shortcut z: %zu/%zu vertices smoothed, range [%.2f, %.2f] -> [%.2f, %.2f]",
                adjusted, n, z_lo0, z_hi0, z_lo1, z_hi1);
        }
        // Committed profile = elementwise max(raw, filtered): neither the
        // filter's crest planing nor its safety clamps may LOWER the cap
        // reference (see the raw_z snapshot comment above).
        for (size_t i = 0; i < raw_z.size() && i < full_route.size(); ++i)
            raw_z[i] = std::max(raw_z[i], full_route[i].z());

        // === STEP 3: Sparse piece boundaries (reference-style). ===
        // The shortcut vertices ARE the geometry; we only subdivide long
        // segments so the optimizer's per-piece obstacle sampling stays dense
        // enough. NO corner densification: pinning extra points at corners is
        // what kept the min-jerk pieces from rounding them — with long free
        // pieces the quintic sweeps through a corner waypoint on a wide arc
        // by itself (Swarm-Formation / GCOPTER structure).
        const double max_seg = std::max(1.0, length_per_piece_ * 4.0);
        clean_path.clear();
        clean_path.reserve(full_route.size() * 4);
        clean_path.push_back(full_route.front());
        // cap_ref mirrors clean_path vertex-for-vertex (SAME skip rule, SAME
        // interpolation) so the optimizer's index pairing holds by
        // construction — it hard-checks sizes and falls back to the scalar
        // cap on mismatch.
        cap_ref.clear();
        cap_ref.reserve(full_route.size() * 4);
        cap_ref.push_back(raw_z.empty() ? full_route.front().z() : raw_z.front());
        // [LEG-POLICY] The subdivision splits an edge; it never merges two, so
        // every sub-edge inherits its parent's leg unchanged. Refuse to guess
        // if the tags and the geometry ever disagree on length.
        const bool leg_tags_ok = (route_leg.size() + 1 == full_route.size());
        if (!leg_tags_ok)
            log_manager_->errorf(
                "[LEG-POLICY] edge tags %zu do not match route %zu — route "
                "provenance dropped",
                route_leg.size(), full_route.size());
        edge_leg.clear();
        edge_leg.reserve(full_route.size() * 4);
        for (size_t i = 0; i + 1 < full_route.size(); ++i) {
            const Eigen::Vector3d &a = full_route[i];
            const Eigen::Vector3d &b = full_route[i + 1];
            const double seg_len = (b - a).norm();
            if (seg_len < 1e-6) continue;
            const int n_sub = std::max(1, (int)std::ceil(seg_len / max_seg));
            const double ra = (i < raw_z.size()) ? raw_z[i] : a.z();
            const double rb = (i + 1 < raw_z.size()) ? raw_z[i + 1] : b.z();
            for (int kk = 1; kk <= n_sub; ++kk) {
                const double t = (double)kk / n_sub;
                clean_path.push_back(a + (b - a) * t);
                cap_ref.push_back(ra + (rb - ra) * t);
                if (leg_tags_ok) edge_leg.push_back(route_leg[i]);
            }
        }
        if (!leg_tags_ok || edge_leg.size() + 1 != clean_path.size())
            edge_leg.clear();
        log_manager_->infof(
            "A* shortcut %zu pts → sparse pieces %zu pts (max_seg %.1f)",
            full_route.size(), clean_path.size(), max_seg);

        publishPipelineDebug(full_route, clean_path);

        if (clean_path.size() < 2) {
            log_manager_->errorf("clean_path too short");
            return false;
        }

        return true;
}

bool PathManager::optimizeStage(std::vector<Eigen::Vector3d> &clean_path,
                                const std::vector<Eigen::Vector3d> &full_route,
                                const Eigen::Vector3d &start_pos,
                                const Eigen::Vector3d &start_vel,
                                const Eigen::Vector3d &start_acc,
                                const std::vector<Eigen::Vector3d> &waypoints,
                                const std::vector<double> &cap_ref,
                                const ego_planner::TailBoundary &tail,
                                const std::vector<size_t> &edge_leg)
{
        // Stage 2 = trajectory optimization. The optimizer owns the MINCO
        // initial-trajectory build + L-BFGS; we only pass the front-end path
        // and store the results. Swap optimizeFromPath() to replace the backend.
        if (!isOptimizerInitialized()) {
            log_manager_->errorf("Optimizer not initialized");
            return false;
        }

        auto t_opt_start = std::chrono::steady_clock::now();

        double global_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        poly_traj::Trajectory global_traj, local_traj;
        // Altitude cap reference for the optimizer's z-cap: the FRONT-END
        // GEODESIC's own max z (not just the endpoints). The eikonal already
        // priced climb-vs-detour in one metric and committed to this profile
        // (its alt band makes it leave mission altitude only where terrain
        // demands), so capping at endpoint+1 re-litigated that decision: a
        // terrain-forced ridge crossing paid ~77k/plan in altitude cost and
        // the down-pressure pushed the crest segment into the terrain
        // (d<0 collision warnings at the crest). The cap's actual job is only
        // to stop the sparse-piece quintic ballooning ABOVE the committed
        // profile — same no-re-litigation rule as the shared risk field.
        {
            double path_max_z = std::max(start_pos.z(), waypoints.back().z());
            double path_min_z = std::min(start_pos.z(), waypoints.back().z());
            for (const auto &p : clean_path) {
                path_max_z = std::max(path_max_z, p.z());
                path_min_z = std::min(path_min_z, p.z());
            }
            // Restore the PRE-denoise committed max (see planFrontEnd): the
            // z-denoise filter planes crests (~0.34 u observed), and a cap
            // referenced to the planed value under-caps the crossing the
            // front-end actually committed to — shoulder grind vs the terrain
            // band, -1004. The filter smooths the PINS; the CAP keeps the
            // committed ceiling. (No symmetric fix for the floor: min-side
            // denoise error only makes the floor laxer, never a conflict.)
            const double path_max_z_denoised = path_max_z;
            path_max_z = std::max(path_max_z, fe_raw_max_z_);
            if (path_max_z - path_max_z_denoised > 1e-6) {
                log_manager_->infof(
                    "[ALT] cap reference restored to pre-denoise max: "
                    "%.3f (denoised profile max %.3f)",
                    path_max_z, path_max_z_denoised);
            }
            // Headroom must leave the quintic a workable vertical corridor
            // above the obstacle-clearance floor: a +1 slack over a sea-level
            // route squeezed the trajectory into a ~4 m band, and the
            // optimizer bought z-compliance with duration (1.8x) until the
            // line search died (-1005). Default 5 ~ the front-end's own alt
            // band tolerance (one FM2 coarse cell).
            //
            // NO terrain floor on the scalar cap (removed). A chord-sampled
            // "terrain max under the route" floor was added while the
            // clearance band was 0.12 to resolve cap-vs-terrain contradictions,
            // but (a) it barely moved the crest dip (-0.117 -> -0.084; the
            // 0.30 band sizing was the actual fix), (b) its 1-D chord/inner-
            // chord sampling provably missed corner-cut knobs anyway, and
            // (c) a single tall islet anywhere on the route raised the GLOBAL
            // ceiling by its full height, licensing high-altitude wandering
            // everywhere else. Terrain safety is owned pointwise by the
            // heightmap-gated cap (zero press within 2*clearance of terrain)
            // plus the 0.30 penalty band sized above the soft-penalty
            // equilibrium (~0.21) — measured invariant to cap weight, so the
            // cap's shoulder pressure cannot drag a crest through the band.
            const double z_hi = path_max_z + alt_cap_headroom_;
            // Floor: mirror of the cap, and the missing LOWER half of the
            // FM2 alt band ("stiff down-side: nothing ever requires diving
            // below mission altitude"). Without it z is only bounded below
            // by the 0.5 collision clearance, so min-jerk pieces sag in big
            // smooth waves and bounce off the water/valley floor. Nothing
            // terrain-forced ever needs to go BELOW the mission endpoints,
            // so a global soft floor is always safe.
            // Floor inherits the GEODESIC MIN, symmetric with the cap: FM2's
            // own down-side band is soft, so its committed profile can dip
            // below the mission endpoints (observed 0.53 vs mission 1.0). A
            // floor pinned to the endpoints then sits ABOVE front-end pins
            // and conflicts with them together with obstacle/risk — a multi-way
            // cost conflict that caused line-search failure on high-altitude
            // missions (low-z missions hid it: their floor was < 0).
            // Keep the soft floor ABOVE the hard ground half-space: for a
            // low mission (cruise 0.1) path_min - headroom went negative, so
            // z in [0, cruise] was penalty-free and the first soft thing a
            // sagging iterate met was the ground plane's unit-gradient CLIFF
            // at z ~ 0 — line searches die on that kink (-1005) and the
            // published mid-iterate shows waves "bouncing off" sea level.
            // A quadratic cushion from half a headroom above the plane
            // catches sag softly before the cliff.
            double z_lo = path_min_z - alt_floor_headroom_;
            if (ground_height_ > -0.5) {
                z_lo = std::max(z_lo, ground_height_ + 0.5 * alt_floor_headroom_);
            }
            // NB: this floor may sit ABOVE a hard-pinned endpoint
            // (low-altitude complex-field stress case: start z 0.05 <
            // cushion top 0.09). Lowering the SCALAR
            // floor to the pin would disable the sag cushion and expose the
            // ground-plane cliff (-1005, tried); instead the optimizer tapers
            // the floor penalty toward pinned endpoints pointwise — see
            // [TERRAIN-TAPER] in poly_traj_optimizer.
            poly_traj_opt_->setAltitudeBand(z_lo, z_hi, weight_altitude_);
            log_manager_->infof(
                "[ALT] optimizer z-band [%.2f, %.2f] (mission min - %.2f, "
                "path max %.2f + %.2f; terrain owned by gated cap + 0.30 band)",
                z_lo, z_hi, alt_floor_headroom_, path_max_z, alt_cap_headroom_);
        }
        // [ZONE-AVOID] pass-1 = committed full avoidance: keep every zone
        // barrier armed (see suppress_crossing_exempt_ in the optimizer).
        poly_traj_opt_->setSuppressCrossingExempt(
            searcher_.zoneAvoidPass() == 1);
        last_piece_leg_.clear();
        bool opt_success = poly_traj_opt_->optimizeFromPath(
            clean_path, start_pos, start_vel, start_acc, waypoints, max_vel_,
            global_traj, local_traj, cap_ref, tail, edge_leg, &last_piece_leg_);
        if (!opt_success) {
            log_manager_->errorf("Trajectory optimization failed");
            return false;
        }

        // Local traj start_time is the trajectory-following reference; stamp it
        // after optimization (matches pre-refactor behavior).
        double local_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        traj_.setGlobalTraj(global_traj, global_time);
        traj_.setLocalTraj(local_traj, local_time, traj_.local_traj.drone_id);

        // [VIZ-TRAJ] The along-trajectory diagnostics (experienced risk and
        // the risk-visibility profile) must sample the OPTIMIZED trajectory
        // that is actually flown and displayed
        // (/planning/trajectory -> /viz/opt_trajectory), NOT the pre-L-BFGS
        // seed. They were wired to the seed (out_global): in zone missions the
        // optimizer dodges/ducks in xy, so the risk band and the colored risk
        // line sat at the seed's positions while the panel drew the optimized
        // trajectory over them — the arc-length axes and the visibility
        // evaluation
        // then disagreed. Measured on the complex-field stress case: seed vs
        // optimized arc
        // length 691 vs 637 u (an 8% axis stretch), xy divergence up to 15.6 u
        // (~20% of the zones' reach, so the dome/roof were sampled well off the
        // flown path near the rims where they vary fastest), z up to 2.5 u.
        // Sample the flown trajectory so the overlay and the trajectory agree.
        publishTrajRisk(local_traj);
        publishRiskProfile(local_traj);
        // Tube renders of the flown (optimized) and seed (global) trajectories.
        // Reference trajectory: pale + translucent, a faint "tunnel" the
        // cursor sphere and the flown path pass through.
        publishTrajTube(local_traj, opt_traj_tube_pub_, "opt_path_drone_",
                        0.55f, 0.72f, 1.0f, 0.22f);
        publishTrajTube(global_traj, global_traj_tube_pub_, "global_path_drone_",
                        0.59f, 0.71f, 1.0f, 0.85f);
        // [CHAIN-VIZ] every ordinary solve clears the per-segment channel;
        // a chained plan repaints it AFTER its last segment solve.
        publishChainSegmentsViz({}, {});

        auto t_opt_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] trajectory optimization: %.1f ms, duration=%.3f max_vel=%.3f",
            std::chrono::duration<double, std::milli>(t_opt_end - t_opt_start).count(),
            local_traj.getTotalDuration(), local_traj.getMaxVelRate());

        return true;
}

// [CHAIN-PAR] One slice, one optimizer instance, no shared mutable state:
// the z-band is a compact per-slice rendition of optimizeStage's (committed
// max from slice+cap plus headroom; floor from the slice minimum with the
// ground cushion), and everything else the solve needs lives inside the
// given instance. No traj_ writes, no viz, no timing members — thread-safe
// against siblings by construction.
bool PathManager::solveSlice(ego_planner::PolyTrajOptimizer &opt,
                             std::vector<Eigen::Vector3d> slice,
                             std::vector<double> cap,
                             const Eigen::Vector3d &head_pos,
                             const Eigen::Vector3d &head_vel,
                             const Eigen::Vector3d &head_acc,
                             const Eigen::Vector3d &goal_pos,
                             const ego_planner::TailBoundary &tail,
                             bool suppress_crossing_exempt,
                             poly_traj::Trajectory *out) const
{
    if (slice.size() < 2 || !out) return false;
    double path_max_z = std::max(head_pos.z(), goal_pos.z());
    double path_min_z = std::min(head_pos.z(), goal_pos.z());
    for (const auto &p : slice) {
        path_max_z = std::max(path_max_z, p.z());
        path_min_z = std::min(path_min_z, p.z());
    }
    // cap_ref carries the committed (pre-denoise-restored) ceiling per
    // vertex — the slice-local stand-in for fe_raw_max_z_.
    for (const double c : cap) path_max_z = std::max(path_max_z, c);
    const double z_hi = path_max_z + alt_cap_headroom_;
    double z_lo = path_min_z - alt_floor_headroom_;
    if (ground_height_ > -0.5) {
        z_lo = std::max(z_lo, ground_height_ + 0.5 * alt_floor_headroom_);
    }
    opt.setAltitudeBand(z_lo, z_hi, weight_altitude_);
    opt.setSuppressCrossingExempt(suppress_crossing_exempt);

    poly_traj::Trajectory out_global;
    const std::vector<Eigen::Vector3d> goal_wps{goal_pos};
    return opt.optimizeFromPath(slice, head_pos, head_vel, head_acc,
                                goal_wps, max_vel_, out_global, *out, cap,
                                tail);
}

// [CHAIN-VIZ] see the header comment. Tube sampling matches
// publishTrajTube (0.1 s + exact endpoint); junction spheres are 3x the
// tube girth with a floating J<n> label so the seams read at map scale.
void PathManager::publishChainSegmentsViz(
    const std::vector<poly_traj::Trajectory> &runs,
    const std::vector<Eigen::Vector3d> &junctions,
    const poly_traj::Trajectory *terminal)
{
    if (!chain_segments_pub_) return;
    visualization_msgs::msg::MarkerArray arr;
    {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "map";
        del.header.stamp = node_->now();
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);
    }
    if (runs.empty()) {  // clear-only
        chain_segments_pub_->publish(arr);
        return;
    }
    // High-contrast palette, cycled; the terminal phase is always WHITE so
    // the prescribed geometry is unmistakable next to the optimized runs.
    static const float kPal[][3] = {
        {0.12f, 0.65f, 1.00f}, {1.00f, 0.55f, 0.10f}, {0.20f, 0.90f, 0.35f},
        {0.95f, 0.30f, 0.85f}, {1.00f, 0.90f, 0.15f}, {0.35f, 0.35f, 1.00f},
        {0.10f, 0.90f, 0.90f}, {1.00f, 0.35f, 0.30f}, {0.60f, 0.95f, 0.20f},
        {0.80f, 0.55f, 1.00f},
    };
    constexpr size_t kPalN = sizeof(kPal) / sizeof(kPal[0]);
    int id = 0;
    const auto tube = [&](const poly_traj::Trajectory &traj, const float *c,
                          const char *ns) {
        const double T = traj.getTotalDuration();
        if (T <= 1e-6) return;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = ns;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = path_scale_;
        m.color.r = c[0]; m.color.g = c[1]; m.color.b = c[2];
        m.color.a = 0.9f;
        m.lifetime = rclcpp::Duration(0, 0);
        for (double t = 0.0; t < T; t += 0.1) {
            const Eigen::Vector3d p = traj.getPos(t);
            geometry_msgs::msg::Point q;
            q.x = p.x(); q.y = p.y(); q.z = p.z();
            m.points.push_back(q);
        }
        const Eigen::Vector3d pe = traj.getPos(T - 1e-9);
        geometry_msgs::msg::Point qe;
        qe.x = pe.x(); qe.y = pe.y(); qe.z = pe.z();
        m.points.push_back(qe);
        arr.markers.push_back(m);
    };
    for (size_t i = 0; i < runs.size(); ++i)
        tube(runs[i], kPal[i % kPalN], "chain_seg");
    if (terminal) {
        static const float kWhite[3] = {1.0f, 1.0f, 1.0f};
        tube(*terminal, kWhite, "chain_terminal");
    }
    for (size_t j = 0; j < junctions.size(); ++j) {
        visualization_msgs::msg::Marker s;
        s.header.frame_id = "map";
        s.header.stamp = node_->now();
        s.ns = "chain_junction";
        s.id = id++;
        s.type = visualization_msgs::msg::Marker::SPHERE;
        s.action = visualization_msgs::msg::Marker::ADD;
        s.pose.position.x = junctions[j].x();
        s.pose.position.y = junctions[j].y();
        s.pose.position.z = junctions[j].z();
        s.pose.orientation.w = 1.0;
        s.scale.x = s.scale.y = s.scale.z = 3.0 * path_scale_;
        s.color.r = s.color.g = s.color.b = 1.0f;
        s.color.a = 0.95f;
        s.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(s);

        visualization_msgs::msg::Marker t = s;
        t.ns = "chain_junction_label";
        t.id = id++;
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.pose.position.z += 4.0 * path_scale_;
        t.scale.x = t.scale.y = 0.0;
        t.scale.z = 4.0 * path_scale_;
        t.text = "J" + std::to_string(j + 1);
        arr.markers.push_back(t);
    }
    chain_segments_pub_->publish(arr);
}

// [CHAIN] Republish the along-trajectory channels for an externally
// assembled trajectory. Same channels, colors and namespaces as the per-plan
// publish in optimizeStage — the chained trajectory replaces the last span's
// partial view, and the reference (baseline) rides the global-tube channel.
void PathManager::publishTrajectoryViz(const poly_traj::Trajectory &opt,
                                       const poly_traj::Trajectory &reference)
{
    publishTrajRisk(opt);
    publishRiskProfile(opt);
    publishTrajTube(opt, opt_traj_tube_pub_, "opt_path_drone_",
                    0.55f, 0.72f, 1.0f, 0.22f);
    publishTrajTube(reference, global_traj_tube_pub_, "global_path_drone_",
                    0.59f, 0.71f, 1.0f, 0.85f);
}

void PathManager::setFormationInfo(int drone_id, const std::string& formation_type,
                                   const std::vector<Eigen::Vector3d>& formation_pattern) {
    current_formation_type_ = formation_type;
    current_formation_pattern_ = formation_pattern;

    log_manager_->infof("PathManager: Set formation info - drone_id=%d, type=%s, pattern_size=%zu",
                drone_id, formation_type.c_str(), formation_pattern.size());
}

bool PathManager::EmergencyStop(const Eigen::Vector3d& stop_pos) {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;

    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    traj_.setLocalTraj(stopMJO.getTraj(), rclcpp::Clock(RCL_ROS_TIME).now().seconds(), traj_.local_traj.drone_id);

    // The stop trajectory replaces the previous route, so every latched
    // route-derived diagnostic must change with it. Otherwise the altitude
    // panel can combine a stationary marker with the old front-end path
    // and old risk colors.
    if (front_end_path_pub_) {
        nav_msgs::msg::Path clear;
        clear.header.frame_id = "map";
        clear.header.stamp = node_->now();
        front_end_path_pub_->publish(clear);
    }
    publishTrajRisk(traj_.local_traj.traj);
    publishRiskProfile(traj_.local_traj.traj);
    publishTrajTube(traj_.local_traj.traj, opt_traj_tube_pub_, "opt_path_drone_",
                    0.55f, 0.72f, 1.0f, 0.22f);

    RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP executed at position (%.2f, %.2f, %.2f)",
                stop_pos.x(), stop_pos.y(), stop_pos.z());
    if (log_manager_) {
        log_manager_->warnf("EMERGENCY STOP executed at position (%.2f, %.2f, %.2f)",
                           stop_pos.x(), stop_pos.y(), stop_pos.z());
    }

    return true;
}

void PathManager::refreshEffectiveRiskZones()
{
    ++zone_policy_generation_;  // [S13] any zone/terrain re-derivation
                                // invalidates every outstanding snapshot
    risk_zones_ = risk_zones_raw_;
    if (!risk_zone_agl_ || risk_zones_.empty()) return;
    if (!terrain_data_.valid) {
        // No DEM yet: raw z is used as-is; setTerrainData() re-derives.
        if (log_manager_) {
            log_manager_->infof(
                "[RISK-ZONE] AGL grounding deferred until DEM arrives "
                "(%zu zones)", risk_zones_.size());
        }
        return;
    }
    for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
        auto &tz = risk_zones_[zi];
        const double z_agl = risk_zones_raw_[zi].center.z();
        // Same water/off-DEM rule as groundedCenter: sea level is 0.
        double base = 0.0;
        const float h = terrain_data_.getElevation(tz.center.x(),
                                                   tz.center.y());
        if (std::isfinite(h) && h > 0.0f) base = static_cast<double>(h);
        tz.center.z() = base + z_agl;
        if (log_manager_) {
            log_manager_->infof(
                "[RISK-ZONE] zone[%zu] grounded: DEM h=%.2f + source height "
                "%.2f -> source z=%.2f", zi, base, z_agl, tz.center.z());
            if (z_agl > 2.0) {
                log_manager_->warnf(
                    "[RISK-ZONE] zone[%zu] source height %.1f units = %.0f m "
                    "AGL — looks like an absolute altitude or a metres "
                    "value (frame is 1 unit = 100 m)", zi, z_agl,
                    100.0 * z_agl);
            }
        }
    }
}

double PathManager::riskEllipsoidRadius(const RiskZone &zone,
                                        const Eigen::Vector3d &pos) const
{
    if (!(zone.reach > 0.0) || !(risk_vertical_ratio_ > 0.0))
        return std::numeric_limits<double>::infinity();
    // CONTRACT: rv (vertical semi-axis) MUST match the value the optimizer and
    // heatmap use — oz.vertical_reach = zone.reach * risk_vertical_ratio_. All
    // three recompute it independently today. If a per-zone vertical override
    // is ever added, thread zone.vertical_reach through HERE and the heatmap so
    // the rendered/penalized moat geometry cannot silently diverge.
    const double rv = zone.reach * risk_vertical_ratio_;
    const Eigen::Vector3d d = pos - zone.center;
    return std::sqrt(d.head<2>().squaredNorm() /
                         (zone.reach * zone.reach) +
                     d.z() * d.z() / (rv * rv));
}

double PathManager::riskZoneValue(size_t zone_index,
                                  const Eigen::Vector3d &pos) const
{
    if (zone_index >= risk_zones_.size()) return 0.0;
    constexpr double kMoatCap = 1.0 - 1e-3;
    const auto &zone = risk_zones_[zone_index];
    const double q = riskEllipsoidRadius(zone, pos);
    if (!(q < 1.0)) return 0.0;
    const double u = 1.0 - q;
    return std::min(zone.peak * u * u *
                        riskVisibilityValue(zone_index, pos),
                    kMoatCap);
}

void PathManager::rebuildTerrainRiskMasks()
{
    terrain_risk_masks_.assign(risk_zones_.size(), TerrainRiskMask{});

    if (!risk_terrain_mask_enable_ || !terrain_data_.valid ||
        risk_zones_.empty()) {
        if (log_manager_ && !risk_zones_.empty()) {
            log_manager_->infof(
                "[RISK-MASK] terrain masking %s; publishing ideal fallback (%zu zones)",
                risk_terrain_mask_enable_ ? "waiting for DEM" : "disabled",
                risk_zones_.size());
        }
        publishEffectiveRiskField();
        return;
    }

    constexpr double kTwoPi = 6.28318530717958647692;
    size_t total_samples = 0;
    for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
        const auto &zone = risk_zones_[zi];
        if (!(zone.reach > 0.0)) continue;

        TerrainRiskMask &mask = terrain_risk_masks_[zi];
        mask.source = zone.center;
        // The optimizer's smooth barrier extends 5% beyond the moat rim. The
        // horizon field covers it as well so both terms use one LOS mask.
        mask.max_range = 1.05 * zone.reach;
        const double requested_step =
            risk_mask_radial_step_ > 0.0 ? risk_mask_radial_step_
                                         : terrain_data_.resolution;
        const double step = std::max(0.02, requested_step);
        // Lower clamp 16, not 2: the ray march can only shadow ring ri from
        // strictly nearer rings, so ring 1 is always clear and a 2-ring mask
        // (any zone with reach <~ one DEM cell) would be ALL clear — masking
        // silently off while "horizon ready" still logs. 16 rings keeps the
        // always-clear ring at <=max_range/15 (sub-DEM-cell, where LOS
        // blocking is geometrically impossible anyway).
        mask.radial_count = std::clamp(
            static_cast<int>(std::ceil(mask.max_range / step)) + 1, 16, 8192);
        mask.radial_step =
            mask.max_range / static_cast<double>(mask.radial_count - 1);
        // At the outer rim, adjacent rays are no farther apart than one
        // radial/DEM sample. This avoids angular gaps behind narrow ridges.
        mask.angular_count = std::clamp(
            static_cast<int>(std::ceil(kTwoPi * mask.max_range /
                                       mask.radial_step)),
            72, 4096);
        const float clear_ceiling = static_cast<float>(
            zone.center.z() - 2.0 * mask.max_range -
            20.0 * risk_mask_softness_ - 1.0);
        mask.shadow_ceiling.assign(
            static_cast<size_t>(mask.angular_count) * mask.radial_count,
            clear_ceiling);

        const float source_ground =
            terrain_data_.getElevation(zone.center.x(), zone.center.y());
        if (std::isfinite(source_ground) && zone.center.z() <= source_ground &&
            log_manager_) {
            log_manager_->warnf(
                "[RISK-MASK] zone[%zu] source z=%.2f is at/below DEM %.2f; "
                "center.z is used literally as the source height",
                zi, zone.center.z(), static_cast<double>(source_ground));
        }

        const double dtheta = kTwoPi / mask.angular_count;
        for (int ai = 0; ai < mask.angular_count; ++ai) {
            const double theta = (static_cast<double>(ai) + 0.5) * dtheta;
            const double ct = std::cos(theta);
            const double st = std::sin(theta);
            double max_slope = -std::numeric_limits<double>::infinity();
            const size_t row = static_cast<size_t>(ai) * mask.radial_count;
            for (int ri = 1; ri < mask.radial_count; ++ri) {
                const double range = ri * mask.radial_step;
                // Liu et al. (2023), Eq. (9): the shadow ceiling at this
                // range is generated by the maximum elevation angle among
                // all NEARER samples. The current sample is incorporated only
                // after storing, so terrain does not occlude a point sitting
                // exactly on its own range sample.
                if (std::isfinite(max_slope)) {
                    mask.shadow_ceiling[row + ri] = static_cast<float>(
                        zone.center.z() + range * max_slope);
                }
                const float h = terrain_data_.getElevation(
                    zone.center.x() + range * ct,
                    zone.center.y() + range * st);
                if (std::isfinite(h)) {
                    max_slope = std::max(
                        max_slope,
                        (static_cast<double>(h) - zone.center.z()) / range);
                }
            }
        }
        mask.valid = true;
        total_samples += mask.shadow_ceiling.size();
        if (log_manager_) {
            log_manager_->infof(
                "[RISK-MASK] zone[%zu] horizon ready: %d azimuth x %d range "
                "(dr=%.3f, R=%.1f)",
                zi, mask.angular_count, mask.radial_count,
                mask.radial_step, zone.reach);
        }
    }
    if (log_manager_) {
        log_manager_->infof(
            "[RISK-MASK] built %zu terrain-horizon samples for %zu zones "
            "(soft edge %.3f z-units)",
            total_samples, risk_zones_.size(), risk_mask_softness_);
    }
    publishEffectiveRiskField();
}

double PathManager::riskShadowCeiling(size_t zone_index,
                                      const Eigen::Vector3d &pos) const
{
    if (!risk_terrain_mask_enable_ || zone_index >= terrain_risk_masks_.size())
        return -std::numeric_limits<double>::infinity();
    const TerrainRiskMask &mask = terrain_risk_masks_[zone_index];
    if (!mask.valid || mask.radial_count < 2 || mask.angular_count < 2)
        return -std::numeric_limits<double>::infinity();

    const double dx = pos.x() - mask.source.x();
    const double dy = pos.y() - mask.source.y();
    const double range = std::hypot(dx, dy);
    if (range <= 1e-9 || range > mask.max_range)
        return -std::numeric_limits<double>::infinity();

    constexpr double kTwoPi = 6.28318530717958647692;
    double theta = std::atan2(dy, dx);
    if (theta < 0.0) theta += kTwoPi;
    // Samples were built at angular cell centres (i+0.5)*dtheta.
    double af = theta * mask.angular_count / kTwoPi - 0.5;
    int a0 = static_cast<int>(std::floor(af));
    const double at = af - std::floor(af);
    a0 = (a0 % mask.angular_count + mask.angular_count) % mask.angular_count;
    const int a1 = (a0 + 1) % mask.angular_count;

    const double rf = std::min(
        range / mask.radial_step,
        static_cast<double>(mask.radial_count - 1));
    const int r0 = std::clamp(static_cast<int>(std::floor(rf)),
                              0, mask.radial_count - 1);
    const int r1 = std::min(r0 + 1, mask.radial_count - 1);
    const double rt = rf - r0;
    auto sample = [&mask](int ai, int ri) -> double {
        return static_cast<double>(mask.shadow_ceiling[
            static_cast<size_t>(ai) * mask.radial_count + ri]);
    };
    const double z0 = (1.0 - rt) * sample(a0, r0) + rt * sample(a0, r1);
    const double z1 = (1.0 - rt) * sample(a1, r0) + rt * sample(a1, r1);
    return (1.0 - at) * z0 + at * z1;
}

// [RISK-GROUNDED] (manager/risk_grounded, default ON) The visibility sigmoid's
// ∂v/∂z = v(1-v)/σ ≥ 0 is a pure DOWN-pull
// ("lower = terrain-occluded"); over a ridge
// inside a zone it out-guns the terrain cubic 48x and settles the equilibrium
// BELOW the surface (measured -0.355 u, VDIAG terr +4803 vs risk -4574).
// Grounding evaluates the sigmoid at z_eff = smoothmax(z, terrain + band):
// below the clearance band there is no legal terrain-occluded state, so the z-pull
// fades to zero while xy-steering (shadow seams) is untouched. Softplus with
// width = risk_mask_softness_ keeps value and analytic z-gradient ONE C1
// surface (project hard rule); the xy central differences query this same
// clamped scalar so they inherit the clamp (incl. its ∂h/∂xy) automatically.
// Cascades to every consumer of the shared field: optimizer moat/barrier,
// FE getRiskNorm, guard getRiskCost, and the barrier-exemption predicates —
// a terrain-hugging forced transit then reads v>0.5 and is exempted as
// designed.
double PathManager::riskGroundedZ(const Eigen::Vector3d &pos,
                                  double *dzeff_dz) const
{
    if (dzeff_dz) *dzeff_dz = 1.0;
    if (!risk_grounded_ || !terrain_data_.valid) return pos.z();
    float h = 0.f, gx = 0.f, gy = 0.f;
    if (!terrain_data_.getElevationAndGrad(pos.x(), pos.y(), &h, &gx, &gy))
        return pos.z();  // off-DEM: no terrain, no clamp
    const double floor_z = static_cast<double>(h) + opt_obstacle_clearance_;
    // Blend width = σ (validated). MEASURED DEAD END (2026-07-20, do not
    // retry): narrowing to σ/4 or σ/2 to stop the softplus tail leaking
    // low-exposure pull below the band (VDIAG: terr +1108 vs risk -902 at
    // clr=0.108 → 3.7 m graze) BROKE the complex-field stress case both
    // times (-1004 + collision). The width is not a local optimizer knob: it
    // reshapes the
    // shared field for the FRONT-END too (max_risk_simple 106.6 → 0.5 =
    // different seed homotopy), reigniting the cost conflict. The 3.7 m graze
    // is instead addressed by the clearance band itself
    // (optimization/obstacle_clearance): a higher band raises the clamp
    // floor AND the terrain cubic at the leak depth quadratically, moving
    // the equilibrium up without touching the field's shape.
    const double w = std::max(1e-6, risk_mask_softness_);
    const double a = (pos.z() - floor_z) / w;
    if (a > 30.0) return pos.z();  // far above the band: identity (s=1)
    if (a < -30.0) {
        if (dzeff_dz) *dzeff_dz = 0.0;
        return floor_z;
    }
    if (dzeff_dz) *dzeff_dz = 1.0 / (1.0 + std::exp(-a));
    return floor_z + w * std::log1p(std::exp(a));
}

double PathManager::riskVisibilityValue(size_t zone_index,
                                        const Eigen::Vector3d &pos) const
{
    const double ceiling = riskShadowCeiling(zone_index, pos);
    if (!std::isfinite(ceiling)) return 1.0;
    const double z_eff = riskGroundedZ(pos, nullptr);
    const double q = (z_eff - ceiling) / risk_mask_softness_;
    if (q >= 40.0) return 1.0;
    if (q <= -40.0) return 0.0;
    return 1.0 / (1.0 + std::exp(-q));
}

double PathManager::riskVisibility(size_t zone_index,
                                   const Eigen::Vector3d &pos,
                                   Eigen::Vector3d *grad) const
{
    const double value = riskVisibilityValue(zone_index, pos);
    if (!grad) return value;
    grad->setZero();
    if (zone_index >= terrain_risk_masks_.size() ||
        !terrain_risk_masks_[zone_index].valid ||
        value <= 1e-12 || value >= 1.0 - 1e-12) {
        return value;
    }

    // The polar horizon table is bilinear but its xy chain rule is awkward at
    // angle wrap and at the source. A small central difference of the SAME
    // scalar query gives L-BFGS a consistent gradient without putting ray
    // marching in the optimization loop. z is analytic.
    const double eps = std::clamp(
        0.25 * terrain_risk_masks_[zone_index].radial_step, 0.02, 0.50);
    Eigen::Vector3d pp = pos;
    Eigen::Vector3d pm = pos;
    pp.x() += eps;
    pm.x() -= eps;
    grad->x() = (riskVisibilityValue(zone_index, pp) -
                 riskVisibilityValue(zone_index, pm)) / (2.0 * eps);
    pp = pos;
    pm = pos;
    pp.y() += eps;
    pm.y() -= eps;
    grad->y() = (riskVisibilityValue(zone_index, pp) -
                 riskVisibilityValue(zone_index, pm)) / (2.0 * eps);
    // Analytic z-grad through the (optional) grounded clamp: chain rule with
    // ∂z_eff/∂z — value itself already came from the clamped scalar above.
    double dzeff_dz = 1.0;
    (void)riskGroundedZ(pos, &dzeff_dz);
    grad->z() = value * (1.0 - value) / risk_mask_softness_ * dzeff_dz;
    return value;
}

// Draped visibility-boundary heatmap. One latched GridMap summarises the
// terrain-masked field as "how low can this column be flown before some zone
// sees me", in four color categories:
//   warm ramp (red -> orange -> yellow)  floor below agl_max: visible
//                                        inside the mission band; red = at
//                                        ground level.
//   green                                floor at/above agl_max: safe in the
//                                        band, seen only if you climb.
//                                        (Transparent instead when
//                                        risk_heatmap_safe_transparent.)
//   deep blue                            NEVER visible in the footprint
//                                        (terrain shadow / envelope wholly
//                                        underground or over the horizon).
// [TRAJ-RISK] The draped heatmap shows the GROUND visibility boundary — not what
// the aircraft experiences at its 3D position, so "red cell under a green
// (safe) flight segment" is common and confusing. This line is the missing
// view where color == cost: sample the OR-combined terrain-masked moat field
// along the planned trajectory and paint the path green (0) -> yellow -> red
// (>= peak-ish). Latched, one marker per plan.
// [DEBUG-PIPELINE] The two invisible stages between the existing channels:
//   /viz/front_end_path      FM2 avoidance path            (existing)
//   shortcut_points  <-- the vertices the front end COMMITTED (this)
//   inner_points     <-- the sparse seed MINCO receives       (this)
//   /viz/global_trajectory   initial min-jerk               (existing)
//   /viz/opt_trajectory      optimized final                (existing)
// One latched MarkerArray per plan (element 0 = DELETEALL), same atomicity
// contract as /viz/risk_field. Debug mode only.
void PathManager::publishPipelineDebug(
    const std::vector<Eigen::Vector3d> &shortcut_route,
    const std::vector<Eigen::Vector3d> &inner_points)
{
    if (!debug_pipeline_viz_ || !debug_pipeline_pub_) return;

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = node_->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    auto sphere_list = [&](const char *ns, double scale,
                           float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = ns;
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.95f;
        return m;
    };

    // Shortcut vertices: what the front end committed. Orange, chunky.
    auto sc = sphere_list("shortcut_points", 0.9, 1.0f, 0.55f, 0.10f);
    for (const auto &p : shortcut_route) {
        geometry_msgs::msg::Point q;
        q.x = p.x(); q.y = p.y(); q.z = p.z();
        sc.points.push_back(q);
    }
    arr.markers.push_back(std::move(sc));

    // Index labels so a bad vertex can be named, not pointed at. Thinned on
    // long routes to keep RViz responsive.
    const size_t n = shortcut_route.size();
    const size_t stride = n > 120 ? (n + 119) / 120 : 1;
    for (size_t i = 0; i < n; i += stride) {
        visualization_msgs::msg::Marker t;
        t.header.frame_id = "map";
        t.header.stamp = node_->now();
        t.ns = "shortcut_index";
        t.id = static_cast<int>(i);
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::msg::Marker::ADD;
        t.pose.position.x = shortcut_route[i].x();
        t.pose.position.y = shortcut_route[i].y();
        t.pose.position.z = shortcut_route[i].z() + 1.4;
        t.pose.orientation.w = 1.0;
        t.scale.z = 1.2;
        t.color.r = 1.0f; t.color.g = 0.75f; t.color.b = 0.35f;
        t.color.a = 0.9f;
        t.text = std::to_string(i);
        arr.markers.push_back(std::move(t));
    }

    // MINCO seed: the sparse pieces the optimizer actually starts from. Cyan,
    // small — dense enough that size must stay subtle next to the shortcut.
    auto ip = sphere_list("inner_points", 0.35, 0.15f, 0.80f, 0.95f);
    for (const auto &p : inner_points) {
        geometry_msgs::msg::Point q;
        q.x = p.x(); q.y = p.y(); q.z = p.z();
        ip.points.push_back(q);
    }
    arr.markers.push_back(std::move(ip));

    debug_pipeline_pub_->publish(arr);
    log_manager_->infof("[DEBUG-PIPELINE] published %zu shortcut + %zu inner points",
                        shortcut_route.size(), inner_points.size());
}

void PathManager::publishTubeMarker(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
    const std::vector<Eigen::Vector3d> &pts, const std::string &ns_prefix,
    float r, float g, float b, float a)
{
    if (!pub || pts.empty()) return;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    const int drone_id = std::max(0, traj_.local_traj.drone_id);
    m.ns = ns_prefix + std::to_string(drone_id);
    m.id = drone_id;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;  // identity quaternion (avoids RViz cull/flicker)
    m.scale.x = m.scale.y = m.scale.z = path_scale_;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.points.reserve(pts.size());
    for (const auto &p : pts) {
        geometry_msgs::msg::Point gp;
        gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
        m.points.push_back(gp);
    }
    pub->publish(m);
}

void PathManager::clearTrajectoryViz()
{
    // The tube and risk channels are transient_local (latched), so a REFUSED
    // plan whose trajectory was invalidated still shows in RViz as the
    // current plan — and any RViz that connects afterwards is handed it from
    // the latch. Zeroing the two execution fields says nothing to a
    // subscriber. Every refusal exit that invalidates must also erase, or
    // the operator is looking at a flight the planner already rejected.
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "world";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    for (const auto &pub : {opt_traj_tube_pub_, global_traj_tube_pub_,
                            traj_risk_pub_}) {
        if (pub) pub->publish(del);
    }
    // The risk profile is a Float64MultiArray, not markers: an EMPTY array
    // is its "nothing to show" value.
    if (risk_profile_pub_) {
        std_msgs::msg::Float64MultiArray empty;
        risk_profile_pub_->publish(empty);
    }
}

void PathManager::publishTrajTube(
    const poly_traj::Trajectory &traj,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
    const std::string &ns_prefix, float r, float g, float b, float a)
{
    const double T = traj.getDurations().sum();
    if (T <= 1e-6) return;
    // 0.1 s sampling plus the exact endpoint — the density the retired
    // bridge node used; dense enough to read as a tube at map scale.
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(static_cast<size_t>(T / 0.1) + 2);
    for (double t = 0.0; t < T; t += 0.1) pts.push_back(traj.getPos(t));
    pts.push_back(traj.getPos(T - 1e-9));
    publishTubeMarker(pub, pts, ns_prefix, r, g, b, a);
}

void PathManager::publishTrajRisk(const poly_traj::Trajectory &traj)
{
    if (!traj_risk_pub_) return;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    m.ns = "traj_risk";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.45;  // line width (u); slightly thinner than the main traj
    m.lifetime = rclcpp::Duration(0, 0);

    const double T = traj.getDurations().sum();
    if (T <= 1e-6 || risk_zones_.empty()) {
        // No zones (or empty traj): publish DELETE so a stale colored line
        // from a previous zoned plan cannot outlive its mission.
        m.action = visualization_msgs::msg::Marker::DELETE;
        traj_risk_pub_->publish(m);
        return;
    }
    const int n = std::min(2000, std::max(50, static_cast<int>(T / 0.5)));
    m.points.reserve(n + 1);
    m.colors.reserve(n + 1);
    for (int i = 0; i <= n; ++i) {
        const double t = T * static_cast<double>(i) / n;
        const Eigen::Vector3d p = traj.getPos(std::min(t, T - 1e-9));
        // OR-combined effective moat (terrain-masked, same field the
        // optimizer prices): r = 1 - prod(1 - m_i).
        double surv = 1.0;
        for (size_t zi = 0; zi < risk_zones_.size(); ++zi)
            surv *= (1.0 - riskZoneValue(zi, p));
        const double r = 1.0 - surv;
        // Perceptual ramp: sqrt lifts the faint-halo values so "slightly
        // visible" is visibly yellow-ish instead of indistinguishable green.
        const double s = std::sqrt(std::clamp(r, 0.0, 1.0));
        geometry_msgs::msg::Point gp;
        gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
        std_msgs::msg::ColorRGBA c;
        c.a = 0.95f;
        if (s < 0.5) {          // green -> yellow
            c.r = static_cast<float>(2.0 * s);
            c.g = 0.85f;
            c.b = 0.05f;
        } else {                // yellow -> red
            c.r = 0.95f;
            c.g = static_cast<float>(0.85 * 2.0 * (1.0 - s));
            c.b = 0.05f;
        }
        m.points.push_back(gp);
        m.colors.push_back(c);
    }
    // Diagnostic: max experienced moat along the line — if this is 0 while
    // the optimizer's risk_cost was nonzero, the viz field and the priced
    // field have diverged (bug), or the cost was pure barrier/velocity terms.
    {
        double max_r = 0.0; Eigen::Vector3d max_p = Eigen::Vector3d::Zero();
        for (int i = 0; i <= n; ++i) {
            const double t = T * static_cast<double>(i) / n;
            const Eigen::Vector3d p = traj.getPos(std::min(t, T - 1e-9));
            double surv = 1.0;
            for (size_t zi = 0; zi < risk_zones_.size(); ++zi)
                surv *= (1.0 - riskZoneValue(zi, p));
            if (1.0 - surv > max_r) { max_r = 1.0 - surv; max_p = p; }
        }
        log_manager_->infof(
            "[TRAJ-RISK] published %d pts, max experienced moat r=%.5f at "
            "(%.1f, %.1f, %.2f), zones=%zu",
            n + 1, max_r, max_p.x(), max_p.y(), max_p.z(), risk_zones_.size());
    }
    traj_risk_pub_->publish(m);
}

bool PathManager::riskProfileVisibleInterval(size_t zone_index,
                                             double x, double y,
                                             double *lower_z,
                                             double *floor_z,
                                             double *top_z) const
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (lower_z) *lower_z = nan;
    if (floor_z) *floor_z = nan;
    if (top_z) *top_z = nan;
    if (!floor_z || !top_z || zone_index >= risk_zones_.size())
        return false;

    const auto &zone = risk_zones_[zone_index];
    if (!(zone.reach > 0.0) || !(zone.peak > 0.0) ||
        !(risk_vertical_ratio_ > 0.0)) {
        return false;
    }

    const double dx = x - zone.center.x();
    const double dy = y - zone.center.y();
    const double rho2 = dx * dx + dy * dy;
    const double reach2 = zone.reach * zone.reach;
    if (!(rho2 < reach2)) return false;

    const double rv = zone.reach * risk_vertical_ratio_;
    const double half_z =
        rv * std::sqrt(std::max(0.0, 1.0 - rho2 / reach2));
    const double ellipsoid_lower = zone.center.z() - half_z;
    const double ellipsoid_upper = zone.center.z() + half_z;

    // Keep the same sea/off-map convention used when risk-zone sources are
    // grounded: finite positive DEM is land; pure water and off-DEM use MSL 0.
    double ground = 0.0;
    float terrain_h = 0.0f, terrain_gx = 0.0f, terrain_gy = 0.0f;
    const bool has_ground_sample =
        terrain_data_.valid &&
        terrain_data_.getElevationAndGrad(
            x, y, &terrain_h, &terrain_gx, &terrain_gy);
    if (has_ground_sample && std::isfinite(terrain_h) && terrain_h > 0.0f)
        ground = static_cast<double>(terrain_h);

    double visible_floor = std::max(ellipsoid_lower, ground);
    if (!(visible_floor < ellipsoid_upper)) return false;

    // riskVisibilityValue is monotone in raw z. Find its configured contour
    // in raw trajectory-z coordinates, rather than mistaking the effective
    // (grounded) z contour for a trajectory altitude.
    const double threshold = risk_mask_viz_threshold_;
    if (threshold >= 1.0) return false;  // no finite visible interval
    if (threshold > 0.0) {
        const double ceiling = riskShadowCeiling(
            zone_index, Eigen::Vector3d(x, y, zone.center.z()));
        if (std::isfinite(ceiling)) {
            const double w = std::max(1e-6, risk_mask_softness_);
            const double z_eff_boundary =
                ceiling + w * std::log(threshold / (1.0 - threshold));
            double raw_z_boundary = z_eff_boundary;

            if (risk_grounded_ && has_ground_sample) {
                const double grounded_floor =
                    static_cast<double>(terrain_h) +
                    opt_obstacle_clearance_;
                const double d =
                    (z_eff_boundary - grounded_floor) / w;
                if (d <= 0.0) {
                    // smoothmax(z, grounded_floor) is strictly above its
                    // asymptote for every finite z: the whole shell is visible.
                    raw_z_boundary =
                        -std::numeric_limits<double>::infinity();
                } else if (d > 50.0) {
                    // log(expm1(d)) = d + log(1-exp(-d)); this form avoids
                    // overflow while retaining the exact inverse.
                    raw_z_boundary =
                        grounded_floor +
                        w * (d + std::log1p(-std::exp(-d)));
                } else {
                    raw_z_boundary =
                        grounded_floor + w * std::log(std::expm1(d));
                }
            }
            visible_floor = std::max(visible_floor, raw_z_boundary);
        }
        // A non-finite horizon is the same full-visibility fallback used by
        // riskVisibilityValue (mask disabled/not ready, source cell, etc.).
    }

    // [SHADOW-FULL] A column whose LOS boundary rises above the ellipsoid cap
    // is FULLY shadowed inside the zone — the safest spot in the footprint,
    // not "no zone". Reporting it as no-interval (old behavior) punched
    // white holes into the panel's zone band that were indistinguishable
    // from uncovered sky. Report a zero-height band AT the cap instead
    // (detection floor == zone top): the red band degenerates to nothing and
    // the shadow-safe fill covers the whole cross-section.
    if (!(visible_floor < ellipsoid_upper)) {
        if (lower_z) *lower_z = ellipsoid_lower;
        *floor_z = ellipsoid_upper;
        *top_z = ellipsoid_upper;
        return true;
    }
    if (lower_z) *lower_z = ellipsoid_lower;
    *floor_z = visible_floor;
    *top_z = ellipsoid_upper;
    return true;
}

// [RISK-PROFILE] Version 3 is self-describing, binds data to source xyz, and
// preserves every zone's
// separate visible vertical interval, so disjoint/overlapping zones cannot be
// painted as one false union band. The risk scalar is evaluated at the actual
// 3-D trajectory sample with the exact same OR-combination as /viz/traj_risk.
void PathManager::publishRiskProfile(const poly_traj::Trajectory &traj)
{
    if (!risk_profile_pub_) return;
    const double T = traj.getTotalDuration();
    std_msgs::msg::Float64MultiArray msg;
    if (!(T > 0.0) || !std::isfinite(T) || risk_zones_.empty()) {
        risk_profile_pub_->publish(msg);  // empty = clear stale profile
        return;
    }
    const double est_len = std::max(1.0, T * max_vel_);
    const int K = std::clamp(static_cast<int>(est_len / 1.0), 256, 16384);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const size_t zone_count = risk_zones_.size();
    // v4: per-zone TRIPLET (geometric lower, detection floor, geometric top)
    // — the panel needs the cross-section lower bound to paint mid-air
    // shadow-safe pockets between overlapping zones (a gap between zone A's
    // cap and zone B's floor was indistinguishable from uncovered sky).
    const size_t record_width = 5 + 3 * zone_count;

    msg.data.reserve(2 + record_width * static_cast<size_t>(K + 1));
    msg.data.push_back(4.0);
    msg.data.push_back(static_cast<double>(zone_count));
    double s = 0.0;
    Eigen::Vector3d prev = traj.getPos(0.0);
    for (int k = 0; k <= K; ++k) {
        const double t = T * static_cast<double>(k) / K;
        const Eigen::Vector3d p = traj.getPos(std::min(t, T - 1e-9));
        s += (p - prev).head<2>().norm();
        prev = p;

        double survival = 1.0;
        for (size_t zi = 0; zi < zone_count; ++zi)
            survival *= (1.0 - riskZoneValue(zi, p));
        const double combined_risk = 1.0 - survival;

        msg.data.push_back(s);
        msg.data.push_back(p.x());
        msg.data.push_back(p.y());
        msg.data.push_back(p.z());
        msg.data.push_back(combined_risk);
        for (size_t zi = 0; zi < zone_count; ++zi) {
            double lower_z = nan;
            double floor_z = nan;
            double top_z = nan;
            (void)riskProfileVisibleInterval(
                zi, p.x(), p.y(), &lower_z, &floor_z, &top_z);
            msg.data.push_back(lower_z);
            msg.data.push_back(floor_z);
            msg.data.push_back(top_z);
        }
    }
    risk_profile_pub_->publish(msg);
    log_manager_->infof(
        "[RISK-PROFILE] v4: %d samples over %.1f u, zones=%zu, width=%zu",
        K + 1, s, zone_count, record_width);
}

// Safe corridors are rendered POSITIVELY instead of being the absence of
// red. Rendering via grid_map_rviz_plugin reuses the terrain-mesh pipeline
// (lighting, one mesh, no alpha-sorted marker soup), which is the point of
// this mode.
void PathManager::publishRiskHeatmap()
{
    if (!risk_heatmap_pub_) return;

    // Message-layout contract shared with terrain_publisher.py and
    // TerrainData ([TERRAIN-FRAME]): matrix ROWS span X (mirrored), COLS
    // span Y (mirrored), data[col * rows + row], length_x = rows * res.
    auto makeLayer = [](int rows, int cols) {
        std_msgs::msg::Float32MultiArray arr;
        arr.layout.dim.resize(2);
        arr.layout.dim[0].label = "column_index";
        arr.layout.dim[0].size = static_cast<uint32_t>(cols);
        arr.layout.dim[0].stride = static_cast<uint32_t>(cols) * rows;
        arr.layout.dim[1].label = "row_index";
        arr.layout.dim[1].size = static_cast<uint32_t>(rows);
        arr.layout.dim[1].stride = static_cast<uint32_t>(rows);
        arr.data.assign(static_cast<size_t>(rows) * cols,
                        std::numeric_limits<float>::quiet_NaN());
        return arr;
    };
    grid_map_msgs::msg::GridMap msg;
    msg.header.frame_id = "map";
    msg.header.stamp = node_->now();
    msg.layers = {"elevation", "color"};

    constexpr double kInf = std::numeric_limits<double>::infinity();
    double x0 = kInf, x1 = -kInf, y0 = kInf, y1 = -kInf;
    for (const auto &zone : risk_zones_) {
        if (!(zone.reach > 0.0) || !(zone.peak > 0.0)) continue;
        x0 = std::min(x0, zone.center.x() - zone.reach);
        x1 = std::max(x1, zone.center.x() + zone.reach);
        y0 = std::min(y0, zone.center.y() - zone.reach);
        y1 = std::max(y1, zone.center.y() + zone.reach);
    }
    const bool hm3d = risk_mask_viz_mode_ == "heatmap3d";
    if ((risk_mask_viz_mode_ != "heatmap" && !hm3d) || !(x1 > x0)) {
        // Latched topic: replace any stale heatmap with an all-NaN stub.
        msg.info.resolution = 1.0;
        msg.info.length_x = 1.0;
        msg.info.length_y = 1.0;
        msg.info.pose.position.x = 0.0;
        msg.info.pose.position.y = 0.0;
        msg.info.pose.orientation.w = 1.0;
        msg.data = {makeLayer(1, 1), makeLayer(1, 1)};
        risk_heatmap_pub_->publish(msg);
        return;
    }

    const double extent = std::max(x1 - x0, y1 - y0);
    const double res = std::max(
        extent / static_cast<double>(risk_heatmap_max_dim_), 0.05);
    const int rows = std::max(1, static_cast<int>(std::ceil((x1 - x0) / res)));
    const int cols = std::max(1, static_cast<int>(std::ceil((y1 - y0) / res)));
    const double length_x = rows * res;
    const double length_y = cols * res;
    msg.info.resolution = res;
    msg.info.length_x = length_x;
    msg.info.length_y = length_y;
    msg.info.pose.position.x = x0 + 0.5 * length_x;
    msg.info.pose.position.y = y0 + 0.5 * length_y;
    msg.info.pose.orientation.w = 1.0;
    auto elevation = makeLayer(rows, cols);
    auto color = makeLayer(rows, cols);

    // grid_map_core colorVectorToValue packing (0xRRGGBB reinterpreted as
    // float32). Self-contained — terrain_publisher.py no longer has a color
    // layer (elevation-only since the TerrainTiles migration).
    auto pack = [](int r, int g, int b) -> float {
        const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                             (static_cast<uint32_t>(g) << 8) |
                             static_cast<uint32_t>(b);
        float f;
        std::memcpy(&f, &rgb, sizeof(f));
        return f;
    };
    // Danger ramp stays WARM only (red at floor 0 -> yellow near agl_max):
    // green is reserved for the explicit safe category so "green = safe"
    // never collides with "high-but-still-visible".
    auto rampColor = [&pack](double t) -> float {
        const double h = 60.0 * std::clamp(t, 0.0, 1.0) / 60.0;
        const double s = 0.90, v = 0.95;
        const int i = static_cast<int>(h);
        const double f = h - i;
        const double p = v * (1.0 - s);
        const double q = v * (1.0 - s * f);
        const double u = v * (1.0 - s * (1.0 - f));
        double r = v, g = p, b = p;
        switch (i) {
            case 0: r = v; g = u; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = u; break;
            default: r = p; g = q; b = v; break;  // 180..185 deg
        }
        return pack(static_cast<int>(255.0 * r + 0.5),
                    static_cast<int>(255.0 * g + 0.5),
                    static_cast<int>(255.0 * b + 0.5));
    };
    const float shadow_color = pack(40, 95, 215);
    const float safe_color = pack(72, 204, 106);

    // Same visibility contour as the marker modes: the floor follows the
    // configured sigmoid threshold, not necessarily the midpoint.
    double vis_offset = 0.0;
    if (risk_mask_viz_threshold_ > 1e-6 &&
        risk_mask_viz_threshold_ < 1.0 - 1e-6) {
        vis_offset = risk_mask_softness_ * std::log(
            risk_mask_viz_threshold_ / (1.0 - risk_mask_viz_threshold_));
    }
    const double rv_ratio = risk_vertical_ratio_;

    // [PREVIEW] No DEM yet (before the first plan loads the corridor map):
    // every column falls back to sea level, so the drape would be BURIED
    // inside the rendered terrain mesh — the ideal-field preview was
    // invisible exactly when the user wants to compare it against the
    // terrain-masked result. Float the whole sheet above the tallest
    // rendered terrain instead. Purely a render height; the field itself is
    // the same ideal (unmasked) field it always was.
    const double preview_lift =
        terrain_data_.valid ? 0.0 : risk_heatmap_preview_lift_;

    size_t painted = 0;
    for (int row = 0; row < rows; ++row) {
        const double wx = x0 + length_x - (row + 0.5) * res;
        for (int col = 0; col < cols; ++col) {
            const double wy = y0 + length_y - (col + 0.5) * res;
            // Water/off-DEM columns sit at sea level 0 (same rule as
            // refreshEffectiveRiskZones / the marker modes).
            double ground = 0.0;
            // Render height uses the MAX elevation within the cell, not the
            // center: on steep ridges the terrain relief inside one coarse
            // heatmap cell exceeds even the 100 m drape offset, so the
            // terrain mesh pierced the drape and z-fought along the
            // intersection line (herringbone bands tracing ridge lines).
            // Semantic quantities (floor AGL, colors) keep the CENTER
            // elevation; only the drawn surface rides the in-cell crest.
            double ground_render = 0.0;
            if (terrain_data_.valid) {
                const float h = terrain_data_.getElevation(wx, wy);
                if (std::isfinite(h) && h > 0.0f)
                    ground = static_cast<double>(h);
                ground_render = ground;
                const double hr = 0.5 * res;
                for (const auto &d :
                     {std::pair<double, double>{-hr, -hr}, {-hr, hr},
                      {hr, -hr}, {hr, hr}}) {
                    const float hc = terrain_data_.getElevation(
                        wx + d.first, wy + d.second);
                    if (std::isfinite(hc))
                        ground_render = std::max(
                            ground_render, static_cast<double>(hc));
                }
            }
            bool in_footprint = false;
            double floor_agl = kInf;
            for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
                const auto &zone = risk_zones_[zi];
                if (!(zone.reach > 0.0) || !(zone.peak > 0.0)) continue;
                const double dx = wx - zone.center.x();
                const double dy = wy - zone.center.y();
                const double rho2 = dx * dx + dy * dy;
                if (rho2 >= zone.reach * zone.reach) continue;
                in_footprint = true;
                const double rv = zone.reach * rv_ratio;
                const double half_z = rv * std::sqrt(std::max(
                    0.0, 1.0 - rho2 / (zone.reach * zone.reach)));
                const double upper = zone.center.z() + half_z;
                if (upper <= ground) continue;  // envelope underground here
                double det = std::max(zone.center.z() - half_z, ground);
                const double horizon = riskShadowCeiling(
                    zi, Eigen::Vector3d(wx, wy, zone.center.z()));
                if (std::isfinite(horizon))
                    det = std::max(det, horizon + vis_offset);
                if (det < upper)
                    floor_agl = std::min(floor_agl, det - ground);
            }
            if (!in_footprint) continue;  // NaN = transparent cell
            float cell_color;
            if (!std::isfinite(floor_agl)) {
                cell_color = shadow_color;  // never visible here
            } else if (floor_agl >= risk_heatmap_agl_max_) {
                // Safe within the band of interest — seen only if you climb
                // above agl_max. Optionally transparent to maximise imagery.
                if (risk_heatmap_safe_transparent_) continue;
                cell_color = safe_color;
            } else {
                cell_color = rampColor(floor_agl / risk_heatmap_agl_max_);
            }
            const size_t idx =
                static_cast<size_t>(col) * rows + row;
            // Drape offset keeps the heatmap clear of the rendered terrain
            // mesh (which locally overshoots the bilinear DEM on slopes)
            // without reading as a floating slab.
            //
            // [HEATMAP3D] mode "heatmap3d": render the VISIBILITY CEILING as a
            // real 3-D surface instead of a terrain drape — surface altitude
            // = ground + visibility-boundary AGL ("fly BELOW this roof and no
            // zone sees you"). Terrain-occluded (never visible) and
            // safe-in-band
            // cells cap flat at agl_max, so the roof height itself is the
            // information: hugging terrain (red) = visible at any altitude,
            // high roof (green/blue) = generous low-exposure envelope. The
            // trajectory piercing the roof = becoming visible.
            double lift = risk_heatmap_offset_;
            if (hm3d) {
                lift = std::isfinite(floor_agl)
                    ? std::min(floor_agl, risk_heatmap_agl_max_)
                    : risk_heatmap_agl_max_;
                // Red (floor≈0) cells sit ON the terrain: keep a sliver of
                // clearance so the roof does not z-fight the terrain mesh.
                lift = std::max(lift, 0.08);
            }
            elevation.data[idx] =
                static_cast<float>(ground_render + lift + preview_lift);
            color.data[idx] = cell_color;
            ++painted;
        }
    }
    msg.data = {elevation, color};
    risk_heatmap_pub_->publish(msg);
    if (log_manager_) {
        log_manager_->infof(
            "[RISK-HEATMAP] %dx%d cells (res=%.3f), %zu painted, "
            "agl_max=%.2f u", rows, cols, res, painted,
            risk_heatmap_agl_max_);
    }
}

// One MarkerArray per refresh is the entire risk field. Element 0 is a
// DELETEALL (RViz honors it inside an array, ahead of the ADDs that follow —
// the same technique publishDynamicObstacles uses), so a repaint is atomic and
// the latched sample a late joiner receives is never a partial field. An empty
// zone set therefore publishes an array holding only that DELETEALL.
void PathManager::publishEffectiveRiskField()
{
    if (!risk_field_pub_) return;

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = node_->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    // The draped GridMap channel (self-clearing when mode != "heatmap").
    publishRiskHeatmap();

    const bool volume_mode = risk_mask_viz_mode_ == "volume";
    const bool heatmap_mode = risk_mask_viz_mode_ == "heatmap" ||
                              risk_mask_viz_mode_ == "heatmap3d";
    constexpr double kTwoPi = 6.28318530717958647692;
    for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
        const auto &zone = risk_zones_[zi];
        if (!(zone.reach > 0.0) || !(zone.peak > 0.0)) continue;
        const double rv = zone.reach * risk_vertical_ratio_;
        const double auto_step = std::max(
            terrain_data_.valid ? terrain_data_.resolution : 0.0,
            zone.reach / 70.0);
        const double step = std::max(
            0.05, risk_mask_viz_step_ > 0.0 ? risk_mask_viz_step_
                                             : auto_step);
        const auto header = [&]() {
            std_msgs::msg::Header h;
            h.frame_id = "map";
            h.stamp = node_->now();
            return h;
        }();
        auto groundAt = [this](double x, double y, double *ground) -> bool {
            if (!terrain_data_.valid) return false;
            const float h = terrain_data_.getElevation(x, y);
            if (!std::isfinite(h)) return false;
            *ground = std::max(0.0, static_cast<double>(h));
            return true;
        };
        auto point = [](const Eigen::Vector3d &p) {
            geometry_msgs::msg::Point gp;
            gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
            return gp;
        };
        auto riskColor = [this, &zone](double risk, double alpha_scale) {
            std_msgs::msg::ColorRGBA c;
            const double peak = std::max(1e-6, std::min(zone.peak, 1.0));
            const double t = std::clamp(risk / peak, 0.0, 1.0);
            c.r = 1.0f;
            c.g = static_cast<float>(0.38 * (1.0 - t));
            c.b = static_cast<float>(0.04 * (1.0 - t));
            c.a = static_cast<float>(
                risk_mask_viz_volume_alpha_ * alpha_scale *
                (0.35 + 0.65 * std::sqrt(t)));
            return c;
        };

        if (volume_mode) {
            // The visible lower boundary of the effective 3-D volume:
            // max(ellipsoid floor, terrain, radial-horizon ceiling). Drawing
            // this surface, rather than an opaque top lid, makes terrain
            // shadows legible and avoids the old floating-disc appearance.
            // (Mode "heatmap" carries this same surface as a draped GridMap
            // instead — see publishRiskHeatmap.)
            visualization_msgs::msg::Marker floor;
            floor.header = header;
            floor.ns = "effective_risk_floor";
            floor.id = static_cast<int>(zi);
            floor.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            floor.action = visualization_msgs::msg::Marker::ADD;
            floor.pose.orientation.w = 1.0;
            floor.scale.x = floor.scale.y = floor.scale.z = 1.0;
            floor.lifetime = rclcpp::Duration(0, 0);

            // The plotted visibility boundary follows the configured contour,
            // not necessarily the sigmoid midpoint.
            double visibility_z_offset = 0.0;
            if (risk_mask_viz_threshold_ > 1e-6 &&
                risk_mask_viz_threshold_ < 1.0 - 1e-6) {
                visibility_z_offset = risk_mask_softness_ * std::log(
                    risk_mask_viz_threshold_ /
                    (1.0 - risk_mask_viz_threshold_));
            }
            struct SurfaceVertex {
                Eigen::Vector3d p{Eigen::Vector3d::Zero()};
                double risk{0.0};
                bool valid{false};
            };
            auto surfaceAt = [&](double x, double y) {
                SurfaceVertex out;
                const double dx = x - zone.center.x();
                const double dy = y - zone.center.y();
                const double rho2 = dx * dx + dy * dy;
                if (rho2 >= zone.reach * zone.reach) return out;
                const double half_z = rv * std::sqrt(std::max(
                    0.0, 1.0 - rho2 / (zone.reach * zone.reach)));
                double lower = zone.center.z() - half_z;
                const double upper = zone.center.z() + half_z;
                double ground = 0.0;
                if (groundAt(x, y, &ground))
                    lower = std::max(lower, ground + 0.02);
                const double horizon = riskShadowCeiling(
                    zi, Eigen::Vector3d(x, y, zone.center.z()));
                if (std::isfinite(horizon))
                    lower = std::max(lower,
                                     horizon + visibility_z_offset);
                if (!(lower < upper)) return out;
                out.p = Eigen::Vector3d(x, y, lower + 0.01);
                out.risk = riskZoneValue(zi, out.p);
                out.valid = true;
                return out;
            };
            auto appendTriangle = [&](const SurfaceVertex &a,
                                      const SurfaceVertex &b,
                                      const SurfaceVertex &c) {
                if (!a.valid || !b.valid || !c.valid) return;
                // Do not bridge a cliff-sized discontinuity in the horizon
                // table; leaving a narrow gap is visually more honest.
                const double zmin = std::min({a.p.z(), b.p.z(), c.p.z()});
                const double zmax = std::max({a.p.z(), b.p.z(), c.p.z()});
                if (zmax - zmin > std::max(3.0 * step, 0.25 * rv)) return;
                for (const auto *v : {&a, &b, &c}) {
                    floor.points.push_back(point(v->p));
                    floor.colors.push_back(riskColor(v->risk, 1.0));
                }
            };
            const int half_cells = std::min(
                180, static_cast<int>(std::ceil(zone.reach / step)));
            const double mesh_step = zone.reach /
                static_cast<double>(std::max(1, half_cells));
            for (int iy = -half_cells; iy < half_cells; ++iy) {
                const double y0 = zone.center.y() + iy * mesh_step;
                const double y1 = y0 + mesh_step;
                for (int ix = -half_cells; ix < half_cells; ++ix) {
                    const double x0 = zone.center.x() + ix * mesh_step;
                    const double x1 = x0 + mesh_step;
                    const SurfaceVertex v00 = surfaceAt(x0, y0);
                    const SurfaceVertex v10 = surfaceAt(x1, y0);
                    const SurfaceVertex v01 = surfaceAt(x0, y1);
                    const SurfaceVertex v11 = surfaceAt(x1, y1);
                    appendTriangle(v00, v10, v11);
                    appendTriangle(v00, v11, v01);
                }
            }
            if (!floor.points.empty()) arr.markers.push_back(std::move(floor));
        }

        if (volume_mode || heatmap_mode) {
            // Sparse altitude contours plus meridians communicate the full
            // ellipsoid without filling it with a visually dominant lid.
            // Heatmap mode keeps only the visibility-clipped equator ring:
            // the draped map already carries the field, so the ring just
            // marks the risk boundary.
            const int n_contours = heatmap_mode ? 1 : risk_mask_viz_contours_;
            const int n_meridians = heatmap_mode ? 0 : 12;
            visualization_msgs::msg::Marker wire;
            wire.header = header;
            wire.ns = "effective_risk_volume";
            wire.id = static_cast<int>(zi);
            wire.type = visualization_msgs::msg::Marker::LINE_LIST;
            wire.action = visualization_msgs::msg::Marker::ADD;
            wire.pose.orientation.w = 1.0;
            // Readability: the rim used to be a semi-transparent red line on
            // the warm draped heatmap — red on red. Opaque, wider, and
            // backed by a darker halo underlay (published below) so the
            // boundary reads on any background.
            //
            // In heatmap mode the ring is also DRAPED onto the terrain (see
            // drapeZ): drawn at the ellipsoid equator's constant altitude it
            // floated over valleys and sank into ridges, reading as a chain
            // hanging in mid-air rather than a boundary. Membership and
            // visibility are still evaluated on the TRUE ellipsoid geometry;
            // only the drawn height follows the ground.
            wire.scale.x = std::max(0.14, 0.22 * step);
            wire.color.r = 1.0f;
            wire.color.g = 0.18f;
            wire.color.b = 0.02f;
            wire.color.a = 1.0f;
            wire.lifetime = rclcpp::Duration(0, 0);
            auto shellPointVisible = [&](const Eigen::Vector3d &p) {
                double ground = 0.0;
                if (groundAt(p.x(), p.y(), &ground) &&
                    p.z() <= ground + 0.02) return false;
                return riskVisibilityValue(zi, p) >=
                       risk_mask_viz_threshold_;
            };
            // Drawn height: terrain-following in heatmap mode (rides just
            // above the drape), true geometry otherwise (volume/wire modes
            // are showing the 3-D shell, where floating IS the point).
            auto drapeZ = [&](const Eigen::Vector3d &p) {
                if (!heatmap_mode) return p;
                double ground = 0.0;
                groundAt(p.x(), p.y(), &ground);
                Eigen::Vector3d q = p;
                q.z() = ground + risk_heatmap_offset_ + 0.05;
                return q;
            };
            auto appendSegment = [&](const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b) {
                if (!shellPointVisible(a) || !shellPointVisible(b)) return;
                wire.points.push_back(point(drapeZ(a)));
                wire.points.push_back(point(drapeZ(b)));
            };
            const int ring_samples = 192;
            for (int li = 0; li < n_contours; ++li) {
                const double f = n_contours == 1 ? 0.0 :
                    -0.70 + 1.40 * li /
                    static_cast<double>(n_contours - 1);
                const double z = zone.center.z() + f * rv;
                const double radius = zone.reach *
                    std::sqrt(std::max(0.0, 1.0 - f * f));
                for (int ai = 0; ai < ring_samples; ++ai) {
                    const double a0 = kTwoPi * ai / ring_samples;
                    const double a1 = kTwoPi * (ai + 1) / ring_samples;
                    appendSegment(
                        Eigen::Vector3d(zone.center.x() + radius * std::cos(a0),
                                        zone.center.y() + radius * std::sin(a0), z),
                        Eigen::Vector3d(zone.center.x() + radius * std::cos(a1),
                                        zone.center.y() + radius * std::sin(a1), z));
                }
            }
            constexpr int kMeridianSamples = 48;
            for (int mi = 0; mi < n_meridians; ++mi) {
                const double az = kTwoPi * mi / n_meridians;
                for (int bi = 0; bi < kMeridianSamples; ++bi) {
                    const double b0 = -0.5 * M_PI +
                        M_PI * bi / kMeridianSamples;
                    const double b1 = -0.5 * M_PI +
                        M_PI * (bi + 1) / kMeridianSamples;
                    appendSegment(
                        Eigen::Vector3d(
                            zone.center.x() + zone.reach * std::cos(b0) * std::cos(az),
                            zone.center.y() + zone.reach * std::cos(b0) * std::sin(az),
                            zone.center.z() + rv * std::sin(b0)),
                        Eigen::Vector3d(
                            zone.center.x() + zone.reach * std::cos(b1) * std::cos(az),
                            zone.center.y() + zone.reach * std::cos(b1) * std::sin(az),
                            zone.center.z() + rv * std::sin(b1)));
                }
            }
            if (!wire.points.empty()) {
                // Cartographic halo: a wider near-black underlay one step
                // closer to the drape; the bright rim renders on top of it.
                visualization_msgs::msg::Marker halo = wire;
                halo.ns = "effective_risk_volume_halo";
                halo.scale.x = 2.0 * wire.scale.x;
                halo.pose.position.z = -0.01;
                halo.color.r = 0.08f;
                halo.color.g = 0.0f;
                halo.color.b = 0.0f;
                halo.color.a = 0.85f;
                arr.markers.push_back(std::move(halo));
                arr.markers.push_back(std::move(wire));
            }
        }

        if (!volume_mode && !heatmap_mode) {
            // Explicit diagnostic cross-section. POINTS avoid the thick,
            // staircase-like CUBE_LIST slab that previously looked physical.
            visualization_msgs::msg::Marker slice;
            slice.header = header;
            slice.ns = risk_mask_viz_mode_ == "fixed_agl"
                           ? "effective_risk_fixed_agl"
                           : "effective_risk_fixed_msl";
            slice.id = static_cast<int>(zi);
            slice.type = visualization_msgs::msg::Marker::POINTS;
            slice.action = visualization_msgs::msg::Marker::ADD;
            slice.pose.orientation.w = 1.0;
            slice.scale.x = slice.scale.y = std::max(0.08, 0.70 * step);
            slice.lifetime = rclcpp::Duration(0, 0);
            const double slice_z =
                zone.center.z() + risk_mask_viz_slice_offset_;
            const int half_cells = static_cast<int>(
                std::ceil(zone.reach / step));
            for (int iy = -half_cells; iy <= half_cells; ++iy) {
                const double y = zone.center.y() + iy * step;
                for (int ix = -half_cells; ix <= half_cells; ++ix) {
                    const double x = zone.center.x() + ix * step;
                    double z = slice_z;
                    if (risk_mask_viz_mode_ == "fixed_agl") {
                        double ground = 0.0;
                        if (!groundAt(x, y, &ground)) continue;
                        z = ground + risk_mask_viz_agl_;
                    }
                    const Eigen::Vector3d p(x, y, z);
                    if (riskVisibilityValue(zi, p) <
                        risk_mask_viz_threshold_) continue;
                    const double risk = riskZoneValue(zi, p);
                    if (risk < 1e-3) continue;
                    slice.points.push_back(point(
                        Eigen::Vector3d(x, y, z + 0.02)));
                    slice.colors.push_back(riskColor(risk, 2.0));
                }
            }
            if (!slice.points.empty()) arr.markers.push_back(std::move(slice));
        }

        visualization_msgs::msg::Marker source;
        source.header = header;
        source.ns = "risk_source";
        source.id = static_cast<int>(zi);
        source.type = visualization_msgs::msg::Marker::SPHERE;
        source.action = visualization_msgs::msg::Marker::ADD;
        source.pose.position.x = zone.center.x();
        source.pose.position.y = zone.center.y();
        source.pose.position.z = zone.center.z();
        source.pose.orientation.w = 1.0;
        source.scale.x = std::max(0.5, 0.008 * zone.reach);
        source.scale.y = source.scale.x;
        source.scale.z = source.scale.x;
        source.color.r = 0.8f;
        source.color.g = 0.0f;
        source.color.b = 0.0f;
        source.color.a = 1.0f;
        source.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(std::move(source));

        // A thin vertical reference visually anchors a ground risk source.
        double source_ground = 0.0;
        if (groundAt(zone.center.x(), zone.center.y(), &source_ground) &&
            zone.center.z() > source_ground + 0.02) {
            visualization_msgs::msg::Marker mast;
            mast.header = header;
            mast.ns = "risk_source_mast";
            mast.id = static_cast<int>(zi);
            mast.type = visualization_msgs::msg::Marker::LINE_LIST;
            mast.action = visualization_msgs::msg::Marker::ADD;
            mast.pose.orientation.w = 1.0;
            mast.scale.x = std::max(0.06, 0.08 * step);
            mast.color.r = 0.8f;
            mast.color.g = 0.05f;
            mast.color.b = 0.02f;
            mast.color.a = 0.9f;
            mast.points.push_back(point(Eigen::Vector3d(
                zone.center.x(), zone.center.y(), source_ground)));
            mast.points.push_back(point(zone.center));
            mast.lifetime = rclcpp::Duration(0, 0);
            arr.markers.push_back(std::move(mast));
        }

        visualization_msgs::msg::Marker label;
        label.header = header;
        label.ns = "risk_label";
        label.id = static_cast<int>(zi);
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = zone.center.x();
        label.pose.position.y = zone.center.y();
        label.pose.position.z = zone.center.z() + rv +
                                std::max(1.0, 0.03 * zone.reach);
        label.pose.orientation.w = 1.0;
        label.scale.z = std::max(1.0, 0.02 * zone.reach);
        label.color.r = 1.0f;
        label.color.g = 0.2f;
        label.color.b = 0.2f;
        label.color.a = 1.0f;
        char label_text[96];
        if (volume_mode || heatmap_mode) {
            std::snprintf(label_text, sizeof(label_text),
                          "terrain-occluded risk zone #%zu  Rh=%.0fm Rv=%.0fm",
                          zi, 100.0 * zone.reach, 100.0 * rv);
        } else if (risk_mask_viz_mode_ == "fixed_agl") {
            std::snprintf(label_text, sizeof(label_text),
                          "risk #%zu @ terrain + %.0f m AGL",
                          zi, 100.0 * risk_mask_viz_agl_);
        } else {
            std::snprintf(label_text, sizeof(label_text),
                          "risk #%zu @ z=%.2f MSL", zi,
                          zone.center.z() + risk_mask_viz_slice_offset_);
        }
        label.text = label_text;
        label.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(std::move(label));
    }

    // Single atomic repaint. With no zones this is the lone DELETEALL, which
    // is exactly the clear that EmergencyStop / a world change / an empty
    // runtime zone set need.
    risk_field_pub_->publish(arr);
}

void PathManager::setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr &msg) {
    if (!msg) {
        log_manager_->warnf("Received null terrain GridMap");
        return;
    }

    // world=none is published as an empty GridMap. It is an explicit state
    // transition, not a malformed update: keeping the previous DEM here while
    // RViz clears it would make the displayed terrain/risk field disagree with
    // the one used for planning.
    if (msg->layers.empty() && msg->data.empty()) {
        const bool had_terrain = terrain_data_.valid;
        const uint64_t next_generation = terrain_data_.generation + 1;
        terrain_data_ = TerrainData{};
        terrain_data_.generation = next_generation;
        terrain_bbox_computed_ = false;
        sdf_built_ = false;
        // initOptimizer() may have captured a previous DEM resolution. Clear
        // that sampling pitch together with the terrain state so optimizer
        // swath/roughness checks cannot keep using a stale map cell size.
        if (poly_traj_opt_) {
            poly_traj_opt_->setTerrainHeightmap(
                [this](double x, double y) -> float {
                    return terrain_data_.valid
                             ? terrain_data_.getElevation(x, y)
                             : -std::numeric_limits<float>::infinity();
                },
                0.0);
        }

        // Every obstacle patch was tied to the old map/grid (including
        // terrain-grounded z and infinite-column bounds), so it cannot safely
        // survive an explicit world clear.
        if (sdf_manager_.numActiveObstacles() > 0 ||
            !dyn_patch_ids_.empty() || !pending_obstacles_.empty()) {
            clearDynamicObstacles();
        }
        // Invalidate the old grid itself as well as PathManager's latch.
        // Otherwise hasData() stays true and an obstacle added while no world
        // is loaded is immediately grounded at sea level, then survives the
        // next DEM load instead of being deferred and re-grounded.
        if (sdf_manager_.hasData() &&
            !sdf_manager_.initialize(sdf_voxel_size_, sdf_voxel_z_)) {
            log_manager_->warnf(
                "[TERRAIN] failed to invalidate SDF grid after world clear");
        }

        refreshEffectiveRiskZones();
        rebuildTerrainRiskMasks();  // no DEM -> the planner's ideal LOS fallback

        if (traj_.local_traj.traj_id > 0 &&
            traj_.local_traj.duration > 0.0 &&
            std::isfinite(traj_.local_traj.duration)) {
            publishTrajRisk(traj_.local_traj.traj);
            publishRiskProfile(traj_.local_traj.traj);
        }
        log_manager_->infof(
            had_terrain
                ? "[TERRAIN] cleared previous DEM (world=none)"
                : "[TERRAIN] empty world confirmed (no DEM loaded)");
        return;
    }

    // A non-empty but malformed message is not an instruction to discard a
    // valid map. Reject it and retain the last accepted DEM on both planner
    // and panel sides.
    if (!(msg->info.resolution > 0.0) || msg->data.empty()) {
        log_manager_->warnf("Invalid non-empty terrain GridMap");
        return;
    }
    if (sdf_voxel_size_auto_ && msg->info.resolution > 1e-6) {
        sdf_voxel_size_ = msg->info.resolution;
    } else if (sdf_voxel_size_ <= 0.0 && msg->info.resolution > 1e-6) {
        sdf_voxel_size_ = msg->info.resolution;
        sdf_voxel_size_auto_ = true;    // keep tracking the DEM cell on re-crop
        log_manager_->infof("[SDF] xy voxel auto-set to DEM cell: %.3f units",
                            sdf_voxel_size_);
    }

    // Find elevation layer
    int elev_idx = -1;
    for (size_t i = 0; i < msg->layers.size(); ++i) {
        if (msg->layers[i] == "elevation") {
            elev_idx = static_cast<int>(i);
            break;
        }
    }
    if (elev_idx < 0) {
        log_manager_->warnf("Elevation layer not found in terrain GridMap");
        return;
    }

    if (static_cast<size_t>(elev_idx) >= msg->data.size()) {
        log_manager_->warnf("Terrain GridMap elevation payload is missing");
        return;
    }
    const auto& elev_data = msg->data[elev_idx];
    if (elev_data.layout.dim.size() < 2) {
        log_manager_->warnf("Invalid terrain GridMap data layout");
        return;
    }
    const size_t cols = elev_data.layout.dim[0].size;
    const size_t rows = elev_data.layout.dim[1].size;
    if (cols == 0 || rows == 0 ||
        cols > std::numeric_limits<size_t>::max() / rows ||
        elev_data.data.size() < cols * rows) {
        log_manager_->warnf("Invalid terrain GridMap dimensions/data size");
        return;
    }

    // Geometry change (new world / corridor re-crop) invalidates everything
    // derived from the OLD map: the boxes-only SDF grid and the cached
    // terrain bbox were one-shot latches sized to the first map ever seen,
    // so a corridor loaded after a nationwide map (or vice versa) kept
    // planning against stale bounds. Reset the latches; the blocks below
    // rebuild them from this message.
    {
        const double new_origin_x = msg->info.pose.position.x - msg->info.length_x / 2.0;
        const double new_origin_y = msg->info.pose.position.y - msg->info.length_y / 2.0;
        const bool geometry_changed = terrain_data_.valid &&
            (std::abs(terrain_data_.resolution - msg->info.resolution) > 1e-9 ||
             std::abs(terrain_data_.length_x - msg->info.length_x) > 1e-6 ||
             std::abs(terrain_data_.length_y - msg->info.length_y) > 1e-6 ||
             std::abs(terrain_data_.origin_x - new_origin_x) > 1e-6 ||
             std::abs(terrain_data_.origin_y - new_origin_y) > 1e-6);
        if (geometry_changed) {
            log_manager_->warnf(
                "[TERRAIN] map geometry changed (res %.4f->%.4f, origin (%.1f,%.1f)->(%.1f,%.1f)) "
                "— rebuilding SDF grid and terrain bbox",
                terrain_data_.resolution, msg->info.resolution,
                terrain_data_.origin_x, terrain_data_.origin_y,
                new_origin_x, new_origin_y);
            sdf_built_ = false;
            terrain_bbox_computed_ = false;
            if (sdf_voxel_size_auto_) {
                sdf_voxel_size_ = msg->info.resolution;   // re-track DEM cell
            }
            // Clear ALL patches: every patch was placed on the OLD grid/DEM —
            // they baked groundedCenter() on the old ground, so they cannot
            // survive a geometry change.
            if (sdf_manager_.numActiveObstacles() > 0 ||
                !dyn_patch_ids_.empty()) {
                clearDynamicObstacles();
            }
        }
    }

    terrain_data_.cols = elev_data.layout.dim[0].size;
    terrain_data_.rows = elev_data.layout.dim[1].size;
    terrain_data_.resolution = msg->info.resolution;
    terrain_data_.length_x = msg->info.length_x;
    terrain_data_.length_y = msg->info.length_y;
    terrain_data_.origin_x = msg->info.pose.position.x - msg->info.length_x / 2.0;
    terrain_data_.origin_y = msg->info.pose.position.y - msg->info.length_y / 2.0;
    terrain_data_.elevation = elev_data.data;
    terrain_data_.valid = true;
    // Invalidate the ELEV-MEMO: a same-geometry re-crop reuses the buffer, so
    // the memo (keyed on generation) must see a new value or it serves stale
    // samples from the previous crop.
    ++terrain_data_.generation;

    // The optimizer is commonly initialized before the first terrain message.
    // Refresh both its callback and, crucially, its DEM-cell sampling pitch on
    // every accepted map (including same-geometry data refreshes and
    // full-map/corridor resolution changes).
    if (poly_traj_opt_) {
        poly_traj_opt_->setTerrainHeightmap(
            [this](double x, double y) -> float {
                return terrain_data_.valid
                         ? terrain_data_.getElevation(x, y)
                         : -std::numeric_limits<float>::infinity();
            },
            terrain_data_.resolution);
    }

    // Return freed arenas to the OS after a map swap. Alternating small/large
    // corridor crops re-allocate every big grid (elevation, FM2 fields) at a
    // different size each epoch; glibc keeps the fragmented arenas and
    // manager RSS ratcheted ~+100 MB per revisit (measured; same-size
    // re-crops stay flat). One trim per terrain message is microseconds.
    malloc_trim(0);

    log_manager_->infof("Terrain data loaded: %dx%d, resolution=%.3f, origin=(%.2f,%.2f), center=(%.2f,%.2f)",
        terrain_data_.cols, terrain_data_.rows, terrain_data_.resolution,
        terrain_data_.origin_x, terrain_data_.origin_y,
        msg->info.pose.position.x, msg->info.pose.position.y);

    // ALIGNMENT SELF-CHECK: terrainToWorld uses the cell-CENTRE convention,
    // which provably matches the grid_map_rviz_plugin mesh (wx = L-(row+.5)res).
    // Sampling the peak cell's world position back through getElevation must
    // therefore return the peak value exactly (bilinear at an exact node).
    // Before the half-cell fix this read the average of the 4 shifted
    // neighbours instead (delta up to the full cell-to-cell relief, ~114 m
    // horizontal displacement) — a nonzero delta here means the sampled
    // surface is offset from the rendered terrain.
    {
        int pc = -1, pr = -1;
        float peak = -std::numeric_limits<float>::infinity();
        for (int c = 0; c < terrain_data_.cols; ++c) {
            for (int r = 0; r < terrain_data_.rows; ++r) {
                const float e = terrain_data_.elevation[c * terrain_data_.rows + r];
                if (std::isfinite(e) && e > peak) { peak = e; pc = c; pr = r; }
            }
        }
        if (pc >= 0) {
            const Eigen::Vector3d w = terrain_data_.terrainToWorld(pc, pr, peak);
            const float back = terrain_data_.getElevation(w.x(), w.y());
            log_manager_->infof(
                "[TERRAIN-ALIGN] peak cell (c=%d,r=%d) h=%.4f -> world (%.2f,%.2f) "
                "-> getElevation=%.4f delta=%.4f %s",
                pc, pr, peak, w.x(), w.y(), back, back - peak,
                std::abs(back - peak) < 1e-3 ? "(ALIGNED)" : "(MISALIGNED!)");
        }
    }

    // Eagerly build the (boxes-only, empty-static) SDF grid as soon as
    // terrain arrives, so the dynamic-obstacle layer can accept clicks before
    // any mission runs — patches need the grid to clip against. This replaced
    // the old eager cache LOAD: the build is now instant (the static layer
    // allocates nothing; terrain lives in the 2.5D heightmap).
    if (!sdf_built_) {
        Eigen::Vector3d lo, hi;
        if (computeTerrainBBox(&lo, &hi)) {
            if (buildSDFForBounds(lo, hi)) {
                sdf_built_ = true;
                flushPendingObstacles();
            }
        }
    }

    // DEM coordinates and elevations are now authoritative. Re-ground the
    // AGL zone sources onto the fresh heightmap, then rebuild the radial
    // viewshed and replace the ideal circular RViz fallback.
    refreshEffectiveRiskZones();
    rebuildTerrainRiskMasks();

    // The displayed risk channels are evaluated against the terrain mask, so
    // a DEM replacement invalidates them even when the trajectory itself did
    // not change. Re-evaluate those channels immediately for the current
    // trajectory.
    if (traj_.local_traj.traj_id > 0 &&
        traj_.local_traj.duration > 0.0 &&
        std::isfinite(traj_.local_traj.duration)) {
        publishTrajRisk(traj_.local_traj.traj);
        publishRiskProfile(traj_.local_traj.traj);
    }
}

const PathManager::ObstacleMeshInfo& PathManager::meshFor(const std::string& model) const
{
    auto it = mesh_catalog_.find(model.empty() ? std::string("building") : model);
    if (it == mesh_catalog_.end()) it = mesh_catalog_.find("building");
    return it->second;
}

Eigen::Vector3d PathManager::groundedCenter(const Eigen::Vector3d& center,
                                            double half_height) const
{
    if (!obstacle_ground_snap_) return center;
    double base = 0.0;  // sea level
    if (terrain_data_.valid) {
        const float h = terrain_data_.getElevation(center.x(), center.y());
        if (std::isfinite(h) && h > 0.0f) base = static_cast<double>(h);
    }
    Eigen::Vector3d out = center;
    out.z() = base + half_height;
    return out;
}

int PathManager::addDynamicSphere(const Eigen::Vector3d& center, double radius,
                                  const std::string& model)
{
    if (!sdf_manager_.hasData()) {
        // Defer: the SDF (ESDF) is built lazily on the first plan. Without a
        // cache it is not ready yet, so queue and add once it exists.
        pending_obstacles_.push_back({false, center,
            Eigen::Vector3d(radius, radius, radius), radius, model});
        log_manager_->infof("addDynamicSphere: SDF not ready, deferred "
                            "(center=%.2f,%.2f,%.2f r=%.2f)",
                            center.x(), center.y(), center.z(), radius);
        return -2;  // deferred (not an error)
    }
    const Eigen::Vector3d gcenter = groundedCenter(center, radius);
    path_planner::sdf::PrimitiveSpec spec;
    spec.kind = path_planner::sdf::PrimitiveKind::kSphere;
    spec.center = gcenter;
    const double d = 2.0 * radius;
    spec.size = Eigen::Vector3d(d, d, d);

    int id = sdf_manager_.addObstacle(spec);
    if (id < 0) {
        log_manager_->warnf("addDynamicSphere: addObstacle failed "
                            "(center=%.2f,%.2f,%.2f r=%.2f)",
                            center.x(), center.y(), center.z(), radius);
        return -1;
    }
    dyn_patch_ids_.push_back(id);
    dyn_patch_centers_.push_back(gcenter);
    dyn_patch_sizes_.push_back(Eigen::Vector3d(d, d, d));  // sphere stored as (2r,2r,2r)
    dyn_patch_is_box_.push_back(0);
    dyn_patch_models_.push_back(model);
    dyn_patch_yaws_.push_back(
        std::uniform_real_distribution<double>(0.0, 6.283185307179586)(yaw_rng_));
    log_manager_->infof("Dynamic sphere added: id=%d center=(%.2f,%.2f,%.2f) r=%.2f "
                        "(request z=%.2f), total=%zu",
                        id, gcenter.x(), gcenter.y(), gcenter.z(), radius,
                        center.z(), sdf_manager_.numActiveObstacles());
    publishDynamicObstacles();
    return id;
}

int PathManager::addDynamicBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                               const std::string& model)
{
    if (!sdf_manager_.hasData()) {
        // Defer until the SDF is built (see addDynamicSphere).
        pending_obstacles_.push_back({true, center, size, 0.0, model});
        log_manager_->infof("addDynamicBox: SDF not ready, deferred "
                            "(center=%.2f,%.2f,%.2f)",
                            center.x(), center.y(), center.z());
        return -2;  // deferred (not an error)
    }
    // One heading per spawn, shared by BOTH the collision primitive and the
    // RViz model marker. It used to be visual-only (collision stayed an
    // axis-aligned box), so a rotated hull stuck out of its own collision
    // volume and trajectories legally clipped the bow/stern.
    const double yaw =
        std::uniform_real_distribution<double>(0.0, 6.283185307179586)(yaw_rng_);
    const Eigen::Vector3d gcenter = groundedCenter(center, 0.5 * size.z());
    path_planner::sdf::PrimitiveSpec spec;
    spec.kind = path_planner::sdf::PrimitiveKind::kCube;
    spec.center = gcenter;
    spec.size = size;  // full extents (sx, sy, sz)
    spec.yaw = yaw;

    int id = sdf_manager_.addObstacle(spec);
    if (id < 0) {
        log_manager_->warnf("addDynamicBox: addObstacle failed "
                            "(center=%.2f,%.2f,%.2f size=%.2f,%.2f,%.2f)",
                            gcenter.x(), gcenter.y(), gcenter.z(),
                            size.x(), size.y(), size.z());
        return -1;
    }
    dyn_patch_ids_.push_back(id);
    dyn_patch_centers_.push_back(gcenter);
    dyn_patch_sizes_.push_back(size);
    dyn_patch_is_box_.push_back(1);
    dyn_patch_models_.push_back(model);
    dyn_patch_yaws_.push_back(yaw);
    log_manager_->infof("Dynamic box added: id=%d center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) "
                        "yaw=%.2f (request z=%.2f), total=%zu",
                        id, gcenter.x(), gcenter.y(), gcenter.z(),
                        size.x(), size.y(), size.z(), yaw,
                        center.z(), sdf_manager_.numActiveObstacles());
    publishDynamicObstacles();
    return id;
}

// [ENV-CHANGE] see the header. Same occupancy predicate the front end plans
// against, so "the route was clear when planned" and "the remainder is clear
// now" are the same question asked twice, not two different standards.
bool PathManager::revalidateStoredTrajectory(double t_from,
                                             Eigen::Vector3d *hit,
                                             EnvChangeReason reason)
{
    auto &lt = traj_.local_traj;
    if (!(lt.duration > 0.0) || lt.traj.getPieceNum() == 0) return true;
    const double t0 = std::clamp(t_from, 0.0, lt.duration);
    if (lt.duration - t0 <= 1e-9) return true;

    // Sample the REMAINDER to a spatial pitch rather than a fixed count: the
    // question is how far apart the samples are in metres, and that must not
    // depend on how long the flight happens to be.
    std::vector<Eigen::Vector3d> pts;
    {
        double len = 0.0;
        Eigen::Vector3d prev = lt.traj.getPos(t0);
        for (int k = 1; k <= 512; ++k) {
            const Eigen::Vector3d p =
                lt.traj.getPos(t0 + (lt.duration - t0) * k / 512.0);
            len += (p - prev).norm();
            prev = p;
        }
        const double pitch = 0.05;          // 5 m at 1 unit = 100 m
        const int n = std::clamp(
            static_cast<int>(std::ceil(len / pitch)), 64, 200000);
        pts.reserve(static_cast<size_t>(n) + 1);
        for (int k = 0; k <= n; ++k)
            pts.push_back(
                lt.traj.getPos(t0 + (lt.duration - t0) * k / n));
    }

    Eigen::Vector3d bad;
    if (searcher_.polylineClear(pts, &bad)) return true;

    if (hit) *hit = bad;
    log_manager_->errorf(
        "[ENV-CHANGE] the remaining trajectory (t=%.2f..%.2f s) is blocked at "
        "(%.2f, %.2f, %.2f) under the current obstacle set — withdrawing it",
        t0, lt.duration, bad.x(), bad.y(), bad.z());
    // The hook runs FIRST, while traj_id still names the trajectory being
    // withdrawn: it is what puts the ABORT on the wire, and clearing the slot
    // first would leave the FSM announcing an id it can no longer read.
    // Detection lives here; what to say about it belongs to the node that
    // owns the publishers.
    if (env_change_hook_)
        env_change_hook_(reason, bad, t0);
    // The same two fields SegmentChainPlanner::invalidateStoredTrajectory
    // zeroes, and the same fields ReplanFSM gates execution on.
    lt.duration = 0.0;
    lt.start_time = 0.0;
    clearTrajectoryViz();
    return false;
}

void PathManager::clearDynamicObstacles()
{
    sdf_manager_.clearObstacles();
    dyn_patch_ids_.clear();
    dyn_patch_centers_.clear();
    dyn_patch_sizes_.clear();
    dyn_patch_is_box_.clear();
    dyn_patch_models_.clear();
    dyn_patch_yaws_.clear();
    pending_obstacles_.clear();
    log_manager_->infof("Dynamic obstacles cleared");
    publishDynamicObstacles();
}

// Add obstacles that were requested before the SDF existed. Called right after
// the SDF is built/loaded, so they make it into the very first plan.
// [ENV-CHANGE] Every path by which the world under a flight can change has to
// reach revalidateStoredTrajectory, not just the obstacle topic. This one is
// the deferred queue: obstacles that arrived before the SDF existed are
// installed HERE, so this is where they first become able to block anything.
uint64_t PathManager::riskZoneFingerprint() const
{
    // Order-sensitive on purpose: a reordered set is a different set to the
    // searcher, which indexes policy by position. Bit patterns rather than
    // rounded values, so a change too small to print is still a change.
    uint64_t h = 1469598103934665603ull;   // FNV-1a
    const auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
    const auto mixd = [&mix](double d) {
        uint64_t bits;
        static_assert(sizeof(bits) == sizeof(d), "double must be 64-bit");
        std::memcpy(&bits, &d, sizeof(bits));
        mix(bits);
    };
    mix(risk_zones_.size());
    for (const auto &z : risk_zones_) {
        mixd(z.center.x()); mixd(z.center.y()); mixd(z.center.z());
        mixd(z.reach); mixd(z.peak);
    }
    return h;
}

void PathManager::invalidateStoredTrajectoryForEnvChange()
{
    if (env_change_hook_ && traj_.local_traj.duration > 0.0)
        env_change_hook_(EnvChangeReason::POLICY_UNEVALUATED,
                         Eigen::Vector3d::Zero(), 0.0);
    traj_.local_traj.duration = 0.0;
    traj_.local_traj.start_time = 0.0;
    clearTrajectoryViz();
}

void PathManager::revalidateAfterEnvChange(const char *what,
                                           EnvChangeReason reason)
{
    auto &lt = traj_.local_traj;
    if (!(lt.duration > 0.0)) return;
    const double t_cur = (lt.start_time > 0.0)
                             ? rclcpp::Clock(RCL_ROS_TIME).now().seconds() -
                                   lt.start_time
                             : 0.0;
    Eigen::Vector3d hit;
    if (!revalidateStoredTrajectory(t_cur, &hit, reason)) {
        log_manager_->errorf(
            "[ENV-CHANGE] %s blocked the flight in progress at "
            "(%.2f, %.2f, %.2f) — the stored trajectory is invalidated. NOTE: "
            "this stops THIS node from continuing to treat it as flyable; it "
            "does not recall a trajectory already published to a consumer "
            "(see docs — the cancel layer is an open decision)",
            what, hit.x(), hit.y(), hit.z());
    }
}

void PathManager::flushPendingObstacles()
{
    if (pending_obstacles_.empty() || !sdf_manager_.hasData()) return;
    std::vector<PendingObstacle> pend;
    pend.swap(pending_obstacles_);  // addDynamic* see an empty queue -> no re-defer
    for (const auto& p : pend) {
        if (p.is_box) addDynamicBox(p.center, p.size, p.model);
        else          addDynamicSphere(p.center, p.radius, p.model);
    }
    log_manager_->infof("Flushed %zu deferred dynamic obstacle(s) after SDF ready",
                        pend.size());
    revalidateAfterEnvChange("a deferred obstacle batch",
                             EnvChangeReason::OBSTACLE_BLOCKED);
}

bool PathManager::captureLegPolicySnapshot(size_t leg, uint64_t search_serial,
                                           LegPolicySnapshot *out) const
{
    if (!out) return false;
    // The plan-wide rule ("exactly one search this epoch") is deliberately
    // NOT reused here — it exists to refuse a plan-wide answer for a
    // multi-leg mission, which is the very case this function serves. What
    // must still hold is everything that makes the searcher's CURRENT
    // buffers describe THIS leg:
    //   - the 3-pass actually ran, when there are zones to have a policy
    //     about (pass 0 means the override buffer may hold a previous
    //     search's values — reading it would be fiction, same as above)
    //   - the search ran on the current zone/terrain data
    //   - the policy buffers are sized for the zones they describe
    const int pass = searcher_.zoneAvoidPass();
    const auto &nb = searcher_.zoneNoBarrier();
    const auto &so = searcher_.zoneSoftOverride();
    const bool ok =
        (risk_zones_.empty() || pass != 0) &&
        zone_policy_epoch_generation_ == zone_policy_generation_ &&
        (risk_zones_.empty() ||
         // EXACTLY one entry per zone. ">= zone count" accepted a buffer left
         // over from a search that saw MORE zones; the extra entries are
         // never read, but their presence means the buffer was not rebuilt
         // for this zone set and the entries that ARE read may be stale.
         (nb.size() == risk_zones_.size() && so.size() == risk_zones_.size()));
    if (!ok) {
        if (log_manager_)
            log_manager_->warnf(
                "[LEG-POLICY] leg %zu NOT capturable (pass %d, %zu zones, "
                "no-barrier %zu, soft-override %zu, search generation %lu vs "
                "data %lu) — no snapshot rather than a wrong one",
                leg, pass, risk_zones_.size(), nb.size(), so.size(),
                static_cast<unsigned long>(zone_policy_epoch_generation_),
                static_cast<unsigned long>(zone_policy_generation_));
        return false;
    }
    out->leg = leg;
    out->search_serial = search_serial;
    out->policy.generation = zone_policy_generation_;
    out->policy.epoch = zone_policy_epoch_;
    // Valid PER LEG: this leg's search really did produce this policy.
    out->policy.valid = true;
    out->policy.zones.clear();
    out->policy.zones.reserve(risk_zones_.size());
    for (size_t i = 0; i < risk_zones_.size(); ++i) {
        ZonePolicySnapshot::Entry e;
        e.zone = risk_zones_[i];
        const bool endpoint = i < nb.size() && nb[i];
        const bool soft_override = i < so.size() && so[i];
        e.disposition = endpoint ? ZoneDisposition::SOFT_ENDPOINT
                        : pass == 2
                            ? ZoneDisposition::SOFT_FALLBACK
                            : soft_override ? ZoneDisposition::SOFT_UNAVOIDABLE
                                            : ZoneDisposition::HARD_AVOID;
        out->policy.zones.push_back(e);
    }
    return true;
}

PathManager::ZonePolicySnapshot PathManager::zonePolicySnapshot() const
{
    ZonePolicySnapshot snap;
    snap.generation = zone_policy_generation_;
    snap.epoch = zone_policy_epoch_;
    const int pass = searcher_.zoneAvoidPass();
    // INVALID, fail-closed:
    //  - zones exist but the 3-pass never ran (pass 0: policy off or a
    //    different front end; zone_soft_override_ may hold a PREVIOUS
    //    search's values — reading it would be fiction)
    //  - no search has run since the last zone/terrain change (the
    //    searcher still holds the previous zone binding)
    // Multi-leg searches rewrite the policy buffers once per leg, so a
    // snapshot after such an epoch would publish the LAST leg's policy as
    // plan-wide — fail-closed instead (review find).
    snap.valid = (risk_zones_.empty() ||
                  (pass != 0 && zone_policy_epoch_searches_ == 1)) &&
                 zone_policy_epoch_generation_ == zone_policy_generation_;
    if (!snap.valid && log_manager_) {
        log_manager_->warnf(
            "[ZONE-POLICY] snapshot INVALID (pass %d, %zu zones, %lu leg "
            "searches this epoch, search generation %lu vs data %lu) — "
            "contract 2 requires ONE single-goal 3-pass on current data",
            pass, risk_zones_.size(),
            static_cast<unsigned long>(zone_policy_epoch_searches_),
            static_cast<unsigned long>(zone_policy_epoch_generation_),
            static_cast<unsigned long>(zone_policy_generation_));
    }
    const auto &nb = searcher_.zoneNoBarrier();
    const auto &so = searcher_.zoneSoftOverride();
    snap.zones.reserve(risk_zones_.size());
    for (size_t i = 0; i < risk_zones_.size(); ++i) {
        ZonePolicySnapshot::Entry e;
        e.zone = risk_zones_[i];
        const bool endpoint = i < nb.size() && nb[i];
        const bool soft_override = i < so.size() && so[i];
        e.disposition = endpoint ? ZoneDisposition::SOFT_ENDPOINT
                        : pass == 2
                            ? ZoneDisposition::SOFT_FALLBACK
                            : soft_override ? ZoneDisposition::SOFT_UNAVOIDABLE
                                            : ZoneDisposition::HARD_AVOID;
        snap.zones.push_back(e);
    }
    return snap;
}

// Staleness shared by contact and exposure: generation, epoch, index
// range, and shape identity (the snapshot's copy must still describe the
// live zone at this index, or the index now points at a DIFFERENT zone).
bool PathManager::zoneSnapshotCurrent(const ZonePolicySnapshot &snap,
                                      size_t idx) const
{
    if (snap.generation != zone_policy_generation_) return false;
    if (snap.epoch != zone_policy_epoch_) return false;
    if (idx >= snap.zones.size() || idx >= risk_zones_.size()) return false;
    const RiskZone &live = risk_zones_[idx];
    const RiskZone &snapz = snap.zones[idx].zone;
    return (live.center - snapz.center).norm() <= 1e-9 &&
           std::abs(live.reach - snapz.reach) <= 1e-9 &&
           std::abs(live.peak - snapz.peak) <= 1e-9;
}

PathManager::ZoneContactResult PathManager::zoneContact(
    const ZonePolicySnapshot &snap, size_t idx,
    const Eigen::Vector3d &p) const
{
    if (!snap.valid) return ZoneContactResult::INVALID;
    if (!zoneSnapshotCurrent(snap, idx)) return ZoneContactResult::STALE;
    // The searcher's OWN hard-exclusion primitive (1.05x ellipsoid +
    // visibility>0.35 standoff) — the exact volume the hard passes keep
    // ROUTES out of, so a candidate is judged by the same standard a
    // route is. The nominal 1.0x/0.5 volume is NOT enough here (review
    // find): visibility in (0.35, 0.5] still carries positive risk.
    return searcher_.zoneHardVolumeContains(idx, p)
               ? ZoneContactResult::CONTACT
               : ZoneContactResult::CLEAR;
}

bool PathManager::zoneContactAuthored(const ZonePolicySnapshot &snap,
                                      size_t idx,
                                      const Eigen::Vector3d &p) const
{
    if (!snap.valid || !zoneSnapshotCurrent(snap, idx)) return false;
    // Same primitive, same visibility floor, ONE difference: no 1.05
    // inflation. What the mission authored, nothing added.
    return searcher_.zoneAuthoredVolumeContains(idx, p);
}

bool PathManager::zoneExposureRaw(const ZonePolicySnapshot &snap,
                                  size_t idx, const Eigen::Vector3d &p,
                                  double *exposure) const
{
    if (!snap.valid || !zoneSnapshotCurrent(snap, idx)) return false;
    if (exposure) *exposure = getEffectiveRisk(idx, p);
    return true;
}

void PathManager::setRiskZonesRuntime(const std::vector<RiskZone>& zones)
{
    // Atomically replace the active zone list. The next planGlobalTraj
    // call will re-bind A* and the optimizer with the new set via the
    // existing setup paths (see planGlobalTraj where searcher_.setRiskZones
    // and poly_traj_opt_->setRiskZones are called).
    risk_zones_raw_ = zones;
    refreshEffectiveRiskZones();
    rebuildTerrainRiskMasks();
    // Avoid leaving the altitude panel and colored trajectory latched to the
    // previous zone set while waiting for another plan request. Empty zones
    // intentionally flow through these publishers as clear/delete messages.
    if (traj_.local_traj.traj_id > 0 &&
        traj_.local_traj.duration > 0.0 &&
        std::isfinite(traj_.local_traj.duration)) {
        publishTrajRisk(traj_.local_traj.traj);
        publishRiskProfile(traj_.local_traj.traj);
    }
    if (log_manager_) {
        log_manager_->infof(
            "[risk_zones] runtime update: %zu zones now active",
            risk_zones_.size());
        for (size_t i = 0; i < risk_zones_.size(); ++i) {
            const auto& tz = risk_zones_[i];
            log_manager_->infof(
                "  zone[%zu] center=(%.1f,%.1f,%.1f) reach=%.1f peak=%.2f",
                i, tz.center.x(), tz.center.y(), tz.center.z(),
                tz.reach, tz.peak);
        }
    }
}

void PathManager::publishDynamicObstacles()
{
    if (!dyn_obstacle_pub_) return;
    visualization_msgs::msg::MarkerArray arr;

    // Single DELETEALL marker first so removed patches disappear in RViz.
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = "dynamic_obstacles";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear_marker);

    for (size_t i = 0; i < dyn_patch_centers_.size(); ++i) {
        // Per-obstacle visual model from the catalog; collision (SDF) is separate.
        const ObstacleMeshInfo& mi = meshFor(dyn_patch_models_[i]);
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "dynamic_obstacles";
        m.id = dyn_patch_ids_[i];
        // color all-zero => RViz uses the mesh's own embedded materials/textures.
        m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        m.mesh_resource = mi.resource;
        m.mesh_use_embedded_materials = true;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.color.r = m.color.g = m.color.b = m.color.a = 0.0f;  // use mesh textures

        const Eigen::Vector3d& c = dyn_patch_centers_[i];
        double foot_x, foot_y, height, base_z;
        if (dyn_patch_is_box_[i]) {
            // Box: mesh fills the collision box exactly (collision == visual bbox).
            foot_x = dyn_patch_sizes_[i].x();
            foot_y = dyn_patch_sizes_[i].y();
            height = dyn_patch_sizes_[i].z();
            base_z = c.z() - 0.5 * height;
        } else {
            // Sphere: fixed-height building from the ball's bottom.
            const double radius = 0.5 * dyn_patch_sizes_[i].x();  // stored as (2r,2r,2r)
            foot_x = foot_y = 2.0 * radius;
            height = obstacle_mesh_height_;
            base_z = c.z() - radius;
        }

        // Visual-only magnification: grows the mesh about the BASE (xy about
        // the center, z upward from the bottom face) so magnified obstacles
        // keep resting on the surface — center-anchored scaling pushed half
        // of every magnified hull underground/underwater. Per-model via
        // obstacle_viz_scale_<model>, global via obstacle_viz_scale.
        // Collision, companion geometry marker and panel overlay untouched.
        const double vs = std::max(1.0, mi.viz_scale);
        m.pose.position.x = c.x();
        m.pose.position.y = c.y();
        m.pose.position.z = base_z;
        // Per-spawn yaw about Z, shared with the SDF collision primitive
        // (PrimitiveSpec.yaw) — what you see is what the planner avoids.
        const double yaw = (i < dyn_patch_yaws_.size()) ? dyn_patch_yaws_[i] : 0.0;
        m.pose.orientation.z = std::sin(0.5 * yaw);
        m.pose.orientation.w = std::cos(0.5 * yaw);
        m.scale.x = vs * foot_x / mi.native_size.x();
        m.scale.y = vs * foot_y / mi.native_size.y();
        m.scale.z = vs * height / mi.native_size.z();
        arr.markers.push_back(m);

        // Companion GEOMETRY marker: the exact collision primitive (center /
        // size / yaw), fully transparent so RViz shows only the mesh above.
        // Panels cannot recover the collision box from a MESH_RESOURCE marker
        // (its scale is relative to unknown native mesh dims), and without
        // this the altitude profile cannot explain box-avoidance climbs —
        // "why is it climbing over open water" turned out to be invisible
        // ships more than once.
        visualization_msgs::msg::Marker g;
        g.header = m.header;
        g.ns = "dynamic_obstacles_geom";
        g.id = dyn_patch_ids_[i];
        g.type = visualization_msgs::msg::Marker::CUBE;
        g.action = visualization_msgs::msg::Marker::ADD;
        g.pose.position.x = c.x();
        g.pose.position.y = c.y();
        g.pose.position.z = c.z();
        g.pose.orientation = m.pose.orientation;
        g.scale.x = dyn_patch_is_box_[i] ? dyn_patch_sizes_[i].x() : foot_x;
        g.scale.y = dyn_patch_is_box_[i] ? dyn_patch_sizes_[i].y() : foot_y;
        g.scale.z = dyn_patch_is_box_[i] ? dyn_patch_sizes_[i].z()
                                         : dyn_patch_sizes_[i].x();
        g.color.a = 0.0f;  // invisible in RViz; geometry carrier only
        arr.markers.push_back(g);
    }
    dyn_obstacle_pub_->publish(arr);
}

// Voxelize terrain + geometry obstacles into an occupancy grid and build ESDF.
// Risk zones are NOT included: they are handled as soft cost elsewhere.
bool PathManager::buildSDFForBounds(const Eigen::Vector3d &lo,
                                    const Eigen::Vector3d &hi)
{
    const double res = sdf_voxel_size_;
    const double res_z = sdf_voxel_z_;
    Eigen::Vector3d ext = hi - lo;
    if ((ext.array() <= 0.0).any()) {
        log_manager_->warnf("buildSDFForBounds: invalid bounds");
        return false;
    }
    int nx = std::max(8, (int)std::ceil(ext.x() / res));
    int ny = std::max(8, (int)std::ceil(ext.y() / res));
    int nz = std::max(8, (int)std::ceil(ext.z() / res_z));

    // NOTE: no occupancy grid is materialized. Terrain and static geometry
    // are never voxelised here (see below), so the grid was PROVABLY all-zero
    // — yet vector-init + the emptiness scan committed nx*ny*nz bytes
    // (~5.8 GB on a 30 m corridor) just for SDFManager to notice it was
    // empty and discard it. buildEmpty() takes that exact path directly.

    // Terrain is NOT voxelised into the SDF any more. It is handled as a 2.5D
    // heightmap (terrain_data_.getElevation, EXACT z) directly by the front end
    // (checkOccupancy_esdf) and the optimizer (heightmap terrain term). Baking
    // the DEM into 3D voxels quantised terrain z to sdf_voxel_z (~10 m) so it
    // under-saw hills, AND double-counted terrain against the heightmap term
    // (SDF-terrain + heightmap both firing -> line-search blowup on tall maps).
    // So the static SDF now holds ONLY geometry obstacles (added as patches
    // after the build): small, box-only, and terrain-precise everywhere.
    if (!terrain_data_.valid) {
        log_manager_->infof("SDF: terrain_data_ INVALID (heightmap terrain unavailable)");
    }

    // NOTE: ground and virtual ceiling are NOT voxelised here. Folding them
    // into the SDF would drag the clearance band above/below the actual
    // plane, so the trajectory would be pushed off a `-0.1` floor by up to
    // `obstacle_clearance` metres. Instead they are enforced as hard
    // half-space constraints inside the optimizer (sdfGradCostP), which
    // applies a unit upward/downward gradient only when the query point
    // crosses the plane — no clearance band, no lateral contamination.

    const Eigen::Vector3d current_voxel = sdf_manager_.voxelSizes();
    if (!sdf_manager_.isInitialized() ||
        std::abs(current_voxel.x() - res) > 1e-12 ||
        std::abs(current_voxel.y() - res) > 1e-12 ||
        std::abs(current_voxel.z() - res_z) > 1e-12) {
        if (!sdf_manager_.initialize(res, res_z)) {
            log_manager_->warnf(
                "buildSDFForBounds: SDF initialization failed");
            return false;
        }
    }
    bool ok = sdf_manager_.buildEmpty(nx, ny, nz, lo);
    if (ok) {
        log_manager_->infof("SDF built: shape=(%d,%d,%d) voxel=(%.2f,%.2f,%.2f) blocks=%zu",
            nx, ny, nz, res, res, res_z, sdf_manager_.numAllocatedBlocks());
    }
    return ok;
}

bool PathManager::computeTerrainBBox(Eigen::Vector3d* lo, Eigen::Vector3d* hi)
{
    if (!terrain_data_.valid) return false;
    if (terrain_bbox_computed_) {
        *lo = terrain_bbox_lo_;
        *hi = terrain_bbox_hi_;
        return true;
    }

    // XY: walk the 4 corners of the terrain grid through terrainToWorld to
    //     get the correct world-frame bounds after the X-mirror / rotation.
    const int cols = terrain_data_.cols;
    const int rows = terrain_data_.rows;
    if (cols <= 0 || rows <= 0) return false;

    std::vector<Eigen::Vector3d> corners = {
        terrain_data_.terrainToWorld(0,        0,        0.0f),
        terrain_data_.terrainToWorld(cols - 1, 0,        0.0f),
        terrain_data_.terrainToWorld(0,        rows - 1, 0.0f),
        terrain_data_.terrainToWorld(cols - 1, rows - 1, 0.0f),
    };
    double min_x = corners[0].x(), max_x = corners[0].x();
    double min_y = corners[0].y(), max_y = corners[0].y();
    for (const auto& c : corners) {
        min_x = std::min(min_x, c.x()); max_x = std::max(max_x, c.x());
        min_y = std::min(min_y, c.y()); max_y = std::max(max_y, c.y());
    }
    // terrainToWorld returns CELL CENTRES, so the corner walk gives a bbox
    // inset by half a cell on every side vs the true published extent
    // [origin, origin+length]. Expand by half a cell so the SDF/obstacle
    // bounds cover the full map footprint — a box or risk zone in the outer
    // half-cell ring was otherwise clipped out of the built grid.
    const double half = 0.5 * terrain_data_.resolution;
    min_x -= half; max_x += half;
    min_y -= half; max_y += half;

    // Z: scan all elevation samples for the actual peak, add flight headroom.
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (float e : terrain_data_.elevation) {
        if (!std::isfinite(e)) continue;
        min_z = std::min(min_z, (double)e);
        max_z = std::max(max_z, (double)e);
    }
    if (!std::isfinite(min_z) || !std::isfinite(max_z)) {
        min_z = 0.0; max_z = 0.0;
    }
    const double z_headroom = 30.0;  // vertical margin above peak for traj room

    terrain_bbox_lo_ = Eigen::Vector3d(min_x, min_y, std::min(0.0, min_z));
    terrain_bbox_hi_ = Eigen::Vector3d(max_x, max_y, max_z + z_headroom);
    terrain_bbox_computed_ = true;
    *lo = terrain_bbox_lo_;
    *hi = terrain_bbox_hi_;
    return true;
}

} // namespace path_manager
