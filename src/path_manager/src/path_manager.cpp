#include "path_manager/path_manager.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
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
        node_->declare_parameter("manager/length_per_piece", 3.0);
        node_->declare_parameter("manager/risk_weight", 1.0);
        node_->declare_parameter("manager/risk_barrier", 100.0);
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
        // ESDF occupancy overlay (RViz debug aid). step=1.0m re-queries the SDF
        // hundreds of millions of times per (re)load — tens of seconds on the
        // critical path. Default 4m: 64x cheaper, still fine for a 3 km map.
        node_->declare_parameter("manager/esdf_viz_step", 4.0);
        node_->declare_parameter("manager/esdf_viz_enable", true);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/length_per_piece", length_per_piece_);
        node_->get_parameter("manager/risk_weight", risk_weight_);
        node_->get_parameter("manager/risk_barrier", risk_barrier_);
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
        node_->get_parameter("manager/esdf_viz_step", esdf_viz_step_);
        node_->get_parameter("manager/esdf_viz_enable", esdf_viz_enable_);
        // Dynamic-obstacle spawn-orientation RNG seed. FIXED by default so the
        // SAME mission spawns the SAME oriented obstacles -> reproducible plans.
        // (The box yaw is NOT "visual only": oriented boxes change the SDF and
        // hence the FM2 route, so a random seed made identical missions plan
        // differently each run.) Set < 0 to randomize per run for robustness tests.
        {
          int yaw_seed = 42;
          node_->get_parameter("manager/dyn_yaw_seed", yaw_seed);
          std::mt19937::result_type applied;
          if (yaw_seed >= 0) {
            applied = static_cast<std::mt19937::result_type>(yaw_seed);
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
                                 std::string("package://mmp_visualization/meshes/building.dae"));
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
        node_->declare_parameter("manager/floor_swath_halfwidth", 5.0);
        node_->get_parameter("obstacle_mesh_resource", obstacle_mesh_resource_);
        node_->get_parameter("obstacle_mesh_height", obstacle_mesh_height_);
        node_->get_parameter("obstacle_viz_scale", obstacle_viz_scale_);
        node_->get_parameter("manager/obstacle_ground_snap", obstacle_ground_snap_);
        node_->get_parameter("manager/floor_swath_halfwidth", floor_swath_halfwidth_);
        if (obstacle_viz_scale_ < 1.0) obstacle_viz_scale_ = 1.0;

        // Visual mesh catalog: model name -> mesh resource + rendered native size [m]
        // (mesh base at z=0, XY centered). Add a model = drop a .dae in
        // mmp_visualization/meshes/ + one line here (measure size with trimesh).
        mesh_catalog_["building"] = { obstacle_mesh_resource_,
                                      Eigen::Vector3d(16.374, 13.358, 17.345) };
        mesh_catalog_["car"]      = { "package://mmp_visualization/meshes/car.dae",
                                      Eigen::Vector3d(17.679, 10.093, 4.620) };
        mesh_catalog_["ship"]     = { "package://mmp_visualization/meshes/simple_ship.dae",
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

        node_->declare_parameter("obstacles", std::vector<double>{});
        std::vector<double> obstacle_params;
        node_->get_parameter("obstacles", obstacle_params);

        // Parse obstacles with flexible format:
        // Basic: [x, y, z] - uses default inflation
        // Circle: [x, y, z, 0, radius]
        // Rectangle: [x, y, z, 1, width, height]
        // Primary obstacle format — 7 fixed fields per entry (ambiguity-free):
        //   Circle:    [cx, cy, cz, 0, radius, 0,      height]
        //   Rectangle: [cx, cy, cz, 1, width,  length, height]
        // `height == 0` means infinite column (legacy semantics).
        //
        // Legacy accepted:
        //   [x, y, z]  — plain point (also used as "no-obstacle" sentinel)
        //
        // Entries are dispatched by peeking at obstacle_params[i + 3]: a
        // recognized shape_type (0 or 1) starts a 7-field record; anything
        // else (including list end) falls back to the 3-field legacy form.
        constexpr size_t kObsFields = 7;
        size_t i = 0;
        while (i + 3 <= obstacle_params.size()) {
            Eigen::Vector3d center(obstacle_params[i + 0],
                                   obstacle_params[i + 1],
                                   obstacle_params[i + 2]);

            const bool have_shape_field = (i + 3 < obstacle_params.size());
            const int shape_type = have_shape_field
                ? static_cast<int>(obstacle_params[i + 3])
                : -1;
            const bool is_full_record =
                have_shape_field &&
                (shape_type == 0 || shape_type == 1) &&
                (i + kObsFields <= obstacle_params.size());

            if (is_full_record) {
                const double p1     = obstacle_params[i + 4];
                const double p2     = obstacle_params[i + 5];
                const double height = obstacle_params[i + 6];
                if (shape_type == 0) {  // CIRCLE: p1=radius, p2 unused
                    if (height > 0.0) {
                        obstacle_centers_.emplace_back(center, p1, height, true);
                    } else {
                        obstacle_centers_.emplace_back(center, p1);
                    }
                } else {  // RECTANGLE: p1=width, p2=length
                    if (height > 0.0) {
                        obstacle_centers_.emplace_back(center, p1, p2, height);
                    } else {
                        obstacle_centers_.emplace_back(center, p1, p2);
                    }
                }
                i += kObsFields;
            } else {
                // Legacy 3-field entry: plain point.
                obstacle_centers_.emplace_back(center);
                i += 3;
            }
        }

        // Latched so a late-joining altitude panel still gets the last
        // front-end route (transient_local pub serves volatile subs fine).
        simple_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/planning/front_end_path",
            rclcpp::QoS(1).reliable().transient_local());
        search_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/debug/search_path", 10);
        esdf_occ_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/debug/esdf_occupied", 1);
        // TRANSIENT_LOCAL so RViz, joining late, still gets the latest set.
        rclcpp::QoS dyn_qos(1);
        dyn_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        dyn_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        dyn_obstacle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/viz/dynamic_obstacles", dyn_qos);
        // Same latched QoS: the altitude panel may (re)join after the plan.
        terrain_influence_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/viz/terrain_influence", dyn_qos);
        // The launch default is drone_id=1 and single-agent runs do not
        // necessarily create drone_0, so risk visualization cannot use the
        // terrain-status publisher's drone_0-only ownership rule. Multiple
        // planners publish an identical id/namespace set and are harmless.
        rclcpp::QoS risk_qos(128);
        risk_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        risk_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        risk_field_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/risk_field", risk_qos);
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

        // Terrain ESDF cache status (drone_0 only, latched).
        if (drone_id == 0) {
            rclcpp::QoS status_qos(1);
            status_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
            terrain_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
                "/planning/terrain_status", status_qos);

            // (The old "ESDF cache missing/ready" startup check is gone with
            // the cache itself — the boxes-only SDF builds instantly when
            // terrain arrives; see setTerrainData.)
        }

        // Terrain may arrive after construction. This first call publishes an
        // ideal circular fallback; setTerrainData() replaces it with the DEM-
        // masked field as soon as the heightmap is ready.
        refreshEffectiveRiskZones();
        rebuildTerrainRiskMasks();
    }

    void PathManager::publishTerrainStatus(const std::string &msg) {
        if (!terrain_status_pub_) return;
        std_msgs::msg::String m;
        m.data = msg;
        terrain_status_pub_->publish(m);
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
            
            // Check prerequisites
            if (!node_) {
                throw std::runtime_error("Node is null");
            }

            poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();

            // Set LogManager for unified logging
            poly_traj_opt_->setLogManager(log_manager_);

            // Set parameters first to ensure node_ is initialized
            poly_traj_opt_->setParam(node_);
            poly_traj_opt_->setDroneId(traj_.local_traj.drone_id);

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

    bool PathManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                     const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                                     const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {
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
            for (auto &wp : wps) {
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
        map_upper_bound_.z() = std::max(max_terrain_z, map_upper_bound_.z()) + 20.0;

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

        // Static yaml obstacles ride the dynamic-patch layer (they are no
        // longer baked into the terrain ESDF — see buildSDFForBounds). Apply
        // once per process; the patches persist on the SDF afterwards.
        if (!static_obstacles_applied_ && !obstacle_centers_.empty()) {
            int applied = 0;
            for (const auto &obs : obstacle_centers_) {
                path_planner::sdf::PrimitiveSpec spec;
                // Cube (L∞) footprint for circles too, matching the old bake:
                // an analytic cylinder has zero horizontal SDF gradient on its
                // axis, which pins L-BFGS at a saddle.
                spec.kind = path_planner::sdf::PrimitiveKind::kCube;
                const double r = (obs.param1 > 0) ? obs.param1 : 0.5;
                const double sx = (obs.shape == ObstacleShape::CIRCLE) ? 2.0 * r : obs.param1;
                const double sy = (obs.shape == ObstacleShape::CIRCLE) ? 2.0 * r : obs.param2;
                // z_extent == 0 means "infinite column" (full span of the
                // BUILT grid — read it from the SDF itself, since the grid
                // may be world-wide rather than mission-scoped).
                const Eigen::Vector3d grid_lo = sdf_manager_.origin();
                const Eigen::Vector3d grid_hi =
                    grid_lo + sdf_manager_.shape().cast<double>().cwiseProduct(
                                  sdf_manager_.voxelSizes());
                const double z0 = (obs.z_extent > 0.0) ? obs.center.z() : grid_lo.z();
                const double z1 = (obs.z_extent > 0.0) ? obs.center.z() + obs.z_extent
                                                       : grid_hi.z();
                spec.center = Eigen::Vector3d(obs.center.x(), obs.center.y(),
                                              0.5 * (z0 + z1));
                spec.size = Eigen::Vector3d(sx, sy, std::max(z1 - z0, sdf_voxel_z_));
                if (sdf_manager_.addObstacle(spec) >= 0) ++applied;
                else log_manager_->warnf("static obstacle patch failed at (%.1f,%.1f)",
                                         obs.center.x(), obs.center.y());
            }
            static_obstacles_applied_ = true;
            log_manager_->infof("Applied %d/%zu static yaml obstacles as SDF patches",
                                applied, obstacle_centers_.size());
        }

        // SDF sanity probe at start/goal.
        {
            float d_start = sdf_manager_.getDistance(start_pos);
            float d_goal  = sdf_manager_.getDistance(wps.back());
            log_manager_->infof("SDF probe: start=%.3f m, goal=%.3f m (margin=%.2f)",
                                d_start, d_goal, obstacle_clearance_);
        }

        // === ESDF occupancy visualization ===
        // Sample the ESDF on a coarse grid and publish occupied voxels as a
        // CUBE_LIST so the user can overlay them on the terrain mesh in RViz
        // to confirm terrain → SDF mapping.
        if (esdf_viz_enable_ && esdf_occ_pub_ &&
            sdf_manager_.revision() != esdf_viz_revision_) {
            esdf_viz_revision_ = sdf_manager_.revision();
            // Re-sampling the whole mission volume (hundreds of millions of
            // SDF queries -> tens of seconds) every plan is pointless while
            // the SDF is unchanged; the revision gate republishes only after
            // a rebuild/load or dynamic-obstacle change.
            const double step = std::max(1.0, esdf_viz_step_);
            Eigen::Vector3d lo = map_lower_bound_;
            Eigen::Vector3d hi = map_upper_bound_;
            visualization_msgs::msg::Marker cubes;
            cubes.header.frame_id = "map";
            cubes.header.stamp = node_->get_clock()->now();
            cubes.ns = "esdf_occupied";
            cubes.id = 0;
            cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
            cubes.action = visualization_msgs::msg::Marker::ADD;
            cubes.pose.orientation.w = 1.0;
            cubes.scale.x = step; cubes.scale.y = step; cubes.scale.z = step;
            cubes.color.r = 1.0f; cubes.color.g = 0.1f; cubes.color.b = 0.1f; cubes.color.a = 0.4f;
            for (double x = lo.x(); x <= hi.x(); x += step) {
                for (double y = lo.y(); y <= hi.y(); y += step) {
                    for (double z = lo.z(); z <= hi.z(); z += step) {
                        Eigen::Vector3d p(x, y, z);
                        float d = sdf_manager_.getDistance(p);
                        if (std::isfinite(d) && d < 0.0f) {
                            geometry_msgs::msg::Point pt;
                            pt.x = x; pt.y = y; pt.z = z;
                            cubes.points.push_back(pt);
                        }
                    }
                }
            }
            log_manager_->infof("ESDF viz: %zu occupied cubes (step=%.1fm)",
                                cubes.points.size(), step);
            esdf_occ_pub_->publish(cubes);
        }


        // === STEP 2~3: front-end search + densification ===
        std::vector<Eigen::Vector3d> full_route, clean_path;
        std::vector<double> cap_ref;
        if (!planFrontEnd(start_pos, wps, full_route, clean_path, cap_ref)) {
            return false;
        }

        // [VEL-ALIGN] A SYNTHESIZED start velocity (default speed x first-leg
        // chord — the FSM knows no route before the front-end runs) is only a
        // proxy for "already cruising along the route". Re-aim it onto the
        // route's ACTUAL initial direction so the head boundary condition
        // agrees with the path the optimizer is about to follow — otherwise
        // the head piece launches down the chord at cruise speed and S-bends
        // onto the route. Explicitly commanded / trajectory-derived
        // velocities are never touched (start_vel_synthesized_ false).
        Eigen::Vector3d start_vel_eff = start_vel;
        if (align_start_vel_to_route_ && start_vel_synthesized_ &&
            clean_path.size() >= 2) {
            Eigen::Vector3d dir = clean_path[1] - clean_path[0];
            dir.z() = 0.0;  // same LEVEL contract as the chord synthesis
            if (dir.head<2>().norm() > 1.0e-9) {
                dir.normalize();
                const Eigen::Vector3d re_aimed = dir * start_vel.norm();
                if ((re_aimed - start_vel).norm() > 1.0e-9) {
                    log_manager_->infof(
                        "[VEL-ALIGN] synthesized start vel re-aimed to the "
                        "route's initial direction: (%.3f, %.3f, %.3f) -> "
                        "(%.3f, %.3f, %.3f) u/s",
                        start_vel.x(), start_vel.y(), start_vel.z(),
                        re_aimed.x(), re_aimed.y(), re_aimed.z());
                }
                start_vel_eff = re_aimed;
            }
        }

        // === STEP 4~5: trajectory optimization (MINCO + L-BFGS) ===
        bool opt_ok = optimizeStage(clean_path, full_route,
                                    start_pos, start_vel_eff, start_acc, wps,
                                    cap_ref);

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
                               std::vector<double> &cap_ref)
{
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
        for (size_t seg = 0; seg < all_points.size() - 1; ++seg)
        {
            std::vector<Eigen::Vector3d> seg_path =
                searcher_.astarSearchAndGetSimplePath(
                    astar_step_size_, all_points[seg], all_points[seg + 1],
                    traj_.local_traj.drone_id);

            log_manager_->infof("A* segment %zu: simple_path_size=%zu",
                seg, seg_path.size());

            if (seg_path.size() < 2)
            {
                RCLCPP_ERROR(node_->get_logger(),
                    "A* failed for segment %zu: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                    seg, all_points[seg].x(), all_points[seg].y(), all_points[seg].z(),
                    all_points[seg+1].x(), all_points[seg+1].y(), all_points[seg+1].z());
                return false;
            }

            for (size_t i = (seg == 0 ? 0 : 1); i < seg_path.size(); ++i)
            {
                if (!full_route.empty() &&
                    (full_route.back() - seg_path[i]).norm() < 1e-3) {
                    continue;
                }
                full_route.push_back(seg_path[i]);
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
            rounded.reserve(full_route.size() * 4);
            rounded.push_back(full_route.front());
            for (size_t n = 1; n + 1 < full_route.size(); ++n) {
                const Eigen::Vector3d &A = full_route[n - 1];
                const Eigen::Vector3d &B = full_route[n];
                const Eigen::Vector3d &C = full_route[n + 1];
                const double d1 = (A - B).norm(), d2 = (C - B).norm();
                if (d1 < 1e-6 || d2 < 1e-6) { rounded.push_back(B); continue; }
                const Eigen::Vector3d u = (A - B) / d1, w = (C - B) / d2;
                const double cosphi = std::clamp(u.dot(w), -1.0, 1.0);
                const double phi = std::acos(cosphi);   // interior angle at B
                if (phi > M_PI - 0.05) { rounded.push_back(B); continue; }  // ~straight

                bool placed = false;
                for (double R = corner_fillet_radius_; R > 1.0; R *= 0.5) {
                    double t = R / std::tan(0.5 * phi);
                    const double t_cap = 0.45 * std::min(d1, d2);
                    double R_eff = R;
                    if (t > t_cap) { t = t_cap; R_eff = t * std::tan(0.5 * phi); }
                    const Eigen::Vector3d P1 = B + u * t, P2 = B + w * t;
                    // Quadratic Bezier P1->B->P2 approximates the arc and is
                    // tangent to both segments; sample every ~3 units.
                    const int N = std::max(3, (int)std::ceil((P1 - P2).norm() / 3.0));
                    std::vector<Eigen::Vector3d> arc;
                    bool ok = true;
                    for (int k = 1; k < N; ++k) {
                        const double a = (double)k / N, b = 1.0 - a;
                        const Eigen::Vector3d p = b * b * P1 + 2 * a * b * B + a * a * P2;
                        if (!sample_ok(p, B)) { ok = false; break; }
                        arc.push_back(p);
                    }
                    if (!ok) continue;            // shrink R, retry
                    rounded.push_back(P1);
                    rounded.insert(rounded.end(), arc.begin(), arc.end());
                    rounded.push_back(P2);
                    placed = true;
                    (void)R_eff;
                    break;
                }
                if (!placed) rounded.push_back(B);  // keep the kink
            }
            rounded.push_back(full_route.back());
            log_manager_->infof("corner fillet R=%.1f: %zu -> %zu pts",
                                corner_fillet_radius_, full_route.size(),
                                rounded.size());
            full_route.swap(rounded);
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

        // Front-end route as nav_msgs/Path → mmp_visualization converts it to a
        // RViz marker (/viz/simple_path).
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
        simple_path_pub_->publish(path_msg);

        // Front-end route as LINE_STRIP + SPHERE_LIST for debugging (cyan)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            line.ns = "search_path_line";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 1.5;
            line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; line.color.a = 1.0;
            line.lifetime = rclcpp::Duration(0, 0);
            for (const auto &p : full_route) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                line.points.push_back(pt);
            }
            search_path_pub_->publish(line);

            visualization_msgs::msg::Marker dots;
            dots.header = line.header;
            dots.ns = "search_path_dots";
            dots.id = 1;
            dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            dots.action = visualization_msgs::msg::Marker::ADD;
            dots.pose.orientation.w = 1.0;
            dots.scale.x = 2.5; dots.scale.y = 2.5; dots.scale.z = 2.5;
            dots.color.r = 0.0; dots.color.g = 0.8; dots.color.b = 1.0; dots.color.a = 1.0;
            dots.lifetime = rclcpp::Duration(0, 0);
            for (const auto &p : full_route) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                dots.points.push_back(pt);
            }
            search_path_pub_->publish(dots);
        }

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
            }
        }
        log_manager_->infof(
            "A* shortcut %zu pts → sparse pieces %zu pts (max_seg %.1f)",
            full_route.size(), clean_path.size(), max_seg);

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
                                const std::vector<double> &cap_ref)
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
        bool opt_success = poly_traj_opt_->optimizeFromPath(
            clean_path, start_pos, start_vel, start_acc, waypoints, max_vel_,
            global_traj, local_traj, cap_ref);
        if (!opt_success) {
            log_manager_->errorf("Trajectory optimization failed");
            return false;
        }

        // Local traj start_time is the trajectory-following reference; stamp it
        // after optimization (matches pre-refactor behavior).
        double local_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        traj_.setGlobalTraj(global_traj, global_time);
        traj_.setLocalTraj(local_traj, local_time, traj_.local_traj.drone_id);

        // [VIZ-TRAJ] The along-trajectory diagnostics (terrain floor underfoot,
        // experienced risk, and the risk-visibility profile) must sample the
        // OPTIMIZED trajectory that is actually flown and displayed
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
        publishTerrainInfluence(local_traj);
        publishTrajRisk(local_traj);
        publishRiskProfile(local_traj);

        auto t_opt_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] trajectory optimization: %.1f ms, duration=%.3f max_vel=%.3f",
            std::chrono::duration<double, std::milli>(t_opt_end - t_opt_start).count(),
            local_traj.getTotalDuration(), local_traj.getMaxVelRate());

        return true;
}

void PathManager::publishTerrainInfluence(const poly_traj::Trajectory &traj)
{
    if (!terrain_influence_pub_) return;
    const double T = traj.getTotalDuration();
    if (!(T > 0.0) || !std::isfinite(T)) return;

    // ~2 samples per FM2 coarse column (~2.3 u) at cruise speed so the
    // staircase edges resolve; capped for pathological durations.
    const double est_len = std::max(1.0, T * max_vel_);
    const int K = std::clamp(static_cast<int>(est_len / 1.0), 256, 16384);

    // Triples [s, floor_underfoot, floor_swath]: the underfoot column floor
    // plus the max floor within floor_swath_halfwidth_ — constraints the
    // route dodged laterally never appear underfoot (that is what dodging
    // means), so the swath channel is what explains avoidance climbs.
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(3 * (K + 1));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double s = 0.0;
    Eigen::Vector3d prev = traj.getPos(0.0);
    for (int k = 0; k <= K; ++k) {
        const double t = T * static_cast<double>(k) / K;
        const Eigen::Vector3d p = traj.getPos(t);
        s += (p - prev).head<2>().norm();
        prev = p;
        const float h = searcher_.fm2ColumnFloor(p.x(), p.y());
        const float w = searcher_.fm2SwathFloor(p.x(), p.y(), floor_swath_halfwidth_);
        msg.data.push_back(s);
        msg.data.push_back(std::isfinite(h) ? static_cast<double>(h) : nan);
        msg.data.push_back(std::isfinite(w) ? static_cast<double>(w) : nan);
    }
    terrain_influence_pub_->publish(msg);
    size_t floored = 0;
    for (size_t i = 1; i < msg.data.size(); i += 3)
        if (!std::isnan(msg.data[i])) ++floored;
    log_manager_->infof("[TERRAIN-INFLUENCE] %d samples, %zu floored columns, "
                        "swath=%.1f u, len=%.1f u",
                        K + 1, floored, floor_swath_halfwidth_, s);
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

// [RISK-PROFILE] Altitude-panel channels: the risk zones as they REALLY are
// in 3-D, cut along the flight path. Per arc-length sample: the zone-union
// DOME cross-section [dome_bot, dome_top] and the visibility ROOF (the
// altitude above which at least one zone can see this column; flying BELOW
// the roof inside a dome = terrain-occluded). The top-down heatmap cannot express
// "under the roof" — the profile view can, which is exactly the ambiguity
// the user hit ("looks like it passes through, but is it safe?").
// Same math as the heatmap cells (single source of truth: effective
// AGL-grounded zones + riskShadowCeiling + the configured sigmoid contour).
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

    double vis_offset = 0.0;
    if (risk_mask_viz_threshold_ > 1e-6 &&
        risk_mask_viz_threshold_ < 1.0 - 1e-6) {
        vis_offset = risk_mask_softness_ * std::log(
            risk_mask_viz_threshold_ / (1.0 - risk_mask_viz_threshold_));
    }
    const double rv_ratio = risk_vertical_ratio_;
    constexpr double kInf = std::numeric_limits<double>::infinity();

    msg.data.reserve(4 * (K + 1));
    double s = 0.0;
    Eigen::Vector3d prev = traj.getPos(0.0);
    for (int k = 0; k <= K; ++k) {
        const double t = T * static_cast<double>(k) / K;
        const Eigen::Vector3d p = traj.getPos(std::min(t, T - 1e-9));
        s += (p - prev).head<2>().norm();
        prev = p;
        double ground = 0.0;
        if (terrain_data_.valid) {
            const float h = terrain_data_.getElevation(p.x(), p.y());
            if (std::isfinite(h) && h > 0.0f) ground = static_cast<double>(h);
        }
        double roof = kInf, dome_top = -kInf, dome_bot = kInf;
        bool covered = false;
        for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
            const auto &zone = risk_zones_[zi];
            if (!(zone.reach > 0.0) || !(zone.peak > 0.0)) continue;
            const double dx = p.x() - zone.center.x();
            const double dy = p.y() - zone.center.y();
            const double rho2 = dx * dx + dy * dy;
            if (rho2 >= zone.reach * zone.reach) continue;
            const double rv = zone.reach * rv_ratio;
            const double half_z = rv * std::sqrt(std::max(
                0.0, 1.0 - rho2 / (zone.reach * zone.reach)));
            const double upper = zone.center.z() + half_z;
            if (upper <= ground) continue;  // envelope fully underground
            covered = true;
            dome_top = std::max(dome_top, upper);
            dome_bot = std::min(dome_bot,
                                std::max(zone.center.z() - half_z, ground));
            double det = std::max(zone.center.z() - half_z, ground);
            const double horizon = riskShadowCeiling(
                zi, Eigen::Vector3d(p.x(), p.y(), zone.center.z()));
            if (std::isfinite(horizon)) det = std::max(det, horizon + vis_offset);
            if (det < upper) roof = std::min(roof, det);
        }
        msg.data.push_back(s);
        if (!covered) {
            msg.data.push_back(nan);
            msg.data.push_back(nan);
            msg.data.push_back(nan);
        } else {
            // Full shadow (no zone can ever see this column): roof caps at
            // the dome top — the visible band [roof, dome_top] is empty.
            msg.data.push_back(std::isfinite(roof) ? roof : dome_top);
            msg.data.push_back(dome_top);
            msg.data.push_back(dome_bot);
        }
    }
    risk_profile_pub_->publish(msg);
    log_manager_->infof("[RISK-PROFILE] %d samples over %.1f u, zones=%zu",
                        K + 1, s, risk_zones_.size());
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

    size_t painted = 0;
    for (int row = 0; row < rows; ++row) {
        const double wx = x0 + length_x - (row + 0.5) * res;
        for (int col = 0; col < cols; ++col) {
            const double wy = y0 + length_y - (col + 0.5) * res;
            // Water/off-DEM columns sit at sea level 0 (same rule as
            // refreshEffectiveRiskZones / the marker modes).
            double ground = 0.0;
            if (terrain_data_.valid) {
                const float h = terrain_data_.getElevation(wx, wy);
                if (std::isfinite(h) && h > 0.0f)
                    ground = static_cast<double>(h);
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
            elevation.data[idx] = static_cast<float>(ground + lift);
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

void PathManager::publishEffectiveRiskField()
{
    if (!risk_field_pub_) return;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = node_->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    risk_field_pub_->publish(clear);

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
            if (!floor.points.empty()) risk_field_pub_->publish(floor);
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
            wire.scale.x = std::max(0.08, 0.12 * step);
            wire.color.r = 1.0f;
            wire.color.g = 0.18f;
            wire.color.b = 0.02f;
            wire.color.a = 0.58f;
            wire.lifetime = rclcpp::Duration(0, 0);
            auto shellPointVisible = [&](const Eigen::Vector3d &p) {
                double ground = 0.0;
                if (groundAt(p.x(), p.y(), &ground) &&
                    p.z() <= ground + 0.02) return false;
                return riskVisibilityValue(zi, p) >=
                       risk_mask_viz_threshold_;
            };
            auto appendSegment = [&](const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b) {
                if (!shellPointVisible(a) || !shellPointVisible(b)) return;
                wire.points.push_back(point(a));
                wire.points.push_back(point(b));
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
            if (!wire.points.empty()) risk_field_pub_->publish(wire);
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
            if (!slice.points.empty()) risk_field_pub_->publish(slice);
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
        risk_field_pub_->publish(source);

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
            risk_field_pub_->publish(mast);
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
        risk_field_pub_->publish(label);
    }
}

void PathManager::setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr &msg) {
    if (!msg || msg->layers.empty()) {
        log_manager_->warnf("Received empty terrain GridMap");
        return;
    }
    if (sdf_voxel_size_ <= 0.0 && msg->info.resolution > 1e-6) {
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

    const auto& elev_data = msg->data[elev_idx];
    if (elev_data.layout.dim.size() < 2) {
        log_manager_->warnf("Invalid terrain GridMap data layout");
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
            // Clear ALL patches (dynamic AND static-yaml) and reset the static
            // latch: every patch was placed on the OLD grid/DEM — dynamic ones
            // baked groundedCenter() on the old ground, and static yaml
            // obstacles with an infinite column (z_extent==0) baked their z
            // span from the old grid bounds. clearDynamicObstacles() wipes the
            // patch layer and resets static_obstacles_applied_ so
            // planGlobalTraj re-applies the yaml obstacles against the NEW grid
            // (column z re-read from new bounds). Previously this ran only when
            // dynamic patches existed, so a static-only config kept a stale
            // short column with a free gap above it after a swap to a taller
            // map.
            if (sdf_manager_.numActiveObstacles() > 0 ||
                !dyn_patch_ids_.empty() || static_obstacles_applied_) {
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
                publishTerrainStatus(
                    "SDF ready (boxes-only grid; terrain via heightmap)");
                flushPendingObstacles();
            }
        }
    }

    // DEM coordinates and elevations are now authoritative. Re-ground the
    // AGL zone sources onto the fresh heightmap, then rebuild the radial
    // viewshed and replace the ideal circular RViz fallback.
    refreshEffectiveRiskZones();
    rebuildTerrainRiskMasks();
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

void PathManager::clearDynamicObstacles()
{
    sdf_manager_.clearObstacles();
    // clearObstacles() wipes ALL patches, including the static yaml obstacles
    // applied in planGlobalTraj. Drop the once-per-process latch so the next
    // plan re-applies them (re-applying also recomputes infinite-column z
    // extents against the current grid, which may have changed).
    static_obstacles_applied_ = false;
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

    // Geometry obstacles (obstacle_centers_) are NOT voxelised here any more.
    // Baking them had two failure modes: with an ESDF cache in use they were
    // silently absent (the build is skipped), and when the cache was first
    // built they were permanently fused into the reusable "terrain" file.
    // They are applied as dynamic patches after the SDF is ready instead
    // (applyStaticObstaclePatches), which works identically for cache-loaded
    // and freshly built SDFs and keeps the cache pure terrain.

    // NOTE: ground and virtual ceiling are NOT voxelised here. Folding them
    // into the SDF would drag the clearance band above/below the actual
    // plane, so the trajectory would be pushed off a `-0.1` floor by up to
    // `obstacle_clearance` metres. Instead they are enforced as hard
    // half-space constraints inside the optimizer (sdfGradCostP), which
    // applies a unit upward/downward gradient only when the query point
    // crosses the plane — no clearance band, no lateral contamination.

    if (!sdf_manager_.isInitialized()) {
        sdf_manager_.initialize(res, res_z);
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
