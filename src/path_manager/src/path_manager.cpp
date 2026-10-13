#include "path_optimizer/ros_parameter_view.h"
#include "path_manager/path_manager.h"
#include "path_manager/optimizer_worker_profile.h"
#include "path_manager/direct_trajectory_solver.h"
#include "path_manager/mission_geometry.h"
#include "path_manager/route_postprocessing.h"
#include "path_manager/mission_route_search.h"
#include "path_manager/builtin_route_search.h"
#include "path_manager/ros_route_search.h"
#include "path_planner/risk_shape.h"

#include <stdexcept>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <malloc.h>

namespace path_manager
{

    PathManager::PathManager(rclcpp::Node::SharedPtr node, bool attach_visualization)
        : node_(node),
          max_vel_(-1.0),
          max_acc_(-1.0),
          is_optimizer_initialized_(false),
          current_formation_type_("")
    {
        log_manager_ = std::make_shared<swarm_formation::LogManager>(
            node_->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);
        
        PlanningEnvironmentOptions environment_options{};
        std::vector<RiskZone> initial_risk_zones;
        enable_debug_logs_ = false;
        if (node_->has_parameter("enable_debug_logs")) {
            node_->get_parameter("enable_debug_logs", enable_debug_logs_);
        }
            
        // [PARAM] drone_id is DATA, not a diagnostic: it reaches the
        // optimizer (setDroneId) and the published PolyTraj. It used to be an
        // uninitialized local whose get_parameter return was discarded, so a
        // failed read published whatever was on the stack — or, once
        // initialized, silently published as drone 0. Neither is acceptable
        // for a value that identifies the aircraft, so this refuses instead.
        // ReplanFSM declares it before constructing PathManager; a direct
        // construction (a test) must declare it itself.
        int drone_id = 0;
        if (!node_->has_parameter("drone_id")) {
            throw std::runtime_error(
                "PathManager: 'drone_id' is not declared — refusing to plan "
                "as drone 0 by default. ReplanFSM declares it before "
                "constructing PathManager; a direct construction must "
                "declare it itself.");
        }
        if (!node_->get_parameter("drone_id", drone_id)) {
            throw std::runtime_error(
                "PathManager: 'drone_id' is declared but could not be read "
                "(type mismatch?) — refusing to plan as drone 0 by default.");
        }
        execution_ = std::make_unique<PlanningExecutionStore>(drone_id);

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
        builtin_route_search_ = std::make_shared<BuiltinRouteSearchBackend>(
            readRosBuiltinRouteSearchConfig(*node_), log_manager_, &searcher_);
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
        node_->declare_parameter("manager/corner_fillet_radius", 0.0);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/length_per_piece", length_per_piece_);
        node_->get_parameter("manager/risk_weight", risk_weight_);
        node_->get_parameter("manager/risk_goal_taper_radius",
                             risk_goal_taper_radius_);
        node_->get_parameter("manager/risk_terrain_mask_enable",
                             environment_options.risk_terrain_mask_enable);
        node_->get_parameter("manager/risk_zone_agl", environment_options.risk_zone_agl);
        node_->get_parameter("manager/risk_vertical_ratio",
                             environment_options.risk_vertical_ratio);
        environment_options.risk_vertical_ratio = std::max(0.01, environment_options.risk_vertical_ratio);
        node_->get_parameter("manager/risk_mask_radial_step",
                             environment_options.risk_mask_radial_step);
        node_->get_parameter("manager/risk_mask_softness",
                             environment_options.risk_mask_softness);
        node_->get_parameter("manager/risk_grounded", environment_options.risk_grounded);
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
        environment_options.risk_mask_softness = std::max(1e-4, environment_options.risk_mask_softness);
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
        node_->get_parameter("manager/sdf_voxel_size", environment_options.sdf_voxel_size);
        node_->get_parameter("manager/sdf_voxel_z", environment_options.sdf_voxel_z);
        // sdf_voxel_size <= 0 resolves to the DEM cell size once terrain
        // arrives (setTerrainData) — finer only burns memory, coarser loses
        // data, and the right value differs per world.
        if (environment_options.sdf_voxel_z <= 0.0) environment_options.sdf_voxel_z = 0.1;  // 10 m real (frame /100)
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
          environment_options.yaw_seed = applied;
        }
        // Patches must extend at least as far as the dynamic berth, or the
        // distance query reads +inf before the margin is reached.
        environment_options.dynamic_influence_radius = dyn_obstacle_margin_;

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
        node_->get_parameter("manager/obstacle_ground_snap", environment_options.obstacle_ground_snap);
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
            // [PA-7] Validate BEFORE adopting anything. Both RiskZone's own
            // declaration and risk_zones.yaml state peak MUST be in (0, 1];
            // this loader used to assign it raw, so 0, a negative, >1, NaN and
            // Inf all became live field parameters.
            //
            // ATOMIC REJECTION: the whole set is refused rather than the
            // offending zone. Dropping one zone silently REMOVES a hazard from
            // the field and the planner then routes straight through it.
            //
            // This does NOT make startup fail-closed at system level: a
            // rejected config leaves zero zones and planning continues. Whether
            // it should is an open policy question (ledger PA-7).
            std::vector<RiskZone> parsed;
            size_t bad = 0;
            for (size_t ti = 0; ti < tz_params.size(); ti += 5) {
                RiskZone tz;
                tz.center = Eigen::Vector3d(tz_params[ti], tz_params[ti+1], tz_params[ti+2]);
                tz.reach = tz_params[ti+3];
                tz.peak = tz_params[ti+4];
                const bool geom_ok = std::isfinite(tz.center.x()) &&
                                     std::isfinite(tz.center.y()) &&
                                     std::isfinite(tz.center.z()) &&
                                     std::isfinite(tz.reach) && tz.reach > 0.0;
                if (!geom_ok || !mmp::risk::isValidPeak(tz.peak)) {
                    ++bad;
                    log_manager_->errorf(
                        "  RiskZone #%zu REJECTED: center=(%.3f,%.3f,%.3f) "
                        "range=%.3f peak=%.3f — peak must be finite and in "
                        "(0, 1], range must be finite and > 0",
                        ti / 5, tz.center.x(), tz.center.y(), tz.center.z(),
                        tz.reach, tz.peak);
                    continue;
                }
                parsed.push_back(tz);
                log_manager_->infof("  RiskZone #%zu: center=(%.1f,%.1f,%.1f) range=%.1f risk=%.1f",
                    parsed.size()-1, tz.center.x(), tz.center.y(), tz.center.z(),
                    tz.reach, tz.peak);
            }
            if (bad > 0) {
                log_manager_->errorf(
                    "risk_zones REJECTED: %zu of %zu zones invalid — loading "
                    "NONE. A partially loaded set would hide %zu hazard(s).",
                    bad, tz_params.size() / 5, bad);
            } else {
                initial_risk_zones = std::move(parsed);
                log_manager_->infof("Loaded %zu risk zones (risk_weight=%.1f)", initial_risk_zones.size(), risk_weight_);
            }
        } else if (tz_params.empty()) {
            log_manager_->infof("No risk zones configured");
        } else {
            log_manager_->warnf("Invalid risk_zones param size: %zu (must be multiple of 5)", tz_params.size());
        }

        // Read legacy parameters even when the optional ROS connection is off.
        node_->declare_parameter("path_scale", 1.5);
        node_->get_parameter("path_scale", path_scale_);
        node_->declare_parameter("manager/debug_pipeline_viz", false);
        node_->get_parameter("manager/debug_pipeline_viz", debug_pipeline_viz_);
        if (attach_visualization)
            visualization_ = std::make_unique<RosPlanningVisualization>(
                *node_, debug_pipeline_viz_);

        // Terrain may arrive after construction. This first call publishes an
        // ideal circular fallback; setTerrainData() replaces it with the DEM-
        // masked field as soon as the heightmap is ready.
        environment_ = std::make_unique<PlanningEnvironmentStore>(
            environment_options, log_manager_, std::move(initial_risk_zones));
        publishEffectiveRiskField();
    }

    void PathManager::setVehicleModel(mmp_vehicle_dynamics::VehicleModel::Ptr model)
    {
        if (model && !model->valid())
            throw std::invalid_argument("invalid vehicle model");
        if (poly_traj_opt_) poly_traj_opt_->setVehicleModel(model);
        vehicle_model_override_ = std::move(model);
        execution_->invalidatePending();
    }

    void PathManager::setRouteSearchBackend(std::shared_ptr<RouteSearchBackend> backend)
    {
        route_search_backend_ = std::move(backend);
        execution_->invalidatePending();
        // Existing flight/policy captures remain meaningful until the next
        // actual search advances the epoch. Do not treat algorithm selection
        // as a physical environment change or emit a synthetic ABORT.
    }

    void PathManager::initOptimizer(bool force_reinit)
    {
        if (is_optimizer_initialized_ && poly_traj_opt_ && !force_reinit)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Optimizer already initialized for drone %d", execution_->current().drone_id);
            return;
        }

        if (force_reinit && is_optimizer_initialized_) {
            RCLCPP_INFO(node_->get_logger(), "Force reinitializing optimizer for drone %d", execution_->current().drone_id);
        }

        // Reset state in case of partial initialization
        is_optimizer_initialized_ = false;
        poly_traj_opt_.reset();

        try {
            RCLCPP_INFO(node_->get_logger(), "Initializing optimizer for drone %d...", execution_->current().drone_id);
            const auto config = readDirectPlanConfig();
            opt_obstacle_clearance_ = config.obstacle_clearance;
            weight_altitude_ = config.weight_altitude;
            alt_cap_headroom_ = config.alt_cap_headroom;
            alt_floor_headroom_ = config.alt_floor_headroom;
            poly_traj_opt_ = configureOptimizer(config, {});
            // Only mark as initialized after all steps succeed
            is_optimizer_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Optimizer initialized successfully for drone %d", execution_->current().drone_id);
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
        return makeConfiguredOptimizer({});
    }

    std::unique_ptr<ego_planner::PolyTrajOptimizer>
    PathManager::makeConfiguredOptimizer(
        const std::vector<rclcpp::Parameter> &parameter_overrides)
    {
        for (const auto &parameter : parameter_overrides) {
            if (!findOptimizerWorkerParameter(parameter.get_name())) {
                throw std::invalid_argument(
                    "unsupported independent optimizer override: " + parameter.get_name());
            }
        }
        return configureOptimizer(readDirectPlanConfig(parameter_overrides), parameter_overrides);
    }

    PathManager::DirectPlanConfig PathManager::readDirectPlanConfig(
        const std::vector<rclcpp::Parameter> &overrides) const
    {
        if (!node_) throw std::runtime_error("Node is null");
        DirectPlanConfig config{opt_obstacle_clearance_, weight_altitude_,
            alt_cap_headroom_, alt_floor_headroom_};
        const ego_planner::RosParameterView parameters(node_, overrides);
        parameters.get_parameter("optimization/obstacle_clearance", config.obstacle_clearance);
        parameters.get_parameter("optimization/weight_altitude", config.weight_altitude);
        parameters.get_parameter("optimization/alt_cap_headroom", config.alt_cap_headroom);
        parameters.get_parameter("optimization/alt_floor_headroom", config.alt_floor_headroom);
        parameters.get_parameter("optimization/dynamics_unit_xy_m", config.dynamics_unit_xy_m);
        if (!node_->has_parameter("optimization/dynamics_flight_path_max_deg"))
            node_->declare_parameter("optimization/dynamics_flight_path_max_deg", 30.0);
        parameters.get_parameter("optimization/dynamics_flight_path_max_deg", config.flight_path_max_deg);
        return config;
    }

    RouteSliceConfig PathManager::sliceSolveConfig() const
    {
        const auto config = readDirectPlanConfig();
        return {max_vel_, config.alt_cap_headroom, config.alt_floor_headroom,
                ground_height_, config.weight_altitude};
    }

    PathManager::DirectSolveContext PathManager::missionSolveContext() const
    {
        auto config = readDirectPlanConfig();
        // These four values are latched by explicit optimizer initialization.
        // Live ROS edits take effect at the same boundary as before.
        config.obstacle_clearance = opt_obstacle_clearance_;
        config.weight_altitude = weight_altitude_;
        config.alt_cap_headroom = alt_cap_headroom_;
        config.alt_floor_headroom = alt_floor_headroom_;
        return {config, poly_traj_opt_.get()};
    }

    std::unique_ptr<ego_planner::PolyTrajOptimizer> PathManager::configureOptimizer(
        const DirectPlanConfig &config,
        const std::vector<rclcpp::Parameter> &parameter_overrides)
    {
        {
            const ego_planner::RosParameterView parameters(node_, parameter_overrides);

            auto poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();

            // Set LogManager for unified logging
            poly_traj_opt_->setLogManager(log_manager_);

            // Set parameters first to ensure node_ is initialized
            // The id goes in FIRST: setParam logs through LOG_* , which
            // formats drone_id_ into every line it emits. Setting it after
            // meant those lines carried the pre-set value.
            poly_traj_opt_->setDroneId(execution_->current().drone_id);
            if (vehicle_model_override_)
                poly_traj_opt_->setVehicleModel(vehicle_model_override_);
            poly_traj_opt_->setParam(node_, parameter_overrides);

            ego_planner::OptimizerRuntimeSettings runtime_settings;
            ego_planner::OptimizerEnvironment environment;
            environment.distance_field = &environment_->distanceField();
            // Strictly below the front-end margin: a margin-respecting path then
            // carries ZERO obstacle cost, so its 10000-weighted gradient cannot
            // pin the geometry to the polyline and smoothness can round corners
            // inside the (margin - clearance) buffer.
            runtime_settings.obstacle_clearance =
                std::min(config.obstacle_clearance, obstacle_clearance_);
            {
                // [REJECT] ancestor safety net (default ON): an audit-failed
                // (terrain/obstacle-overlapping) trajectory is discarded, not
                // published. optimization/collision_reject: false restores the
                // old logs-only behavior.
                bool reject_on = true;
                if (!node_->has_parameter("optimization/collision_reject"))
                    node_->declare_parameter("optimization/collision_reject", true);
                parameters.get_parameter("optimization/collision_reject", reject_on);
                runtime_settings.collision_reject = reject_on;

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
                parameters.get_parameter("optimization/audit_envelope_reject",
                                     env_reject_on);
                parameters.get_parameter(
                    "optimization/audit_envelope_violation_max", env_viol_max);
                parameters.get_parameter(
                    "optimization/audit_envelope_violation_hard_max",
                    env_viol_hard);
                runtime_settings.envelope_reject = env_reject_on;
                runtime_settings.envelope_violation_max = env_viol_max;
                runtime_settings.envelope_violation_hard_max = env_viol_hard;

                // [GNRON] endpoint moat taper — same radius as the front-end
                // (wired in initSearcher) so both stages price one field.
                runtime_settings.risk_goal_taper_radius = risk_goal_taper_radius_;

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
                    parameters.get_parameter("optimization/lbfgs_mem_size", mem);
                    parameters.get_parameter("optimization/lbfgs_g_epsilon", geps);
                    parameters.get_parameter("optimization/lbfgs_past", past);
                    parameters.get_parameter("optimization/lbfgs_delta", delta);
                    parameters.get_parameter("optimization/lbfgs_max_linesearch", maxls);
                    parameters.get_parameter("optimization/lbfgs_f_dec", fdec);
                    parameters.get_parameter("optimization/lbfgs_s_curv", scurv);
                    runtime_settings.lbfgs_mem_size = mem;
                    runtime_settings.lbfgs_g_epsilon = geps;
                    runtime_settings.lbfgs_past = past;
                    runtime_settings.lbfgs_delta = delta;
                    runtime_settings.lbfgs_max_linesearch = maxls;
                    runtime_settings.lbfgs_f_dec = fdec;
                    runtime_settings.lbfgs_s_curv = scurv;
                }
            }
            runtime_settings.ground_height = ground_height_;
            runtime_settings.virtual_ceil_height = virtual_ceil_height_;
            poly_traj_opt_->setRuntimeSettings(runtime_settings);
            // 2.5D terrain heightmap: the optimizer queries the DEM elevation
            // directly (exact z) instead of the coarse voxelised SDF, so terrain
            // clearance matches the panel/DEM. The terrain owner outlives the optimizer.
            environment.terrain_height =
                [this](double x, double y) -> float {
                    return environment_->terrain().valid
                             ? environment_->terrain().getElevation(x, y)
                             : -std::numeric_limits<float>::infinity();
                };
            // DEM cell in frame units — scales the SWATH-FLOOR sampling
            // pitch to the actual grid (corridor 30 m vs full_map 250 m).
            environment.terrain_cell_u =
                environment_->terrain().valid ? environment_->terrain().resolution : 0.0;
            // Value + analytic slope of the same surface, for the terrain term
            // and alt-cap gate: cost and gradient must agree exactly (see
            // TerrainData::getElevationAndGrad).
            environment.terrain_gradient =
                [this](double x, double y, float *h, float *gx, float *gy) -> bool {
                    return environment_->terrain().getElevationAndGrad(x, y, h, gx, gy);
                };

            // Pass risk zones to optimizer for trajectory fine-tuning (2nd stage)
            if (!environment_->riskZones().empty()) {
                for (const auto &tz : environment_->riskZones()) {
                    ego_planner::RiskZone oz;
                    oz.center = tz.center;
                    oz.reach = tz.reach;
                    oz.peak = tz.peak;
                    oz.vertical_reach = tz.reach * environment_->verticalRatio();
                    environment.risk.zones.push_back(oz);
                }
            }
            environment.risk.visibility =
                [this, clearance = config.obstacle_clearance](size_t zi, const Eigen::Vector3d &p,
                       Eigen::Vector3d *grad) -> double {
                    return riskVisibility(zi, p, grad, clearance);
                };
            // [OCCLUSION-CAP] raw LOS horizon for the visibility-aware cap: same
            // zone indexing as the visibility callback above.
            environment.risk.shadow_ceiling =
                [this](size_t zi, const Eigen::Vector3d &p) -> double {
                    return riskShadowCeiling(zi, p);
                };
            poly_traj_opt_->setEnvironment(std::move(environment));
            if (!environment_->riskZones().empty()) {
                log_manager_->infof("Passed %zu risk zones to optimizer", environment_->riskZones().size());
            }

            // initOptimizer adopts this instance; parallel workers each
            // keep their own session. Both borrow this manager's providers.
            return poly_traj_opt_;
        }
    }

    double PathManager::effectiveHandoffMaxMps() const
{
    const auto model = vehicleModel();
    const double model_max =
        model ? model->limits().speed_max_mps : std::numeric_limits<double>::infinity();
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
        const auto &model = poly_traj_opt_->vehicleModel();
        const auto limits = model->limits();
        const double floor_mps =
            poly_traj_opt_->dynamicsMinSpeedFloorUnits() * um_xy;
        const double vmax_pre = effectiveHandoffMaxMps();
        // The JUDGEMENT is mmp_vehicle_dynamics::speedConeVerdict — one
        // definition shared with every off-line harness, so none of them can
        // certify a state this gate would refuse. What stays here is the two
        // ROS-parameterised scalars above and the wording below.
        const Eigen::Vector3d vel_mps(vel_units.x() * um_xy,
                                      vel_units.y() * um_xy,
                                      vel_units.z() * um_z);
        const auto verdict = model->velocityVerdict(vel_mps, floor_mps, vmax_pre);
        const double vm = verdict.speed_mps;
        char buf[160];
        // [SPEED-BOUNDARY] see path_manager.h. Commanding exactly the floor
        // must pass whatever the direction; the unit round trip puts some
        // directions 2.8e-14 m/s under it — (1,1,0) and r5's own first-leg
        // aim (-18.3, -199.2, 0) among them.
        if (verdict.reject ==
            mmp_vehicle_dynamics::HandoffReject::SpeedBelowFloor) {
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
        const double vmax_mps = vmax_pre;
        double explicit_cap = 0.0;
        if (node_->has_parameter("planning/handoff_max_vel_mps"))
            node_->get_parameter("planning/handoff_max_vel_mps",
                                 explicit_cap);
        // [SPEED-BOUNDARY] INCLUSIVE at the cap, same rule as the floor.
        // Eight of the ten shipped missions state exactly
        // optimization/max_vel * unit = 200.0 m/s, and a strict > put every
        // one of them on the last bits of the direction normalization.
        if (verdict.reject ==
            mmp_vehicle_dynamics::HandoffReject::SpeedAboveMaximum) {
            const char *which =
                vmax_mps >= limits.speed_max_mps - 1e-9
                    ? "model maximum"
                    : (explicit_cap > 1e-9 ? "explicit handoff cap"
                                           : "default planning cap");
            snprintf(buf, sizeof buf,
                     "speed %.1f m/s above the effective maximum %.1f m/s "
                     "(%s)", vm, vmax_mps, which);
            return buf;
        }
        const double gamma = verdict.flight_path_angle_rad;
        if (verdict.reject ==
            mmp_vehicle_dynamics::HandoffReject::FlightPathOutsideCone) {
            snprintf(buf, sizeof buf,
                     "flight path angle %.1f deg outside the +/-%.1f deg "
                     "validity cone",
                     gamma * 180.0 / M_PI,
                     limits.flight_path_angle_max_rad * 180.0 / M_PI);
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
        const auto &model = poly_traj_opt_->vehicleModel();
        // Same shared judgement as the velocity half. The speed/cone part
        // re-runs here and has already passed above; what this call adds is
        // the inverse-dynamics and envelope verdict. NOT
        // "envelopeUtilization() <= 1.0": below the model activation speed
        // that comparison certifies everything (utilization is 0.0 there
        // while isWithinEnvelope is false), which is why the predicate and
        // not the ratio decides.
        const auto verdict = model->handoffVerdict(
            S.cwiseProduct(pos_units), S.cwiseProduct(vel_units),
            S.cwiseProduct(acc_units), poly_traj_opt_->dynamicsMinSpeedFloorUnits() * um_xy,
            effectiveHandoffMaxMps());
        char buf[160];
        if (verdict.reject ==
            mmp_vehicle_dynamics::HandoffReject::InverseDynamicsUndefined) {
            return "inverse dynamics undefined for this state — cannot be "
                   "certified";
        }
        if (verdict.reject ==
            mmp_vehicle_dynamics::HandoffReject::EnvelopeExceeded) {
            snprintf(buf, sizeof buf,
                     "flying this exact state demands %.0f%% of the %s limit",
                     100.0 * verdict.utilization,
                     mmp_vehicle_dynamics::envelopeLimitName(verdict.limit));
            return buf;
        }
        return {};
    }

    std::optional<PathManager::PreparedRoute> PathManager::prepareRoute(
        const DirectSolveContext &context, const StartHead &head,
        const std::vector<Eigen::Vector3d> &waypoints, bool junction_goal,
        const std::vector<Eigen::Vector3d> *route_override,
        const std::vector<double> *cap_ref_override,
        const RouteProvenance *provenance)
    {
        beginPlanningComputation();
        PreparedRoute prepared;
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
            return std::nullopt;
        }
        if (!start_pos.allFinite() || !start_vel.allFinite() ||
            !start_acc.allFinite()) {
            log_manager_->errorf(
                "[HEAD-POLICY] planGlobalTraj called with a non-finite start "
                "state — refusing");
            return std::nullopt;
        }
        log_manager_->infof("Planning global trajectory with %zu waypoints", waypoints.size());
        prepared.total_start = std::chrono::steady_clock::now();

        if (waypoints.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "planGlobalTraj: No waypoints provided!");
            return std::nullopt;
        }

        // Geometry preparation borrows the current terrain synchronously.
        // An absent callback preserves the no-terrain waypoint semantics.
        ElevationQuery elevation;
        if (environment_->terrain().valid) {
            elevation = [this](double x, double y) {
                return environment_->terrain().getElevation(x, y);
            };
        }
        auto resolved = resolveMissionWaypoints(
            waypoints, junction_goal, min_goal_agl_, elevation);
        prepared.absolute_waypoints = std::move(resolved.waypoints);
        const auto &wps = prepared.absolute_waypoints;
        for (const auto &change : resolved.altitude_changes) {
            const auto &wp = wps[change.index];
            log_manager_->infof(
                "[GOAL AGL] waypoint (%.1f, %.1f) z=%.2f AGL "
                "-> absolute %.2f (terrain %.2f + agl %.2f)",
                wp.x(), wp.y(), change.original_z, change.absolute_z,
                change.terrain_base, change.agl);
        }

        // Diagnostic guard: warn if the mission start or any waypoint falls
        // OUTSIDE the terrain bbox. Off-DEM points are treated as sea-level (0)
        // water by getElevation — legitimate for over-water legs. But after a
        // corridor/map publisher failure the TRANSIENT_LOCAL topic can still
        // serve a STALE map whose bbox no longer covers the mission, which
        // would silently plan terrain-less. We WARN (not refuse) so genuine
        // over-water missions still plan, but a stale-map swap is visible.
        if (environment_->terrain().valid) {
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

        const MissionBoundsResult bounds = computeMissionBounds(
            start_pos, wps, environment_->riskZones(),
            {map_ceiling_headroom_, environment_->verticalRatio()}, elevation);
        map_lower_bound_ = bounds.lower;
        map_upper_bound_ = bounds.upper;
        if (bounds.included_zone_count < environment_->riskZones().size()) {
            log_manager_->infof(
                "[BBOX] %zu/%zu risk zones excluded from map bounds "
                "(moat cannot reach the mission area)",
                environment_->riskZones().size() - bounds.included_zone_count,
                environment_->riskZones().size());
        }
        log_manager_->infof(
            "[BBOX] headroom=%.2f terrain_peak=%.2f zone_top=%.2f (%d zones, "
            "not covered) -> ceiling %.2f",
            map_ceiling_headroom_, bounds.max_terrain_z, bounds.max_zone_top,
            static_cast<int>(bounds.included_zone_count), map_upper_bound_.z());
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
        if (!environment_->ensureDistanceField(map_lower_bound_, map_upper_bound_)) {
            RCLCPP_ERROR(node_->get_logger(), "SDF build failed");
            return std::nullopt;
        }


        // The SDF is now ready — add any obstacles deferred before it existed
        // (no ESDF cache on a fresh run), so they make it into this first plan.
        flushPendingObstacles();

        // SDF sanity probe at start/goal.
        {
            float d_start = environment_->distanceField().getDistance(start_pos);
            float d_goal  = environment_->distanceField().getDistance(wps.back());
            log_manager_->infof("SDF probe: start=%.3f m, goal=%.3f m (margin=%.2f)",
                                d_start, d_goal, obstacle_clearance_);
        }


        // === STEP 2~3: front-end search + densification ===
        auto &clean_path = prepared.route.path;
        auto &cap_ref = prepared.route.cap;
        auto &edge_leg = prepared.route.edge_leg;
        if (route_override && route_override->size() >= 2) {
            // [CHAIN] Inherited committed route (see the header comment):
            // the optimizer re-solves this exact geometry; no re-litigation
            // of the route homotopy, the altitude band/cap tables inherit
            // the baseline's committed profile over the span.
            clean_path = *route_override;
            if (cap_ref_override) cap_ref = *cap_ref_override;
            prepared.raw_max_z = -1e9;
            for (const auto &p : clean_path)
                prepared.raw_max_z = std::max(prepared.raw_max_z, p.z());
            log_manager_->infof(
                "[CHAIN] inherited committed route: %zu vertices, cap_ref "
                "%zu, committed max z %.2f (front-end search skipped)",
                clean_path.size(), cap_ref.size(), prepared.raw_max_z);
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
        } else {
            auto profile = planFrontEnd(start_pos, wps,
                                         allowsTakeoffRelief(head.src), context);
            if (!profile) return std::nullopt;
            clean_path = std::move(profile->path);
            cap_ref = std::move(profile->cap_ref);
            edge_leg = std::move(profile->edge_leg);
            prepared.raw_max_z = profile->raw_max_z;
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
        } else {
            edge_leg.clear();
        }
        prepared.route.epoch = last_route_epoch_;
        return prepared;
    }

    std::optional<CommittedRoute> PathManager::planCommittedRoute(
        const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints)
    {
        auto prepared = prepareRoute(missionSolveContext(), head, waypoints, false, nullptr, nullptr,
                                      nullptr);
        if (!prepared) return std::nullopt;
        log_manager_->infof("[CHAIN-PAR] front-end only: %zu vertices "
                            "committed, optimization skipped",
                            prepared->route.path.size());
        prepared->route.front_end_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - prepared->total_start).count();
        return std::move(prepared->route);
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
        if (front_end_only) {
            auto prepared = prepareRoute(missionSolveContext(), head, waypoints, junction_goal,
                route_override, cap_ref_override, provenance);
            if (!prepared) return false;
            log_manager_->infof("[CHAIN-PAR] front-end only: %zu vertices "
                                "committed, optimization skipped",
                                prepared->route.path.size());
            return true;
        }
        auto product = computeGlobalTrajectory(head, waypoints, tail,
            junction_goal, route_override, cap_ref_override, provenance);
        if (!product) return false;
        PlanningComputation computed;
        computed.status = PlanResult::success();
        computed.recordProduct(std::move(*product));
        return applyPlanningComputation(
            sealPlanningComputation(std::move(computed))).applied;
    }

    PlanningComputation PathManager::sealPlanningComputation(
        PlanningComputation value) const
    {
        // Environment preparation may already have revoked an older flight.
        // Seal against the live execution owner AFTER calculation completes.
        return execution_->seal(std::move(value), environmentRevision());
    }

    ExecutionReceipt PathManager::applyPlanningComputation(
        const PlanningComputation &value)
    {
        const auto receipt = execution_->apply(value, environmentRevision());
        if (receipt.applied && value.invalidate_execution) clearTrajectoryViz();
        return receipt;
    }

    std::optional<PlannedTrajectory> PathManager::computeGlobalTrajectory(const StartHead &head,
                                     const std::vector<Eigen::Vector3d> &waypoints,
                                     const ego_planner::TailBoundary &tail,
                                     bool junction_goal,
                                     const std::vector<Eigen::Vector3d> *route_override,
                                     const std::vector<double> *cap_ref_override,
                                     const RouteProvenance *provenance)
    {
        return computeGlobalTrajectory(missionSolveContext(), head, waypoints,
            tail, junction_goal, route_override, cap_ref_override, provenance);
    }

    std::optional<PlannedTrajectory> PathManager::computeGlobalTrajectory(
                                     const DirectPlanSettings &settings, const StartHead &head,
                                     const std::vector<Eigen::Vector3d> &waypoints,
                                     const ego_planner::TailBoundary &tail,
                                     bool junction_goal,
                                     const std::vector<Eigen::Vector3d> *route_override,
                                     const std::vector<double> *cap_ref_override,
                                     const RouteProvenance *provenance)
    {
        std::unique_ptr<ego_planner::PolyTrajOptimizer> optimizer;
        DirectPlanConfig config;
        try {
            config = readDirectPlanConfig(settings.overrides);
            optimizer = configureOptimizer(config, settings.overrides);
            optimizer->setSwarmTrajs(&swarm_trajectories_);
        } catch (const std::exception &error) {
            throw DirectPlanConfigurationError(error.what());
        }
        // The searcher retains a borrowed visibility provider after prepare().
        // Whole-mission evaluation must see the mission field on every exit,
        // including exceptions, without resetting captured per-leg policies.
        struct MissionVisibilityBinding {
            decltype(searcher_) &searcher;
            std::function<double(size_t, const Eigen::Vector3d &)> visibility;
            ~MissionVisibilityBinding() { searcher.setRiskVisibility(std::move(visibility)); }
        } binding{searcher_, [this](size_t i, const Eigen::Vector3d &p) {
            return riskVisibilityValue(i, p);
        }};
        return computeGlobalTrajectory(DirectSolveContext{config, optimizer.get()},
            head, waypoints, tail, junction_goal, route_override, cap_ref_override, provenance);
    }

    std::optional<PlannedTrajectory> PathManager::computeGlobalTrajectory(
                                     const DirectSolveContext &context, const StartHead &head,
                                     const std::vector<Eigen::Vector3d> &waypoints,
                                     const ego_planner::TailBoundary &tail,
                                     bool junction_goal,
                                     const std::vector<Eigen::Vector3d> *route_override,
                                     const std::vector<double> *cap_ref_override,
                                     const RouteProvenance *provenance)
    {
        auto prepared = prepareRoute(context, head, waypoints, junction_goal,
                                      route_override, cap_ref_override,
                                      provenance);
        if (!prepared) return std::nullopt;
        const auto &start_pos = head.pos_u;
        const auto &start_vel = head.vel_u;
        const auto &start_acc = head.acc_u;
        auto &clean_path = prepared->route.path;
        const auto &cap_ref = prepared->route.cap;
        const auto &wps = prepared->absolute_waypoints;

        // [HEAD-POLICY] One implementation, in start_state.h. This block
        // and its twin in SegmentChainPlanner::planOverRoute were the same
        // two rules written twice, and they had already drifted — the
        // numeric-equality rule at the speed boundaries reached this copy
        // and not the other. The DECISION now has one owner; the log tags
        // stay here because the two callers report differently.
        //
        // The head arrives with its own provenance; nothing here derives it.
        const double v_floor = context.optimizer
            ? context.optimizer->dynamicsMinSpeedFloorUnits() : 0.0;
        const double um_xy_head = context.config.dynamics_unit_xy_m;
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
        auto product = optimizeStage(context, clean_path,
                                    start_pos, start_vel_eff, start_acc, wps,
                                    cap_ref, prepared->raw_max_z, tail, last_route_edge_leg_);

        auto t_total_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] === TOTAL planGlobalTraj: %.1f ms ===",
            std::chrono::duration<double, std::milli>(t_total_end - prepared->total_start).count());

        // Second half of the anti-fragmentation pair (see setTerrainData):
        // the FM2 fields are re-allocated INSIDE the plan, so the post-swap
        // trim can't see their old epochs — trim again now that the plan's
        // transient allocations are dead. ~ms on a GB heap vs a 0.5-3 s plan.
        malloc_trim(0);

        return product;
    }

bool PathManager::configureFrontEnd(RouteSearchProblem &problem, const DirectSolveContext &context)
{
    bindRiskZonesToSearcher();
    searcher_.setLogManager(log_manager_);
    searcher_.setSDF(&environment_->distanceField(), map_lower_bound_,
                     map_upper_bound_ - map_lower_bound_, environment_->voxelSize(), environment_->voxelZ());
    log_manager_->infof("[PM DBG] setRiskZones: %zu zones weight=%.3f",
        environment_->searchRiskZones().size(), risk_weight_);
    if (context.optimizer)
        bindRiskZonesToOptimizer(*context.optimizer, context.config.obstacle_clearance);

    problem = RouteSearchProblem{};
    problem.identity.context_id = nextRouteSearchIdentity();
    problem.identity.environment_revision = environmentRevision();
    problem.identity.zone_generation = environment_->zoneGeneration();
    problem.identity.policy_epoch = zone_policy_epoch_;
    problem.constraints.obstacle_clearance = obstacle_clearance_;
    problem.constraints.ground_height = ground_height_;
    problem.constraints.virtual_ceiling = virtual_ceil_height_;
    problem.constraints.dynamic_obstacle_margin = dyn_obstacle_margin_;
    problem.model = vehicle_model_override_ ? vehicle_model_override_
        : (context.optimizer && context.optimizer->dynamicsEnabled()
               ? context.optimizer->vehicleModel() : nullptr);
    {
        const double fpa_deg = context.config.flight_path_max_deg;
        // Explicit models own this limit. The legacy ROS path deliberately
        // reads the live parameter at each search, even before optimizer
        // initialization or after a runtime parameter update.
        const auto &model = vehicle_model_override_;
        problem.constraints.flight_path_angle_max_rad = model
            ? model->limits().flight_path_angle_max_rad
            : fpa_deg * M_PI / 180.0;
    }

    auto &environment = problem.environment;
    environment.distance_field = &environment_->distanceField();
    environment.lower = map_lower_bound_;
    environment.upper = map_upper_bound_;
    environment.voxel_xy = environment_->voxelSize();
    environment.voxel_z = environment_->voxelZ();
    environment.fine_grid_shape = environment_->distanceField().shape();
    environment.terrain_cell_u = environment_->terrain().valid ? environment_->terrain().resolution : 0.0;
    for (const auto &zone : environment_->searchRiskZones())
        environment.zones.push_back({zone.center, zone.reach, zone.peak, zone.vertical_reach});
    environment.elevation = [this](double x, double y) -> float {
        return environment_->terrain().valid ? environment_->terrain().getElevation(x, y)
            : -std::numeric_limits<float>::infinity();
    };
    environment.elevation_gradient =
        [this](double x, double y, float *h, float *gx, float *gy) {
            return environment_->terrain().getElevationAndGrad(x, y, h, gx, gy);
        };
    if (context.optimizer == poly_traj_opt_.get()) {
        environment.risk_visibility = [this](size_t i, const Eigen::Vector3d &p) {
            return riskVisibilityValue(i, p);
        };
    } else {
        environment.risk_visibility =
            [this, clearance = context.config.obstacle_clearance](size_t i, const Eigen::Vector3d &p) {
                return riskVisibilityValue(i, p, clearance);
            };
    }
    // This object still supplies canonical occupancy/risk queries even when
    // another backend generates the route. Its algorithm tuning is private
    // to the built-in adapter; an external backend needs no FM2/A* options.
    return builtin_route_search_->prepare(problem, !route_search_backend_);
}

void PathManager::publishFrontEndRoute(
    const std::vector<Eigen::Vector3d> &route, double search_ms, double clearance)
{
    if (!visualization_) return;
    log_manager_->infof("A* route: %zu waypoints (%.1f ms)",
        route.size(),
        search_ms);
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
    for (size_t ri = 0; ri < route.size(); ++ri) {
        const auto &p = route[ri];
        double survival = 1.0;
        std::string per_zone;
        const bool dump = route.size() <= kRiskDumpMax;
        for (size_t zi = 0; zi < environment_->riskZones().size(); ++zi) {
            const auto &tz = environment_->riskZones()[zi];
            const double q = riskEllipsoidRadius(tz, p);
            double moat = 0.0;
            double visibility = 0.0;
            if (q < 1.0) {
                visibility = riskVisibilityValue(zi, p, clearance);
                moat = tz.peak * mmp::risk::shape(q) * visibility;
            }
            survival *= (1.0 - std::min(moat, mmp::risk::kMoatCap));
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
    if (route.size() > kRiskDumpMax) {
        log_manager_->infof(
            "  A* route risk: %zu wp, max=%.3f mean=%.4f (per-wp dump skipped)",
            route.size(), dbg_risk_max,
            dbg_risk_sum / std::max<size_t>(1, route.size()));
    }

    visualization_->frontEndPath(visualization::makeFrontEndPath(
        route, rclcpp::Clock(RCL_ROS_TIME).now()));
    publishTubeMarker(visualization::TubeChannel::FrontEnd, route, "opt_path_drone_",
                      0.12f, 0.39f, 1.0f, 0.7f);
}

std::optional<RouteProfileResult> PathManager::planFrontEnd(
    const Eigen::Vector3d &start_pos,
    const std::vector<Eigen::Vector3d> &waypoints, bool takeoff_start,
    const DirectSolveContext &context)
{
    std::vector<Eigen::Vector3d> full_route;
    // [S13] policy epoch: dispositions are per-search state, so every
    // front-end run invalidates outstanding zone-policy snapshots; the
    // recorded generation says WHICH zone data this search saw.
    ++zone_policy_epoch_;
    zone_policy_epoch_generation_ = environment_->zoneGeneration();
    zone_policy_epoch_searches_ = 0;
    leg_policies_.clear();
    last_search_policy_ = RouteSearchPolicy{};
    last_search_policy_measured_ = false;

    const MissionRouteSearchRequest request{
        start_pos, waypoints, 1.0, execution_->current().drone_id,
        takeoff_start};
    // The backend only borrows this input during synchronous calls. Keep it
    // local so its model reference is released on every return/exception;
    // an idle route cache must not retain a previously replaced model.
    RouteSearchProblem problem;
    if (!configureFrontEnd(problem, context)) return std::nullopt;

    auto t_astar_start = std::chrono::steady_clock::now();
    bool leg_capture_voided = false;
    const RouteLegSearch search = [this, &problem](const RouteLegSearchRequest &leg) {
        problem.start = leg.start;
        problem.goal = leg.goal;
        problem.drone_id = leg.drone_id;
        problem.is_takeoff_leg = leg.is_takeoff_leg;
        problem.identity.request_id = nextRouteSearchIdentity();
        problem.identity.search_serial = zone_policy_epoch_searches_ + 1;
        problem.identity.leg = leg.leg;
        const auto backend = route_search_backend_ ? route_search_backend_
            : std::static_pointer_cast<RouteSearchBackend>(builtin_route_search_);
        RouteSearchResult result;
        if (route_search_backend_) {
            try {
                result = backend->search(problem);
            } catch (const std::exception &error) {
                result.status = RouteSearchStatus::BACKEND_FAILURE;
                result.detail = std::string("external backend exception: ") + error.what();
            } catch (...) {
                result.status = RouteSearchStatus::BACKEND_FAILURE;
                result.detail = "external backend threw an unknown exception";
            }
        } else {
            result = backend->search(problem);
        }
        last_search_policy_ = std::move(result.policy);
        last_search_policy_measured_ = routeSearchPolicyMeasured(problem, last_search_policy_) &&
            problem.identity.environment_revision == environmentRevision() &&
            problem.identity.zone_generation == environment_->zoneGeneration();
        if (!routeSearchGeometryUsable(result)) {
            last_search_policy_measured_ = false;
            if (!result.detail.empty())
                log_manager_->errorf("[ROUTE-SEARCH] %s", result.detail.c_str());
            return std::vector<Eigen::Vector3d>{};
        }
        return std::move(result.points);
    };
    // Capture live policy immediately after each actual search, including
    // failed attempts. The next leg must not overwrite the searcher buffers
    // before this observer has recorded or voided their attribution.
    const LegSearchObserver observe = [&](const RouteLegSearchRequest &leg,
                                           size_t point_count) {
        const size_t seg = leg.leg;
        ++zone_policy_epoch_searches_;

        log_manager_->infof("A* segment %zu: simple_path_size=%zu",
            seg, point_count);

        if (point_count < 2)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "A* failed for segment %zu: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                seg, leg.start.x(), leg.start.y(), leg.start.z(),
                leg.goal.x(), leg.goal.y(), leg.goal.z());
            // [LEG-POLICY] A leg that produced no path produced no policy
            // either. Everything captured so far described a route that is
            // now not going to exist, and the legs after this one will
            // never run — so the EPOCH is void, not merely incomplete. A
            // surviving prefix would be a set of snapshots for a plan that
            // failed, sitting behind an accessor whose only size check is
            // against a trajectory from some other plan.
            leg_policies_.clear();
            return;
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
    };
    auto searched = searchMissionRoute(request, search, observe);
    if (searched.status != MissionRouteSearchStatus::SUCCESS) return std::nullopt;
    full_route = std::move(searched.route.points);
    std::vector<size_t> route_leg = std::move(searched.route.edge_leg);

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
            if (environment_->distanceField().hasData() &&
                environment_->distanceField().getDistance(p) < obstacle_clearance_) return false;
            for (size_t zi = 0; zi < environment_->riskZones().size(); ++zi) {
                const auto &tz = environment_->riskZones()[zi];
                const bool corner_in =
                    riskEllipsoidRadius(tz, corner) < 1.0 &&
                    riskVisibilityValue(zi, corner, context.config.obstacle_clearance) > 0.5;
                const bool p_in =
                    riskEllipsoidRadius(tz, p) < 1.0 &&
                    riskVisibilityValue(zi, p, context.config.obstacle_clearance) > 0.5;
                if (p_in && !corner_in) return false;  // arc dips INTO a zone
            }
            return true;
        };

        const size_t original_points = full_route.size();
        TaggedRoute rounded = roundRouteCorners(
            {std::move(full_route), std::move(route_leg)},
            corner_fillet_radius_, sample_ok);
        log_manager_->infof("corner fillet R=%.1f: %zu -> %zu pts",
                            corner_fillet_radius_, original_points,
                            rounded.points.size());
        full_route = std::move(rounded.points);
        route_leg = std::move(rounded.edge_leg);
    }

    auto t_rrt_end = std::chrono::steady_clock::now();
    publishFrontEndRoute(full_route,
        std::chrono::duration<double, std::milli>(t_rrt_end - t_astar_start).count(),
        context.config.obstacle_clearance);

    // Publish the filleted front-end route above before denoising its
    // optimization pins. The value result preserves the pre-filter cap
    // reference and edge provenance through every subdivision.
    ElevationQuery elevation;
    if (environment_->terrain().valid) {
        elevation = [this](double x, double y) {
            return environment_->terrain().getElevation(x, y);
        };
    }
    const size_t input_tag_count = route_leg.size();
    RouteProfileResult profile = prepareRouteProfile(
        {std::move(full_route), std::move(route_leg)}, start_pos, waypoints,
        {length_per_piece_, obstacle_clearance_}, elevation, &environment_->distanceField());
    const auto &filtered_route = profile.filtered_route;
    const auto &clean_path = profile.path;
    if (profile.denoise.applied) {
        const auto &stats = profile.denoise;
        log_manager_->infof(
            "[Z-DENOISE] shortcut z: %zu/%zu vertices smoothed, range [%.2f, %.2f] -> [%.2f, %.2f]",
            stats.adjusted, filtered_route.size(), stats.min_before,
            stats.max_before, stats.min_after, stats.max_after);
    }
    if (!profile.input_tags_match) {
        log_manager_->errorf(
            "[LEG-POLICY] edge tags %zu do not match route %zu — route "
            "provenance dropped",
            input_tag_count, filtered_route.size());
    }
    log_manager_->infof(
        "A* shortcut %zu pts → sparse pieces %zu pts (max_seg %.1f)",
        filtered_route.size(), clean_path.size(), profile.max_segment_length);

    publishPipelineDebug(filtered_route, clean_path);

    if (clean_path.size() < 2) {
        log_manager_->errorf("clean_path too short");
        return std::nullopt;
    }

    return profile;
}

std::optional<PlannedTrajectory> PathManager::optimizeStage(
                                const DirectSolveContext &context, std::vector<Eigen::Vector3d> &clean_path,
                                const Eigen::Vector3d &start_pos,
                                const Eigen::Vector3d &start_vel,
                                const Eigen::Vector3d &start_acc,
                                const std::vector<Eigen::Vector3d> &waypoints,
                                const std::vector<double> &cap_ref,
                                double raw_max_z,
                                const ego_planner::TailBoundary &tail,
                                const std::vector<size_t> &edge_leg)
{
        // Stage 2 = trajectory optimization. The optimizer owns the MINCO
        // initial-trajectory build + L-BFGS; we only pass the front-end path
        // through solveDirectTrajectory/PolyTrajOptimizer::solve and return
        // owned results. Legacy optimizeFromPath is not a virtual backend seam.
        if (!context.optimizer) {
            log_manager_->errorf("Optimizer not initialized");
            return std::nullopt;
        }

        auto t_opt_start = std::chrono::steady_clock::now();

        double global_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        // Keep the direct solve's pre-denoise ceiling restoration distinct
        // from slice solving: cap_ref values do not expand this scalar band.
        const DirectAltitudeConfig altitude_config{
            raw_max_z, context.config.alt_cap_headroom, context.config.alt_floor_headroom,
            ground_height_, context.config.weight_altitude};
        const DirectAltitudeBand band = computeDirectAltitudeBand(
            clean_path, start_pos, waypoints.back(), altitude_config);
        if (band.path_max_z - band.path_max_z_denoised > 1e-6) {
            log_manager_->infof(
                "[ALT] cap reference restored to pre-denoise max: "
                "%.3f (denoised profile max %.3f)",
                band.path_max_z, band.path_max_z_denoised);
        }
        log_manager_->infof(
            "[ALT] optimizer z-band [%.2f, %.2f] (mission min - %.2f, "
            "path max %.2f + %.2f; terrain owned by gated cap + 0.30 band)",
            band.z_lo, band.z_hi, context.config.alt_floor_headroom, band.path_max_z,
            context.config.alt_cap_headroom);

        DirectTrajectoryRequest request;
        request.path = std::move(clean_path);
        request.waypoints = waypoints;
        request.cap_ref = cap_ref;
        request.edge_leg = edge_leg;
        request.head_pos = start_pos;
        request.head_vel = start_vel;
        request.head_acc = start_acc;
        request.tail = tail;
        request.band = band;
        request.max_vel = max_vel_;
        // Pass 1 committed full avoidance: retain all zone barriers.
        request.suppress_crossing_exempt = suppressCrossingExemptNow();
        last_piece_leg_.clear();
        DirectTrajectoryResult solved =
            solveDirectTrajectory(*context.optimizer, std::move(request));
        // optimizeFromPath may insert points before it fails. Retain those
        // diagnostics and the exact piece mapping on both result paths.
        clean_path = std::move(solved.optimizer_path);
        last_piece_leg_ = std::move(solved.piece_leg);
        if (!solved.hasTrajectory()) {
            log_manager_->errorf("Trajectory optimization failed");
            return std::nullopt;
        }
        poly_traj::Trajectory global_traj = std::move(solved.seed);
        poly_traj::Trajectory local_traj = std::move(solved.trajectory);

        // Local traj start_time is the trajectory-following reference; stamp it
        // after optimization (matches pre-refactor behavior).
        double local_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

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
        publishTrajRisk(local_traj, context.config.obstacle_clearance);
        publishRiskProfile(local_traj, context.config.obstacle_clearance);
        // Tube renders of the flown (optimized) and seed (global) trajectories.
        // Reference trajectory: pale + translucent, a faint "tunnel" the
        // cursor sphere and the flown path pass through.
        publishTrajTube(local_traj, visualization::TubeChannel::Optimized, "opt_path_drone_",
                        0.55f, 0.72f, 1.0f, 0.22f);
        publishTrajTube(global_traj, visualization::TubeChannel::Reference, "global_path_drone_",
                        0.59f, 0.71f, 1.0f, 0.85f);
        // [CHAIN-VIZ] every ordinary solve clears the per-segment channel;
        // a chained plan repaints it AFTER its last segment solve.
        publishChainSegmentsViz({}, {});

        auto t_opt_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] trajectory optimization: %.1f ms, duration=%.3f max_vel=%.3f",
            std::chrono::duration<double, std::milli>(t_opt_end - t_opt_start).count(),
            local_traj.getTotalDuration(), local_traj.getMaxVelRate());

        return PlannedTrajectory{std::move(local_traj), std::move(global_traj),
                                 local_time, global_time};
}

// Existing callers keep the bool/out-parameter contract. Computation and its
// owned result live in solveRouteSlice; this adapter only snapshots settings
// and transfers a successful trajectory to the caller.
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
    if (!out) return false;
    RouteSliceResult result = solveRouteSlice(
        opt, {std::move(slice), std::move(cap), head_pos, head_vel, head_acc,
              goal_pos, tail, suppress_crossing_exempt}, sliceSolveConfig());
    if (!result.hasTrajectory()) return false;
    *out = std::move(result.trajectory);
    return true;
}

// [CHAIN-VIZ] see the header comment. Tube sampling matches
// publishTrajTube (0.1 s + exact endpoint); junction spheres are 3x the
// tube girth with a floating J<n> label so the seams read at map scale.
void PathManager::publishChainSegmentsViz(
    const std::vector<poly_traj::Trajectory> &runs,
    const std::vector<Eigen::Vector3d> &junctions,
    const poly_traj::Trajectory *terminal)
{
    if (!visualization_) return;
    visualization_->chain(visualization::makeChain(
        runs, junctions, terminal, path_scale_, visualizationStamp()));
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
    publishTrajTube(opt, visualization::TubeChannel::Optimized, "opt_path_drone_",
                    0.55f, 0.72f, 1.0f, 0.22f);
    publishTrajTube(reference, visualization::TubeChannel::Reference, "global_path_drone_",
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

    if (!execution_->adoptEmergencyTrajectory(
            stopMJO.getTraj(), rclcpp::Clock(RCL_ROS_TIME).now().seconds()).applied)
        return false;

    // The stop trajectory replaces the previous route, so every latched
    // route-derived diagnostic must change with it. Otherwise the altitude
    // panel can combine a stationary marker with the old front-end path
    // and old risk colors.
    if (visualization_) {
        visualization_->frontEndPath(visualization::makeFrontEndPath({}, node_->now()));
    }
    publishTrajRisk(execution_->current().traj);
    publishRiskProfile(execution_->current().traj);
    publishTrajTube(execution_->current().traj, visualization::TubeChannel::Optimized, "opt_path_drone_",
                    0.55f, 0.72f, 1.0f, 0.22f);

    RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP executed at position (%.2f, %.2f, %.2f)",
                stop_pos.x(), stop_pos.y(), stop_pos.z());
    if (log_manager_) {
        log_manager_->warnf("EMERGENCY STOP executed at position (%.2f, %.2f, %.2f)",
                           stop_pos.x(), stop_pos.y(), stop_pos.z());
    }

    return true;
}



double PathManager::riskEllipsoidRadius(const RiskZone &zone, const Eigen::Vector3d &pos) const
{
    return environment_->ellipsoidRadius(zone, pos);
}

double PathManager::riskZoneValue(size_t zone_index, const Eigen::Vector3d &pos) const
{
    return riskZoneValue(zone_index, pos, opt_obstacle_clearance_);
}

double PathManager::riskZoneValue(size_t zone_index, const Eigen::Vector3d &pos,
                                  double clearance) const
{
    return environment_->riskValue(zone_index, pos, clearance);
}

double PathManager::riskShadowCeiling(size_t zone_index, const Eigen::Vector3d &pos) const
{
    return environment_->shadowCeiling(zone_index, pos);
}

double PathManager::riskGroundedZ(const Eigen::Vector3d &pos, double *dzeff_dz) const
{
    return environment_->groundedZ(pos, opt_obstacle_clearance_, dzeff_dz);
}

double PathManager::riskVisibilityValue(size_t zone_index, const Eigen::Vector3d &pos) const
{
    return riskVisibilityValue(zone_index, pos, opt_obstacle_clearance_);
}

double PathManager::riskVisibilityValue(size_t zone_index, const Eigen::Vector3d &pos,
                                        double clearance) const
{
    return environment_->visibilityValue(zone_index, pos, clearance);
}

double PathManager::riskVisibility(size_t zone_index, const Eigen::Vector3d &pos,
                                   Eigen::Vector3d *grad) const
{
    return riskVisibility(zone_index, pos, grad, opt_obstacle_clearance_);
}

double PathManager::riskVisibility(size_t zone_index, const Eigen::Vector3d &pos,
                                   Eigen::Vector3d *grad, double clearance) const
{
    return environment_->visibility(zone_index, pos, clearance, grad);
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
visualization::Stamp PathManager::visualizationStamp() const
{
    return [this]() -> builtin_interfaces::msg::Time { return node_->now(); };
}

visualization::RiskView PathManager::riskVisualizationView() const
{
    return riskVisualizationView(opt_obstacle_clearance_);
}

visualization::RiskView PathManager::riskVisualizationView(double clearance) const
{
    return {environment_->riskZones(),
        {environment_->terrain().valid, environment_->terrain().resolution,
         [this](double x, double y) { return environment_->terrain().getElevation(x, y); },
         [this](double x, double y, float *h, float *dx, float *dy) {
             return environment_->terrain().getElevationAndGrad(x, y, h, dx, dy);
         }},
        [this, clearance](size_t i, const Eigen::Vector3d &p) { return riskZoneValue(i, p, clearance); },
        [this, clearance](size_t i, const Eigen::Vector3d &p) { return riskVisibilityValue(i, p, clearance); },
        [this](size_t i, const Eigen::Vector3d &p) { return riskShadowCeiling(i, p); }};
}

visualization::RiskSettings PathManager::riskVisualizationSettings() const
{
    return riskVisualizationSettings(opt_obstacle_clearance_);
}

visualization::RiskSettings PathManager::riskVisualizationSettings(double clearance) const
{
    visualization::RiskSettings settings;
    settings.vertical_ratio = environment_->verticalRatio();
    settings.softness = environment_->maskSoftness();
    settings.threshold = risk_mask_viz_threshold_;
    settings.obstacle_clearance = clearance;
    settings.grounded = environment_->groundedRisk();
    settings.max_velocity = max_vel_;
    settings.step = risk_mask_viz_step_;
    settings.slice_offset = risk_mask_viz_slice_offset_;
    settings.agl = risk_mask_viz_agl_;
    settings.volume_alpha = risk_mask_viz_volume_alpha_;
    settings.contours = risk_mask_viz_contours_;
    settings.mode = risk_mask_viz_mode_;
    settings.heatmap_agl_max = risk_heatmap_agl_max_;
    settings.heatmap_offset = risk_heatmap_offset_;
    settings.heatmap_preview_lift = risk_heatmap_preview_lift_;
    settings.heatmap_max_dim = risk_heatmap_max_dim_;
    settings.heatmap_safe_transparent = risk_heatmap_safe_transparent_;
    return settings;
}

void PathManager::publishPipelineDebug(
    const std::vector<Eigen::Vector3d> &shortcut_route,
    const std::vector<Eigen::Vector3d> &inner_points)
{
    if (!visualization_ || !visualization_->pipelineEnabled()) return;
    auto result = visualization::makePipeline(shortcut_route, inner_points,
                                              visualizationStamp());
    visualization_->pipeline(result.message);
    if (log_manager_ && !result.diagnostic.empty())
        log_manager_->infof("%s", result.diagnostic.c_str());
}

void PathManager::publishTubeMarker(
    visualization::TubeChannel channel,
    const std::vector<Eigen::Vector3d> &pts, const std::string &ns_prefix,
    float r, float g, float b, float a)
{
    if (!visualization_) return;
    auto message = visualization::makeTube(pts, execution_->current().drone_id,
        path_scale_, ns_prefix, r, g, b, a, visualizationStamp());
    if (message) visualization_->tube(channel, *message);
}

void PathManager::clearTrajectoryViz()
{
    if (!visualization_) return;
    visualization_->clearTrajectory(visualization::makeTrajectoryClear());
}

void PathManager::publishTrajTube(
    const poly_traj::Trajectory &traj, visualization::TubeChannel channel,
    const std::string &ns_prefix, float r, float g, float b, float a)
{
    if (!visualization_) return;
    auto message = visualization::makeTrajectoryTube(traj,
        execution_->current().drone_id, path_scale_, ns_prefix, r, g, b, a,
        visualizationStamp());
    if (message) visualization_->tube(channel, *message);
}

void PathManager::publishTrajRisk(const poly_traj::Trajectory &traj)
{
    publishTrajRisk(traj, opt_obstacle_clearance_);
}

void PathManager::publishTrajRisk(const poly_traj::Trajectory &traj, double clearance)
{
    if (!visualization_) return;
    auto result = visualization::makeTrajectoryRisk(
        traj, riskVisualizationView(clearance), visualizationStamp());
    if (log_manager_ && !result.diagnostic.empty())
        log_manager_->infof("%s", result.diagnostic.c_str());
    visualization_->trajectoryRisk(result.message);
}



// [RISK-PROFILE] Version 3 is self-describing, binds data to source xyz, and
// preserves every zone's
// separate visible vertical interval, so disjoint/overlapping zones cannot be
// painted as one false union band. The risk scalar is evaluated at the actual
// 3-D trajectory sample with the exact same OR-combination as /viz/traj_risk.
void PathManager::publishRiskProfile(const poly_traj::Trajectory &traj)
{
    publishRiskProfile(traj, opt_obstacle_clearance_);
}

void PathManager::publishRiskProfile(const poly_traj::Trajectory &traj, double clearance)
{
    if (!visualization_) return;
    auto result = visualization::makeRiskProfile(
        traj, riskVisualizationView(clearance), riskVisualizationSettings(clearance));
    visualization_->riskProfile(result.message);
    if (log_manager_ && !result.diagnostic.empty())
        log_manager_->infof("%s", result.diagnostic.c_str());
}

// Safe corridors are rendered POSITIVELY instead of being the absence of
// red. Rendering via grid_map_rviz_plugin reuses the terrain-mesh pipeline
// (lighting, one mesh, no alpha-sorted marker soup), which is the point of
// this mode.
void PathManager::publishRiskHeatmap()
{
    if (!visualization_) return;
    auto result = visualization::makeRiskHeatmap(
        riskVisualizationView(), riskVisualizationSettings(), visualizationStamp());
    visualization_->riskHeatmap(result.message);
    if (log_manager_ && !result.diagnostic.empty())
        log_manager_->infof("%s", result.diagnostic.c_str());
}

// One MarkerArray per refresh is the entire risk field. Element 0 is a
// DELETEALL (RViz honors it inside an array, ahead of the ADDs that follow —
// the same technique publishDynamicObstacles uses), so a repaint is atomic and
// the latched sample a late joiner receives is never a partial field. An empty
// zone set therefore publishes an array holding only that DELETEALL.
void PathManager::publishEffectiveRiskField()
{
    if (!visualization_) return;
    // Preserve the existing streaming order; LOS masks were already rebuilt
    // by the caller and remain active even when this connection is absent.
    publishRiskHeatmap();
    visualization_->riskField(visualization::makeRiskField(
        riskVisualizationView(), riskVisualizationSettings(), visualizationStamp()));
}

TerrainUpdate PathManager::setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr &msg)
{
    const auto update = environment_->ingestTerrain(msg);
    if (!update.accepted()) return update;
    // The primary optimizer borrows the owner, but its DEM pitch is a value.
    if (poly_traj_opt_) {
        poly_traj_opt_->setTerrainHeightmap(
            [this](double x, double y) -> float {
                return environment_->terrain().valid ? environment_->terrain().getElevation(x, y)
                    : -std::numeric_limits<float>::infinity();
            }, environment_->terrain().valid ? environment_->terrain().resolution : 0.0);
    }
    if (update.obstacles_cleared) publishDynamicObstacles();
    if (update.kind == TerrainUpdateKind::REPLACED &&
        environment_->ensureTerrainDistanceField()) {
        // Installation and revalidation precede the risk-mask refresh, as in
        // the original ingestion path. The environment never reads execution.
        flushPendingObstacles();
    }
    environment_->refreshRisk();
    publishEffectiveRiskField();
    if (execution_->current().traj_id > 0 && execution_->current().duration > 0.0 &&
        std::isfinite(execution_->current().duration)) {
        publishTrajRisk(execution_->current().traj);
        publishRiskProfile(execution_->current().traj);
    }
    if (update.kind == TerrainUpdateKind::CLEARED)
        log_manager_->infof(update.had_terrain
            ? "[TERRAIN] cleared previous DEM (world=none)"
            : "[TERRAIN] empty world confirmed (no DEM loaded)");
    return update;
}

const PathManager::ObstacleMeshInfo& PathManager::meshFor(const std::string& model) const
{
    auto it = mesh_catalog_.find(model.empty() ? std::string("building") : model);
    if (it == mesh_catalog_.end()) it = mesh_catalog_.find("building");
    return it->second;
}



int PathManager::addDynamicSphere(const Eigen::Vector3d &center, double radius,
                                  const std::string &model)
{
    const int id = environment_->addSphere(center, radius, model);
    if (id == -2) execution_->invalidatePending();  // queued inputs also stale computations
    if (id >= 0) publishDynamicObstacles();
    return id;
}

int PathManager::addDynamicBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size,
                               const std::string &model)
{
    const int id = environment_->addBox(center, size, model);
    if (id == -2) execution_->invalidatePending();  // queued inputs also stale computations
    if (id >= 0) publishDynamicObstacles();
    return id;
}

// [ENV-CHANGE] see the header. Same occupancy predicate the front end plans
// against, so "the route was clear when planned" and "the remainder is clear
// now" are the same question asked twice, not two different standards.
bool PathManager::revalidateStoredTrajectory(double t_from,
                                             Eigen::Vector3d *hit,
                                             EnvChangeReason reason)
{
    const auto &lt = execution_->current();
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
    // The execution owner captures the withdrawn identity and notifies
    // before clearing the slot. Detection lives here; the ROS adapter owns
    // publication and state-machine recovery.
    execution_->revoke(reason, bad, t0, environmentRevision());
    clearTrajectoryViz();
    return false;
}

void PathManager::clearDynamicObstacles()
{
    environment_->clearObstacles();
    publishDynamicObstacles();
}

// Query the active zone content; the caller decides whether a replacement
// requires execution withdrawal under the existing fail-closed policy.
uint64_t PathManager::riskZoneFingerprint() const
{
    return environment_->riskZoneFingerprint();
}

void PathManager::invalidateStoredTrajectoryForEnvChange()
{
    execution_->revoke(EnvChangeReason::POLICY_UNEVALUATED,
                       Eigen::Vector3d::Zero(), 0.0, environmentRevision());
    clearTrajectoryViz();
}

void PathManager::revalidateAfterEnvChange(const char *what,
                                           EnvChangeReason reason)
{
    const auto &lt = execution_->current();
    if (!(lt.duration > 0.0)) return;
    const double t_cur = (lt.start_time > 0.0)
                             ? rclcpp::Clock(RCL_ROS_TIME).now().seconds() -
                                   lt.start_time
                             : 0.0;
    Eigen::Vector3d hit;
    if (!revalidateStoredTrajectory(t_cur, &hit, reason)) {
        // revoke() has already notified any registered observer. In ReplanFSM
        // that observer publishes ABORT (when its publisher is available) and
        // enters WAIT_EXTERNAL_RECOVERY. Without that adapter only the local
        // execution slot is withdrawn. The legacy log text below is retained
        // for compatibility; its "open decision" wording predates the observer.
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
    // Deferred inputs first become collision obstacles once an SDF exists.
    // Revalidate after this batch; ordinary environment setter calls do not
    // all perform execution revalidation.
    const auto pending = environment_->takePendingObstacles();
    if (pending.empty()) return;
    for (const auto &p : pending) {
        if (p.is_box) addDynamicBox(p.center, p.size, p.model);
        else addDynamicSphere(p.center, p.radius, p.model);
    }
    log_manager_->infof("Flushed %zu deferred dynamic obstacle(s) after SDF ready",
                        pending.size());
    revalidateAfterEnvChange("a deferred obstacle batch",
                             EnvChangeReason::OBSTACLE_BLOCKED);
}

bool PathManager::captureLegPolicySnapshot(size_t leg, uint64_t search_serial,
                                           LegPolicySnapshot *out) const
{
    if (!out) return false;
    // Capture the owned policy returned by THIS invocation. The admission
    // check already matched request identity, ordered zones and diagnostics.
    // A backend without canonical policy remains unmeasured; the default
    // adapter only attests the existing FM2 three-pass (or no-zone case).
    // This per-leg capture deliberately does not require a single-leg epoch.
    const int pass = zoneAvoidPassNow();
    const auto &nb = last_search_policy_.endpoint_exempt;
    const auto &so = last_search_policy_.soft_override;
    const bool ok =
        (environment_->riskZones().empty() || (last_search_policy_measured_ &&
          last_search_policy_.identity.leg == leg &&
          last_search_policy_.identity.search_serial == search_serial)) &&
        zone_policy_epoch_generation_ == environment_->zoneGeneration();
    if (!ok) {
        if (log_manager_)
            log_manager_->warnf(
                "[LEG-POLICY] leg %zu NOT capturable (pass %d, %zu zones, "
                "no-barrier %zu, soft-override %zu, search generation %lu vs "
                "data %lu) — no snapshot rather than a wrong one",
                leg, pass, environment_->riskZones().size(), nb.size(), so.size(),
                static_cast<unsigned long>(zone_policy_epoch_generation_),
                static_cast<unsigned long>(environment_->zoneGeneration()));
        return false;
    }
    out->leg = leg;
    out->search_serial = search_serial;
    out->policy.generation = environment_->zoneGeneration();
    out->policy.epoch = zone_policy_epoch_;
    // Valid PER LEG: this leg's search really did produce this policy.
    out->policy.valid = true;
    out->policy.zones.clear();
    out->policy.zones.reserve(environment_->riskZones().size());
    for (size_t i = 0; i < environment_->riskZones().size(); ++i) {
        ZonePolicySnapshot::Entry e;
        e.zone = environment_->riskZones()[i];
        e.disposition = last_search_policy_measured_ && i < last_search_policy_.dispositions.size()
            ? last_search_policy_.dispositions[i] : ZoneDisposition::HARD_AVOID;
        out->policy.zones.push_back(e);
    }
    return true;
}

PathManager::ZonePolicySnapshot PathManager::zonePolicySnapshot() const
{
    ZonePolicySnapshot snap;
    snap.generation = environment_->zoneGeneration();
    snap.epoch = zone_policy_epoch_;
    const int pass = zoneAvoidPassNow();
    // A plan-wide answer requires one measured search on current data.
    // Multi-leg epochs still need their separately captured leg policies;
    // an unknown external policy must never borrow the canonical searcher's
    // last FM2 buffers. Empty zones retain the existing trivial verdict.
    snap.valid = (environment_->riskZones().empty() ||
                  (last_search_policy_measured_ && zone_policy_epoch_searches_ == 1)) &&
                 zone_policy_epoch_generation_ == environment_->zoneGeneration();
    if (!snap.valid && log_manager_) {
        log_manager_->warnf(
            "[ZONE-POLICY] snapshot INVALID (pass %d, %zu zones, %lu leg "
            "searches this epoch, search generation %lu vs data %lu) — "
            "contract 2 requires ONE measured single-goal search on current data",
            pass, environment_->riskZones().size(),
            static_cast<unsigned long>(zone_policy_epoch_searches_),
            static_cast<unsigned long>(zone_policy_epoch_generation_),
            static_cast<unsigned long>(environment_->zoneGeneration()));
    }
    snap.zones.reserve(environment_->riskZones().size());
    for (size_t i = 0; i < environment_->riskZones().size(); ++i) {
        ZonePolicySnapshot::Entry e;
        e.zone = environment_->riskZones()[i];
        e.disposition = last_search_policy_measured_ && i < last_search_policy_.dispositions.size()
            ? last_search_policy_.dispositions[i] : ZoneDisposition::HARD_AVOID;
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
    if (snap.generation != environment_->zoneGeneration()) return false;
    if (snap.epoch != zone_policy_epoch_) return false;
    if (idx >= snap.zones.size() || idx >= environment_->riskZones().size()) return false;
    const RiskZone &live = environment_->riskZones()[idx];
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

void PathManager::bindRiskZonesToOptimizer()
{
    // Extracted from planFrontEnd for the same reason as the searcher bind:
    // the optimizer only learned about a zone update at the next plan, so
    // nothing could read what the BACK END prices without running a solve.
    if (poly_traj_opt_) bindRiskZonesToOptimizer(*poly_traj_opt_, opt_obstacle_clearance_);
}

void PathManager::bindRiskZonesToOptimizer(ego_planner::PolyTrajOptimizer &optimizer,
                                          double clearance)
{
    ego_planner::OptimizerRiskEnvironment environment;
    environment.zones.reserve(environment_->riskZones().size());
    for (const auto &tz : environment_->riskZones()) {
        ego_planner::RiskZone oz;
        oz.center = tz.center;
        oz.reach = tz.reach;
        oz.peak = tz.peak;
        oz.vertical_reach = tz.reach * environment_->verticalRatio();
        environment.zones.push_back(oz);
    }
    environment.visibility =
        [this, clearance](size_t zi, const Eigen::Vector3d &p,
               Eigen::Vector3d *grad) -> double {
            return riskVisibility(zi, p, grad, clearance);
        };
    environment.shadow_ceiling =
        [this](size_t zi, const Eigen::Vector3d &p) -> double {
            return riskShadowCeiling(zi, p);
        };
    optimizer.setRiskEnvironment(std::move(environment));
}

void PathManager::bindRiskZonesToSearcher()
{
    environment_->refreshSearchRiskZones();
    const auto &zones = environment_->searchRiskZones();
    searcher_.setRiskZones(zones.empty() ? nullptr : &zones);
    searcher_.setRiskVisibility(
        [this](size_t zi, const Eigen::Vector3d &p) -> double {
            return riskVisibilityValue(zi, p);
        });
}

bool PathManager::setRiskZonesRuntime(const std::vector<RiskZone> &zones)
{
    if (!environment_->setRiskZones(zones)) return false;
    publishEffectiveRiskField();
    bindRiskZonesToSearcher();
    bindRiskZonesToOptimizer();
    // Avoid leaving the altitude panel and colored trajectory latched to the
    // previous zone set while waiting for another plan request. Empty zones
    // intentionally flow through these publishers as clear/delete messages.
    if (execution_->current().traj_id > 0 &&
        execution_->current().duration > 0.0 &&
        std::isfinite(execution_->current().duration)) {
        publishTrajRisk(execution_->current().traj);
        publishRiskProfile(execution_->current().traj);
    }
    return true;
}

void PathManager::publishDynamicObstacles()
{
    if (!visualization_) return;
    std::vector<visualization::Obstacle> obstacles;
    obstacles.reserve(environment_->obstacles().size());
    for (const auto &patch : environment_->obstacles()) {
        const auto &mesh = meshFor(patch.model);
        visualization::Obstacle obstacle;
        obstacle.id = patch.id;
        obstacle.center = patch.center;
        obstacle.size = patch.size;
        obstacle.is_box = patch.is_box;
        obstacle.yaw = patch.yaw;
        obstacle.mesh_resource = mesh.resource;
        obstacle.mesh_native_size = mesh.native_size;
        obstacle.mesh_scale = mesh.viz_scale;
        obstacles.push_back(std::move(obstacle));
    }
    visualization_->dynamicObstacles(visualization::makeDynamicObstacles(
        obstacles, obstacle_mesh_height_, visualizationStamp()));
}

// Voxelize terrain + geometry obstacles into an occupancy grid and build ESDF.
// Risk zones are NOT included: they are handled as soft cost elsewhere.


bool PathManager::computeTerrainBBox(Eigen::Vector3d *lo, Eigen::Vector3d *hi)
{
    return environment_->terrainBounds(lo, hi);
}

} // namespace path_manager
