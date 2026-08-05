#include "path_manager/replan_fsm.h"
#include <cmath>
#include <sys/resource.h>
#include <numeric>
using namespace std::chrono_literals;

namespace path_manager {

ReplanFSM::ReplanFSM(rclcpp::Node::SharedPtr node)
    : node_(node),
      exec_state_(FSM_EXEC_STATE::INIT),
      have_target_(false),
      have_local_traj_(false),
      have_recv_pre_agent_(false),
      drone_id_(0),
      flag_escape_emergency_(false),
      num_drones_(4),
      current_formation_type_("square"),
      current_formation_scale_(2.0),
      last_received_sequence_(-1),
      current_mission_id_(""),
      next_mission_id_(""),
      is_final_mission_(false)
    {
        log_manager_ = std::make_unique<swarm_formation::LogManager>(
            node->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);

    node_->declare_parameter("enable_debug_logs", false);
    node_->get_parameter("enable_debug_logs", enable_debug_logs_);

    node_->declare_parameter("enable_lbfgs_detail_logs", false);
    // Note: enable_lbfgs_detail_logs will be read by PolyTrajOptimizer::setParam()

    node_->declare_parameter("drone_id", 0);
    node_->get_parameter("drone_id", drone_id_);
    FSM_LOG_INFO("Starting ReplanFSM for drone_id: %d", drone_id_);

    // Formation manager parameters
    node_->declare_parameter("num_drones", 4);
    node_->declare_parameter("formation_type", "square");
    node_->declare_parameter("formation_scale", 2.0);
    node_->declare_parameter("formation_center_x", 80.0);
    node_->declare_parameter("formation_center_y", -1.5);
    node_->declare_parameter("formation_center_z", 0.0);
    
    node_->get_parameter("num_drones", num_drones_);
    node_->get_parameter("formation_type", current_formation_type_);
    node_->get_parameter("formation_scale", current_formation_scale_);
    
    double center_x, center_y, center_z;
    node_->get_parameter("formation_center_x", center_x);
    node_->get_parameter("formation_center_y", center_y);
    node_->get_parameter("formation_center_z", center_z);
    current_formation_center_ = Eigen::Vector3d(center_x, center_y, center_z);

    node_->declare_parameter("enable_waypoint_markers", true);
    node_->get_parameter("enable_waypoint_markers", enable_waypoint_markers_);
    FSM_LOG_INFO("enable_waypoint_markers: %s", enable_waypoint_markers_ ? "true" : "false");

    node_->declare_parameter("enable_global_trajectory_pub", true);
    node_->get_parameter("enable_global_trajectory_pub", enable_global_trajectory_pub_);
    FSM_LOG_INFO("enable_global_trajectory_pub: %s", enable_global_trajectory_pub_ ? "true" : "false");

    // --- TEST: initial-state injection (vel/acc boundary condition demo) ---
    node_->declare_parameter("test/inject_init_state", false);
    node_->get_parameter("test/inject_init_state", inject_init_state_);

    std::vector<double> init_vel_vec{0.0, 0.0, 0.0};
    std::vector<double> init_acc_vec{0.0, 0.0, 0.0};
    node_->declare_parameter("test/init_vel", init_vel_vec);
    node_->declare_parameter("test/init_acc", init_acc_vec);
    node_->get_parameter("test/init_vel", init_vel_vec);
    node_->get_parameter("test/init_acc", init_acc_vec);

    inject_init_vel_.setZero();
    inject_init_acc_.setZero();
    if (init_vel_vec.size() >= 3)
        inject_init_vel_ = Eigen::Vector3d(init_vel_vec[0], init_vel_vec[1], init_vel_vec[2]);
    if (init_acc_vec.size() >= 3)
        inject_init_acc_ = Eigen::Vector3d(init_acc_vec[0], init_acc_vec[1], init_acc_vec[2]);

    node_->declare_parameter("manager/initial_speed_unit_m", 100.0);
    node_->get_parameter("manager/initial_speed_unit_m", initial_speed_unit_m_);
    if (initial_speed_unit_m_ <= 0.0) {
        RCLCPP_WARN(node_->get_logger(),
                    "manager/initial_speed_unit_m must be positive; using 100 m");
        initial_speed_unit_m_ = 100.0;
    }

    if (inject_init_state_) {
        FSM_LOG_WARN("[TEST] inject_init_state ENABLED: init vel=(%.2f,%.2f,%.2f) acc=(%.2f,%.2f,%.2f)",
                     inject_init_vel_(0), inject_init_vel_(1), inject_init_vel_(2),
                     inject_init_acc_(0), inject_init_acc_(1), inject_init_acc_(2));
    }


    // Start position will be received from TrajectoryCommand message
    // Initialize with zero until we receive the command
    RCLCPP_INFO(node_->get_logger(), "ReplanFSM parameters:");
    log_manager_->infof("ReplanFSM parameters:");
    RCLCPP_INFO(node_->get_logger(), "  Start position will be set from TrajectoryCommand");
    log_manager_->infof("  Start position will be set from TrajectoryCommand");
    RCLCPP_INFO(node_->get_logger(), "  Waiting for trajectory command...");
    log_manager_->infof("  Waiting for trajectory command...");

    start_pt_ = Eigen::Vector3d::Zero();
    current_pos_ = Eigen::Vector3d::Zero();  // Updated from TrajectoryCommand
    current_vel_ = Eigen::Vector3d::Zero();  // Initialize velocity to zero
    end_pt_ = Eigen::Vector3d::Zero();  // Initialize end_pt_ to avoid uninitialized access
    have_target_ = false;  // Wait for trajectory command
    start_position_received_ = false;  // Flag to track if we received start position
    
    RCLCPP_INFO(node_->get_logger(), "Initial position set to: (%.2f, %.2f, %.2f)", 
                current_pos_(0), current_pos_(1), current_pos_(2));
    log_manager_->infof("Initial position set to: (%.2f, %.2f, %.2f)", 
                current_pos_(0), current_pos_(1), current_pos_(2));

    // Initialize PathManager in constructor to avoid nullptr access
    path_manager_ = std::make_shared<PathManager>(node_);

    RCLCPP_INFO(node_->get_logger(), "PathManager initialized, waiting for trajectory command");
    log_manager_->infof("PathManager initialized, waiting for trajectory command");

    // [CHAIN] Stage-1 segment-chained planning experiment (default off).
    node_->declare_parameter("chain/enable", false);
    node_->get_parameter("chain/enable", chain_enable_);
    int chain_segments = 3;
    node_->declare_parameter("chain/segments", 3);
    node_->get_parameter("chain/segments", chain_segments);
    bool chain_inherit_route = true;
    node_->declare_parameter("chain/inherit_route", true);
    node_->get_parameter("chain/inherit_route", chain_inherit_route);
    chain_planner_ = std::make_unique<SegmentChainPlanner>(
        node_, path_manager_, log_manager_.get(), chain_segments,
        chain_inherit_route);
    if (chain_enable_) {
        const std::string seg_desc = chain_segments > 0
            ? std::to_string(chain_segments) + " segments per mission"
            : "auto-sized segments";
        FSM_LOG_WARN("[CHAIN] segment-chained planning ENABLED "
                     "(%s) — /planning/"
                     "initial_trajectory carries the baseline OPTIMIZED "
                     "trajectory in this mode (the chained flight itself "
                     "under chain/author_from_route), not the MINCO seed",
                     seg_desc.c_str());
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Create callback groups with dedicated separation to prevent timer stalls:
    // - timer_callback_group: FSM timer only (MutuallyExclusive, runs independently)
    // - subscription_callback_group: Formation/broadcast callbacks (MutuallyExclusive)
    // - position_callback_group: Position updates (MutuallyExclusive, separate to avoid blocking)
    // With MultiThreadedExecutor, these groups can run in parallel threads
    timer_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscription_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    FSM_LOG_INFO("Callback groups created: timer, subscription (all MutuallyExclusive)");

    // Single-drone: topics are flat (no per-drone prefix).
    // For swarm/multiple drones, restore a per-drone prefix here
    // (e.g. "/drone" + std::to_string(drone_id_)) so topics don't collide.
    std::string topic_prefix = "";

    // Keep the latest polynomial available to panels/visualizers opened after
    // planning completed. RELIABLE, not sensor/best-effort: a best-effort
    // WRITER latches its sample but delivers it best-effort, so a late joiner
    // could still miss the one message that matters. Reliable is stricter on
    // the writer side and stays compatible with every best-effort subscriber.
    auto trajectory_qos = rclcpp::QoS(5).reliable();
    trajectory_qos.transient_local();
    optimized_path_pub_ = node_->create_publisher<mmp_traj_msgs::msg::PolyTraj>(
        topic_prefix + "/planning/trajectory", trajectory_qos);
    global_path_pub_ = node_->create_publisher<mmp_traj_msgs::msg::PolyTraj>(
        topic_prefix + "/planning/initial_trajectory", trajectory_qos);

    rclcpp::SubscriptionOptions trajectory_cmd_options;
    trajectory_cmd_options.callback_group = subscription_callback_group_;
    // Reliable: this is THE mission command. A best-effort reader may drop the
    // single Run message on a timing edge and the FSM would just sit there.
    trajectory_cmd_sub_ = node_->create_subscription<mmp_mission_msgs::msg::TrajectoryCommand>(
        topic_prefix + "/mission/trajectory_command", rclcpp::QoS(5).reliable(),
        std::bind(&ReplanFSM::trajectoryCommandCallback, this, std::placeholders::_1),
        trajectory_cmd_options);

    waypoint_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "/viz/waypoints", 10);

    // Every callback that MUTATES PathManager shared state (terrain_data_,
    // risk_zones_, terrain_risk_masks_, the SDF dynamic layer) must sit on
    // subscription_callback_group_, the same MutuallyExclusive group that
    // runs trajectoryCommandCallback (and through it the whole planning
    // pipeline, which READS that state). A subscription created without a
    // group lands on the node DEFAULT group, which the MultiThreadedExecutor
    // runs in parallel with subscription_callback_group_: at startup the
    // latched /terrain/grid_map and the panel's /mission/risk_zones both fired
    // rebuildTerrainRiskMasks() concurrently, and the two unsynchronized
    // std::vector reallocations (risk_zones_, terrain_risk_masks_) segfaulted
    // the node before WAIT_TARGET. None of these are hot paths, so
    // serializing them costs nothing.
    rclcpp::SubscriptionOptions state_mutator_options;
    state_mutator_options.callback_group = subscription_callback_group_;

    // Terrain GridMap subscription (TRANSIENT_LOCAL to receive latched message)
    rclcpp::QoS terrain_qos(1);
    terrain_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    terrain_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    terrain_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
        "/terrain/grid_map", terrain_qos,
        std::bind(&ReplanFSM::terrainCallback, this, std::placeholders::_1),
        state_mutator_options);

    // No separate clear topic: an empty DynamicObstacleArray with replace=true
    // is the clear verb (the /dynamic_obstacles/clear Empty topic was retired).
    // reliable+transient_local, same as the zone sub below: the panel latches
    // the scenario, and a volatile reader never collects a latch — a planner
    // started after Load would silently plan with zero obstacles (the exact
    // bug class fixed for zones in P0).
    load_obstacles_sub_ =
        node_->create_subscription<mmp_mission_msgs::msg::DynamicObstacleArray>(
            "/mission/obstacles", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&ReplanFSM::loadObstaclesCallback, this,
                      std::placeholders::_1),
            state_mutator_options);
    // Runtime risk-zone reset. Subscribe on the same MutuallyExclusive
    // subscription_callback_group_ used by trajectoryCommandCallback so
    // that publish-order from the mission panel is preserved: the panel
    // publishes RiskZoneArray first, then TrajectoryCommand; the panel's
    // QoS is reliable, ROS preserves FIFO per publisher, and the shared
    // MutuallyExclusive group serializes the two callbacks. No race.
    {
        rclcpp::SubscriptionOptions risk_zone_options;
        risk_zone_options.callback_group = subscription_callback_group_;
        // transient_local: the ObstacleScenario panel LATCHES the zone set for
        // exactly this case (its comment says "a planner restarted after the
        // scenario was loaded still receives the last published zone set") —
        // but a volatile reader never requests the latched sample, so a
        // planner started after Load silently planned with ZERO zones. A TL
        // reader is what actually collects the latch.
        load_risk_zones_sub_ =
            node_->create_subscription<mmp_mission_msgs::msg::RiskZoneArray>(
                "/mission/risk_zones", rclcpp::QoS(1).reliable().transient_local(),
                std::bind(&ReplanFSM::loadRiskZonesCallback, this,
                          std::placeholders::_1),
                risk_zone_options);
    }

    // The FSM timer shares the SUBSCRIPTION group ON PURPOSE (it used to have
    // its own group "to prevent timer stalls"): with separate MutuallyExclusive
    // groups the MultiThreadedExecutor runs the 10 ms tick CONCURRENTLY with
    // trajectoryCommandCallback -> planGlobalTraj, so the tick read traj_ and
    // exec_state_ while the planner wrote them (data race, proven empirically
    // with a temporary overlap counter). Sharing the group serializes them;
    // ticks during the multi-second plan are only flag-polling, so delaying
    // them until the plan finishes costs nothing in single-shot mode.
    timer_ = node_->create_wall_timer(10ms, std::bind(&ReplanFSM::computeAndPublishPaths, this), subscription_callback_group_);
    FSM_LOG_INFO("FSM timer created on the subscription callback group (10ms period, serialized with planning)");
}

void ReplanFSM::computeAndPublishPaths() {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        fsm_num = 0;
    }

    switch (exec_state_) {
        case INIT: {
            changeFSMExecState(WAIT_POSITION, "FSM");
            break;
        }

        case WAIT_POSITION: {
            if (!have_target_) {
                return;
            }
            changeFSMExecState(SEQUENTIAL_START, "FSM");
            break;
        }

        case SEQUENTIAL_START: {
            if (drone_id_ <= 0 || (drone_id_ >= 1 && have_recv_pre_agent_)) 
            {
                bool success = false;
                try {
                    success = planFromGlobalTraj(1);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "Exception during trajectory planning in SEQUENTIAL_START: %s", e.what());
                    log_manager_->errorf("Exception during trajectory planning in SEQUENTIAL_START: %s", e.what());
                    success = false;
                }
                
                static int sequential_start_failures = 0;  // Track consecutive failures

                if (success)
                {
                    sequential_start_failures = 0;  // Reset on success
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
                    log_manager_->errorf("MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
                    // Instead of going back to SEQUENTIAL_START immediately, wait a bit
                    sequential_start_failures++;
                    if (sequential_start_failures > 10) {
                        RCLCPP_ERROR(node_->get_logger(), "Too many failures in SEQUENTIAL_START, going to EMERGENCY_STOP");
                        log_manager_->errorf("Too many failures in SEQUENTIAL_START, going to EMERGENCY_STOP");
                        flag_escape_emergency_ = true;  // entry tick commands the hover
                        changeFSMExecState(EMERGENCY_STOP, "FSM");
                        sequential_start_failures = 0;
                    } else {
                        changeFSMExecState(WAIT_POSITION, "FSM");
                    }
                }
            }
            break;
        }

        case GEN_NEW_TRAJ: {
            bool success = planFromGlobalTraj(1);
            if (success) {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            } else {
                have_target_ = false;
                changeFSMExecState(WAIT_POSITION, "FSM");
            }
            break;
        }

        case EXEC_TRAJ:
        {
            if (!path_manager_) {
                RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
                log_manager_->errorf("PathManager is not initialized!");
                flag_escape_emergency_ = true;  // entry tick commands the hover
                changeFSMExecState(EMERGENCY_STOP, "FSM");
                break;
            }
            auto local_traj = &path_manager_->traj_.local_traj;
            double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - local_traj->start_time;
            t_cur = std::min(local_traj->duration, t_cur);

            // Single-shot execution: no replan, just check if trajectory completed
            if (t_cur > local_traj->duration - 0.2)
            {
                have_target_ = false;
                have_local_traj_ = false;
                changeFSMExecState(WAIT_POSITION, "FSM");
                RCLCPP_INFO(node_->get_logger(), "[drone %d reached goal]", drone_id_);
                log_manager_->infof("[drone %d reached goal]", drone_id_);
                return;
            }
            break;
        }

        case EMERGENCY_STOP: {
            if (flag_escape_emergency_) {
                // Entry tick (flag set on the transition): command the hover
                // ONCE. The flag was never set before, so this state published
                // nothing and fell straight through to GEN_NEW_TRAJ.
                callEmergencyStop(current_pos_);
                flag_escape_emergency_ = false;
            } else {
                // No odometry exists in this architecture (current_vel_ was
                // never written), so a "wait until stopped" gate is
                // unknowable. Park and await the next mission instead.
                have_target_ = false;
                have_local_traj_ = false;
                RCLCPP_WARN(node_->get_logger(), "Emergency hover commanded; awaiting a new mission");
                log_manager_->warnf("Emergency hover commanded; awaiting a new mission");
                changeFSMExecState(WAIT_POSITION, "FSM");
            }
            break;
        }
    }
}

void ReplanFSM::polyTraj2ROSMsg(mmp_traj_msgs::msg::PolyTraj &msg)
{
    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return;
    }
    auto data = &path_manager_->traj_.local_traj;

    msg.drone_id = drone_id_;
    msg.order = 5;

    const double s = data->start_time;
    msg.start_time.sec     = static_cast<int32_t>(std::floor(s));
    msg.start_time.nanosec = static_cast<uint32_t>(std::llround((s - msg.start_time.sec) * 1e9));

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
}

void ReplanFSM::globalTraj2ROSMsg(mmp_traj_msgs::msg::PolyTraj &msg)
{
    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return;
    }
    msg.drone_id = drone_id_;

    auto data = &path_manager_->traj_.global_traj;

    rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    msg.start_time.sec = now.seconds();
    msg.start_time.nanosec = now.nanoseconds() % 1000000000;
    msg.order = 5;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
}

bool ReplanFSM::planFromGlobalTraj(int trial_times) {
    // In SFC single-shot mode, planGlobalTraj() already optimized and set local_traj.
    // Just verify local_traj exists and publish it.
    auto local_traj = &path_manager_->traj_.local_traj;
    if (local_traj->duration > 0 && local_traj->start_time > 0)
    {
        // local_traj was already set by planGlobalTraj() — publish and go
        log_manager_->infof("[planFromGlobalTraj] Using pre-optimized trajectory (duration=%.3f)", local_traj->duration);

        mmp_traj_msgs::msg::PolyTraj msg;
        polyTraj2ROSMsg(msg);
        optimized_path_pub_->publish(msg);

        have_local_traj_ = true;

        if (enable_global_trajectory_pub_) {
            mmp_traj_msgs::msg::PolyTraj msg2;
            globalTraj2ROSMsg(msg2);
            global_path_pub_->publish(msg2);
        }

        return true;
    }

    // local_traj not set (shouldn't happen in normal flow)
    log_manager_->errorf("[planFromGlobalTraj] local_traj not ready — planGlobalTraj() may have failed");
    return false;
}

void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
    static std::string state_str[6] = {"INIT", "WAIT_POSITION", "GEN_NEW_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    // Throttle frequent state transitions
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();

    bool should_log = (exec_state_ != new_state) &&  // State actually changed
                      (new_state == WAIT_POSITION || new_state == SEQUENTIAL_START || new_state == GEN_NEW_TRAJ || elapsed >= 2);

    if (should_log) {
        RCLCPP_INFO(node_->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[static_cast<int>(exec_state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
        last_log_time = now;
    }

    log_manager_->infof("[%s]: from %s to %s", pos_call.c_str(), state_str[static_cast<int>(exec_state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
    exec_state_ = new_state;
}

// Single entry point for starting a mission plan. This used to be a pair of
// functions bridged by a ROS message that never went on the wire: the sender
// built the message and then hand-delivered it to the receiver by shared_ptr,
// so no publisher, subscriber or topic ever existed and the message only
// carried arguments across a plain function call. The target and the route
// waypoints are the only values that were ever read back out of it.
void ReplanFSM::startMissionPlan(const Eigen::Vector3d& target,
                                 const std::vector<Eigen::Vector3d>& waypoints) {
    auto plan_start = std::chrono::high_resolution_clock::now();
    double ros_time_start = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

    if (target.z() < -0.1) {
        return;
    }

    FSM_LOG_INFO("[DEBUG TARGET] startMissionPlan STARTED for drone %d at time %.3f!",
                 drone_id_, ros_time_start);

    // Mission progress: move next_mission to current, clear next
    // This signals that we're working on the "next" mission now (which becomes current)
    if (!next_mission_id_.empty() && next_mission_id_ != "MISSION_END") {
        current_mission_id_ = next_mission_id_;
        next_mission_id_.clear();  // Empty means we need new mission data
        FSM_LOG_INFO("Mission progressed: now executing %s, next mission cleared (will request new data)",
                    current_mission_id_.c_str());
    }

    // Initialize optimizer if not already initialized
    if (!path_manager_->isOptimizerInitialized()) {
        auto opt_init_start = std::chrono::high_resolution_clock::now();
        try {
            RCLCPP_INFO(node_->get_logger(), "Initializing optimizer for drone %d...", drone_id_);
            log_manager_->infof("Initializing optimizer for drone %d...", drone_id_);
            FSM_LOG_INFO("[TIMING] Optimizer initialization started");

            path_manager_->initOptimizer();
            path_manager_->deliverTrajToOptimizer();

            auto opt_init_end = std::chrono::high_resolution_clock::now();
            auto opt_init_duration = std::chrono::duration_cast<std::chrono::milliseconds>(opt_init_end - opt_init_start).count();

            RCLCPP_INFO(node_->get_logger(), "Optimizer initialized successfully for drone %d", drone_id_);
            log_manager_->infof("Optimizer initialized successfully for drone %d", drone_id_);
            FSM_LOG_INFO("[TIMING] Optimizer initialization took %ld ms", opt_init_duration);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize optimizer for drone %d: %s", drone_id_, e.what());
            log_manager_->errorf("Failed to initialize optimizer for drone %d: %s", drone_id_, e.what());
            return;
        }
    }

    if (have_local_traj_) {
        LocalTrajData *info = &path_manager_->traj_.local_traj;
        double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - info->start_time;
        Eigen::Vector3d theoretical_pos = info->traj.getPos(t_cur);
        Eigen::Vector3d theoretical_vel = info->traj.getVel(t_cur);
        Eigen::Vector3d theoretical_acc = info->traj.getAcc(t_cur);

        // Use trajectory position for smooth formation change
        // This prevents jumps in commanded position during formation transitions
        start_pt_ = theoretical_pos;
        start_vel_ = theoretical_vel;
        start_acc_ = theoretical_acc;
        start_vel_synthesized_ = false;  // trajectory-derived: real motion state

        double pos_error = (current_pos_ - theoretical_pos).norm();
        log_manager_->infof("Formation change - using TRAJECTORY position/vel/acc (error from actual: %.2fm)",
                   pos_error);
        log_manager_->infof("  Trajectory pos: (%.2f, %.2f, %.2f), Actual pos: (%.2f, %.2f, %.2f)",
                   start_pt_(0), start_pt_(1), start_pt_(2),
                   current_pos_(0), current_pos_(1), current_pos_(2));
    } else {
        start_pt_ = current_pos_;
        start_vel_.setZero();
        start_vel_synthesized_ = false;
        // Provenance tag for the log below: the derived first-leg velocity
        // was repeatedly misread as an applied use_initial_velocity vector.
        const char *vel_src = "rest (zero)";
        if (use_commanded_initial_velocity_) {
            start_vel_ = commanded_initial_velocity_;
            vel_src = "commanded vector (use_initial_velocity)";
        } else if (commanded_initial_speed_ > 0.0) {
            // Aim the initial velocity along the FIRST ROUTE LEG, not the
            // final target: with intermediate waypoints the two differ, and
            // a target-aimed start manufactured an immediate high-load bank
            // onto leg 1 (measured: n=1.65 / 52.6 deg / 113.8% thrust at
            // t=9.3 s on a dogleg whose real corner was 12.8 km away).
            // First waypoint == target for plain missions, so the simple
            // case is unchanged.
            Eigen::Vector3d leg_end(target.x(), target.y(), 0.0);
            for (const auto &wp : waypoints) {
                const double dx = wp.x() - start_pt_.x();
                const double dy = wp.y() - start_pt_.y();
                if (dx * dx + dy * dy > 1.0e-6) {
                    leg_end = Eigen::Vector3d(wp.x(), wp.y(), 0.0);
                    break;
                }
            }
            Eigen::Vector3d initial_direction(
                leg_end.x() - start_pt_.x(),
                leg_end.y() - start_pt_.y(), 0.0);
            if (initial_direction.head<2>().norm() < 1.0e-9) {
                initial_direction = Eigen::Vector3d::UnitX();
            } else {
                initial_direction.normalize();
            }
            start_vel_ = initial_direction * commanded_initial_speed_;
            vel_src = "initial_speed x first-leg direction (level)";
            // Chord is only a PROXY for "cruising along the route" (the route
            // does not exist yet) — mark it so planGlobalTraj can re-aim onto
            // the front-end route's real initial direction ([VEL-ALIGN]).
            start_vel_synthesized_ = true;
        }
        start_acc_ = use_commanded_initial_acceleration_
            ? commanded_initial_acceleration_
            : Eigen::Vector3d::Zero();
        log_manager_->infof(
            "Using command start state: pos=(%.2f, %.2f, %.2f), "
            "speed=%.1f m/s, vel_mps=(%.1f, %.1f, %.1f) [src: %s], "
            "acc_mps2=(%.1f, %.1f, %.1f)",
            start_pt_(0), start_pt_(1), start_pt_(2),
            start_vel_.norm() * initial_speed_unit_m_,
            start_vel_(0) * initial_speed_unit_m_,
            start_vel_(1) * initial_speed_unit_m_,
            start_vel_(2) * initial_speed_unit_m_,
            vel_src,
            start_acc_(0) * initial_speed_unit_m_,
            start_acc_(1) * initial_speed_unit_m_,
            start_acc_(2) * initial_speed_unit_m_);
    }

    // TEST: override the resolved start vel/acc with the injected boundary condition.
    // Position is left untouched; only the initial motion state is forced.
    if (inject_init_state_) {
        start_vel_ = inject_init_vel_;
        start_acc_ = inject_init_acc_;
        start_vel_synthesized_ = false;  // injected = explicit, never re-aim
        log_manager_->infof("[TEST] Injected initial vel=(%.2f,%.2f,%.2f) acc=(%.2f,%.2f,%.2f)",
                   start_vel_(0), start_vel_(1), start_vel_(2),
                   start_acc_(0), start_acc_(1), start_acc_(2));
    }

    // The commanded waypoints are passed through as-is: they are already
    // center points with any formation offset applied.
    std::vector<Eigen::Vector3d> route = waypoints;

    if (!route.empty()) {
        log_manager_->infof(
                   "Drone %d: Using %zu commanded waypoints (already offset-applied)",
                   drone_id_, route.size());

        end_pt_ = route.back();
    } else {
        route = { target };
        end_pt_ = target;

        RCLCPP_INFO(node_->get_logger(),
                   "Drone %d: Using single target as waypoint (already offset-applied)",
                   drone_id_);
        log_manager_->infof(
                   "Drone %d: Using single target as waypoint (already offset-applied)",
                   drone_id_);
    }

    log_manager_->infof("start_pt_: %.2f, %.2f, %.2f", start_pt_(0), start_pt_(1), start_pt_(2));
    log_manager_->infof("waypoints size: %zu", route.size());
    for (const auto& wp : route) {
        log_manager_->infof("waypoint: %.2f, %.2f, %.2f", wp(0), wp(1), wp(2));
    }

    if (enable_waypoint_markers_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "waypoints_drone_" + std::to_string(drone_id_);
        marker.id = drone_id_;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;  // Sphere size

        if (drone_id_ == 0) {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;  // Red
        } else if (drone_id_ == 1) {
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;  // Blue
        } else if (drone_id_ == 2) {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;  // Green
        } else if (drone_id_ == 3) {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;  // Yellow
        }
        marker.color.a = 1.0;

        for (const auto& wp : route) {
            geometry_msgs::msg::Point p;
            p.x = wp(0);
            p.y = wp(1);
            // Waypoint z is AGL ([GOAL AGL]); render the marker at the
            // ABSOLUTE altitude planGlobalTraj will fly (elevation + max(z,
            // min_goal_agl)) — the raw value drew the sphere inside the
            // terrain on any non-flat DEM. Same rule and gate as the
            // conversion in path_manager (water/off-DEM base = 0).
            p.z = wp(2);
            if (path_manager_ && path_manager_->hasTerrainData()) {
                double base = 0.0;
                path_manager_->terrainElevation(wp(0), wp(1), &base);
                p.z = base + std::max(wp(2), path_manager_->minGoalAgl());
            }
            marker.points.push_back(p);
        }

        waypoint_marker_pub_->publish(marker);
        log_manager_->infof("Drone %d: Published %zu waypoint markers to RViz (frame: %s, ns: %s)",
                    drone_id_, route.size(), marker.header.frame_id.c_str(), marker.ns.c_str());
    }

    triggerGlobalPlan(route);

    auto plan_end = std::chrono::high_resolution_clock::now();
    auto plan_duration = std::chrono::duration_cast<std::chrono::milliseconds>(plan_end - plan_start).count();
    double ros_time_end = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    FSM_LOG_INFO("[DEBUG TARGET] startMissionPlan COMPLETED in %ld ms at time %.3f (total elapsed: %.3f ms)",
                 plan_duration, ros_time_end, (ros_time_end - ros_time_start) * 1000);
}

// [START AGL] The commanded seed start rides the same AGL contract as
// [GOAL AGL]: mission yaml / panel z is height above terrain (above sea
// level over water), because mission sources cannot know the DEM. Applied
// exactly once, at the first plan attempt — consumed even without terrain
// (kept absolute then, matching goal behavior) so a DEM arriving mid-flight
// can never re-shift a trajectory-derived start.
void ReplanFSM::resolveCommandedStartAgl()
{
    if (!start_seed_agl_pending_) return;
    start_seed_agl_pending_ = false;
    if (!path_manager_) return;
    if (!path_manager_->hasTerrainData()) {
        FSM_LOG_WARN("[START AGL] no terrain at first plan — commanded start "
                     "z=%.2f kept ABSOLUTE", start_pt_.z());
        return;
    }
    double base = 0.0;  // water / off-DEM: AGL == ASL, same as [GOAL AGL]
    path_manager_->terrainElevation(start_pt_.x(), start_pt_.y(), &base);
    const double agl = std::max(start_pt_.z(), path_manager_->minGoalAgl());
    const double z_abs = base + agl;
    if (std::abs(z_abs - start_pt_.z()) > 1e-9) {
        FSM_LOG_INFO("[START AGL] commanded start z=%.2f AGL -> absolute %.2f "
                     "(terrain %.2f + agl %.2f)",
                     start_pt_.z(), z_abs, base, agl);
    }
    start_pt_.z() = z_abs;
    current_pos_.z() = z_abs;
}

void ReplanFSM::triggerGlobalPlan(const std::vector<Eigen::Vector3d>& waypoints) {
    last_plan_succeeded_ = false;
    resolveCommandedStartAgl();
    // Use formation pattern received via TrajectoryCommand
    auto formation_setup_start = std::chrono::high_resolution_clock::now();
    path_manager_->setFormationInfo(drone_id_, current_formation_type_, current_formation_pattern_);

    auto global_traj_start = std::chrono::high_resolution_clock::now();
    FSM_LOG_INFO("[TIMING] Starting global trajectory planning");
    bool success;
    if (chain_enable_ && chain_planner_) {
        // [CHAIN] baseline + N chained segment runs; the planner forwards
        // the synthesized flag itself (per-run semantics differ).
        success = chain_planner_->plan(start_pt_, start_vel_, start_acc_,
                                       waypoints, start_vel_synthesized_);
    } else {
        path_manager_->setStartVelSynthesized(start_vel_synthesized_);
        success = path_manager_->planGlobalTraj(
            start_pt_, start_vel_, start_acc_,
            waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }

    auto global_traj_end = std::chrono::high_resolution_clock::now();
    auto global_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(global_traj_end - global_traj_start).count();
    FSM_LOG_INFO("[TIMING] Global trajectory planning took %ld ms", global_traj_duration);

    if (success) {
        // Pass formation pattern to optimizer for formation cost calculation
        FSM_LOG_INFO("Trajectory planning successful for formation type: %s", current_formation_type_.c_str());

        if (!current_formation_pattern_.empty()) {
            log_manager_->infof("Formation pattern for optimizer (relative coordinates):");
            for (size_t i = 0; i < current_formation_pattern_.size(); ++i) {
                log_manager_->infof("  Drone %zu: [%.3f, %.3f, %.3f]",
                                   i, current_formation_pattern_[i].x(),
                                   current_formation_pattern_[i].y(),
                                   current_formation_pattern_[i].z());
            }
            path_manager_->setFormationToOptimizer(current_formation_pattern_, num_drones_);
        } else {
            FSM_LOG_WARN("Formation pattern is empty, optimizer may not apply formation constraints");
            path_manager_->setFormationToOptimizer(current_formation_pattern_, num_drones_);
        }

        have_target_ = true;
        last_plan_succeeded_ = true;

        if (exec_state_ == WAIT_POSITION)
            changeFSMExecState(SEQUENTIAL_START, "startMissionPlan");
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(SEQUENTIAL_START, "startMissionPlan");  // Single-shot: new global plan already has local_traj

        // NOTE: Global trajectory publish removed from the mission-plan path to prevent blocking
        // The global trajectory will be published in computeAndPublishPaths instead
        // This fixes the race condition that caused 20-60 second freezes during formation transitions

        RCLCPP_INFO(node_->get_logger(), "Successfully generated global trajectory for drone %d", drone_id_);
        log_manager_->infof("Successfully generated global trajectory for drone %d", drone_id_);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory for drone %d!", drone_id_);
        log_manager_->errorf("Unable to generate global trajectory for drone %d!", drone_id_);
    }
}

bool ReplanFSM::callEmergencyStop(const Eigen::Vector3d& stop_pos) {
    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return false;
    }

    path_manager_->EmergencyStop(stop_pos);

    mmp_traj_msgs::msg::PolyTraj msg;
    polyTraj2ROSMsg(msg);
    optimized_path_pub_->publish(msg);

    return true;
}

// trajectoryCommandCallback: receives a trajectory command (target position
// and waypoints) from the RViz MissionConfig panel.
void ReplanFSM::trajectoryCommandCallback(const mmp_mission_msgs::msg::TrajectoryCommand::SharedPtr msg) {
    auto callback_start = std::chrono::high_resolution_clock::now();
    FSM_LOG_INFO("[TRAJECTORY CMD] Received trajectory command (seq: %d, drone: %d) at time %.3f",
                 msg->sequence, msg->drone_id, rclcpp::Clock(RCL_ROS_TIME).now().seconds());

    // Check if this command is for this drone
    if (msg->drone_id != drone_id_) {
        FSM_LOG_DEBUG("Ignoring trajectory command for drone %d (I am drone %d)",
                     msg->drone_id, drone_id_);
        return;
    }

    // Check for duplicate messages using sequence number
    if (msg->sequence <= last_received_sequence_) {
        FSM_LOG_DEBUG("Ignoring duplicate/old trajectory command (seq: %d, last: %d)",
                     msg->sequence, last_received_sequence_);
        return;
    }

    // Capture the pre-command sequencing state: if the plan below fails or is
    // rejected, both are ROLLED BACK at the end of this callback so a resend
    // of the SAME command retries instead of dying in the dedup gate above.
    const int prev_sequence = last_received_sequence_;
    const std::string prev_mission_id = current_mission_id_;

    last_received_sequence_ = msg->sequence;

    // A NEW mission (different mission_id) is not a mid-flight continuation of
    // the current trajectory — its commanded start must be adopted. Capture
    // this BEFORE current_mission_id_ is overwritten below. (Same mission_id =
    // formation change / replan, which keeps the trajectory-continuation path.)
    const bool mission_changed =
        start_position_received_ && !msg->mission_id.empty() &&
        msg->mission_id != current_mission_id_;

    current_mission_id_ = msg->mission_id;
    commanded_initial_speed_ =
        std::max(0.0, msg->initial_speed) / initial_speed_unit_m_;
    use_commanded_initial_velocity_ = msg->use_initial_velocity;
    use_commanded_initial_acceleration_ = msg->use_initial_acceleration;
    commanded_initial_velocity_ = Eigen::Vector3d(
        msg->initial_velocity.x, msg->initial_velocity.y,
        msg->initial_velocity.z) / initial_speed_unit_m_;
    commanded_initial_acceleration_ = Eigen::Vector3d(
        msg->initial_acceleration.x, msg->initial_acceleration.y,
        msg->initial_acceleration.z) / initial_speed_unit_m_;
    if (use_commanded_initial_velocity_ &&
        !commanded_initial_velocity_.allFinite()) {
        FSM_LOG_WARN("Ignoring non-finite commanded initial velocity");
        use_commanded_initial_velocity_ = false;
        commanded_initial_velocity_.setZero();
    }
    if (use_commanded_initial_acceleration_ &&
        !commanded_initial_acceleration_.allFinite()) {
        FSM_LOG_WARN("Ignoring non-finite commanded initial acceleration");
        use_commanded_initial_acceleration_ = false;
        commanded_initial_acceleration_.setZero();
    }

    // Adopt the commanded start for the FIRST command AND for every new
    // mission. Previously this was a write-once latch (only the first command
    // ever set the start), so missions 2..N silently planned from the stale
    // first start / previous trajectory end — there is no odometry sub to
    // refresh current_pos_. A new mission drops the previous local trajectory
    // so startMissionPlan plans from the commanded start, not the old
    // trajectory position.
    if (!start_position_received_ || mission_changed) {
        Eigen::Vector3d new_start_pos(
            msg->start_position.x,
            msg->start_position.y,
            msg->start_position.z
        );

        start_pt_ = new_start_pos;
        current_pos_ = new_start_pos;
        start_position_received_ = true;
        start_seed_agl_pending_ = true;  // z is AGL; resolved at first plan
        if (mission_changed) {
            have_local_traj_ = false;    // new mission: do not continue prev traj
        }

        FSM_LOG_INFO("Received start position from TrajectoryCommand: (%.2f, %.2f, %.2f)%s",
                    new_start_pos.x(), new_start_pos.y(), new_start_pos.z(),
                    mission_changed ? " [new mission]" : "");
    }

    FSM_LOG_INFO("Trajectory command (seq: %d): mission=%s, start=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
                msg->sequence,
                msg->mission_id.c_str(),
                msg->start_position.x, msg->start_position.y, msg->start_position.z,
                msg->target_position.x, msg->target_position.y, msg->target_position.z);

    std::vector<Eigen::Vector3d> waypoints;
    for (const auto& wp : msg->waypoints) {
        waypoints.emplace_back(wp.x, wp.y, wp.z);
    }

    Eigen::Vector3d target_position(
        msg->target_position.x,
        msg->target_position.y,
        msg->target_position.z
    );

    // Waypoint-less command: the TARGET is the mission. The old message path
    // substituted current_formation_center_ for an empty waypoint list —
    // (0,0,0) on a fresh node — so a valid headless/CLI command (target set,
    // waypoints omitted) planned a flight to the map corner while logging
    // success. The RViz panel always fills waypoints[], which is why that
    // stayed dormant; filling the route here keeps the target authoritative.
    if (waypoints.empty()) {
        waypoints.push_back(target_position);
        FSM_LOG_INFO("Trajectory command has no waypoints — using target_position "
                     "(%.2f, %.2f, %.2f) as the single waypoint",
                     target_position.x(), target_position.y(), target_position.z());
    }

    // Extract formation offset (for reference/logging)
    Eigen::Vector3d formation_offset(
        msg->formation_offset.x,
        msg->formation_offset.y,
        msg->formation_offset.z
    );

    std::vector<Eigen::Vector3d> formation_pattern;
    formation_pattern.reserve(msg->formation_pattern.size());
    for (const auto& pt : msg->formation_pattern) {
        formation_pattern.emplace_back(pt.x, pt.y, pt.z);
    }

    current_formation_pattern_ = formation_pattern;

    // Update formation parameters (for compatibility with existing code)
    current_formation_type_ = msg->formation_type;
    current_formation_scale_ = msg->formation_scale;

    // Pass trajectory parameters to PathManager
    if (msg->length_per_piece > 0.0) {
        path_manager_->setLengthPerPiece(msg->length_per_piece);
    }
    if (msg->obstacle_clearance > 0.0) {
        path_manager_->setObstacleClearance(msg->obstacle_clearance);
    }
    FSM_LOG_INFO("Drone %d: target=(%.2f, %.2f, %.2f), offset=(%.2f, %.2f, %.2f), %zu waypoints",
                drone_id_,
                target_position.x(), target_position.y(), target_position.z(),
                formation_offset.x(), formation_offset.y(), formation_offset.z(),
                waypoints.size());

    last_plan_succeeded_ = false;  // also covers early returns before triggerGlobalPlan
    startMissionPlan(target_position, waypoints);

    if (!last_plan_succeeded_) {
        // Plan failed or was rejected (e.g. the optimizer's collision audit).
        // Un-consume the sequencing state so a resend of the SAME command
        // retries; without this the dedup gate silently dropped it and the
        // mission was stranded with no reject signal.
        last_received_sequence_ = prev_sequence;
        current_mission_id_ = prev_mission_id;
        RCLCPP_ERROR(node_->get_logger(),
                     "[PLAN REJECTED] mission '%s' (seq %d) produced no trajectory — "
                     "sequence rolled back, a resend will retry",
                     msg->mission_id.c_str(), msg->sequence);
        log_manager_->errorf(
                     "[PLAN REJECTED] mission '%s' (seq %d) produced no trajectory — "
                     "sequence rolled back, a resend will retry",
                     msg->mission_id.c_str(), msg->sequence);
    }

    auto callback_end = std::chrono::high_resolution_clock::now();
    auto callback_duration = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end - callback_start).count();
    FSM_LOG_INFO("[TRAJECTORY CMD] Callback completed (took %ld ms) at time %.3f",
                 callback_duration, rclcpp::Clock(RCL_ROS_TIME).now().seconds());
}

void ReplanFSM::terrainCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    if (path_manager_) {
        path_manager_->setTerrainData(msg);
        FSM_LOG_INFO("Terrain data received and forwarded to PathManager");
    }
}

// Applies a dynamic-obstacle batch. msg->replace clears the live set first, so
// an EMPTY array with replace=true is the "clear all obstacles" verb (the old
// /dynamic_obstacles/clear std_msgs/Empty topic was retired in its favor).
void ReplanFSM::loadObstaclesCallback(
    const mmp_mission_msgs::msg::DynamicObstacleArray::SharedPtr msg)
{
    if (!path_manager_) return;
    if (msg->replace) {
        path_manager_->clearDynamicObstacles();
    }
    size_t added = 0;
    size_t skipped = 0;
    size_t deferred = 0;  // queued until the SDF exists (no cache on first run)
    for (const auto& spec : msg->obstacles) {
        const Eigen::Vector3d c(spec.center.x, spec.center.y, spec.center.z);
        int id = -1;
        if (spec.kind == mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_CUBE) {
            const Eigen::Vector3d size(spec.size.x, spec.size.y, spec.size.z);
            id = path_manager_->addDynamicBox(c, size, spec.model);
            if (id == -1) {
                FSM_LOG_WARN(
                    "loadObstacles: addDynamicBox rejected at (%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f)",
                    c.x(), c.y(), c.z(), size.x(), size.y(), size.z());
            }
        } else if (spec.kind == mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_SPHERE) {
            id = path_manager_->addDynamicSphere(c, spec.radius, spec.model);
            if (id == -1) {
                FSM_LOG_WARN(
                    "loadObstacles: addDynamicSphere rejected at (%.2f,%.2f,%.2f) r=%.2f",
                    c.x(), c.y(), c.z(), spec.radius);
            }
        } else {
            FSM_LOG_WARN(
                "loadObstacles: skipping unsupported spec kind=%u",
                spec.kind);
            ++skipped;
            continue;
        }
        if (id == -1) ++skipped; else if (id == -2) ++deferred; else ++added;
    }
    FSM_LOG_INFO("loadObstacles: added=%zu deferred=%zu skipped=%zu (total in msg=%zu)",
                 added, deferred, skipped, msg->obstacles.size());
}

// Applies a risk-zone batch. msg->replace mirrors DynamicObstacleArray:
// true overwrites the active set, false appends to it. The FSM owns the
// accumulated set (active_risk_zones_) because setRiskZonesRuntime is
// replace-only — before this, Append silently dropped previously loaded zones.
// An EMPTY array with replace=true is therefore the "clear all zones" verb.
void ReplanFSM::loadRiskZonesCallback(
    const mmp_mission_msgs::msg::RiskZoneArray::SharedPtr msg)
{
    if (!path_manager_) return;
    // Convert the wire format to PathManager's internal RiskZone struct.
    std::vector<path_manager::RiskZone> zones;
    zones.reserve(msg->zones.size());
    size_t dropped = 0;
    for (const auto& z : msg->zones) {
        if (!std::isfinite(z.center.x) ||
            !std::isfinite(z.center.y) ||
            !std::isfinite(z.center.z) ||
            !std::isfinite(z.reach) ||
            !std::isfinite(z.peak) ||
            z.reach <= 0.0 || z.peak <= 0.0) {
            ++dropped;
            continue;
        }
        path_manager::RiskZone tz;
        tz.center = Eigen::Vector3d(z.center.x, z.center.y, z.center.z);
        tz.reach = z.reach;
        tz.peak = z.peak;  // raw max_risk_level, same semantics as the yaml
                           // loader: values >= 1 saturate at the field-level
                           // moat cap (1-1e-3) in getRiskNorm/RiskGradCostP
        zones.push_back(tz);
    }

    const size_t previous = active_risk_zones_.size();
    if (msg->replace) {
        active_risk_zones_ = zones;
    } else {
        active_risk_zones_.insert(active_risk_zones_.end(),
                                  zones.begin(), zones.end());
    }
    path_manager_->setRiskZonesRuntime(active_risk_zones_);
    FSM_LOG_INFO(
        "loadRiskZones: %s %zu zones (dropped %zu invalid), active set %zu -> %zu",
        msg->replace ? "replaced with" : "appended", zones.size(), dropped,
        previous, active_risk_zones_.size());
}

}  // namespace path_manager
