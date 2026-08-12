#ifndef REPLAN_FSM_H
#define REPLAN_FSM_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <map>
#include <vector>
#include "mmp_traj_msgs/msg/poly_traj.hpp"
#include "mmp_mission_msgs/msg/trajectory_command.hpp"
#include "mmp_mission_msgs/msg/dynamic_obstacle_array.hpp"
#include "mmp_mission_msgs/msg/dynamic_obstacle_spec.hpp"
#include "mmp_mission_msgs/msg/risk_zone_array.hpp"
#include "mmp_mission_msgs/msg/risk_zone_spec.hpp"
#include "path_manager/path_manager.h"
#include "path_manager/planning_result.h"
#include "path_manager/start_state.h"
#include "path_manager/segment_chain_planner.h"
#include "path_optimizer/plan_container.hpp"
#include "../../common/log_manager.hpp"

// Conditional logging macros to avoid code duplication
#define FSM_LOG_INFO(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_INFO(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->infof(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define FSM_LOG_WARN(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_WARN(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->warnf(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define FSM_LOG_ERROR(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_ERROR(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->errorf(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define FSM_LOG_DEBUG(msg, ...) do { \
    if (enable_debug_logs_ && log_manager_) { \
        log_manager_->debugf(msg, ##__VA_ARGS__); \
    } \
} while(0)

namespace path_manager {

class ReplanFSM {
public:
    enum FSM_EXEC_STATE {
        INIT,
        WAIT_POSITION,
        GEN_NEW_TRAJ,
        EXEC_TRAJ,
        EMERGENCY_STOP,
        SEQUENTIAL_START
    };

    ReplanFSM(rclcpp::Node::SharedPtr node);
    ~ReplanFSM() {};

    void computeAndPublishPaths();
    void trajectoryCommandCallback(const mmp_mission_msgs::msg::TrajectoryCommand::SharedPtr msg);
    void terrainCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    // Obstacles have no separate clear verb: an empty DynamicObstacleArray with
    // replace=true is the clear (see loadObstaclesCallback).
    void loadObstaclesCallback(
        const mmp_mission_msgs::msg::DynamicObstacleArray::SharedPtr msg);
    void loadRiskZonesCallback(
        const mmp_mission_msgs::msg::RiskZoneArray::SharedPtr msg);
    void polyTraj2ROSMsg(mmp_traj_msgs::msg::PolyTraj &msg);
    void globalTraj2ROSMsg(mmp_traj_msgs::msg::PolyTraj &msg);
    // Callback groups:
    // - timer_callback_group: FSM timer only (MutuallyExclusive, dedicated for 10ms timer)
    // - subscription_callback_group: Formation and broadcast subscriptions (MutuallyExclusive)
    // - position_callback_group: Position updates (MutuallyExclusive, separate to avoid blocking)
    // This separation prevents timer stalls when subscriptions are processing
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

private:
    rclcpp::Node::SharedPtr node_;
    bool planFromGlobalTraj(int trial_times = 1);
    // Pure planning trigger: runs global planning for given waypoints and handles
    // success/FSM transition. No message dependency (uses member state + waypoints).
    void triggerGlobalPlan(const std::vector<Eigen::Vector3d>& waypoints);
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    bool callEmergencyStop(const Eigen::Vector3d& stop_pos);
    
    // Mission entry point: resolves the start state from the commanded target
    // and route, draws the waypoint markers, then runs the global plan.
    void startMissionPlan(const Eigen::Vector3d& target,
                          const std::vector<Eigen::Vector3d>& waypoints);

    std::shared_ptr<PathManager> path_manager_;

    // [CHAIN] Stage-1 segment-chained planning (chain/enable, default off).
    // Lives beside the state machine, not inside it: the FSM decides WHEN to
    // plan, the chain planner knows how to turn one mission into N chained
    // pipeline runs. When disabled, triggerGlobalPlan takes the single-shot
    // path exactly as before.
    std::unique_ptr<SegmentChainPlanner> chain_planner_;
    bool chain_enable_{false};

    rclcpp::Publisher<mmp_traj_msgs::msg::PolyTraj>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<mmp_traj_msgs::msg::PolyTraj>::SharedPtr global_path_pub_;
    rclcpp::Subscription<mmp_mission_msgs::msg::TrajectoryCommand>::SharedPtr trajectory_cmd_sub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
    // [TERRAIN-READY] latched ingestion receipt: [resolution_u, origin_x_u,
    // origin_y_u] of the terrain the planner actually consumed — the panel's
    // Run flow gates the mission command on it (see terrainCallback).
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr terrain_ready_pub_;
    rclcpp::Subscription<mmp_mission_msgs::msg::DynamicObstacleArray>::SharedPtr
        load_obstacles_sub_;
    rclcpp::Subscription<mmp_mission_msgs::msg::RiskZoneArray>::SharedPtr
        load_risk_zones_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    FSM_EXEC_STATE exec_state_;
    bool have_target_;
    bool have_local_traj_;
    bool have_recv_pre_agent_;
    bool start_position_received_;  // Track if we received start position from TrajectoryCommand
    // Commanded seed start z is AGL (same contract as [GOAL AGL]: mission
    // sources cannot know the DEM). Converted to absolute ONCE at the first
    // plan; replans continue from trajectory positions, already absolute.
    bool start_seed_agl_pending_ = false;
    void resolveCommandedStartAgl();
    int drone_id_;      // Internal index (0,1,2,3...)
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d start_pt_, start_vel_, start_acc_;
    Eigen::Vector3d end_pt_;
    // Initial speed from TrajectoryCommand is physical m/s. The planner state
    // uses frame units/s, so convert once and apply it along the first route
    // chord when there is no preceding trajectory.
    double commanded_initial_speed_{0.0};
    // [INITIAL-STATE] The validated claim from the last accepted command.
    // Parsed at the top of the callback, before any state is consumed.
    path_manager::MissionStartClaim mission_start_claim_{};
    double initial_speed_unit_m_{100.0};
    bool use_commanded_initial_velocity_{false};
    bool use_commanded_initial_acceleration_{false};
    // True when start_vel_ was SYNTHESIZED (default speed x first-leg chord)
    // rather than commanded/trajectory-derived. planGlobalTraj may re-aim a
    // synthesized velocity onto the route's actual initial direction
    // ([VEL-ALIGN]); explicit vectors are never touched.
    bool start_vel_synthesized_{false};
    // [ENVELOPE] True when start_vel_ is an EXPLICIT operator input
    // (use_initial_velocity vector or test injection). Commanded starts are
    // envelope-validated at plan entry (contract 1: outside the cruise
    // validity region -> FAILED(INITIAL_MODE_UNSUPPORTED), never clamped);
    // synthesized and trajectory-derived starts are not inputs and keep the
    // [STALL-FLOOR] clamp doctrine.
    bool start_vel_commanded_{false};
    // True when the START STATE HAS A STATED ORIGIN — the mission gave a
    // velocity vector or an initial speed, or the state was read off a
    // trajectory this stack itself authored (replan / formation change).
    // False means the mission said nothing about how the flight begins,
    // which this planner treats as an incomplete mission rather than a
    // request to start at cruise: the initial phase is part of the product,
    // so an initial state is part of the input.
    bool start_state_stated_{false};
    Eigen::Vector3d commanded_initial_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d commanded_initial_acceleration_{Eigen::Vector3d::Zero()};

    // --- TEST: inject a non-zero initial velocity/acceleration into the plan's
    // head-state boundary condition, to visualize its effect (e.g. start_vel up
    // while the goal is left → trajectory shoots up then curves to the goal).
    // Off by default; toggled by the "test/inject_init_state" ROS parameter.
    bool inject_init_state_;
    Eigen::Vector3d inject_init_vel_;
    Eigen::Vector3d inject_init_acc_;
    bool flag_escape_emergency_;
    bool enable_debug_logs_;
    bool enable_waypoint_markers_;
    bool enable_global_trajectory_pub_;

    // Formation manager variables
    int num_drones_;
    std::string current_formation_type_;
    double current_formation_scale_;
    Eigen::Vector3d current_formation_center_;
    std::vector<Eigen::Vector3d> current_formation_pattern_;  // Full formation pattern from TrajectoryCommand

    // Mission sequencing for robustness
    int last_received_sequence_;        // Last received sequence number to check duplicates
    // Outcome of the last triggerGlobalPlan run. trajectoryCommandCallback
    // consumes the sequence number only when this is true: bumping it before
    // a failed/rejected plan made an identical retry a "duplicate", silently
    // stranding the mission with no reject signal.
    bool last_plan_succeeded_{false};
    // [PLAN-OUTCOME] tri-state contract from the planner (planning_result.h):
    // last_plan_succeeded_ stays the retry/rollback gate; these carry the
    // "flyable but a requirement was relaxed" distinction a bool cannot.
    // [PHASE] mission final boundary from the command (planner units).
    ego_planner::TailBoundary mission_tail_;
    PlanOutcome last_plan_outcome_{PlanOutcome::FAILED};
    PlanReason last_plan_reason_{PlanReason::NONE};
    std::string current_mission_id_;    // Current mission being executed
    std::string next_mission_id_;       // Next mission to execute
    bool is_final_mission_;             // True if no more missions after current

    // Authoritative copy of the risk zones this FSM has accepted from
    // /mission/risk_zones. RiskZoneArray::replace decides whether an incoming
    // batch overwrites this set or appends to it; the whole set is then pushed
    // to PathManager (setRiskZonesRuntime is replace-only by contract).
    std::vector<RiskZone> active_risk_zones_;

    std::unique_ptr<swarm_formation::LogManager> log_manager_;
};

}  // namespace path_manager

#endif  // REPLAN_FSM_H
