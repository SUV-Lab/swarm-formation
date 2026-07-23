#ifndef REPLAN_FSM_H
#define REPLAN_FSM_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <mutex>
#include <map>
#include "path_manager/msg/poly_traj.hpp"
#include "path_manager/msg/formation_target.hpp"
#include "formation_msgs/msg/trajectory_command.hpp"
#include "path_manager/msg/position_command.hpp"
#include "path_manager/msg/dynamic_obstacle_array.hpp"
#include "path_manager/msg/dynamic_obstacle_spec.hpp"
#include "path_manager/msg/risk_zone_array.hpp"
#include "path_manager/msg/risk_zone_spec.hpp"
#include "path_manager/path_manager.h"
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

    // [RACE-PROBE] temporary diagnostics: proves/refutes timer-vs-subscription
    // concurrency on traj_ (set around planGlobalTraj, checked in the timer).
    std::atomic<bool> plan_writer_active_{false};
    std::atomic<int> race_overlap_count_{0};
    
    void init();
    void computeAndPublishPaths();
    void formationTargetCallback(const path_manager::msg::FormationTarget::SharedPtr msg);
    void trajectoryCommandCallback(const formation_msgs::msg::TrajectoryCommand::SharedPtr msg);
    void terrainCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    // RViz-driven dynamic obstacle layer. Click in RViz with the "Publish Point"
    // tool → /clicked_point → spawn a fixed-radius sphere obstacle into the SDF.
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void clearObstaclesCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void loadObstaclesCallback(
        const path_manager::msg::DynamicObstacleArray::SharedPtr msg);
    void loadRiskZonesCallback(
        const path_manager::msg::RiskZoneArray::SharedPtr msg);
    void polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg);
    void globalTraj2ROSMsg(path_manager::msg::PolyTraj &msg);
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
    
    // Formation target publishing (now receives pre-calculated targets)
    void publishFormationTarget(const Eigen::Vector3d& target, const std::vector<Eigen::Vector3d>& waypoints = {}, bool formation_changed = false, const Eigen::Vector3d& formation_offset = Eigen::Vector3d::Zero());

    std::shared_ptr<PathManager> path_manager_;

    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr global_path_pub_;
    rclcpp::Subscription<formation_msgs::msg::TrajectoryCommand>::SharedPtr trajectory_cmd_sub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_obstacles_sub_;
    rclcpp::Subscription<path_manager::msg::DynamicObstacleArray>::SharedPtr
        load_obstacles_sub_;
    rclcpp::Subscription<path_manager::msg::RiskZoneArray>::SharedPtr
        load_risk_zones_sub_;
    double dynamic_obstacle_radius_;  // m, applied to clicked-point spheres
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
    double initial_speed_unit_m_{100.0};
    bool use_commanded_initial_velocity_{false};
    bool use_commanded_initial_acceleration_{false};
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
    std::string current_mission_id_;    // Current mission being executed
    std::string next_mission_id_;       // Next mission to execute
    bool is_final_mission_;             // True if no more missions after current

    std::unique_ptr<swarm_formation::LogManager> log_manager_;
};

}  // namespace path_manager

#endif  // REPLAN_FSM_H
