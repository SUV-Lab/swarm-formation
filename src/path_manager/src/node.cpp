#include <rclcpp/rclcpp.hpp>
#include "path_manager/replan_fsm.h"
#include <rclcpp/executors.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<path_manager::ReplanFSM>();
    auto node = std::make_shared<rclcpp::Node>("path_manager");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{});
    path_manager::ReplanFSM ego_replan(node);
    // The FSM idles until a /trajectory_command arrives (startMissionPlan).
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}