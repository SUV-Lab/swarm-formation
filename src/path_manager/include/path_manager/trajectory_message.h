#ifndef PATH_MANAGER_TRAJECTORY_MESSAGE_H
#define PATH_MANAGER_TRAJECTORY_MESSAGE_H

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <mmp_traj_msgs/msg/poly_traj.hpp>

#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {

// ROS boundary only: copy an owned calculation result into the existing wire
// format. The caller chooses identity and timestamp; this does not read a
// planner, allocate an execution ID, convert coordinates, or publish anything.
mmp_traj_msgs::msg::PolyTraj makeTrajectoryMessage(
    const poly_traj::Trajectory &trajectory, int drone_id,
    uint64_t trajectory_id, const builtin_interfaces::msg::Time &start_time);

}  // namespace path_manager

#endif  // PATH_MANAGER_TRAJECTORY_MESSAGE_H
