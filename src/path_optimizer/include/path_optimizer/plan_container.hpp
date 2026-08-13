#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "poly_traj_utils.hpp"

using std::vector;

namespace ego_planner
{

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;
    double global_start_time; // world time
    double duration;

    /* Global traj time. 
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;
    /* Global traj time. 
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;
  };

  struct LocalTrajData
  {
    poly_traj::Trajectory traj;
    int drone_id; // A negative value indicates no received trajectories.
    // Monotone per node, never reset while a mission runs. On the wire as
    // PolyTraj.trajectory_id, and TrajectoryExecutionControl names it to say
    // WHICH trajectory an abort refers to — so it must not wrap during a
    // flight, hence 64 bits rather than the int it was while nothing read it.
    uint64_t traj_id;
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_pos;
  };

  typedef std::vector<LocalTrajData> SwarmTrajData;

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj;
    LocalTrajData local_traj;
    SwarmTrajData swarm_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
      local_traj.drone_id = -1;  // Initialize to -1, will be set by PathManager
    }
    ~TrajContainer() {}

    void setGlobalTraj(const poly_traj::Trajectory &trajectory, const double &world_time)
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;

      // Don't reset drone_id to -1 here, keep the existing value
      local_traj.duration = 0.0;
      local_traj.traj_id = 0;
    }

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const double &world_time, const int drone_id = -1)
    {
      // Only update drone_id if a valid one is provided
      if (drone_id >= 0) {
        local_traj.drone_id = drone_id;
      }
      local_traj.traj_id++;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
    }

  };

} // namespace ego_planner

#endif