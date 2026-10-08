#include "path_manager/trajectory_message.h"

namespace path_manager {

mmp_traj_msgs::msg::PolyTraj makeTrajectoryMessage(
    const poly_traj::Trajectory &trajectory, int drone_id,
    uint64_t trajectory_id, const builtin_interfaces::msg::Time &start_time)
{
  mmp_traj_msgs::msg::PolyTraj message;
  message.drone_id = drone_id;
  message.trajectory_id = trajectory_id;
  message.start_time = start_time;
  message.order = 5;
  const Eigen::VectorXd durations = trajectory.getDurations();
  const int pieces = trajectory.getPieceNum();
  message.duration.resize(pieces);
  message.coef_x.resize(6 * pieces);
  message.coef_y.resize(6 * pieces);
  message.coef_z.resize(6 * pieces);
  for (int i = 0; i < pieces; ++i) {
    message.duration[i] = durations(i);
    const auto coefficients = trajectory.getPiece(i).getCoeffMat();
    for (int j = 0; j < 6; ++j) {
      const int index = i * 6 + j;
      message.coef_x[index] = coefficients(0, j);
      message.coef_y[index] = coefficients(1, j);
      message.coef_z[index] = coefficients(2, j);
    }
  }
  return message;
}

}  // namespace path_manager
