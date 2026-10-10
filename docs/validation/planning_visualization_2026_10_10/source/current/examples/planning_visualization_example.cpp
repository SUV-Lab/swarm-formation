// Owned visualization preparation through the installed public API. This
// example creates no ROS node and no publisher; ROS message types remain.
#include "path_manager/planning_visualization_data.h"
#include <iostream>

int main() {
  namespace viz = path_manager::visualization;
  const viz::Stamp stamp = [] { return builtin_interfaces::msg::Time{}; };
  nav_msgs::msg::Path route;
  std::optional<visualization_msgs::msg::Marker> tube;
  {
    const std::vector<Eigen::Vector3d> points{{0, 0, 2}, {2, 0, 2}};
    route = viz::makeFrontEndPath(points, stamp());
    auto coefficients = poly_traj::CoefficientMat::Zero().eval();
    coefficients(2, 5) = 2.0;
    coefficients(0, 4) = 1.0;
    const poly_traj::Trajectory trajectory({2.0}, {coefficients});
    tube = viz::makeTrajectoryTube(trajectory, 1, 1.5, "opt_path_drone_",
                                  0.55f, 0.72f, 1.0f, 0.22f, stamp);
  }
  // Source points/trajectory are gone; these messages own all their data.
  if (!tube || route.poses.size() != 2 || tube->points.empty()) return 1;
  std::cout << "route_points=" << route.poses.size()
            << " tube_points=" << tube->points.size()
            << " frame=" << tube->header.frame_id << '\n';
  return 0;
}
