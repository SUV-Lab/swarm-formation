#ifndef PATH_MANAGER_COMMITTED_ROUTE_H
#define PATH_MANAGER_COMMITTED_ROUTE_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <Eigen/Core>

namespace path_manager {

// One front-end result in existing planner units. Geometry, altitude caps
// and leg provenance travel together, including after a transition cut.
// epoch identifies the manager's policy captures for these edge tags; it
// is not an environment snapshot or a whole-flight policy verdict.
struct CommittedRoute {
  std::vector<Eigen::Vector3d> path;
  std::vector<double> cap;
  std::vector<size_t> edge_leg;
  uint64_t epoch{0};
  double front_end_ms{0.0};
};

}  // namespace path_manager

#endif  // PATH_MANAGER_COMMITTED_ROUTE_H
