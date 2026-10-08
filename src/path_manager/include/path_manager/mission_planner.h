#pragma once

#include <memory>
#include "path_manager/mission_planning_backend.h"

namespace path_manager {

// Whole-mission entry point with owned inputs/results and an explicit backend.
// This facade creates no ROS node and stores no second copy of live settings.
// Serialize synchronous calls on an instance; configured internal segment
// workers may still run in parallel. Replace model/search implementations only
// while idle between missions. Live environment updates during execution retain
// the ROS adapter's existing revalidation and trajectory withdrawal policy.
class MissionPlanner {
 public:
  MissionPlanner(PlanSettings settings, PlanningEnvironment environment,
                 std::unique_ptr<MissionPlanningBackend> backend);
  ~MissionPlanner();
  MissionPlanResult plan(MissionPlanRequest request);
  // A model is immutable and shared by the ROS backend's optimizers. Null
  // restores that backend's configured default. Update only between missions.
  void setVehicleModel(mmp_vehicle_dynamics::VehicleModel::Ptr model);
  void setRouteSearchBackend(std::shared_ptr<RouteSearchBackend> backend);
  bool setTerrain(TerrainGrid terrain, std::string *error = nullptr);
  void clearTerrain();
  bool setRiskZones(const std::vector<RiskZone> &zones);
  int addDynamicSphere(const Eigen::Vector3d &center, double radius,
                       const std::string &model = "");
  int addDynamicBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size,
                    const std::string &model = "");
  void clearDynamicObstacles();

 private:
  std::unique_ptr<MissionPlanningBackend> backend_;
};
}  // namespace path_manager
