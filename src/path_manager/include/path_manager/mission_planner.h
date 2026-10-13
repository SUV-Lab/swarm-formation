#pragma once

#include <memory>
#include "path_manager/mission_planning_backend.h"

namespace path_manager {

// Whole-mission entry point with owned inputs/results and an explicit backend.
// This facade creates no ROS node and stores no second copy of live settings.
// Serialize synchronous calls on an instance; configured internal segment
// workers may still run in parallel. Replace model/search implementations only
// while idle between missions. Environment setters delegate to the backend.
// With RosPlanningBackend, ordinary setter calls update planning inputs without
// revalidating or withdrawing the adopted flight; ReplanFSM's topic callbacks
// perform that execution policy. Terrain installation can also flush deferred
// obstacles, which revalidates that newly installed batch.
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
  // RosPlanningBackend also clears the SDF and installed/pending dynamic
  // obstacles. Authored risk zones are retained.
  void clearTerrain();
  bool setRiskZones(const std::vector<RiskZone> &zones);
  // RosPlanningBackend: >=0 installed ID, -2 accepted but deferred until the
  // SDF is ready, -1 rejected. Acceptance does not imply execution validation.
  int addDynamicSphere(const Eigen::Vector3d &center, double radius,
                       const std::string &model = "");
  int addDynamicBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size,
                    const std::string &model = "");
  void clearDynamicObstacles();

 private:
  std::unique_ptr<MissionPlanningBackend> backend_;
};
}  // namespace path_manager
