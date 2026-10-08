#pragma once

#include "path_manager/mission_plan.h"
#include "path_manager/plan_settings.h"
#include "path_manager/planning_environment.h"

namespace path_manager {

// A synchronous, explicitly selected implementation. initialize is called once
// by MissionPlanner construction; invalid startup inputs throw with detail.
// Instances are serially used; an implementation may run internal workers.
class MissionPlanningBackend {
 public:
  virtual ~MissionPlanningBackend() = default;
  virtual void initialize(PlanSettings settings, PlanningEnvironment environment) = 0;
  virtual MissionPlanResult plan(MissionPlanRequest request) = 0;
  virtual void setVehicleModel(mmp_vehicle_dynamics::VehicleModel::Ptr model) = 0;
  virtual void setRouteSearchBackend(std::shared_ptr<RouteSearchBackend> backend) = 0;
  // A rejected value grid retains the old environment. The error pointer is
  // synchronous, not retained, and is cleared on success. NaN cells are valid.
  virtual bool setTerrain(TerrainGrid terrain, std::string *error = nullptr) = 0;
  virtual void clearTerrain() = 0;
  virtual bool setRiskZones(const std::vector<RiskZone> &zones) = 0;
  // ROS compatibility backend preserves existing IDs: >=0 added, -2 accepted
  // pending the first SDF build, -1 rejected. No value here implies execution.
  virtual int addDynamicSphere(const Eigen::Vector3d &center, double radius,
                               const std::string &model) = 0;
  virtual int addDynamicBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size,
                            const std::string &model) = 0;
  virtual void clearDynamicObstacles() = 0;
};
}  // namespace path_manager
