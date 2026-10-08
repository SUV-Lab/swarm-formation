#include "path_manager/mission_planner.h"

#include <stdexcept>
#include <utility>

namespace path_manager {
MissionPlanner::MissionPlanner(PlanSettings settings, PlanningEnvironment environment,
                               std::unique_ptr<MissionPlanningBackend> backend)
    : backend_(std::move(backend))
{
  if (!backend_) throw std::invalid_argument("MissionPlanner requires an explicit backend");
  backend_->initialize(std::move(settings), std::move(environment));
}
MissionPlanner::~MissionPlanner() = default;
MissionPlanResult MissionPlanner::plan(MissionPlanRequest request)
{
  return backend_->plan(std::move(request));
}
void MissionPlanner::setVehicleModel(mmp_vehicle_dynamics::VehicleModel::Ptr model)
{
  backend_->setVehicleModel(std::move(model));
}
void MissionPlanner::setRouteSearchBackend(std::shared_ptr<RouteSearchBackend> backend)
{
  backend_->setRouteSearchBackend(std::move(backend));
}
bool MissionPlanner::setTerrain(TerrainGrid terrain, std::string *error)
{
  return backend_->setTerrain(std::move(terrain), error);
}
void MissionPlanner::clearTerrain() { backend_->clearTerrain(); }
bool MissionPlanner::setRiskZones(const std::vector<RiskZone> &zones)
{
  return backend_->setRiskZones(zones);
}
int MissionPlanner::addDynamicSphere(const Eigen::Vector3d &center, double radius,
                                    const std::string &model)
{
  return backend_->addDynamicSphere(center, radius, model);
}
int MissionPlanner::addDynamicBox(const Eigen::Vector3d &center,
                                 const Eigen::Vector3d &size, const std::string &model)
{
  return backend_->addDynamicBox(center, size, model);
}
void MissionPlanner::clearDynamicObstacles() { backend_->clearDynamicObstacles(); }
}  // namespace path_manager
