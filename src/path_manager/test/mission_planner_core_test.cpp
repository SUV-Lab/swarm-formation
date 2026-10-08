// Compile the production facade and value validation with Eigen/STL only.
// A scripted backend verifies the ownership/delegation boundary without ROS.
#include "path_manager/mission_planner.h"

#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pm = path_manager;
namespace {
int failures = 0;
void check(bool ok, const std::string &message)
{
  std::cout << (ok ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!ok) ++failures;
}
template<class Exception, class F> bool throws(F f)
{
  try { f(); } catch (const Exception &) { return true; }
  return false;
}
pm::TerrainGrid terrain()
{
  pm::TerrainGrid value;
  value.rows = 3; value.cols = 2; value.resolution = 0.3;
  value.center_x = -2.7; value.center_y = 9.125;
  value.length_x = 0.9123; value.length_y = 0.6234;
  value.elevation = {-1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 99.0f};
  return value;
}
struct Observations {
  int initialized{0}, plans{0}, destroyed{0}, clears{0};
  pm::PlanSettings settings;
  pm::PlanningEnvironment environment;
  pm::MissionPlanRequest request;
};
class ScriptedBackend final : public pm::MissionPlanningBackend {
 public:
  explicit ScriptedBackend(std::shared_ptr<Observations> observations)
      : observations_(std::move(observations)) {}
  ~ScriptedBackend() override { ++observations_->destroyed; }
  void initialize(pm::PlanSettings settings, pm::PlanningEnvironment environment) override
  {
    ++observations_->initialized;
    observations_->settings = std::move(settings);
    observations_->environment = std::move(environment);
  }
  pm::MissionPlanResult plan(pm::MissionPlanRequest request) override
  {
    ++observations_->plans;
    observations_->request = std::move(request);
    pm::MissionPlanResult result;
    result.status = pm::PlanResult::success();
    poly_traj::CoefficientMat coefficients = poly_traj::CoefficientMat::Zero();
    coefficients(0, 5) = observations_->request.head.pos_u.x();
    result.trajectory.emplace_back(2.0, coefficients);
    result.reference_trajectory = result.trajectory;
    result.environment_generation = 17;
    return result;
  }
  void setVehicleModel(mmp_vehicle_dynamics::VehicleModel::Ptr model) override
  { observations_->settings.vehicle_model = std::move(model); }
  void setRouteSearchBackend(std::shared_ptr<pm::RouteSearchBackend> backend) override
  { observations_->settings.route_search_backend = std::move(backend); }
  bool setTerrain(pm::TerrainGrid value, std::string *error) override
  {
    if (!pm::validateTerrainGrid(value, error)) return false;
    observations_->environment.terrain = std::move(value); return true;
  }
  void clearTerrain() override { observations_->environment.terrain.reset(); ++observations_->clears; }
  bool setRiskZones(const std::vector<pm::RiskZone> &zones) override
  { observations_->environment.risk_zones = zones; return true; }
  int addDynamicSphere(const Eigen::Vector3d &center, double radius,
                       const std::string &model) override
  {
    observations_->environment.dynamic_obstacles.emplace();
    observations_->environment.dynamic_obstacles->push_back(pm::SphereObstacle{center, radius, model});
    return -2;
  }
  int addDynamicBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size,
                    const std::string &model) override
  {
    observations_->environment.dynamic_obstacles->push_back(pm::BoxObstacle{center, size, model});
    return 41;
  }
  void clearDynamicObstacles() override { observations_->environment.dynamic_obstacles.emplace(); }
 private:
  std::shared_ptr<Observations> observations_;
};
void validation()
{
  auto valid = terrain(); std::string error = "stale";
  const uint32_t payload = 0x7fc12345;
  std::memcpy(&valid.elevation[1], &payload, sizeof(payload));
  valid.elevation[2] = std::numeric_limits<float>::infinity();
  check(pm::validateTerrainGrid(valid, &error) && error.empty(),
        "rectangular geometry, negative/NaN/infinite cells and extra payload remain valid");
  const auto reject = [&](pm::TerrainGrid invalid, const char *why) {
    check(!pm::validateTerrainGrid(invalid, &error) && !error.empty(), why);
  };
  auto invalid = valid; invalid.rows = 0; reject(invalid, "zero dimensions rejected");
  invalid = valid; invalid.rows = std::numeric_limits<uint32_t>::max();
  invalid.cols = std::numeric_limits<uint32_t>::max(); reject(invalid, "index overflow rejected before multiplication/narrowing");
  invalid = valid; invalid.elevation.resize(5); reject(invalid, "short payload rejected");
  for (double resolution : {0.0, -0.1, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()}) {
    invalid = valid; invalid.resolution = resolution;
    reject(invalid, "nonpositive/nonfinite resolution rejected");
  }
  invalid = valid; invalid.center_x = std::numeric_limits<double>::infinity();
  reject(invalid, "nonfinite center rejected");
  invalid = valid; invalid.length_y = 0.0; reject(invalid, "zero physical length rejected");
  invalid = valid; invalid.center_x = std::numeric_limits<double>::max();
  invalid.length_x = std::numeric_limits<double>::max(); reject(invalid, "overflowing upper bound rejected");
  invalid.center_x = -std::numeric_limits<double>::max(); reject(invalid, "overflowing lower bound rejected");
}
}  // namespace
int main()
{
  validation();
  check(throws<std::invalid_argument>([] { pm::MissionPlanner planner({}, {}, nullptr); }),
        "generic facade refuses an unspecified backend");
  auto seen = std::make_shared<Observations>();
  pm::MissionPlanResult retained;
  {
    pm::PlanSettings settings;
    settings.parameters = {{"integer", int64_t{7}}, {"double", 7.0}, {"boolean", true},
      {"text", std::string("value")}, {"bytes", std::vector<uint8_t>{0, 255}},
      {"bools", std::vector<bool>{true, false}}, {"integers", std::vector<int64_t>{1, 2}},
      {"doubles", std::vector<double>{1.25, 2.5}}, {"strings", std::vector<std::string>{"a", "b"}}};
    pm::PlanningEnvironment environment; environment.terrain = terrain();
    environment.risk_zones = std::vector<pm::RiskZone>{};
    const auto expected_settings = settings.parameters;
    pm::MissionPlanner planner(settings, environment, std::make_unique<ScriptedBackend>(seen));
    settings.parameters.clear(); environment.terrain->elevation.assign(7, -9.0f);
    check(seen->initialized == 1 && seen->settings.parameters == expected_settings &&
              seen->environment.terrain->elevation == terrain().elevation,
          "construction forwards one owned settings/environment copy with exact variant types");
    check(seen->environment.risk_zones.has_value() && seen->environment.risk_zones->empty() &&
              !seen->environment.dynamic_obstacles.has_value(),
          "absent environment field remains distinct from explicitly empty field");
    pm::MissionPlanRequest request; request.head.pos_u.x() = 13.0;
    retained = planner.plan(request); request.head.pos_u.x() = 99.0;
    planner.plan(request);
    check(seen->initialized == 1 && seen->plans == 2 && retained.trajectory.getPos(0.0).x() == 13.0,
          "planning neither reapplies settings nor rewrites previous owned results");
    auto invalid = terrain(); invalid.elevation.clear(); std::string error;
    check(!planner.setTerrain(invalid, &error) && !error.empty() &&
              seen->environment.terrain->elevation == terrain().elevation,
          "terrain refusal and old-map retention cross the facade unchanged");
    check(planner.setTerrain(terrain(), &error) && error.empty(), "successful terrain update clears synchronous error");
    planner.clearTerrain(); check(!seen->environment.terrain && seen->clears == 1, "terrain clear is explicit");
    check(planner.setRiskZones({}) && seen->environment.risk_zones->empty(), "zone clear forwards explicitly");
    check(planner.addDynamicSphere(Eigen::Vector3d(1, 2, 3), 4.0, "sphere") == -2 &&
              planner.addDynamicBox(Eigen::Vector3d(4, 5, 6), Eigen::Vector3d(7, 8, 9), "box") == 41 &&
              seen->environment.dynamic_obstacles->size() == 2,
          "obstacle arguments/order and deferred/add IDs reach the caller");
    planner.clearDynamicObstacles(); check(seen->environment.dynamic_obstacles->empty(), "obstacle clear forwarded");
  }
  check(seen->destroyed == 1 && retained.hasTrajectory() &&
            retained.trajectory.getPos(1.0).x() == 13.0 &&
            retained.reference_trajectory.getPos(1.0).x() == 13.0 &&
            retained.environment_generation == 17,
        "owned final/reference results survive all planner/backend input lifetimes");
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
