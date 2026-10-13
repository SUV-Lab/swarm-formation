#include "path_manager/planning_environment_store.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <type_traits>

namespace pm = path_manager;
namespace {
int failures = 0;
void check(bool ok, const char *message) {
  std::cout << (ok ? "[OK] " : "[FAIL] ") << message << '\n';
  failures += !ok;
}
pm::PlanningEnvironmentOptions options() {
  return {1.0, 0.1, false, 3.0, true, 42,
          true, 0.35, true, 0.0, 0.1, true};
}
grid_map_msgs::msg::GridMap::SharedPtr terrain(float height, double resolution = 1.0) {
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  msg->info.resolution = resolution;
  msg->info.length_x = msg->info.length_y = 20.0 * resolution;
  msg->layers = {"elevation"}; msg->data.resize(1);
  auto &data = msg->data.front(); data.layout.dim.resize(2);
  data.layout.dim[0].size = data.layout.dim[1].size = 20;
  data.data.assign(400, height);
  return msg;
}
void finishTerrain(pm::PlanningEnvironmentStore &store) {
  if (store.ensureTerrainDistanceField()) {
    for (const auto &p : store.takePendingObstacles()) {
      if (p.is_box) store.addBox(p.center, p.size, p.model);
      else store.addSphere(p.center, p.radius, p.model);
    }
  }
  store.refreshRisk();
}
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  static_assert(!std::is_copy_constructible_v<pm::PlanningEnvironmentStore>);
  static_assert(!std::is_move_constructible_v<pm::PlanningEnvironmentStore>);
  auto log = std::make_shared<swarm_formation::LogManager>(
      "environment_store_test", "/tmp/mmp_environment_store_test_logs", swarm_formation::LogManager::WARN);
  const pm::RiskZone zone{Eigen::Vector3d(.5,.5,2.), 10., .8};
  pm::PlanningEnvironmentStore store(options(), log, {zone});
  check(store.riskZones().size() == 1 && store.zoneGeneration() == 1,
        "construction owns and derives the initial zone list");
  const auto before_queue = store.revision();
  check(store.addSphere(Eigen::Vector3d(0,0,99), .5) == -2 &&
            store.revision() == before_queue && store.obstacles().empty(),
        "deferred input is reported without inventing a live SDF mutation");
  auto map = terrain(2.f);
  const auto first = store.ingestTerrain(map);
  check(first.kind == pm::TerrainUpdateKind::REPLACED && !first.obstacles_cleared,
        "first DEM returns an explicit accepted update");
  finishTerrain(store);
  map->data[0].data.assign(400, 99.f);
  check(store.terrain().getElevation(0,0) == 2.f &&
            store.riskZones()[0].center.z() == 4.,
        "terrain and AGL sources are owned after caller input changes");
  check(store.obstacles().size() == 1 &&
            store.obstacles()[0].center.z() == 2.5 &&
            store.distanceField().getDistance(Eigen::Vector3d(0,0,2.5)) < 0.f,
        "deferred obstacle is grounded only after its terrain arrives");
  check(store.takePendingObstacles().empty(), "deferred queue transfers exactly once");
  const auto old_fingerprint = store.riskZoneFingerprint();
  const auto old_zone_generation = store.zoneGeneration();
  auto invalid_zone = zone; invalid_zone.peak = std::numeric_limits<double>::quiet_NaN();
  check(!store.setRiskZones({zone, invalid_zone}) &&
            store.riskZoneFingerprint() == old_fingerprint &&
            store.zoneGeneration() == old_zone_generation,
        "malformed zone update preserves the complete live field");
  const auto revision = store.revision();
  store.ingestTerrain(terrain(3.f)); finishTerrain(store);
  float h=0, gx=0, gy=0;
  check(store.terrain().getElevationAndGrad(0,0,&h,&gx,&gy) && h==3.f &&
            store.revision()>revision && store.riskZones()[0].center.z()==5.,
        "same-geometry DEM refresh advances generations and recomputes AGL sources");
  check(store.obstacles().size()==1 && store.obstacles()[0].center.z()==2.5,
        "same-geometry update retains the existing obstacle grounding policy");
  auto malformed=terrain(7.f); malformed->data[0].data.clear();
  const auto valid_revision=store.revision();
  check(!store.ingestTerrain(malformed).accepted() &&
            store.revision()==valid_revision && store.terrain().getElevation(0,0)==3.f,
        "malformed DEM cannot discard the last accepted field");
  const auto recrop = store.ingestTerrain(terrain(1.f, .5)); finishTerrain(store);
  check(recrop.obstacles_cleared && store.obstacles().empty() &&
            store.distanceField().numActiveObstacles()==0,
        "geometry replacement reports and removes old-world obstacle patches");
  store.addBox(Eigen::Vector3d(0,0,99), Eigen::Vector3d(1,2,3));
  const auto clear = store.ingestTerrain(std::make_shared<grid_map_msgs::msg::GridMap>());
  store.refreshRisk();
  check(clear.kind==pm::TerrainUpdateKind::CLEARED && clear.had_terrain &&
            clear.obstacles_cleared && !store.terrain().valid &&
            !store.distanceField().hasData() && store.obstacles().empty(),
        "explicit world clear drops terrain, obstacle patches and old SDF geometry");
  check(store.addSphere(Eigen::Vector3d(0,0,0),.5)==-2,
        "obstacles after world clear are deferred for the next terrain");
  store.clearObstacles();
  store.ingestTerrain(terrain(0.f));
  check(store.ensureTerrainDistanceField() && store.takePendingObstacles().empty(),
        "explicit obstacle clear drops pending inputs before the next grid becomes ready");
  store.refreshRisk();

  auto ridge = terrain(0.f);
  for (int col=0;col<20;++col) ridge->data[0].data[col*20+6]=4.f;
  pm::PlanningEnvironmentStore risk(options(), log, {zone});
  risk.ingestTerrain(ridge); finishTerrain(risk);
  const Eigen::Vector3d behind(6.5,.5,2.);
  const double low=risk.visibilityValue(0,behind,.15);
  const double high=risk.visibilityValue(0,behind,10.);
  check(low<.01 && high>.99 && risk.visibilityValue(0,behind,.15)==low,
        "per-call grounding clearance changes queries without changing another caller's field");
  Eigen::Vector3d gradient;
  check(risk.visibility(0,behind,.15,&gradient)==low && gradient.allFinite(),
        "visibility and gradient share the explicit clearance input");
  check(risk.riskValue(0,behind,10.)>risk.riskValue(0,behind,.15),
        "composed risk uses the same per-call grounded visibility");

  pm::PlanningEnvironmentStore deferred(options(),log), immediate(options(),log);
  deferred.addSphere(Eigen::Vector3d(0,0,9),.5);
  deferred.ingestTerrain(terrain(0.f)); finishTerrain(deferred);
  immediate.ingestTerrain(terrain(0.f)); finishTerrain(immediate);
  immediate.addSphere(Eigen::Vector3d(0,0,9),.5);
  deferred.addBox(Eigen::Vector3d(2,2,9),Eigen::Vector3d(1,2,3));
  immediate.addBox(Eigen::Vector3d(2,2,9),Eigen::Vector3d(1,2,3));
  check(deferred.obstacles()[0].yaw==immediate.obstacles()[0].yaw &&
            deferred.obstacles()[1].yaw==immediate.obstacles()[1].yaw &&
            deferred.distanceField().getDistance(Eigen::Vector3d(2.2,2.1,1.5))==
                immediate.distanceField().getDistance(Eigen::Vector3d(2.2,2.1,1.5)),
        "deferred replay preserves RNG order and oriented obstacle collision geometry");
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed checks\n";
  return failures ? 1 : 0;
}
