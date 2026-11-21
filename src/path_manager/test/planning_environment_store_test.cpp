#include "path_manager/planning_environment_store.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <type_traits>
#include <functional>

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
  msg->header.frame_id = "map";
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
    store.installPendingObstacles();
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
  const double first_yaw = store.obstacles()[0].yaw;
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
  check(store.obstacles().size()==1 && store.obstacles()[0].center.z()==3.5 &&
            store.obstacles()[0].yaw==first_yaw &&
            store.obstacles()[0].authored.center.z()==99. &&
            store.distanceField().getDistance(Eigen::Vector3d(0,0,3.5))<0.f &&
            store.distanceField().getDistance(Eigen::Vector3d(0,0,2.5))>0.f,
        "same-geometry height update re-grounds raw obstacle input and retains its yaw");
  auto malformed=terrain(7.f); malformed->data[0].data.clear();
  const auto valid_revision=store.revision();
  check(!store.ingestTerrain(malformed).accepted() &&
            store.revision()==valid_revision && store.terrain().getElevation(0,0)==3.f,
        "malformed DEM cannot discard the last accepted field");
  const auto recrop = store.ingestTerrain(terrain(1.f, .5)); finishTerrain(store);
  check(!recrop.obstacles_cleared && recrop.obstacles_requeued &&
            store.obstacles().size()==1 && store.distanceField().numActiveObstacles()==1 &&
            store.obstacles()[0].center.z()==1.5 && store.obstacles()[0].yaw==first_yaw &&
            store.distanceField().getDistance(Eigen::Vector3d(0,0,1.5))<0.f,
        "same-frame crop replacement retains the obstacle and rebuilds its grounded collision patch");
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

  // Append-style individual additions survive repeated re-crops without
  // acquiring new headings or duplicate patches. Compare the NEXT spawn to
  // a control that did not re-crop to catch accidental RNG advancement.
  pm::PlanningEnvironmentStore reloaded(options(),log), yaw_control(options(),log);
  for (auto *s : {&reloaded, &yaw_control}) {
    s->ingestTerrain(terrain(2.f)); finishTerrain(*s);
    s->addBox(Eigen::Vector3d(1,1,91), Eigen::Vector3d(1,3,2), "building");
    s->addSphere(Eigen::Vector3d(-2,-2,92), .4, "sphere");
  }
  const double box_yaw=reloaded.obstacles()[0].yaw;
  const double sphere_yaw=reloaded.obstacles()[1].yaw;
  for (const auto &next : {terrain(7.f,.5), terrain(4.f,.5), terrain(1.f,1.)}) {
    const auto update=reloaded.ingestTerrain(next); finishTerrain(reloaded);
    check(update.obstacles_requeued && !update.obstacles_cleared &&
              reloaded.obstacles().size()==2 && reloaded.distanceField().numActiveObstacles()==2 &&
              reloaded.obstacles()[0].yaw==box_yaw && reloaded.obstacles()[1].yaw==sphere_yaw &&
              reloaded.obstacles()[0].model=="building" && reloaded.obstacles()[1].model=="sphere" &&
              reloaded.obstacles()[0].authored.center==Eigen::Vector3d(1,1,91) &&
              reloaded.obstacles()[0].size==Eigen::Vector3d(1,3,2) &&
              reloaded.distanceField().getDistance(reloaded.obstacles()[0].center)<0.f,
          "repeated crop/height updates preserve both authored shapes, models, yaw and collision occupancy");
  }
  reloaded.addBox(Eigen::Vector3d(3,3,0),Eigen::Vector3d(2,1,1));
  yaw_control.addBox(Eigen::Vector3d(3,3,0),Eigen::Vector3d(2,1,1));
  check(reloaded.obstacles().back().yaw==yaw_control.obstacles().back().yaw,
        "terrain replay consumes no RNG: next appended spawn matches an unreloaded control");
  reloaded.clearObstacles();  // Same primitive used by replace=true and Clear.
  reloaded.addSphere(Eigen::Vector3d(2,2,0),.6,"replacement");
  reloaded.ingestTerrain(terrain(8.f,.5)); finishTerrain(reloaded);
  check(reloaded.obstacles().size()==1 && reloaded.obstacles()[0].model=="replacement" &&
            std::abs(reloaded.obstacles()[0].center.z()-8.6)<1e-12,
        "explicit replace removes old authored shapes; only its replacement survives the next crop");
  reloaded.clearObstacles();
  reloaded.ingestTerrain(terrain(6.f)); finishTerrain(reloaded);
  check(reloaded.obstacles().empty() && reloaded.distanceField().numActiveObstacles()==0 &&
            reloaded.installPendingObstacles()==0,
        "explicit obstacle Clear cannot resurrect objects on a later terrain reload");

  auto no_snap_options=options(); no_snap_options.obstacle_ground_snap=false;
  pm::PlanningEnvironmentStore absolute(no_snap_options,log);
  absolute.ingestTerrain(terrain(1.f)); finishTerrain(absolute);
  absolute.addBox(Eigen::Vector3d(1,1,7),Eigen::Vector3d(1,2,3));
  const double absolute_yaw=absolute.obstacles()[0].yaw;
  absolute.ingestTerrain(terrain(9.f,.5)); finishTerrain(absolute);
  check(absolute.obstacles()[0].center==Eigen::Vector3d(1,1,7) &&
            absolute.obstacles()[0].yaw==absolute_yaw &&
            absolute.distanceField().getDistance(Eigen::Vector3d(1,1,7))<0.f,
        "ground-snap disabled preserves authored absolute height across a crop and DEM change");
  auto different_frame=terrain(2.f); different_frame->header.frame_id="other_map";
  const auto frame_update=absolute.ingestTerrain(different_frame); finishTerrain(absolute);
  check(frame_update.obstacles_cleared && absolute.obstacles().empty() &&
            absolute.installPendingObstacles()==0,
        "explicit coordinate-frame change clears old-frame authored obstacles");

  pm::PlanningEnvironmentStore queued(options(),log);
  queued.addBox(Eigen::Vector3d(0,0,12),Eigen::Vector3d(1,2,2));
  bool surfaced_failure=false;
  try { queued.installPendingObstacles(); }
  catch (const std::runtime_error &) { surfaced_failure=true; }
  queued.ingestTerrain(terrain(5.f)); finishTerrain(queued);
  check(surfaced_failure && queued.obstacles().size()==1 && queued.obstacles()[0].center.z()==6.,
        "attempted replay before SDF readiness fails visibly without losing the accepted pending input");
  queued.ingestTerrain(std::make_shared<grid_map_msgs::msg::GridMap>());
  queued.ingestTerrain(terrain(3.f)); finishTerrain(queued);
  check(queued.obstacles().empty() && queued.distanceField().numActiveObstacles()==0,
        "explicit world clear purges live authored state and later DEM loading cannot resurrect it");
  queued.ingestTerrain(std::make_shared<grid_map_msgs::msg::GridMap>());
  queued.addSphere(Eigen::Vector3d(0,0,11),.3);
  queued.ingestTerrain(std::make_shared<grid_map_msgs::msg::GridMap>());
  queued.ingestTerrain(terrain(3.f)); finishTerrain(queued);
  check(queued.obstacles().empty() && queued.installPendingObstacles()==0,
        "world clear also purges pending-only authored input");

  pm::PlanningEnvironmentStore mixed_pending(options(),log);
  mixed_pending.ingestTerrain(terrain(1.f)); finishTerrain(mixed_pending);
  mixed_pending.addBox(Eigen::Vector3d(1,1,50),Eigen::Vector3d(1,2,2));
  const double pending_box_yaw=mixed_pending.obstacles()[0].yaw;
  mixed_pending.ingestTerrain(terrain(2.f,.5));
  const int new_pending=mixed_pending.addSphere(Eigen::Vector3d(-1,-1,60),.4);
  mixed_pending.ingestTerrain(terrain(4.f));
  auto bad_pending_map=terrain(9.f); bad_pending_map->data[0].data.clear();
  const bool pending_rejected=!mixed_pending.ingestTerrain(bad_pending_map).accepted();
  finishTerrain(mixed_pending);
  check(new_pending==-2 && pending_rejected && mixed_pending.obstacles().size()==2 &&
            mixed_pending.obstacles()[0].yaw==pending_box_yaw &&
            mixed_pending.obstacles()[0].center.z()==5. &&
            std::abs(mixed_pending.obstacles()[1].center.z()-4.4)<1e-12 &&
            mixed_pending.installPendingObstacles()==0,
        "existing replay plus newly appended pending input survives repeated and rejected terrain updates exactly once");

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

  auto auto_options = options(); auto_options.sdf_voxel_size_auto = true;
  pm::PlanningEnvironmentStore transactional(auto_options, log, {zone});
  transactional.ingestTerrain(terrain(2.f)); finishTerrain(transactional);
  transactional.addSphere(Eigen::Vector3d(0,0,0), .5);
  const auto accepted_revision = transactional.revision();
  const auto accepted_fingerprint = transactional.riskZoneFingerprint();
  using Message = grid_map_msgs::msg::GridMap;
  const std::vector<std::function<void(Message &)>> malformed_updates{
    [](Message &m) { m.data[0].data.clear(); },
    [](Message &m) { m.layers = {"wrong"}; },
    [](Message &m) { m.data[0].layout.dim.resize(1); },
    [](Message &m) { m.info.resolution = std::numeric_limits<double>::infinity(); },
    [](Message &m) { m.info.length_x = std::numeric_limits<double>::quiet_NaN(); },
    [](Message &m) { m.info.length_y = -1.; },
    [](Message &m) { m.info.pose.position.x = std::numeric_limits<double>::infinity(); },
    [](Message &m) { m.data[0].layout.dim[0].size = std::numeric_limits<uint32_t>::max(); }
  };
  bool rejected_atomically = true;
  for (const auto &mutate : malformed_updates) {
    auto bad = terrain(9.f, 2.); mutate(*bad);
    rejected_atomically &= !transactional.ingestTerrain(bad).accepted() &&
        transactional.revision() == accepted_revision &&
        transactional.voxelSize() == 1. &&
        transactional.terrain().getElevation(0,0) == 2.f &&
        transactional.obstacles().size() == 1 &&
        transactional.riskZoneFingerprint() == accepted_fingerprint;
  }
  check(rejected_atomically,
        "all malformed ROS terrain forms preserve data, auto pitch, revisions, risk and obstacles");
  transactional.ingestTerrain(terrain(4.f, .5)); finishTerrain(transactional);
  check(transactional.voxelSize() == .5 && transactional.terrain().getElevation(0,0) == 4.f,
        "valid recrop after rejected inputs still updates auto pitch and terrain");

  pm::PlanningEnvironmentStore invalid_obstacles(options(), log), control(options(), log);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejectObstacles = [&](pm::PlanningEnvironmentStore &s) {
    const auto before = s.revision();
    return s.addSphere(Eigen::Vector3d(nan,0,0), 1.) == -1 &&
        s.addSphere(Eigen::Vector3d::Zero(), -1.) == -1 &&
        s.addSphere(Eigen::Vector3d::Zero(), nan) == -1 &&
        s.addBox(Eigen::Vector3d(0,nan,0), Eigen::Vector3d::Ones()) == -1 &&
        s.addBox(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.,0.,1.)) == -1 &&
        s.addBox(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.,nan,1.)) == -1 &&
        s.revision() == before && s.obstacles().empty();
  };
  check(rejectObstacles(invalid_obstacles),
        "malformed obstacles are rejected before deferral and do not change revisions");
  invalid_obstacles.ingestTerrain(terrain(0.f)); finishTerrain(invalid_obstacles);
  control.ingestTerrain(terrain(0.f)); finishTerrain(control);
  check(invalid_obstacles.obstacles().empty() && rejectObstacles(invalid_obstacles),
        "rejected obstacles never enter the deferred queue or live distance field");
  invalid_obstacles.addBox(Eigen::Vector3d::Zero(), Eigen::Vector3d(1,2,3));
  control.addBox(Eigen::Vector3d::Zero(), Eigen::Vector3d(1,2,3));
  check(invalid_obstacles.obstacles()[0].yaw == control.obstacles()[0].yaw &&
            invalid_obstacles.distanceField().getDistance(Eigen::Vector3d(.2,.1,1.5)) ==
                control.distanceField().getDistance(Eigen::Vector3d(.2,.1,1.5)),
        "rejected obstacles consume no yaw RNG and cannot affect the next valid shape");
  pm::PendingEnvironmentObstacle replacement{false,Eigen::Vector3d(4,4,0),
      Eigen::Vector3d::Zero(),.7,"replacement"};
  auto bad_replacement=replacement;bad_replacement.radius=nan;
  const auto batch_revision=control.revision();
  const auto retained=control.obstacles().front();
  check(!control.replaceObstacles({replacement,bad_replacement})&&
      control.revision()==batch_revision&&control.obstacles().size()==1&&
      control.obstacles().front().id==retained.id&&control.obstacles().front().yaw==retained.yaw,
      "invalid batch is rejected before replacing any accepted obstacle or advancing RNG");
  check(control.replaceObstacles({replacement})&&control.obstacles().size()==1&&
      control.obstacles().front().model=="replacement"&&
      control.distanceField().getDynamicDistance(Eigen::Vector3d(4,4,.7))<0,
      "complete replacement installs the new collision geometry");
  check(control.replaceObstacles({})&&control.obstacles().empty()&&
      control.distanceField().numActiveObstacles()==0,
      "empty complete batch removes every old collision patch");
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed checks\n";
  return failures ? 1 : 0;
}
