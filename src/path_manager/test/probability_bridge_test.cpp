#include <path_manager/path_manager.h>
#include <risk_model/model.hpp>
#include <mmp_mission_msgs/msg/risk_environment.hpp>
#include <mmp_traj_msgs/msg/trajectory_risk_evaluation.hpp>
#include <mmp_traj_msgs/msg/poly_traj.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {
using Models=mmp_mission_msgs::msg::RiskModelArray;
using Source=mmp_mission_msgs::msg::RiskModelSpec;
using Environment=mmp_mission_msgs::msg::RiskEnvironment;
using Evaluation=mmp_traj_msgs::msg::TrajectoryRiskEvaluation;
using Wire=mmp_traj_msgs::msg::PolyTraj;
int checks=0,failures=0;
void check(bool ok,const std::string& label) {
  ++checks;
  std::cout<<(ok?"[OK] ":"[FAIL] ")<<label<<'\n';
  if(!ok)++failures;
}
bool near(double a,double b,double abs=1e-9,double rel=1e-7) {
  return std::isfinite(a)&&std::isfinite(b)&&
      std::abs(a-b)<=abs+rel*std::max(std::abs(a),std::abs(b));
}
bool until(rclcpp::executors::SingleThreadedExecutor& executor,
           const std::function<bool()>& ready,int milliseconds=2500) {
  const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(milliseconds);
  do {
    executor.spin_some(std::chrono::milliseconds(10));
    if(ready())return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while(std::chrono::steady_clock::now()<end);
  return ready();
}
Models fixture() {
  Models value;
  value.schema_version=2;value.replace=true;
  value.unit_xy_m=100;value.unit_z_m=100;
  value.rcs_a=.3172;value.rcs_b=.1784;value.rcs_c=1.003;
  value.sigma_ref_m2=1;value.fallback_v0_mps=1;value.fallback_v1_mps=5;
  value.height_width_m=5;value.radius_width_m=5;
  value.center_rho_width=.001;value.plateau_rho_width=.01;
  value.visibility_width_m=5;value.clearance_m=10;value.grounded=true;
  value.reference_length_configured=true;value.reference_length_m=100;
  Source source;
  source.source_id="A";source.center.z=.1;source.height_mode=Source::MSL;
  source.c1=1;source.c2=risk_model::calibratedC2(700,1);
  source.calibration_from_r50=true;
  source.calibration_r50_m=700;source.calibration_sigma_m2=1;
  source.r_max_m=1000;source.h_max_m=1000;
  source.r_min_m=0;source.h_min_m=0;source.p0=.8;source.rho_min=0;source.exponent=2;
  source.policy_reach_frame=.5;
  value.sources.push_back(source);
  return value;
}
grid_map_msgs::msg::GridMap::SharedPtr flatTerrain(float height) {
  auto result=std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr int size=24;
  result->info.resolution=1;result->info.length_x=size;result->info.length_y=size;
  result->info.pose.orientation.w=1;
  result->layers={"elevation"};result->data.resize(1);
  auto& layer=result->data.front();layer.layout.dim.resize(2);
  layer.layout.dim[0].label="column_index";layer.layout.dim[0].size=size;
  layer.layout.dim[0].stride=size*size;
  layer.layout.dim[1].label="row_index";layer.layout.dim[1].size=size;
  layer.layout.dim[1].stride=size;
  layer.data.assign(size*size,height);
  return result;
}
Wire linearTrajectory(const path_manager::PathManager& manager,std::uint64_t id) {
  Wire value;
  value.drone_id=17;value.trajectory_id=id;value.planner_session_id=manager.riskSessionId();
  value.start_time.sec=1234;value.start_time.nanosec=5678;
  value.order=1;value.duration={1.0f};
  // x=2+t, y=0, z=3 frame units: a 100 m 3D path at 100 m/s.
  value.coef_x={1.0f,2.0f};value.coef_y={0.0f,0.0f};value.coef_z={0.0f,3.0f};
  return value;
}
} // namespace

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.append_parameter_override("manager/risk_mask_viz_mode",std::string("volume"));
  options.append_parameter_override("optimization/dynamics_unit_xy_m",100.0);
  options.append_parameter_override("optimization/dynamics_unit_z_m",100.0);
  auto node=std::make_shared<rclcpp::Node>("probability_bridge_test",options);
  node->declare_parameter("drone_id",17);
  path_manager::PathManager manager(node,false);
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  manager.setProbabilityCallbackGroup(group);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto qos=rclcpp::QoS(1).reliable().transient_local();
  auto trajectory_pub=node->create_publisher<Wire>("/planning/trajectory",qos);
  std::vector<Evaluation> evaluations;
  auto evaluation_sub=node->create_subscription<Evaluation>(
      "/planning/trajectory_risk_evaluation",qos,[&](Evaluation::ConstSharedPtr message){
        if(message->session_id==manager.riskSessionId()&&message->drone_id==17)
          evaluations.push_back(*message);
      });
  check(manager.probabilityRiskMode(),"probability model is selected by default");
  check(node->get_parameter("manager/risk_model_mode").as_string()=="default",
      "omitted risk_model_mode resolves to default");
  check(!manager.riskProbabilitySnapshot(),"missing environment is not a valid empty environment");
  check(!manager.riskSessionId().empty(),"planner session identity exists");
  check(until(executor,[&]{return trajectory_pub->get_subscription_count()>0;}),
      "trajectory evaluation subscriber discovered");
  trajectory_pub->publish(linearTrajectory(manager,1));
  check(until(executor,[&]{return !evaluations.empty()&&evaluations.back().trajectory_id==1;}),
      "trajectory without an environment produces explicit evaluation status");
  check(!evaluations.empty()&&evaluations.back().status==Evaluation::INVALID&&
      std::isnan(evaluations.back().cumulative_probability),
      "missing probability environment is INVALID/N/A, never valid zero");

  auto models=fixture();std::string reason;
  check(manager.setRiskModelsRuntime(models,&reason),"MSL source accepted before DEM: "+reason);
  auto first=manager.riskProbabilitySnapshot();
  check(first&&first->sources.size()==1,"one source installed");
  if(!first){rclcpp::shutdown();return 1;}
  const Eigen::Vector3d query(2,0,1);
  const auto reference=risk_model::evaluateFrame(first->sources,{2,0,1},{0,0,0},
      first->scale,first->settings,risk_model::FormulaMode::Planning,
      risk_model::RcsMode::Reference,first->terrain.get());
  check(reference.risk.value>0&&reference.flags.visibility_assumed,
      "MSL with missing DEM retains risk and records assumed visibility");
  check(manager.getZoneEllipsoidRadius(0,query)>1,
      "test query is outside the independent policy ellipsoid");
  check(near(manager.getFrontEndRisk(query),reference.risk.value),
      "FM calls common baseline-RCS model beyond policy reach");
  check(near(manager.getEffectiveRisk(0,query),reference.risk.value),
      "manager physical field equals common core");

  auto appended=models;appended.replace=false;appended.sources[0].source_id="B";
  appended.sources[0].center.y=.3;
  check(manager.setRiskModelsRuntime(appended,&reason),"append accepted");
  const auto two=manager.riskProbabilitySnapshot();
  check(two&&two->sources.size()==2&&two->revision>first->revision,"append creates full versioned snapshot");
  check(first->sources.size()==1,"previous snapshot remains immutable after append");

  std::optional<Environment> late_environment;
  auto late=node->create_subscription<Environment>("/planning/risk_environment",qos,
      [&](Environment::ConstSharedPtr message){
        if(message->session_id==manager.riskSessionId())late_environment=*message;
      });
  check(until(executor,[&]{return late_environment&&late_environment->resolved.sources.size()==2;}),
      "late subscriber receives A+B instead of only append delta");
  check(late_environment&&late_environment->valid&&late_environment->resolved.replace&&
      late_environment->revision==two->revision,"latched environment is full resolved snapshot");
  check(late_environment&&late_environment->resolved.sources[0].calibration_from_r50&&
      late_environment->resolved.sources[0].calibration_r50_m==700&&
      late_environment->resolved.sources[0].calibration_sigma_m2==1,
      "accepted full snapshot preserves sensor calibration provenance");

  auto tampered=models;tampered.sources[0].calibration_sigma_m2=2;
  check(!manager.setRiskModelsRuntime(tampered,&reason)&&manager.riskProbabilitySnapshot()==two,
      "inconsistent preserved calibration is rejected without replacing accepted snapshot");
  tampered=models;tampered.sources[0].calibration_from_r50=false;
  check(!manager.setRiskModelsRuntime(tampered,&reason),
      "direct c2 declaration rejects nonzero hidden calibration metadata");

  auto bad=models;bad.sources[0].c2=-1;
  check(!manager.setRiskModelsRuntime(bad,&reason)&&!reason.empty(),"invalid Replace rejected with reason");
  check(manager.riskProbabilitySnapshot()==two,"rejected update preserves previous snapshot atomically");
  auto zero=models;zero.reference_length_m=0;
  check(!manager.setRiskModelsRuntime(zero,&reason),"explicit configured zero exposure is rejected");
  auto wrong_frame=models;wrong_frame.frame_id="odom";
  check(!manager.setRiskModelsRuntime(wrong_frame,&reason),"unknown coordinate frame is rejected");
  auto agl=models;agl.sources[0].height_mode=Source::AGL;
  agl.request_id="agl-initial";
  check(manager.setRiskModelsRuntime(agl,&reason)&&reason.find("Awaiting DEM")!=std::string::npos,
      "valid unresolved AGL request is queued, not reported rejected");
  check(manager.riskProbabilitySnapshot()==two,"waiting AGL does not delete prior valid sources");
  check(until(executor,[&]{return late_environment&&late_environment->has_pending&&
      late_environment->pending_request_id=="agl-initial"&&
      late_environment->request_status==Environment::AWAITING_DEM;}),
      "AWAITING_DEM and request identity are published explicitly");
  check(late_environment&&late_environment->resolved.sources.size()==2&&
      late_environment->pending.sources.size()==1,
      "accepted full environment and pending full request are separate payloads");
  bad.request_id="malformed-during-wait";
  check(!manager.setRiskModelsRuntime(bad,&reason),"malformed input is rejected during DEM wait");
  check(until(executor,[&]{return late_environment&&late_environment->request_id==bad.request_id&&
      late_environment->request_status==Environment::REJECTED&&late_environment->has_pending&&
      late_environment->pending_request_id=="agl-initial";}),
      "rejected request preserves earlier valid pending request");

  manager.setTerrainData(flatTerrain(1.0f));
  const auto grounded=manager.riskProbabilitySnapshot();
  check(grounded&&grounded->sources.size()==1&&grounded->revision>two->revision,
      "DEM receipt automatically accepts waiting AGL without resubmission");
  check(until(executor,[&]{return late_environment&&!late_environment->has_pending&&
      late_environment->request_id=="agl-initial"&&late_environment->request_status==Environment::ACCEPTED;}),
      "automatic approval clears pending and identifies the applied request");
  check(grounded&&near(grounded->sources.front().center_m.z,110),
      "AGL 0.1 frame plus terrain 1.0 frame resolves to 110 m MSL");
  manager.setTerrainData(flatTerrain(2.0f));
  auto changed=manager.riskProbabilitySnapshot();
  check(changed&&grounded&&changed->revision>grounded->revision&&
      changed->terrain_revision>grounded->terrain_revision,
      "DEM replacement advances environment and terrain revisions");
  check(changed&&near(changed->sources.front().center_m.z,210)&&
      grounded&&near(grounded->sources.front().center_m.z,110),
      "DEM change re-resolves AGL while old snapshots retain original center");
  if(!changed){rclcpp::shutdown();return 1;}

  auto outside=agl;outside.request_id="agl-outside";outside.sources[0].center.x=50;
  check(manager.setRiskModelsRuntime(outside,&reason),"outside-DEM AGL request is queued");
  check(until(executor,[&]{return late_environment&&late_environment->has_pending&&
      late_environment->pending_request_id=="agl-outside";}),"outside AGL is published for crop coverage");
  check(late_environment&&late_environment->resolved.sources[0].center.x==0&&
      late_environment->pending.sources[0].center.x==50,
      "new pending XY is visible even while an accepted environment exists");
  manager.setTerrainData(flatTerrain(2.0f));
  check(until(executor,[&]{return late_environment&&late_environment->has_pending&&
      late_environment->revision>changed->revision;}),
      "accepted DEM rebuild preserves an unresolved pending request");
  Models cancel;cancel.cancel_pending=true;cancel.request_id="cancel-outside";
  const auto before_cancel=manager.riskProbabilitySnapshot();
  check(manager.setRiskModelsRuntime(cancel,&reason),"explicit cancellation accepted");
  check(manager.riskProbabilitySnapshot()==before_cancel,"cancellation retains accepted world");
  check(until(executor,[&]{return late_environment&&!late_environment->has_pending&&
      late_environment->request_status==Environment::CANCELLED;}),"cancellation clears only pending coverage");
  check(manager.setRiskModelsRuntime(outside,&reason),"outside AGL may be submitted again after cancellation");
  auto extended=flatTerrain(2.0f);extended->info.pose.position.x=50;
  manager.setTerrainData(extended);
  const auto outside_accepted=manager.riskProbabilitySnapshot();
  check(outside_accepted&&near(outside_accepted->sources[0].center_m.x,5000)&&
      near(outside_accepted->sources[0].center_m.z,210),
      "DEM covering pending XY accepts the request without resubmission");
  check(until(executor,[&]{return late_environment&&!late_environment->has_pending&&
      late_environment->request_id=="agl-outside"&&late_environment->request_status==Environment::ACCEPTED;}),
      "outside-DEM request reaches ACCEPTED after DEM coverage arrives");
  manager.setTerrainData(flatTerrain(2.0f));
  check(manager.setRiskModelsRuntime(agl,&reason),"restore local AGL fixture for trajectory evaluation");
  changed=manager.riskProbabilitySnapshot();

  const auto trajectory=linearTrajectory(manager,10);
  trajectory_pub->publish(trajectory);
  check(until(executor,[&]{return !evaluations.empty()&&evaluations.back().trajectory_id==10&&
      evaluations.back().status!=Evaluation::INVALID;}),"linear polynomial receives finite cumulative evaluation");
  if(evaluations.empty()||evaluations.back().trajectory_id!=10){rclcpp::shutdown();return 1;}
  const auto evaluated=evaluations.back();
  check(evaluated.environment_revision==changed->revision&&
      evaluated.start_time==trajectory.start_time&&evaluated.session_id==manager.riskSessionId(),
      "evaluation binds exact trajectory, start time, session and environment");
  check(evaluated.formula_mode=="exact"&&near(evaluated.length_m,100,1e-6)&&
      evaluated.cumulative_probability>0&&evaluated.cumulative_probability<1,
      "evaluation uses exact probability and physical 3D path length");
  check(evaluated.status==Evaluation::VALID&&evaluated.boundaries_resolved&&
      evaluated.unresolved_boundary_count==0,
      "clear flat terrain receives a certified valid probability evaluation");
  check(evaluated.evaluation_schema_version==2&&evaluated.frame_id=="map"&&
      evaluated.rcs_mode=="heading"&&evaluated.exposure_mode=="distance"&&
      std::isfinite(evaluated.hazard_error_estimate)&&std::isfinite(evaluated.length_error_estimate_m),
      "evaluation publishes explicit modes and numerical error estimates");
  check(evaluated.interval_lengths_m.size()==evaluated.probabilities.size()&&
      !evaluated.probabilities.empty(),"wire carries reproducible exposure intervals");
  std::vector<risk_model::ExposureInterval> intervals;
  for(std::size_t i=0;i<std::min(evaluated.probabilities.size(),evaluated.interval_lengths_m.size());++i)
    intervals.push_back({evaluated.interval_lengths_m[i],evaluated.probabilities[i],
                         evaluated.interval_log_survivals.at(i)});
  check(evaluated.interval_time_begin_s.size()==intervals.size()&&
      evaluated.interval_time_end_s.size()==intervals.size()&&
      evaluated.interval_arc_begin_m.size()==intervals.size()&&
      evaluated.interval_arc_end_m.size()==intervals.size()&&
      near(evaluated.interval_time_begin_s.front(),0)&&near(evaluated.interval_time_end_s.back(),1)&&
      near(evaluated.interval_arc_end_m.back(),evaluated.length_m),
      "wire exposure leaves cover final trajectory time and physical arc length");
  const auto replay=risk_model::accumulatePathRisk(intervals,evaluated.reference_length_m);
  check(replay.probability&&near(*replay.probability,evaluated.cumulative_probability,1e-12),
      "RViz common accumulation reproduces planner probability from wire samples");

  until(executor,[]{return false;},100);
  const auto before_wrong=evaluations.size();
  auto wrong=linearTrajectory(manager,999);wrong.planner_session_id="different-planner-session";
  trajectory_pub->publish(wrong);
  until(executor,[]{return false;},200);
  check(evaluations.size()==before_wrong,"foreign session rejected without poisoning trajectory ID sequence");

  auto reduced=agl;reduced.sources[0].p0=.4;
  check(manager.setRiskModelsRuntime(reduced,&reason),"physical parameter update accepted");
  const auto latest=manager.riskProbabilitySnapshot();
  check(latest&&latest->revision>changed->revision,"physical parameter change advances revision");
  check(until(executor,[&]{return !evaluations.empty()&&evaluations.back().status==Evaluation::INVALID&&
      latest&&evaluations.back().environment_revision==latest->revision;}),
      "environment change invalidates previously displayed trajectory risk");
  trajectory_pub->publish(linearTrajectory(manager,11));
  check(until(executor,[&]{return !evaluations.empty()&&evaluations.back().trajectory_id==11&&
      evaluations.back().status!=Evaluation::INVALID;}),"next own-session trajectory evaluates after foreign replay");
  check(!evaluations.empty()&&latest&&evaluations.back().environment_revision==latest->revision&&
      evaluations.back().cumulative_probability<evaluated.cumulative_probability,
      "reevaluation uses new environment and responds to lower damage probability");
  until(executor,[]{return false;},100);
  const auto before_old=evaluations.size();trajectory_pub->publish(trajectory);
  until(executor,[]{return false;},200);
  check(evaluations.size()==before_old,"older trajectory ID cannot replace a newer risk result");

  // Exercise the production wire -> polynomial partition -> message path,
  // including a window that uniform-time samples previously missed.
  auto narrow=fixture();narrow.reference_length_m=1;
  auto& narrow_source=narrow.sources.front();
  narrow_source.center.x=.502;narrow_source.center.y=0;narrow_source.center.z=3;
  narrow_source.height_mode=Source::MSL;narrow_source.c1=2;narrow_source.c2=1e-300;
  narrow_source.calibration_from_r50=false;
  narrow_source.calibration_r50_m=0;narrow_source.calibration_sigma_m2=0;
  narrow_source.r_max_m=.01;narrow_source.h_max_m=1;
  narrow_source.r_min_m=0;narrow_source.h_min_m=0;
  narrow_source.p0=.8;narrow_source.rho_min=.5;
  check(manager.setRiskModelsRuntime(narrow,&reason),"two-centimetre physical risk configured");
  auto narrow_wire=linearTrajectory(manager,20);narrow_wire.coef_x={1,0};
  trajectory_pub->publish(narrow_wire);
  check(until(executor,[&]{return !evaluations.empty()&&evaluations.back().trajectory_id==20&&
      evaluations.back().status!=Evaluation::INVALID;}),"narrow risk final-wire evaluation received");
  check(evaluations.back().status==Evaluation::VALID&&evaluations.back().boundaries_resolved&&
      near(evaluations.back().cumulative_probability,.019711230215148607,2e-8),
      "production evaluator automatically captures a two-centimetre exposure");

  auto saturated=narrow;auto& saturated_source=saturated.sources.front();
  saturated_source.center.x=0;saturated_source.r_max_m=100;saturated_source.h_max_m=100;
  saturated_source.p0=1;saturated_source.rho_min=.9;
  check(manager.setRiskModelsRuntime(saturated,&reason),"rounded certainty fixture configured");
  auto micro=linearTrajectory(manager,21);
  micro.duration={1e-6f};micro.coef_x={.03f,.01f};
  trajectory_pub->publish(micro);
  check(until(executor,[&]{return !evaluations.empty()&&evaluations.back().trajectory_id==21&&
      evaluations.back().status!=Evaluation::INVALID;}),"micrometre exposure evaluation received");
  check(evaluations.back().cumulative_probability>0&&evaluations.back().cumulative_probability<.01&&
      std::isfinite(evaluations.back().hazard)&&!evaluations.back().interval_log_survivals.empty(),
      "wire log survival prevents rounded T=1 from forcing a tiny exposure to 100 percent");

  auto empty=fixture();empty.sources.clear();
  check(manager.setRiskModelsRuntime(empty,&reason),"explicit empty probability world accepted");
  trajectory_pub->publish(linearTrajectory(manager,22));
  check(until(executor,[&]{return !evaluations.empty()&&evaluations.back().trajectory_id==22&&
      evaluations.back().status!=Evaluation::INVALID;}),"empty-world evaluation received");
  check(evaluations.back().status==Evaluation::VALID&&evaluations.back().cumulative_probability==0&&
      near(evaluations.back().length_m,100),"empty-world final trajectory is valid zero, not degraded");

  mmp_mission_msgs::msg::ScenarioEnvironment scenario;
  scenario.risk_models=fixture();scenario.obstacles.replace=true;
  mmp_mission_msgs::msg::DynamicObstacleSpec obstacle;
  obstacle.kind=obstacle.KIND_CUBE;obstacle.center.x=5;obstacle.center.y=5;
  obstacle.size.x=1;obstacle.size.y=2;obstacle.size.z=3;
  scenario.obstacles.obstacles.push_back(obstacle);
  check(manager.setScenarioEnvironmentRuntime(scenario,&reason),
      "one scenario installs its probability model and obstacle together: "+reason);
  const auto scenario_snapshot=manager.riskProbabilitySnapshot();
  check(scenario_snapshot&&scenario_snapshot->sources.size()==1&&manager.numDynamicObstacles()==1,
      "accepted scenario owns both environment components");
  auto invalid_scenario=scenario;
  invalid_scenario.risk_models.sources.front().p0=.2;
  invalid_scenario.obstacles.obstacles.push_back(obstacle);
  invalid_scenario.obstacles.obstacles.back().size.z=-1;
  check(!manager.setScenarioEnvironmentRuntime(invalid_scenario,&reason)&&
      manager.riskProbabilitySnapshot()==scenario_snapshot&&manager.numDynamicObstacles()==1,
      "invalid last obstacle retains the previous complete environment");
  invalid_scenario=scenario;invalid_scenario.risk_models.sources.front().c2=-1;
  invalid_scenario.obstacles.obstacles.clear();
  check(!manager.setScenarioEnvironmentRuntime(invalid_scenario,&reason)&&
      manager.riskProbabilitySnapshot()==scenario_snapshot&&manager.numDynamicObstacles()==1,
      "invalid risk model cannot clear the previous obstacles");
  invalid_scenario=scenario;invalid_scenario.risk_models.sources.front().center.x=1000;
  invalid_scenario.risk_models.sources.front().height_mode=Source::AGL;
  invalid_scenario.obstacles.obstacles.clear();
  check(!manager.setScenarioEnvironmentRuntime(invalid_scenario,&reason)&&
      manager.riskProbabilitySnapshot()==scenario_snapshot&&manager.numDynamicObstacles()==1,
      "scenario AGL outside DEM is rejected without replacing the accepted world");
  auto recropped=flatTerrain(2.f);recropped->info.pose.position.x=1000;
  manager.setTerrainData(recropped);
  check(manager.riskProbabilitySnapshot()&&
      manager.riskProbabilitySnapshot()->sources.front().center_m.x==0,
      "rejected scenario never queues an AGL update for a later terrain crop");
  manager.setTerrainData(flatTerrain(2.f));
  scenario.risk_models.sources.clear();scenario.obstacles.obstacles.clear();
  check(manager.setScenarioEnvironmentRuntime(scenario,&reason)&&
      manager.riskProbabilitySnapshot()&&manager.riskProbabilitySnapshot()->sources.empty()&&
      manager.numDynamicObstacles()==0,
      "empty full scenario clears prior risk sources and obstacles");
  scenario.risk_models.replace=false;
  check(!manager.setScenarioEnvironmentRuntime(scenario,&reason),
      "scenario input cannot append to a previous environment");
  scenario.risk_models.replace=true;scenario.risk_models.cancel_pending=true;
  check(!manager.setScenarioEnvironmentRuntime(scenario,&reason),
      "scenario input cannot use pending-request cancellation as an environment");
  scenario.risk_models.cancel_pending=false;
  rclcpp::NodeOptions legacy_options;
  legacy_options.append_parameter_override("manager/risk_model_mode",std::string("legacy"));
  auto legacy_node=std::make_shared<rclcpp::Node>("scenario_legacy_test",legacy_options);
  legacy_node->declare_parameter("drone_id",18);
  path_manager::PathManager legacy(legacy_node,false);
  check(!legacy.setScenarioEnvironmentRuntime(scenario,&reason)&&reason.find("default")!=std::string::npos,
      "legacy mode explicitly refuses a default-model unified scenario");
  manager.setTerrainData(std::make_shared<grid_map_msgs::msg::GridMap>());
  check(!manager.setScenarioEnvironmentRuntime(scenario,&reason),
      "scenario refuses planning before its terrain is ingested");

  std::cout<<(checks-failures)<<'/'<<checks<<" ROS probability bridge checks passed\n";
  executor.remove_node(node);
  rclcpp::shutdown();
  return failures?1:0;
}
