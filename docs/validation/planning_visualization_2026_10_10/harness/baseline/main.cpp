#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <thread>
#include <rclcpp/serialization.hpp>
#include "path_manager/path_manager.h"
namespace fs=std::filesystem;
struct Capture {
 std::string root, phase="initial"; int count=0; std::ofstream index;
 std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
 Capture(std::string r):root(std::move(r)){fs::create_directories(root);index.open(root+"/index.tsv");}
 template<class M> void subscribe(rclcpp::Node::SharedPtr n,const char* topic,const char* type){
  subscriptions.push_back(n->create_subscription<M>(topic,rclcpp::QoS(64).reliable().transient_local(),[this,topic,type](std::shared_ptr<M> m){
   rclcpp::Serialization<M> codec;rclcpp::SerializedMessage bytes;codec.serialize_message(m.get(),&bytes);
   std::string name=std::to_string(count++)+".cdr";std::ofstream out(root+"/"+name,std::ios::binary);const auto &buf=bytes.get_rcl_serialized_message();out.write(reinterpret_cast<const char*>(buf.buffer),buf.buffer_length);
   index<<phase<<'\t'<<topic<<'\t'<<type<<'\t'<<name<<'\n';index.flush();
  }));
 }
 void connect(rclcpp::Node::SharedPtr n){
  subscribe<nav_msgs::msg::Path>(n,"/planning/front_end_path","nav_msgs/msg/Path");
  for(auto topic:{"/viz/opt_trajectory","/viz/global_trajectory","/viz/front_end_path","/viz/traj_risk"})subscribe<visualization_msgs::msg::Marker>(n,topic,"visualization_msgs/msg/Marker");
  for(auto topic:{"/viz/chain_segments","/viz/risk_field","/viz/dynamic_obstacles","/debug/pipeline"})subscribe<visualization_msgs::msg::MarkerArray>(n,topic,"visualization_msgs/msg/MarkerArray");
  subscribe<std_msgs::msg::Float64MultiArray>(n,"/viz/risk_profile","std_msgs/msg/Float64MultiArray");
  subscribe<grid_map_msgs::msg::GridMap>(n,"/viz/risk_heatmap","grid_map_msgs/msg/GridMap");
 }
};
void settle(rclcpp::Node::SharedPtr n,int ms=200){for(int i=0;i<ms/10;++i){rclcpp::spin_some(n);std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
poly_traj::Trajectory line(double x=-7,double y=.5,double z=2,double vx=1.5,double T=8){poly_traj::CoefficientMat c=poly_traj::CoefficientMat::Zero();c.col(5)=Eigen::Vector3d(x,y,z);c(0,4)=vx;c(2,3)=.015;return poly_traj::Trajectory({T},{c});}
grid_map_msgs::msg::GridMap::SharedPtr terrain(){auto m=std::make_shared<grid_map_msgs::msg::GridMap>();m->info.resolution=1.;m->info.length_x=m->info.length_y=20.;m->layers={"elevation"};m->data.resize(1);auto &d=m->data[0];d.layout.dim.resize(2);d.layout.dim[0].label="column_index";d.layout.dim[0].size=20;d.layout.dim[0].stride=400;d.layout.dim[1].label="row_index";d.layout.dim[1].size=20;d.layout.dim[1].stride=20;d.data.assign(400,0.f);for(int c=0;c<20;++c){d.data[c*20+6]=4.f;for(int r=0;r<20;++r){if(r>12)d.data[c*20+r]=.1f*(r-12);if(c<2&&r<2)d.data[c*20+r]=std::numeric_limits<float>::quiet_NaN();}}return m;}
int main(int argc,char**argv){rclcpp::init(argc,argv);if(argc<2)return 2;rclcpp::NodeOptions opts;
 opts.append_parameter_override("manager/debug_pipeline_viz",true);
 opts.append_parameter_override("manager/risk_zone_agl",true);
 opts.append_parameter_override("manager/risk_heatmap_max_dim",48);
 opts.append_parameter_override("manager/risk_mask_viz_step",1.0);
 opts.append_parameter_override("manager/max_vel",2.0);
 opts.append_parameter_override("obstacle_viz_scale_ship",1.7);
 auto node=std::make_shared<rclcpp::Node>("focused_visual_fixture",opts);node->declare_parameter("drone_id",1);
 Capture cap(std::string(argv[1])+"/events");cap.connect(node);
#ifdef STAGE4_VISUALIZATION
 bool connected = argc < 3 || std::string(argv[2]) != "off";
 path_manager::PathManager pm(node, connected);
#else
 path_manager::PathManager pm(node);
#endif
 pm.yaw_rng_.seed(24681357);settle(node,400);
 std::ofstream state(std::string(argv[1])+"/physical_state.txt");state<<std::hexfloat;
 auto physical=[&]{state<<"phase="<<cap.phase<<" revision="<<pm.environmentRevision()<<" zones="<<pm.numRiskZones()<<" obstacles="<<pm.numDynamicObstacles()<<'\n';
  for(size_t i=0;i<pm.numRiskZones();++i)for(const Eigen::Vector3d p:std::vector<Eigen::Vector3d>{{-6.5,.5,2},{.5,.5,2},{6.5,.5,2},{6.5,.5,8}})state<<i<<' '<<p.transpose()<<' '<<pm.getEffectiveRisk(i,p)<<' '<<pm.getRiskVisibility(i,p)<<' '<<pm.getRiskShadowCeiling(i,p)<<'\n';
  for(double yaw:pm.dyn_patch_yaws_)state<<"collision_yaw="<<yaw<<'\n';state.flush();};
 auto phase=[&](std::string p,auto action){cap.phase=std::move(p);action();settle(node);physical();};
 path_manager::RiskZone a,b;a.center={.5,.5,2.};a.reach=10.;a.peak=.8;b.center={3.5,.5,.5};b.reach=8.;b.peak=.65;
 phase("zones_no_terrain",[&]{pm.setRiskZonesRuntime({a,b});});
 phase("terrain_ridge",[&]{pm.setTerrainData(terrain());});
 for(std::string mode:{"heatmap","heatmap3d","volume","fixed_agl","fixed_msl"})phase("field_"+mode,[&]{pm.risk_mask_viz_mode_=mode;pm.publishEffectiveRiskField();});
 phase("heatmap_safe_transparent",[&]{pm.risk_mask_viz_mode_="heatmap";pm.risk_heatmap_safe_transparent_=true;pm.publishEffectiveRiskField();});
 auto opt=line(),ref=line(-7,1.5,3,1.2,9);std::vector<Eigen::Vector3d> route={{-7,.5,2},{0,.5,2.5},{5,.5,3}};
 phase("front_end",[&]{pm.publishFrontEndRoute(route,12.5);});
 phase("pipeline",[&]{pm.publishPipelineDebug(route,{{-4,.5,2.2},{2,.5,2.7}});});
 phase("trajectory",[&]{pm.publishTrajectoryViz(opt,ref);});
 phase("chain",[&]{auto t=line(5,.5,3,.2,2);pm.publishChainSegmentsViz({line(-7,.5,2,1.5,4),line(-1,.5,2.3,1.5,4)},{{-1,.5,2.3},{5,.5,3}},&t);});
 phase("obstacle_sphere",[&]{pm.buildSDFForBounds({-20,-20,-2},{20,20,30});if(pm.addDynamicSphere({-5,-5,2},1.2,"car")<0)throw std::runtime_error("sphere insertion failed");});
 phase("obstacle_box",[&]{if(pm.addDynamicBox({5,-5,3},{3,4,2},"ship")<0)throw std::runtime_error("box insertion failed");});
 // A late subscriber must receive the complete retained view from every channel.
 {auto late=std::make_shared<rclcpp::Node>("focused_visual_late_full");Capture c(std::string(argv[1])+"/late_full");c.phase="late_full";c.connect(late);settle(late,600);}
 phase("trajectory_clear",[&]{pm.clearTrajectoryViz();});
 phase("chain_clear",[&]{pm.publishChainSegmentsViz({},{});});
 phase("obstacle_clear",[&]{pm.clearDynamicObstacles();});
 phase("zone_clear",[&]{pm.setRiskZonesRuntime({});});
 phase("terrain_clear",[&]{pm.setTerrainData(std::make_shared<grid_map_msgs::msg::GridMap>());});
 phase("invalid_trajectory",[&]{pm.publishTrajectoryViz({},{});});
 {auto late=std::make_shared<rclcpp::Node>("focused_visual_late_clear");Capture c(std::string(argv[1])+"/late_clear");c.phase="late_clear";c.connect(late);settle(late,600);}
 state<<"environment_revision="<<pm.environmentRevision()<<"\nrng_next="<<pm.yaw_rng_()<<"\nrisk_zones="<<pm.numRiskZones()<<"\nobstacles="<<pm.numDynamicObstacles()<<'\n';
 std::cout<<"recorded "<<cap.count<<" live visual messages\n";rclcpp::shutdown();return 0;}
