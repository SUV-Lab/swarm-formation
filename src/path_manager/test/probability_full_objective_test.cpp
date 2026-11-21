#include <path_optimizer/poly_traj_optimizer.h>
#include <risk_model/model.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {
int checks=0,failures=0;
void check(bool condition,const std::string& label) {
  ++checks;
  if (!condition) { ++failures; std::cerr<<"[FAIL] "<<label<<'\n'; }
}
bool close(double a,double b,double absolute=2e-6,double relative=2e-5) {
  return std::abs(a-b)<=absolute+relative*std::max(std::abs(a),std::abs(b));
}

struct PhysicalFixture {
  risk_model::Settings settings;
  std::vector<risk_model::Source> sources;
  Eigen::Vector3d scale{20,20,30};
  std::atomic<std::size_t> queries{0},fallback_queries{0};
  PhysicalFixture() {
    settings.rcs={.3172,.1784,1.003,1.0,20,60};
    settings.smooth={2,2,.002,.02,5,1,true};
    sources={{"first",{40,20,0},{1.1,1e-8},{300,200,0,0,.7,.15,2}},
             {"second",{90,-20,5},{.9,8e-9},{240,180,0,0,.6,.1,2}}};
    risk_model::validate(settings);
    for (const auto& source:sources) risk_model::validate(source);
  }
  double query(std::size_t index,const Eigen::Vector3d& p,
               const Eigen::Vector3d& v,Eigen::Vector3d* gp,Eigen::Vector3d* gv,
               risk_model::FormulaMode mode=risk_model::FormulaMode::Planning) {
    const Eigen::Vector3d pm=scale.cwiseProduct(p),vm=scale.cwiseProduct(v);
    risk_model::VisibilitySample visibility;
    visibility.state=risk_model::VisibilityState::KnownHorizon;
    visibility.ceiling_m=60+.01*pm.x()+.015*pm.y();
    visibility.ceiling_gradient={.01,.015,0};
    visibility.ground_known=true;
    visibility.ground_m=0;
    const auto result=risk_model::evaluateSource(sources.at(index),
        {pm.x(),pm.y(),pm.z()},{vm.x(),vm.y(),vm.z()},settings,mode,
        risk_model::RcsMode::Heading,visibility);
    ++queries;
    if (result.flags.heading_fallback) ++fallback_queries;
    if(gp) *gp=scale.cwiseProduct(Eigen::Vector3d(result.risk.grad_p.x,result.risk.grad_p.y,result.risk.grad_p.z));
    if(gv) *gv=scale.cwiseProduct(Eigen::Vector3d(result.risk.grad_v.x,result.risk.grad_v.y,result.risk.grad_v.z));
    return result.risk.value;
  }
};

std::vector<rclcpp::Parameter> parameters(int threads,bool bounded_z) {
  return {{"optimization/constrain_points_perPiece",20},
          {"optimization/risk_parallel_threads",threads},
          {"optimization/weight_Risk",35.0},
          {"optimization/weight_Risk_barrier",.6},
          {"optimization/weight_smoothness",.02},
          {"optimization/weight_time",2.0},
          {"optimization/weight_sqrvariance",.7},
          {"optimization/weight_feasibility",.03},
          {"optimization/weight_obstacle",3.0},
          {"optimization/max_vel",2.1},
          {"optimization/max_acc",2.0},
          {"optimization/weight_dynamics",.001},
          {"optimization/dynamics_enable",true},
          {"optimization/dynamics_unit_xy_m",20.0},
          {"optimization/dynamics_unit_z_m",30.0},
          {"optimization/dynamics_mass_kg",5.0},
          {"optimization/dynamics_speed_min_mps",20.0},
          {"optimization/dynamics_speed_max_mps",60.0},
          {"optimization/dynamics_activation_speed_mps",5.0},
          {"optimization/h5_bounded_z",bounded_z}};
}

void configure(ego_planner::PolyTrajOptimizer& optimizer,
               const rclcpp::Node::SharedPtr& node,
               const std::shared_ptr<PhysicalFixture>& fixture,
               int threads,bool bounded_z=false) {
  optimizer.setParam(node,parameters(threads,bounded_z));
  // A flat height surface avoids a float-valued DEM's quantization while
  // keeping the real terrain-floor penalty and endpoint taper active.
  optimizer.setTerrainHeightmap([](double,double){return 1.75f;},.1);
  optimizer.setTerrainHeightGrad([](double,double,float* h,float* x,float* y){
    *h=1.75f;*x=0;*y=0;return true;
  });
  optimizer.setObstacleClearance(.4);
  optimizer.setAltitudeBand(-1,1.95,1.3);
  optimizer.setRiskGoalTaperRadius(.9);
  ego_planner::OptimizerRiskEnvironment environment;
  environment.zones={{{2,1,0},5,1,4},{{4.5,-1,.1666666666667},1.2,1,3}};
  environment.probability=[fixture](std::size_t i,const Eigen::Vector3d& p,
      const Eigen::Vector3d& v,Eigen::Vector3d* gp,Eigen::Vector3d* gv){
    return fixture->query(i,p,v,gp,gv);
  };
  environment.visibility=[](std::size_t,const Eigen::Vector3d&,Eigen::Vector3d* g){
    if (g) g->setZero();
    return 1.0;
  };
  optimizer.setRiskEnvironment(std::move(environment));
  optimizer.prepareRiskBarrier({0,0,2},{{6,1,2.2}});
}

struct Candidate {
  Eigen::MatrixXd head=Eigen::MatrixXd::Zero(3,3),tail=Eigen::MatrixXd::Zero(3,3);
  Eigen::MatrixXd inner=Eigen::MatrixXd(3,2);
  Eigen::VectorXd duration=Eigen::VectorXd(3);
  Candidate() {
    head.col(0)<<0,0,2;head.col(1)<<2,.25,.05;
    tail.col(0)<<6,1,2.2;tail.col(1)<<2.2,.15,0;
    inner.col(0)<<1.8,.3,2.1;inner.col(1)<<4.4,-.4,1.9;
    duration<<.8,1.4,1.1; // Both branches of the actual virtual-time warp.
  }
};

ego_planner::PolyTrajOptimizer::ObjectiveEvaluation checkGradient(
    ego_planner::PolyTrajOptimizer& optimizer,const Eigen::VectorXd& x,
    const std::string& label) {
  const auto analytic=optimizer.inspectObjectiveAt(x);
  check(std::isfinite(analytic.cost)&&analytic.gradient.allFinite(),label+": finite full objective");
  double worst=0;
  for(Eigen::Index index=0;index<x.size();++index) {
    for(double relative_step:{1e-5,5e-6}) {
      const double h=relative_step*std::max(1.0,std::abs(x[index]));
      Eigen::VectorXd plus=x,minus=x;plus[index]+=h;minus[index]-=h;
      const double fd=(optimizer.inspectObjectiveAt(plus).cost-
                       optimizer.inspectObjectiveAt(minus).cost)/(2*h);
      const double error=std::abs(fd-analytic.gradient[index]);
      worst=std::max(worst,error/std::max(1.0,std::abs(fd)));
      if(!close(fd,analytic.gradient[index]))
        std::cerr<<label<<" variable="<<index<<" h="<<h
                 <<" analytic="<<analytic.gradient[index]<<" fd="<<fd<<'\n';
      check(close(fd,analytic.gradient[index]),label+": decision variable "+std::to_string(index));
    }
  }
  std::cout<<label<<": cost="<<analytic.cost<<", variables="<<x.size()
           <<", max scaled gradient error="<<worst
           <<", integrated terms="<<analytic.integrated_costs.transpose()<<'\n';
  return analytic;
}

// A separate exact/heading diagnostic on the actual generated polynomial.
// This is deliberately not the optimization objective and is not asserted
// to decrease whenever that objective decreases.
double rawRiskLength(const poly_traj::Trajectory& trajectory,PhysicalFixture& fixture) {
  constexpr int samples=1200;
  const double dt=trajectory.getTotalDuration()/samples;
  double total=0;
  for(int i=0;i<samples;++i){
    const double t=(i+.5)*dt;
    const auto p=trajectory.getPos(t),v=trajectory.getVel(t);
    double survival=1;
    for(std::size_t source=0;source<fixture.sources.size();++source)
      survival*=1-fixture.query(source,p,v,nullptr,nullptr,risk_model::FormulaMode::Exact);
    total+=(1-survival)*fixture.scale.cwiseProduct(v).norm()*dt;
  }
  return total;
}
}

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("probability_full_objective_test");
  node->declare_parameter("enable_debug_logs",true);
  std::cout<<std::setprecision(12);
  auto fixture=std::make_shared<PhysicalFixture>();
  const Candidate candidate;
  ego_planner::PolyTrajOptimizer serial;
  configure(serial,node,fixture,1);
  const auto variables=serial.prepareObjectiveEvaluation(candidate.head,candidate.tail,
      candidate.inner,candidate.duration);
  check(variables.size()==9,"two inner xyz points and all three virtual times are tested");
  check(variables[6]<0&&variables[7]>0&&variables[8]>0,"both virtual-time branches exercised");
  const auto result=checkGradient(serial,variables,"serial actual objective");
  check(close(result.cost,result.smoothness_cost+result.time_cost+result.integrated_costs.sum()),
        "full cost equals reported smoothness, time and all integrated terms");
  check(result.smoothness_cost>0&&result.time_cost>0,"smoothness and duration costs active");
  for(int slot:{3,4,5,6,7,8})
    check(result.integrated_costs[slot]>0,"risk/feasibility/variance/altitude/dynamics/terrain slot "+std::to_string(slot)+" active");
  check(fixture->queries>0&&fixture->fallback_queries>0,"actual D/Q/V/RCS and heading fallback exercised");

  ego_planner::PolyTrajOptimizer parallel;
  configure(parallel,node,fixture,2);
  const auto parallel_variables=parallel.prepareObjectiveEvaluation(candidate.head,candidate.tail,
      candidate.inner,candidate.duration);
  const auto parallel_result=checkGradient(parallel,parallel_variables,"parallel actual objective");
  check(close(result.cost,parallel_result.cost,1e-10,1e-12)&&
        (result.gradient-parallel_result.gradient).norm()<1e-8,
        "serial and parallel objective and full decision gradients agree");

  ego_planner::PolyTrajOptimizer bounded;
  configure(bounded,node,fixture,1,true);
  const auto bounded_variables=bounded.prepareObjectiveEvaluation(candidate.head,candidate.tail,
      candidate.inner,candidate.duration);
  checkGradient(bounded,bounded_variables,"bounded-z actual objective");

  // Exercise an accepted objective descent and independently reevaluate the
  // ORIGINAL Exact model on both actual MINCO trajectories. No monotonicity
  // assertion is imposed on the different reporting metric.
  const Eigen::VectorXd direction=-result.gradient/std::max(1.0,result.gradient.norm());
  auto improved=result;
  double step=.02;
  for(int attempt=0;attempt<20;++attempt,step*=.5){
    improved=serial.inspectObjectiveAt(variables+step*direction);
    if(improved.cost<result.cost)break;
  }
  check(improved.cost<result.cost,"full objective admits a finite descent step");
  const double original_before=rawRiskLength(result.trajectory,*fixture);
  const double original_after=rawRiskLength(improved.trajectory,*fixture);
  check(std::isfinite(original_before)&&std::isfinite(original_after)&&
        original_before>=0&&original_after>=0,"original exact risk independently reevaluated before and after descent");
  std::cout<<"Objective descent "<<result.cost<<" -> "<<improved.cost
           <<"; separate exact risk-weighted SI length "<<original_before<<" -> "<<original_after<<'\n';
  std::cout<<(checks-failures)<<'/'<<checks<<" full MINCO objective checks passed\n";
  rclcpp::shutdown();return failures?1:0;
}
