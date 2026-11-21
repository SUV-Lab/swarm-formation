#include <path_optimizer/poly_traj_optimizer.h>
#include <path_planner/dyn_a_star.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
int checks=0,failures=0;
void check(bool ok,const std::string& label) {
  ++checks;
  if (!ok) { ++failures; std::cerr<<"[FAIL] "<<label<<'\n'; }
}
bool close(double a,double b,double abs=1e-7,double rel=1e-5) {
  return std::abs(a-b)<=abs+rel*std::max(std::abs(a),std::abs(b));
}
double physical(std::size_t source,const Eigen::Vector3d& p,const Eigen::Vector3d& v,
                Eigen::Vector3d* gp,Eigen::Vector3d* gv) {
  const double t=1/(1+std::exp(-(.3*p.x()+.2*v.y()+.1*source)));
  if (gp) *gp=Eigen::Vector3d(.06*t*(1-t),0,0);
  if (gv) *gv=Eigen::Vector3d(0,.04*t*(1-t),0);
  return .1+.2*t;
}
double constant(std::size_t,const Eigen::Vector3d&,const Eigen::Vector3d&,
                Eigen::Vector3d* gp,Eigen::Vector3d* gv) {
  if (gp) gp->setZero();
  if (gv) gv->setZero();
  return .4;
}
} // namespace

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("probability_optimizer_test");
  ego_planner::PolyTrajOptimizer optimizer;
  constexpr double weight=7;
  optimizer.setParam(node,{rclcpp::Parameter("optimization/weight_Risk",weight),
      rclcpp::Parameter("optimization/weight_Risk_barrier",0.0)});
  ego_planner::OptimizerRiskEnvironment environment;
  environment.zones={{{0,0,0},1,.8,1},{{0,0,0},1,.8,1}};
  environment.probability=physical;
  environment.visibility=[](std::size_t,const Eigen::Vector3d&,Eigen::Vector3d* g){
    if(g)g->setZero();
    return 0.0;
  };
  optimizer.setRiskEnvironment(environment);
  const Eigen::Vector3d p(10,3,2),v(3,4,1);
  Eigen::Vector3d gp,gv;
  double cost;
  check(optimizer.inspectRiskCostAt(p,v,gp,gv,cost),"probability survives policy AABB rejection");
  const double combined=1-(1-physical(0,p,v,nullptr,nullptr))*(1-physical(1,p,v,nullptr,nullptr));
  check(close(cost,weight*combined*v.norm()),"MINCO consumes probability without legacy shape or extra visibility");
  const auto value=[&](Eigen::Vector3d point,Eigen::Vector3d velocity){
    Eigen::Vector3d g1,g2;double c=0;
    optimizer.inspectRiskCostAt(point,velocity,g1,g2,c); return c;
  };
  for(int axis=0;axis<6;++axis){
    auto pp=p,pm=p,vp=v,vm=v;const double h=1e-5;
    if(axis<3){pp[axis]+=h;pm[axis]-=h;}else{vp[axis-3]+=h;vm[axis-3]-=h;}
    const double fd=(value(pp,vp)-value(pm,vm))/(2*h);
    check(close(fd,axis<3?gp[axis]:gv[axis-3]),"full probability p/v derivative "+std::to_string(axis));
  }
  check(std::abs(value(p,{5,0,1})-value(p,{0,5,1}))>1e-3,
      "same position and speed can have different heading risk cost");

  // Integrate the real production integrand over a polynomial candidate and
  // differentiate a coefficient and its time interval, including direct T.
  struct Integral {double value=0,coefficient=0,time=0;};
  const auto integral=[&](double duration,double vx){
    Integral out;constexpr int n=80;
    const Eigen::Vector3d velocity0(vx,4,1),acceleration(.1,-.2,.03);
    for(int i=0;i<=n;++i){
      const double u=static_cast<double>(i)/n,t=duration*u;
      const double w=(i==0||i==n?1:(i%2?4:2))/(3.0*n);
      const Eigen::Vector3d point=p+velocity0*t+.5*acceleration*t*t;
      const Eigen::Vector3d velocity=velocity0+acceleration*t;
      Eigen::Vector3d g1,g2;double c=0;
      optimizer.inspectRiskCostAt(point,velocity,g1,g2,c);
      out.value+=w*duration*c;
      out.coefficient+=w*duration*(g1.x()*t+g2.x());
      out.time+=w*(c+duration*u*(g1.dot(velocity)+g2.dot(acceleration)));
    }return out;
  };
  const auto integrated=integral(2.3,3.0);const double h=1e-5;
  check(close(integrated.coefficient,(integral(2.3,3+h).value-integral(2.3,3-h).value)/(2*h)),
      "integrated polynomial coefficient chain includes velocity gradient");
  check(close(integrated.time,(integral(2.3+h,3).value-integral(2.3-h,3).value)/(2*h)),
      "integrated interval time derivative includes gp dot v plus gv dot a and direct duration");

  environment.probability=[](std::size_t source,const Eigen::Vector3d& point,
      const Eigen::Vector3d& velocity,Eigen::Vector3d* g1,Eigen::Vector3d* g2){
    if(source==0){if(g1)g1->setZero();if(g2)g2->setZero();return 1.0;}
    return physical(source,point,velocity,g1,g2);
  };
  optimizer.setRiskEnvironment(environment);
  optimizer.inspectRiskCostAt(p,v,gp,gv,cost);
  check(close(cost,weight*v.norm())&&gp.isZero()&&gv.allFinite(),
      "T=1 produces full risk and finite division-free gradients");

  environment.probability=[](std::size_t,const Eigen::Vector3d&,
      const Eigen::Vector3d&,Eigen::Vector3d* g1,Eigen::Vector3d* g2){
    if(g1)g1->setZero();
    if(g2)g2->setZero();
    return 1e-20;
  };
  optimizer.setRiskEnvironment(environment);
  optimizer.inspectRiskCostAt(p,v,gp,gv,cost);
  check(cost>0&&std::abs(cost/(weight*2e-20*v.norm())-1)<1e-12,
      "tiny probabilities retain nonzero cost without cancellation");

  environment.zones.resize(1);environment.probability=constant;
  environment.visibility=[](std::size_t,const Eigen::Vector3d&,Eigen::Vector3d* g){
    if(g)g->setZero();
    return 1.0;
  };
  optimizer.setRiskEnvironment(environment);
  optimizer.setRiskGoalTaperRadius(5);
  optimizer.prepareRiskBarrier(Eigen::Vector3d::Zero(),{Eigen::Vector3d(20,0,0)});
  optimizer.inspectRiskCostAt({1,0,0},v,gp,gv,cost);
  check(close(cost,weight*.4*(.2*.2*(3-2*.2))*v.norm()),"per-source endpoint taper applies to planning cost");
  optimizer.inspectRiskCostAt(Eigen::Vector3d::Zero(),v,gp,gv,cost);
  check(cost==0,"contained endpoint has zero planned cost without changing physical callback");
  check(constant(0,{},v,nullptr,nullptr)==.4,"physical callback remains untapered");
  environment.probability={};optimizer.setRiskEnvironment(environment);
  optimizer.inspectRiskCostAt(p,v,gp,gv,cost);
  check(cost==0,"rebinding legacy environment clears probability callback");

  path_planner::search::PathSearcher searcher;
  std::vector<path_planner::search::RiskZoneLite> zones={{{0,0,0},1,.8,1},{{0,0,0},1,.8,1}};
  searcher.setRiskZones(&zones);
  searcher.setRiskVisibility([](std::size_t,const Eigen::Vector3d&){return 0.0;});
  searcher.setRiskProbability([](std::size_t i,const Eigen::Vector3d&){return i==0?.4:.3;});
  check(close(searcher.riskFieldAt(p),.58),"FM probability outside policy support uses OR without double visibility");
  searcher.setRiskProbability([](std::size_t,const Eigen::Vector3d&){return 1.0;});
  check(searcher.riskFieldAt(p)==1,"FM supports probability one without 0.999 cap");
  searcher.setRiskProbability({});
  check(searcher.riskFieldAt(p)==0,"FM legacy field restored by empty callback");
  searcher.setRiskProbability([](std::size_t,const Eigen::Vector3d&){return std::numeric_limits<double>::quiet_NaN();});
  bool rejected=false;
  try { (void)searcher.riskFieldAt(p); } catch(const std::domain_error&) {rejected=true;}
  check(rejected,"invalid callback probability is rejected, never turned into zero risk");

  std::cout<<(checks-failures)<<'/'<<checks<<" production probability checks passed\n";
  rclcpp::shutdown();return failures?1:0;
}
