// Real solves compare local configuration with the former node-mutation path.
#include "path_optimizer/poly_traj_optimizer.h"
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {
using namespace ego_planner;
int failures=0;
void check(bool ok, const std::string &message) {
  std::cout << (ok ? "[OK] " : "[FAIL] ") << message << '\n';
  failures += !ok;
}
std::vector<double> coefficients(const poly_traj::Trajectory &trajectory) {
  std::vector<double> values;
  for (const auto &piece : trajectory) {
    values.push_back(piece.getDuration());
    const auto &matrix=piece.getCoeffMat();
    values.insert(values.end(),matrix.data(),matrix.data()+matrix.size());
  }
  return values;
}
bool same(const OptimizerSolveResult &a,const OptimizerSolveResult &b) {
  if (!a.hasTrajectory() || a.status!=b.status || a.envelope_peak!=b.envelope_peak ||
      coefficients(a.seed)!=coefficients(b.seed) ||
      coefficients(a.trajectory)!=coefficients(b.trajectory) ||
      a.piece_leg!=b.piece_leg || a.optimizer_path.size()!=b.optimizer_path.size())
    return false;
  for (size_t i=0;i<a.optimizer_path.size();++i)
    if ((a.optimizer_path[i].array()!=b.optimizer_path[i].array()).any()) return false;
  return true;
}
OptimizerSolveRequest request() {
  OptimizerSolveRequest r;
  r.head_pos={20.0,30.0,4.0}; r.head_vel={2.0,0.0,0.0};
  r.waypoints={{100.0,30.0,4.0}}; r.path={r.head_pos,r.waypoints.back()};
  r.cap_ref={4.0,4.0}; r.edge_leg={7};
  r.tail=TailBoundary::pinned(r.head_vel,Eigen::Vector3d::Zero());
  r.band={3.5,4.4,1000.0}; r.max_vel=2.0; r.suppress_crossing_exempt=true;
  return r;
}
using Snapshot=std::map<std::string,rclcpp::Parameter>;
Snapshot snapshot(const rclcpp::Node::SharedPtr &node) {
  Snapshot result;
  for (const auto &name:node->list_parameters({},100).names)
    result.emplace(name,node->get_parameter(name));
  return result;
}
void environment(PolyTrajOptimizer &optimizer) {
  OptimizerRuntimeSettings settings;
  settings.obstacle_clearance=0.3; settings.risk_goal_taper_radius=30.0;
  optimizer.setRuntimeSettings(settings); optimizer.setEnvironment({});
}
}
int main(int argc,char **argv) {
  if (argc!=2) return 2;
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args","--params-file",argv[1]});
  options.parameter_overrides({rclcpp::Parameter("optimization/risk_parallel_threads",1)});
  auto shared=rclcpp::Node::make_shared("configuration_shared",options);
  auto legacy_node=rclcpp::Node::make_shared("configuration_legacy",options);
  shared->declare_parameter("enable_debug_logs",false);
  legacy_node->declare_parameter("enable_debug_logs",false);
  PolyTrajOptimizer baseline,legacy;
  baseline.setParam(shared); environment(baseline);
  legacy.setParam(legacy_node); environment(legacy);
  const auto pristine=snapshot(shared);
  int parameter_writes=0;
  const auto callback=shared->add_on_set_parameters_callback(
      [&](const std::vector<rclcpp::Parameter>&) {
        ++parameter_writes;
        rcl_interfaces::msg::SetParametersResult r; r.successful=true; return r;
      });
  const auto input=request();
  const auto original=baseline.solve(input);
  check(original.hasTrajectory(),"pristine fixture executes real L-BFGS");
  const std::vector<rclcpp::Parameter> first={
    {"optimization/weight_time",12.0},
    {"optimization/max_vel",1.8},
    {"optimization/constrain_points_perPiece",5},
    {"optimization/risk_parallel_threads",1},
    {"optimization/alt_cap_headroom",0.8},
    {"optimization/dynamics_mass_kg",1250.0},
    {"optimization/diag_vertical",false}};
  for (const auto &status:legacy_node->set_parameters(first))
    check(status.successful,"legacy fixture accepts typed override");
  legacy.setParam(legacy_node); environment(legacy);
  PolyTrajOptimizer first_worker;
  first_worker.setParam(shared,first); environment(first_worker);
  const auto first_result=first_worker.solve(input);
  check(same(first_result,legacy.solve(input)),
        "local overrides and node overrides produce identical complete solves");
  check(coefficients(first_result.trajectory)!=coefficients(original.trajectory),
        "fixture exercises settings that actually change the optimized trajectory");
  check(snapshot(shared)==pristine && parameter_writes==0,
        "worker configuration and solve leave shared typed values and callbacks untouched");
  PolyTrajOptimizer following_worker;
  following_worker.setParam(shared); environment(following_worker);
  check(same(original,following_worker.solve(input)),
        "a following worker reads the mission defaults without profile leakage");
  // The view is borrowed only during configuration; the configured solver owns
  // the numerical values after the caller's temporary vector is destroyed.
  PolyTrajOptimizer temporary_worker;
  temporary_worker.setParam(shared,std::vector<rclcpp::Parameter>(first));
  environment(temporary_worker);
  check(same(first_result,temporary_worker.solve(input)),
        "configured worker remains valid after override input destruction");
  PolyTrajOptimizer malformed;
  bool threw=false;
  try { malformed.setParam(shared,{{"optimization/weight_time",std::string("wrong type")}}); }
  catch (const std::exception &) { threw=true; }
  check(threw && snapshot(shared)==pristine && parameter_writes==0,
        "typed-read failure cannot partially mutate shared mission parameters");
  check(malformed.solve(input).status==OptimizerSolveStatus::NOT_CONFIGURED,
        "partially configured solver cannot execute after a typed-read failure");
  rclcpp::shutdown();
  return failures ? 1 : 0;
}
