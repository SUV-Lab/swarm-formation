// Exercise the installed optimizer's complete owned solve boundary without
// PathManager: real L-BFGS, legacy equivalence, live bindings and lifetimes.
#include "path_optimizer/poly_traj_optimizer.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace ego_planner;
int failures = 0;
void check(bool condition, const std::string &description) {
  std::cout << (condition ? "[OK] " : "[FAIL] ") << description << '\n';
  if (!condition) ++failures;
}
std::vector<double> coefficients(const poly_traj::Trajectory &trajectory) {
  std::vector<double> values;
  for (const auto &piece : trajectory) {
    values.push_back(piece.getDuration());
    const auto &matrix = piece.getCoeffMat();
    values.insert(values.end(), matrix.data(), matrix.data() + matrix.size());
  }
  return values;
}
bool samePath(const std::vector<Eigen::Vector3d> &a,
              const std::vector<Eigen::Vector3d> &b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    for (int axis = 0; axis < 3; ++axis)
      if (a[i][axis] != b[i][axis] &&
          !(std::isnan(a[i][axis]) && std::isnan(b[i][axis]))) return false;
  return true;
}
bool sameRequest(const OptimizerSolveRequest &a, const OptimizerSolveRequest &b) {
  return samePath(a.path,b.path) && samePath(a.waypoints,b.waypoints) &&
      a.cap_ref == b.cap_ref && a.edge_leg == b.edge_leg &&
      (a.head_pos-b.head_pos).norm() == 0.0 &&
      (a.head_vel-b.head_vel).norm() == 0.0 &&
      (a.head_acc-b.head_acc).norm() == 0.0 &&
      a.tail.prescribe_vel == b.tail.prescribe_vel &&
      a.tail.prescribe_acc == b.tail.prescribe_acc &&
      (a.tail.vel-b.tail.vel).norm() == 0.0 &&
      (a.tail.acc-b.tail.acc).norm() == 0.0 &&
      a.band.z_lo == b.band.z_lo && a.band.z_hi == b.band.z_hi &&
      a.band.weight == b.band.weight && a.max_vel == b.max_vel &&
      a.suppress_crossing_exempt == b.suppress_crossing_exempt;
}
OptimizerSolveRequest request() {
  OptimizerSolveRequest r;
  r.head_pos = Eigen::Vector3d(20.0,30.0,4.0);
  r.head_vel = Eigen::Vector3d(2.0,0.0,0.0);
  r.waypoints = {Eigen::Vector3d(100.0,30.0,4.0)};
  r.path = {r.head_pos,r.waypoints.back()};
  r.cap_ref = {4.0,4.0};
  r.edge_leg = {7};
  r.tail = TailBoundary::pinned(r.head_vel,Eigen::Vector3d::Zero());
  r.band = {3.5,4.4,1000.0};
  r.max_vel = 2.0;
  r.suppress_crossing_exempt = true;
  return r;
}
bool emptyFlight(const OptimizerSolveResult &r) {
  return !r.hasTrajectory() && r.seed.getPieceNum() == 0 &&
      r.trajectory.getPieceNum() == 0 && r.envelope_peak == 0.0;
}
void legacySettings(PolyTrajOptimizer &opt, const OptimizerRuntimeSettings &s) {
  opt.setObstacleClearance(s.obstacle_clearance);
  opt.setCollisionReject(s.collision_reject);
  opt.setEnvelopeReject(s.envelope_reject,s.envelope_violation_max,
                        s.envelope_violation_hard_max);
  opt.setLbfgsParams(s.lbfgs_mem_size,s.lbfgs_g_epsilon,s.lbfgs_past,
                     s.lbfgs_delta,s.lbfgs_max_linesearch,
                     s.lbfgs_f_dec,s.lbfgs_s_curv);
  opt.setGroundHeight(s.ground_height);
  opt.setVirtualCeilHeight(s.virtual_ceil_height);
  opt.setRiskGoalTaperRadius(s.risk_goal_taper_radius);
}
OptimizerSolveResult legacySolve(PolyTrajOptimizer &opt,
                                 const OptimizerSolveRequest &r,
                                 bool apply_request_settings = true) {
  OptimizerSolveResult result;
  result.optimizer_path = r.path;
  if (apply_request_settings) {
    opt.setAltitudeBand(r.band.z_lo,r.band.z_hi,r.band.weight);
    opt.setSuppressCrossingExempt(r.suppress_crossing_exempt);
  }
  const bool ok = opt.optimizeFromPath(result.optimizer_path,r.head_pos,
      r.head_vel,r.head_acc,r.waypoints,r.max_vel,result.seed,
      result.trajectory,r.cap_ref,r.tail,r.edge_leg,&result.piece_leg);
  if (ok) {
    result.status = OptimizerSolveStatus::SUCCESS;
    result.envelope_peak = opt.lastEnvPeak();
  }
  return result;
}
void compare(const OptimizerSolveResult &a, const OptimizerSolveResult &b,
             const std::string &label) {
  check(a.status == b.status && a.hasTrajectory(),label+" accepts the same real solve");
  check(coefficients(a.seed) == coefficients(b.seed) &&
        coefficients(a.trajectory) == coefficients(b.trajectory) &&
        a.envelope_peak == b.envelope_peak,
        label+" has exactly equal seed/final coefficients, durations and envelope");
  check(samePath(a.optimizer_path,b.optimizer_path) && a.piece_leg == b.piece_leg,
        label+" has exactly equal edited route and leg attribution");
}
struct BlockedField final : path_planner::sdf::IDistanceField {
  mutable int queries{0};
  float getDistance(const Eigen::Vector3d &) const override {
    ++queries; return -1.0f;
  }
  bool getDistanceAndGradient(const Eigen::Vector3d &, float *distance,
                              Eigen::Vector3d *gradient) const override {
    ++queries; *distance=-1.0f; gradient->setZero(); return true;
  }
  bool hasData() const override { return true; }
};
}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: optimizer_session_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc,argv);
  auto r=request();
  const auto original=r;
  {
    PolyTrajOptimizer unconfigured;
    auto short_r=r; short_r.path.resize(1);
    const auto short_result=unconfigured.solve(short_r);
    check(short_result.status == OptimizerSolveStatus::ROUTE_TOO_SHORT &&
          emptyFlight(short_result) && samePath(short_result.optimizer_path,short_r.path),
          "short routes refuse before any configured optimizer or callback is needed");
    auto no_goal=r; no_goal.waypoints.clear();
    const auto no_goal_result=unconfigured.solve(no_goal);
    check(no_goal_result.status == OptimizerSolveStatus::NO_WAYPOINTS &&
          emptyFlight(no_goal_result),"missing waypoint has a distinct empty refusal");
    const auto missing=unconfigured.solve(r);
    check(missing.status == OptimizerSolveStatus::NOT_CONFIGURED && emptyFlight(missing),
          "a valid request cannot enter a never-configured backend");
  }
  OptimizerSolveResult retained;
  std::vector<double> saved_seed,saved_final;
  std::vector<Eigen::Vector3d> saved_path;
  std::vector<size_t> saved_tags;
  double saved_peak=0.0, sample_time=0.0;
  Eigen::Vector3d saved_sample=Eigen::Vector3d::Zero();
  std::weak_ptr<int> provider_lifetime;
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args","--params-file",argv[1]});
    options.parameter_overrides({
        rclcpp::Parameter("optimization/risk_parallel_threads",1),
        rclcpp::Parameter("optimization/alt_cap_headroom",0.65)});
    auto node=rclcpp::Node::make_shared("optimizer_session_test",options);
    node->declare_parameter("enable_debug_logs",false);
    const bool headroom_not_predeclared=
        !node->has_parameter("optimization/alt_cap_headroom");
    PolyTrajOptimizer optimizer,legacy;
    optimizer.setParam(node);
    check(headroom_not_predeclared &&
          node->get_parameter("optimization/alt_cap_headroom").as_double()==0.65,
          "standalone parameter setup declares and honors the headroom override without PathManager");
    legacy.setParam(node);
    OptimizerRuntimeSettings settings;
    settings.obstacle_clearance=0.3;
    settings.risk_goal_taper_radius=30.0;
    optimizer.setRuntimeSettings(settings);
    legacySettings(legacy,settings);
    optimizer.setEnvironment({});
    retained=optimizer.solve(r);
    compare(retained,legacySolve(legacy,r),"owned API versus legacy settings+solve");
    check(sameRequest(r,original),"the complete caller request survives midpoint and lead-in insertion unchanged");
    check(retained.optimizer_path.size()>r.path.size() &&
          retained.piece_leg.size()==static_cast<size_t>(retained.trajectory.getPieceNum()) &&
          !retained.piece_leg.empty() &&
          std::all_of(retained.piece_leg.begin(),retained.piece_leg.end(),[](size_t x){return x==7;}),
          "insertions preserve the authored edge's piece provenance");
    if (!retained.hasTrajectory()) { rclcpp::shutdown(); return 1; }
    saved_seed=coefficients(retained.seed); saved_final=coefficients(retained.trajectory);
    saved_path=retained.optimizer_path; saved_tags=retained.piece_leg;
    saved_peak=retained.envelope_peak;
    sample_time=retained.trajectory.getTotalDuration()*0.5;
    saved_sample=retained.trajectory.getPos(sample_time);

    const double nan=std::numeric_limits<double>::quiet_NaN();
    const double inf=std::numeric_limits<double>::infinity();
    int admission_queries=0;
    BlockedField admission_sdf;
    OptimizerEnvironment admission_environment;
    admission_environment.distance_field=&admission_sdf;
    admission_environment.terrain_height=[&admission_queries](double,double) {
      ++admission_queries; return -std::numeric_limits<float>::infinity();
    };
    admission_environment.terrain_gradient=
        [&admission_queries](double,double,float *,float *,float *) {
          ++admission_queries; return false;
        };
    optimizer.setEnvironment(std::move(admission_environment));
    std::vector<std::pair<std::string,OptimizerSolveRequest>> malformed;
    const auto bad=[&](const std::string &name,
                       const std::function<void(OptimizerSolveRequest &)> &change) {
      auto candidate=r; change(candidate);
      malformed.emplace_back(name,std::move(candidate));
    };
    bad("non-finite route",[&](auto &x){x.path.back().x()=nan;});
    bad("non-finite waypoint",[&](auto &x){x.waypoints.back().y()=inf;});
    bad("non-finite head position",[&](auto &x){x.head_pos.z()=nan;});
    bad("non-finite head velocity",[&](auto &x){x.head_vel.y()=inf;});
    bad("non-finite head acceleration",[&](auto &x){x.head_acc.x()=nan;});
    bad("non-finite paired cap",[&](auto &x){x.cap_ref.back()=nan;});
    bad("infinite paired cap",[&](auto &x){x.cap_ref.front()=inf;});
    bad("zero seed speed",[](auto &x){x.max_vel=0.0;});
    bad("negative seed speed",[](auto &x){x.max_vel=-2.0;});
    bad("NaN seed speed",[&](auto &x){x.max_vel=nan;});
    bad("infinite seed speed",[&](auto &x){x.max_vel=inf;});
    bad("non-finite floor",[&](auto &x){x.band.z_lo=nan;});
    bad("non-finite ceiling",[&](auto &x){x.band.z_hi=inf;});
    bad("non-finite altitude weight",[&](auto &x){x.band.weight=nan;});
    bad("non-finite prescribed tail velocity",[&](auto &x){x.tail.vel.x()=inf;});
    bad("non-finite prescribed tail acceleration",[&](auto &x){x.tail.acc.z()=nan;});
    // Valid but drastically different settings must not be applied before
    // the malformed head is refused. Probe the old low-level API below,
    // which does not itself reset these settings before using them.
    bad("malformed head with conflicting solve settings",[&](auto &x){
      x.head_pos.x()=nan; x.band.z_lo=1000.0;
      x.suppress_crossing_exempt=false;
    });
    for (const auto &entry : malformed) {
      const auto refusal=optimizer.solve(entry.second);
      check(refusal.status==OptimizerSolveStatus::MALFORMED_INPUT &&
            emptyFlight(refusal) && refusal.piece_leg.empty() &&
            samePath(refusal.optimizer_path,entry.second.path),
            entry.first+" returns an empty malformed result with the untouched diagnostic route");
    }
    check(admission_queries==0 && admission_sdf.queries==0,
          "all malformed requests refuse before terrain or SDF providers are queried");
    optimizer.setEnvironment({});
    compare(legacySolve(optimizer,r,false),retained,
            "malformed admission preserves the backend's previously applied solve settings");
    compare(optimizer.solve(r),retained,"valid input recovers after all malformed refusals");

    auto ignored_tail=r;
    ignored_tail.tail.prescribe_vel=false;
    ignored_tail.tail.prescribe_acc=false;
    ignored_tail.tail.vel.setConstant(nan);
    ignored_tail.tail.acc.setConstant(inf);
    compare(optimizer.solve(ignored_tail),legacySolve(legacy,ignored_tail),
            "unprescribed non-finite tail fields remain ignored");
    auto ignored_tail_acc=r;
    ignored_tail_acc.tail.prescribe_acc=false;
    ignored_tail_acc.tail.acc.setConstant(nan);
    compare(optimizer.solve(ignored_tail_acc),legacySolve(legacy,ignored_tail_acc),
            "a prescribed velocity does not activate the ignored acceleration field");
    auto ignored_tail_vel=r;
    ignored_tail_vel.tail.prescribe_vel=false;
    ignored_tail_vel.tail.vel.setConstant(inf);
    compare(optimizer.solve(ignored_tail_vel),legacySolve(legacy,ignored_tail_vel),
            "a prescribed acceleration does not activate the ignored velocity field");
    auto ignored_cap=r;
    ignored_cap.cap_ref={nan,inf,0.0};
    compare(optimizer.solve(ignored_cap),legacySolve(legacy,ignored_cap),
            "mismatched non-finite cap values keep the scalar fallback");

    auto mismatch=r; mismatch.cap_ref={9999.0}; mismatch.edge_leg={7,8};
    const auto fallback=optimizer.solve(mismatch);
    compare(fallback,legacySolve(legacy,mismatch),"mismatched cap/provenance fallback");
    check(fallback.piece_leg.empty(),"malformed edge attribution is dropped as one whole map");

    BlockedField blocked;
    int terrain_queries=0;
    {
      auto owner=std::make_shared<int>(0); provider_lifetime=owner;
      OptimizerEnvironment environment;
      environment.distance_field=&blocked;
      environment.terrain_cell_u=0.3;
      environment.terrain_gradient=[owner,&terrain_queries](double,double,float *h,float *gx,float *gy) {
        ++*owner; ++terrain_queries; *h=0.0f; *gx=0.0f; *gy=0.0f; return true;
      };
      optimizer.setEnvironment(std::move(environment));
    }
    check(!provider_lifetime.expired(),"environment retains callback objects after their source object is destroyed");
    const auto rejected=optimizer.solve(r);
    check(rejected.status == OptimizerSolveStatus::OPTIMIZER_FAILED &&
          emptyFlight(rejected) && rejected.optimizer_path.size()>r.path.size() &&
          rejected.piece_leg.empty(),
          "real collision refusal returns edited geometry but no partial seed, stale flight or envelope");
    check(blocked.queries>0 && terrain_queries>0,
          "the real backend queries both borrowed SDF and owned terrain callback bindings");

    // A risk-only replacement must not silently detach the terrain or SDF.
    const int old_sdf_queries=blocked.queries, old_terrain_queries=terrain_queries;
    optimizer.setRiskEnvironment({});
    const auto still_blocked=optimizer.solve(r);
    check(still_blocked.status == OptimizerSolveStatus::OPTIMIZER_FAILED &&
          blocked.queries>old_sdf_queries && terrain_queries>old_terrain_queries,
          "risk-only rebinding retains the existing terrain and distance field");
    optimizer.setEnvironment({});
    check(provider_lifetime.expired(),"an empty full environment releases old callback captures");
    const int cleared_sdf_queries=blocked.queries, cleared_terrain_queries=terrain_queries;
    const auto recovered=optimizer.solve(r);
    compare(recovered,retained,"empty environment removes the rejected solve's bindings");
    check(blocked.queries==cleared_sdf_queries && terrain_queries==cleared_terrain_queries,
          "cleared terrain and SDF providers are never queried by subsequent solves");

    int visibility_queries=0;
    OptimizerRiskEnvironment risk;
    risk.zones.push_back({Eigen::Vector3d(60.0,80.0,4.0),5.0,0.5,5.0});
    risk.visibility=[&visibility_queries](size_t index,const Eigen::Vector3d &,Eigen::Vector3d *grad) {
      ++visibility_queries;
      if (grad) grad->setZero();
      return index==0 ? 0.5 : 0.0;
    };
    optimizer.setRiskEnvironment(risk);
    risk.zones.front().peak=0.0; risk.zones.front().center.setConstant(1e6);
    Eigen::Vector3d gp,gv; double cost=0.0;
    optimizer.inspectRiskCostAt(Eigen::Vector3d(60.0,80.0,4.0),r.head_vel,gp,gv,cost);
    check(cost>0.0 && std::isfinite(cost) && visibility_queries>0,
          "risk binding owns copied zone geometry and invokes its ordered visibility callback");
    optimizer.setRiskEnvironment({});
    cost=123.0;
    const bool risk_active=optimizer.inspectRiskCostAt(Eigen::Vector3d(60.0,80.0,4.0),r.head_vel,gp,gv,cost);
    check(!risk_active && cost==0.0,"empty risk binding clears prior zones and providers");

    auto later=r; later.path.back().x()+=20.0; later.waypoints.back()=later.path.back();
    later.edge_leg={9};
    check(optimizer.solve(later).hasTrajectory(),"one configured session can solve a different mission after failures and environment replacements");
    check(coefficients(retained.seed)==saved_seed && coefficients(retained.trajectory)==saved_final &&
          samePath(retained.optimizer_path,saved_path) && retained.piece_leg==saved_tags &&
          retained.envelope_peak==saved_peak && sameRequest(r,original),
          "later successes, failures and environment updates cannot rewrite earlier results or input");

    rclcpp::NodeOptions bad_options;
    bad_options.parameter_overrides({rclcpp::Parameter("optimization/constrain_points_perPiece","wrong-type")});
    auto bad_node=rclcpp::Node::make_shared("optimizer_session_bad_setup",bad_options);
    bool threw=false;
    try { optimizer.setParam(bad_node); } catch (const std::exception &) { threw=true; }
    const auto refused=optimizer.solve(r);
    check(threw && refused.status==OptimizerSolveStatus::NOT_CONFIGURED && emptyFlight(refused),
          "a failed parameter reconfiguration revokes the previous setup's readiness");
    optimizer.setParam(node); optimizer.setRuntimeSettings(settings); optimizer.setEnvironment({});
    compare(optimizer.solve(r),retained,"successful reconfiguration restores the session");
  }
  check(coefficients(retained.seed)==saved_seed && coefficients(retained.trajectory)==saved_final &&
        samePath(retained.optimizer_path,saved_path) && retained.piece_leg==saved_tags &&
        retained.envelope_peak==saved_peak &&
        (retained.trajectory.getPos(sample_time)-saved_sample).norm()==0.0,
        "owned result remains evaluable after optimizer, terrain callbacks and nodes are destroyed");
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
