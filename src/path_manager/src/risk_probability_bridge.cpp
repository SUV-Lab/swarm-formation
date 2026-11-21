#include "path_manager/path_manager.h"
#include "path_manager/terrain_risk.h"
#include "path_manager/probability_visibility_partition.h"
#include <mmp_mission_msgs/msg/risk_environment.hpp>
#include <mmp_traj_msgs/msg/trajectory_risk_evaluation.hpp>

#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace path_manager {
namespace {

using ModelMessage = mmp_mission_msgs::msg::RiskModelArray;
using EnvironmentMessage = mmp_mission_msgs::msg::RiskEnvironment;
using EvaluationMessage = mmp_traj_msgs::msg::TrajectoryRiskEvaluation;

void require(bool condition, const char* reason) {
  if (!condition) throw std::invalid_argument(reason);
}

// Owns a DEM COPY and each source's polar horizon table. Published snapshots
// never borrow PathManager's mutable terrain or mask vectors.
class FrozenProbabilityTerrain final : public risk_model::TerrainSnapshot {
 public:
  FrozenProbabilityTerrain(TerrainData terrain, risk_model::FrameScale scale,
      const risk_model::Settings& settings, bool enabled, double radial_step)
      : terrain_(std::move(terrain)), scale_(scale), enabled_(enabled),
        settings_{enabled, false, radial_step,
                  settings.smooth.visibility_width_m / scale.z_m,
                  settings.smooth.clearance_m / scale.z_m} {}

  std::uint64_t revision() const override { return terrain_.generation; }

  risk_model::BoundaryPartition visibilityPartition(const risk_model::Source& source,
      const risk_model::PolynomialPiece& piece) const override {
    const auto found=masks_.find(source.id);
    return partitionProbabilityVisibility(terrain_,
        found==masks_.end()?nullptr:&found->second,enabled_,
        elevation(source.center_m.x,source.center_m.y).has_value(),scale_,piece);
  }

  std::optional<double> elevation(double x, double y) const override {
    if (!terrain_.valid) return std::nullopt;
    const double h = terrain_.getElevation(x / scale_.xy_m, y / scale_.xy_m);
    if (!std::isfinite(h)) return std::nullopt;
    return h * scale_.z_m;
  }

  void build(const std::vector<risk_model::Source>& sources,
             const ModelMessage& authored) {
    for (std::size_t i = 0; i < sources.size(); ++i) {
      const auto& source = sources[i];
      RiskZone zone;
      zone.center = Eigen::Vector3d(source.center_m.x / scale_.xy_m,
          source.center_m.y / scale_.xy_m, source.center_m.z / scale_.z_m);
      // Damage support and its smoothing can extend beyond the independent
      // HARD_AVOID boundary. The horizon must cover both query domains.
      zone.reach = std::max(authored.sources[i].policy_reach_frame,
          (source.damage.r_max_m + authored.radius_width_m) / scale_.xy_m);
      const double step=std::max(0.02,settings_.radial_step>0
          ? settings_.radial_step : terrain_.resolution);
      require(std::isfinite(zone.reach) && zone.reach>0 &&
              1.05L*zone.reach/step+1 <= std::numeric_limits<int>::max(),
              "risk support exceeds the polar terrain mask numeric range");
      zone.peak = 1;
      masks_.emplace(source.id, terrain_risk::buildMask(zone, terrain_, settings_).mask);
    }
  }

  risk_model::VisibilitySample sample(const risk_model::Source& source,
                                      risk_model::Vec3 p) const override {
    risk_model::VisibilitySample result;
    if (!enabled_) { result.state = risk_model::VisibilityState::Disabled; return result; }
    if (!terrain_.valid) return result;
    const Eigen::Vector3d frame(p.x / scale_.xy_m, p.y / scale_.xy_m, p.z / scale_.z_m);
    float height=0, gx=0, gy=0;
    if (!terrain_.getElevationAndGrad(frame.x(), frame.y(), &height, &gx, &gy)) {
      result.state = risk_model::VisibilityState::OutsideMap;
      return result;
    }
    result.ground_known = true;
    result.ground_m = height * scale_.z_m;
    result.ground_gradient = {gx * scale_.z_m / scale_.xy_m,
                              gy * scale_.z_m / scale_.xy_m, 0};
    // Missing source coverage is not equivalent to proven absence of a ridge.
    if (!elevation(source.center_m.x, source.center_m.y)) {
      result.state = risk_model::VisibilityState::OutsideMap;
      return result;
    }
    const auto found = masks_.find(source.id);
    if (found == masks_.end() || !found->second.valid) return result;
    const auto& mask = found->second;
    const double ceiling = terrain_risk::shadowCeiling(&mask, true, frame);
    if (!std::isfinite(ceiling)) {
      const double radius = std::hypot(frame.x()-mask.source.x(), frame.y()-mask.source.y());
      result.state = radius > mask.max_range ? risk_model::VisibilityState::OutsideMap
                                             : risk_model::VisibilityState::NoOcclusion;
      return result;
    }
    result.state = risk_model::VisibilityState::KnownHorizon;
    result.ceiling_m = ceiling * scale_.z_m;
    const double epsilon = std::clamp(0.01 * mask.radial_step, 1e-5, 0.01);
    for (int axis=0; axis<2; ++axis) {
      auto plus=frame, minus=frame;
      plus[axis] += epsilon;
      minus[axis] -= epsilon;
      const double hi = terrain_risk::shadowCeiling(&mask, true, plus);
      const double lo = terrain_risk::shadowCeiling(&mask, true, minus);
      if (std::isfinite(hi) && std::isfinite(lo))
        result.ceiling_gradient[axis] = (hi-lo) / (2*epsilon) * scale_.z_m / scale_.xy_m;
    }
    return result;
  }

 private:
  TerrainData terrain_;
  risk_model::FrameScale scale_;
  bool enabled_;
  terrain_risk::Settings settings_;
  std::unordered_map<std::string, terrain_risk::Mask> masks_;
};

risk_model::Settings settingsFrom(const ModelMessage& message) {
  risk_model::Settings result;
  result.rcs = {message.rcs_a, message.rcs_b, message.rcs_c,
      message.sigma_ref_m2, message.fallback_v0_mps, message.fallback_v1_mps};
  result.smooth = {message.height_width_m, message.radius_width_m,
      message.center_rho_width, message.plateau_rho_width,
      message.visibility_width_m, message.clearance_m, message.grounded};
  risk_model::validate(result);
  return result;
}

risk_model::SourceInput inputFrom(const mmp_mission_msgs::msg::RiskModelSpec& message) {
  require(message.height_mode <= message.AGL, "unknown source height mode");
  require(std::isfinite(message.policy_reach_frame) && message.policy_reach_frame > 0,
          "policy reach must be finite and positive");
  if (message.calibration_from_r50) {
    const double calibrated=risk_model::calibratedC2(
        message.calibration_r50_m,message.calibration_sigma_m2);
    require(std::isfinite(message.c2) && message.c2>0 &&
        std::abs(message.c2-calibrated)<=16*std::numeric_limits<double>::epsilon()*
            std::max(message.c2,calibrated),
        "c2 disagrees with preserved R50/RCS calibration");
  } else {
    require(message.calibration_r50_m==0 && message.calibration_sigma_m2==0,
        "direct c2 input must not carry undeclared calibration provenance");
  }
  risk_model::SourceInput result;
  result.id = message.source_id;
  result.center_frame = {message.center.x, message.center.y, message.center.z};
  result.height_mode = message.height_mode == message.AGL
      ? risk_model::HeightMode::Agl : risk_model::HeightMode::Msl;
  result.sensor = {message.c1, message.c2};
  result.damage = {message.r_max_m, message.h_max_m, message.r_min_m,
      message.h_min_m, message.p0, message.rho_min, message.exponent};
  return result;
}

risk_model::SourceResult querySource(const risk_model::EnvironmentSnapshot& snapshot,
    std::size_t index, const Eigen::Vector3d& p, const Eigen::Vector3d& v,
    risk_model::FormulaMode mode, risk_model::RcsMode rcs) {
  if (index >= snapshot.sources.size()) throw std::out_of_range("risk source index");
  const auto scale = snapshot.scale;
  const risk_model::Vec3 p_si{p.x()*scale.xy_m, p.y()*scale.xy_m, p.z()*scale.z_m};
  const risk_model::Vec3 v_si{v.x()*scale.xy_m, v.y()*scale.xy_m, v.z()*scale.z_m};
  auto visibility = snapshot.terrain ? snapshot.terrain->sample(snapshot.sources[index], p_si)
                                    : risk_model::VisibilitySample{};
  auto result = risk_model::evaluateSource(snapshot.sources[index], p_si, v_si,
      snapshot.settings, mode, rcs, visibility);
  for (std::size_t axis=0; axis<3; ++axis) {
    const double unit = axis==2 ? scale.z_m : scale.xy_m;
    result.risk.grad_p[axis] *= unit;
    result.risk.grad_v[axis] *= unit;
  }
  return result;
}

} // namespace

class RiskProbabilityBridge {
 public:
  bool enabled=false, valid=false, configured=false;
  int drone_id=0;
  std::string session;
  risk_model::EnvironmentStore store;
  ModelMessage authored;
  std::optional<ModelMessage> pending;
  bool refreshing_active=false;
  std::uint64_t request_sequence=0;
  std::uint8_t request_status=EnvironmentMessage::NO_REQUEST;
  std::string request_id;
  double maximum_dt=0.1, absolute_tolerance=1e-6, relative_tolerance=1e-5;
  std::size_t maximum_samples=200000;
  rclcpp::Publisher<EnvironmentMessage>::SharedPtr environment_pub;
  rclcpp::Publisher<EvaluationMessage>::SharedPtr evaluation_pub;
  rclcpp::Subscription<ModelMessage>::SharedPtr model_sub;
  rclcpp::Subscription<mmp_traj_msgs::msg::PolyTraj>::SharedPtr trajectory_sub;
  mmp_traj_msgs::msg::PolyTraj::ConstSharedPtr last_wire;
  std::uint64_t invalidated_through_id=0;

  void publishEnvironment(double vertical_ratio, const std::string& reason={}) const {
    if (!environment_pub) return;
    EnvironmentMessage output;
    output.session_id=session;
    output.valid=valid;
    output.reason=reason;
    output.policy_vertical_ratio=vertical_ratio;
    output.model_version="probability_v1";
    output.request_status=request_status;
    output.request_id=request_id;
    output.has_pending=pending.has_value();
    if (pending) {
      output.pending=*pending;
      output.pending_request_id=pending->request_id;
    }
    output.resolved=authored;
    output.resolved.replace=true;
    const auto snapshot=store.snapshot();
    if (snapshot) {
      output.revision=snapshot->revision;
      output.terrain_revision=snapshot->terrain_revision;
      for (std::size_t i=0; i<snapshot->sources.size(); ++i) {
        auto& entry=output.resolved.sources[i];
        const auto& p=snapshot->sources[i].center_m;
        entry.center.x=p.x/snapshot->scale.xy_m;
        entry.center.y=p.y/snapshot->scale.xy_m;
        entry.center.z=p.z/snapshot->scale.z_m;
        entry.height_mode=entry.MSL;
      }
    }
    environment_pub->publish(output);
  }

  EvaluationMessage baseEvaluation() const {
    EvaluationMessage output;
    output.drone_id=drone_id;
    output.session_id=session;
    output.model_version="probability_v1";
    output.formula_mode="exact";
    output.evaluation_schema_version=2;
    output.frame_id=authored.frame_id;
    output.rcs_mode="heading";
    output.exposure_mode="distance";
    output.status=EvaluationMessage::INVALID;
    output.cumulative_probability=std::numeric_limits<double>::quiet_NaN();
    if (last_wire) {
      output.trajectory_id=last_wire->trajectory_id;
      output.start_time=last_wire->start_time;
    }
    if (const auto snapshot=store.snapshot()) {
      output.environment_revision=snapshot->revision;
      output.reference_length_m=snapshot->reference_length_m.value_or(0);
      output.unit_xy_m=snapshot->scale.xy_m;
      output.unit_z_m=snapshot->scale.z_m;
    }
    return output;
  }

  void invalidateEvaluation(const std::string& reason) const {
    if (!evaluation_pub) return;
    auto output=baseEvaluation();
    output.reason=reason;
    evaluation_pub->publish(output);
  }
};

bool PathManager::probabilityRiskMode() const {
  return probability_bridge_ && probability_bridge_->enabled;
}

bool PathManager::probabilityRiskReady() const {
  return !probabilityRiskMode() || probability_bridge_->valid;
}

std::string PathManager::riskSessionId() const {
  return probability_bridge_ ? probability_bridge_->session : std::string{};
}

std::shared_ptr<const risk_model::EnvironmentSnapshot> PathManager::riskProbabilitySnapshot() const {
  return probabilityRiskMode() && probability_bridge_->valid
      ? probability_bridge_->store.snapshot() : nullptr;
}

void PathManager::initRiskProbabilityBridge() {
  probability_bridge_=std::make_shared<RiskProbabilityBridge>();
  auto& bridge=*probability_bridge_;
  node_->get_parameter("drone_id", bridge.drone_id);
  std::random_device entropy;
  std::ostringstream session;
  session << node_->get_name() << '-' << std::hex << entropy() << entropy()
          << std::chrono::steady_clock::now().time_since_epoch().count();
  bridge.session=session.str();
  if (!node_->has_parameter("manager/risk_model_mode"))
    node_->declare_parameter("manager/risk_model_mode", std::string("default"));
  const auto mode=node_->get_parameter("manager/risk_model_mode").as_string();
  require(mode=="legacy" || mode=="default", "unknown manager/risk_model_mode");
  bridge.enabled=mode=="default";
  const auto qos=rclcpp::QoS(1).reliable().transient_local();
  bridge.environment_pub=node_->create_publisher<EnvironmentMessage>("/planning/risk_environment",qos);
  if (!bridge.enabled) {
    EnvironmentMessage legacy;
    legacy.session_id=bridge.session;
    legacy.model_version="legacy";
    legacy.valid=false;
    legacy.reason="legacy risk mode";
    bridge.environment_pub->publish(legacy);
    return;
  }
  for (const auto& item : std::vector<std::pair<std::string,double>>{
      {"optimization/dynamics_unit_xy_m",100.0}, {"optimization/dynamics_unit_z_m",100.0},
      {"manager/risk_evaluation_max_dt_s",0.1}, {"manager/risk_evaluation_abs_tol",1e-6},
      {"manager/risk_evaluation_rel_tol",1e-5}})
    if (!node_->has_parameter(item.first)) node_->declare_parameter(item.first,item.second);
  if (!node_->has_parameter("manager/risk_evaluation_max_samples"))
    node_->declare_parameter("manager/risk_evaluation_max_samples",200000);
  bridge.maximum_dt=node_->get_parameter("manager/risk_evaluation_max_dt_s").as_double();
  bridge.absolute_tolerance=node_->get_parameter("manager/risk_evaluation_abs_tol").as_double();
  bridge.relative_tolerance=node_->get_parameter("manager/risk_evaluation_rel_tol").as_double();
  const auto samples=node_->get_parameter("manager/risk_evaluation_max_samples").as_int();
  require(samples>0, "risk evaluation maximum samples must be positive");
  bridge.maximum_samples=static_cast<std::size_t>(samples);
  bridge.evaluation_pub=node_->create_publisher<EvaluationMessage>("/planning/trajectory_risk_evaluation",qos);
  // Explicit mode selection prevents existing legacy parameters from being
  // silently interpreted as probability sources.
  environment_->setResolvedRiskZones({});
  setProbabilityCallbackGroup(nullptr);
  bridge.publishEnvironment(environment_->verticalRatio(),"probability environment not received");
  bridge.invalidateEvaluation("probability environment not received");
}

void PathManager::setProbabilityCallbackGroup(rclcpp::CallbackGroup::SharedPtr group) {
  if (!probabilityRiskMode()) return;
  auto& bridge=*probability_bridge_;
  bridge.model_sub.reset();
  bridge.trajectory_sub.reset();
  rclcpp::SubscriptionOptions options;
  options.callback_group=std::move(group);
  const auto qos=rclcpp::QoS(1).reliable().transient_local();
  bridge.model_sub=node_->create_subscription<ModelMessage>("/mission/risk_models",qos,
      [this](ModelMessage::ConstSharedPtr message) {
        std::string reason;
        if (!setRiskModelsRuntime(*message,&reason))
          RCLCPP_ERROR(node_->get_logger(),"Risk model update rejected: %s",reason.c_str());
      },options);
  bridge.trajectory_sub=node_->create_subscription<mmp_traj_msgs::msg::PolyTraj>(
      "/planning/trajectory",qos,
      [this](mmp_traj_msgs::msg::PolyTraj::ConstSharedPtr message) {
        evaluateProbabilityWire(message);
      },options);
}

bool PathManager::setRiskModelsRuntime(const ModelMessage& message,std::string* reason) {
  return setRiskModelsRuntimeImpl(message,reason,false);
}

bool PathManager::setScenarioEnvironmentRuntime(
    const mmp_mission_msgs::msg::ScenarioEnvironment& scenario,std::string* reason) {
  try {
    require(probabilityRiskMode(),"unified scenarios require manager/risk_model_mode=default");
    require(scenario.risk_models.replace && !scenario.risk_models.cancel_pending &&
            scenario.obstacles.replace,"a scenario must replace its complete environment");
    require(environment_->terrain().valid && environment_->distanceField().hasData(),
            "scenario terrain must be ingested before its mission");
    std::vector<PendingEnvironmentObstacle> obstacles;
    obstacles.reserve(scenario.obstacles.obstacles.size());
    for (const auto& spec : scenario.obstacles.obstacles) {
      using Spec=mmp_mission_msgs::msg::DynamicObstacleSpec;
      require(spec.kind==Spec::KIND_CUBE || spec.kind==Spec::KIND_SPHERE,
              "unsupported scenario obstacle kind");
      const Eigen::Vector3d center(spec.center.x,spec.center.y,spec.center.z);
      const Eigen::Vector3d size(spec.size.x,spec.size.y,spec.size.z);
      require(center.allFinite(),"scenario obstacle center must be finite");
      if (spec.kind==Spec::KIND_CUBE)
        require(size.allFinite() && (size.array()>0.0).all(),
                "scenario obstacle size must be finite and positive");
      else
        require(std::isfinite(spec.radius) && spec.radius>0.0,
                "scenario obstacle radius must be finite and positive");
      obstacles.push_back({spec.kind==Spec::KIND_CUBE,center,size,spec.radius,spec.model});
    }
    const bool accepted=setRiskModelsRuntimeImpl(scenario.risk_models,reason,true,[&] {
      require(environment_->replaceObstacles(obstacles),"scenario obstacle batch could not be installed");
    });
    if (!accepted) return false;
    execution_->invalidatePending();
    publishDynamicObstacles();
    revalidateAfterEnvChange("a new scenario obstacle set",EnvChangeReason::OBSTACLE_BLOCKED);
    return true;
  } catch (const std::exception& error) {
    if (reason) *reason=error.what();
    return false;
  }
}

bool PathManager::setRiskModelsRuntimeImpl(const ModelMessage& message,std::string* reason,
    bool require_ready,const std::function<void()>& before_commit) {
  if (!probabilityRiskMode()) {
    if (reason) *reason="probability input requires manager/risk_model_mode=default";
    return false;
  }
  auto& bridge=*probability_bridge_;
  std::optional<ModelMessage> proposed;
  std::string submitted_id=message.request_id;
  if (submitted_id.empty())
    submitted_id=bridge.session+"/request-"+std::to_string(++bridge.request_sequence);
  try {
    require(message.schema_version==2,"unsupported risk model schema_version (expected 2)");
    require(message.frame_id=="map","risk frame_id must be map");
    require(message.model_version=="probability_v1","unsupported risk model_version");
    require(message.rcs_model=="ellipsoid_heading_v1","unsupported risk rcs_model");
    if (message.cancel_pending) {
      require(!bridge.refreshing_active,"internal terrain refresh cannot cancel requests");
      require(message.sources.empty(),"cancel_pending must not contain sources");
      bridge.pending.reset();
      bridge.request_id=submitted_id;
      bridge.request_status=EnvironmentMessage::CANCELLED;
      bridge.publishEnvironment(environment_->verticalRatio(),"pending request cancelled; accepted environment retained");
      if (reason) *reason="pending request cancelled";
      return true;
    }
    const risk_model::FrameScale scale{message.unit_xy_m,message.unit_z_m};
    risk_model::validate(scale);
    require(scale.xy_m==node_->get_parameter("optimization/dynamics_unit_xy_m").as_double() &&
            scale.z_m==node_->get_parameter("optimization/dynamics_unit_z_m").as_double(),
            "risk model units must match optimization dynamics units");
    const auto settings=settingsFrom(message);
    require(std::isfinite(message.reference_length_m),"reference_length_m must be finite");
    require(message.reference_length_configured ? message.reference_length_m>0
                                               : message.reference_length_m==0,
            "configured reference_length_m must be positive; unconfigured exposure must carry zero");

    ModelMessage full=message;
    full.request_id=submitted_id;
    full.replace=true;
    if (!message.replace && bridge.configured) {
      full.sources=bridge.authored.sources;
      full.sources.insert(full.sources.end(),message.sources.begin(),message.sources.end());
    }
    // Validate the entire proposed world before it may become a DEM request.
    std::unordered_set<std::string> ids;
    std::vector<risk_model::SourceInput> request;
    request.reserve(full.sources.size());
    for (const auto& entry : full.sources) {
      const auto input=inputFrom(entry);
      require(ids.insert(input.id).second,"duplicate source ID");
      risk_model::Source provisional{input.id,
          {input.center_frame.x*scale.xy_m,input.center_frame.y*scale.xy_m,
           input.center_frame.z*scale.z_m},input.sensor,input.damage};
      risk_model::validate(provisional);
      request.push_back(input);
    }
    proposed=full;
    const auto old=bridge.store.snapshot();
    auto comparison=full;
    comparison.request_id=bridge.authored.request_id;
    if (bridge.valid && comparison==bridge.authored && old &&
        old->terrain_revision==environment_->terrain().generation) {
      if (before_commit) before_commit();
      if (!bridge.refreshing_active) {
        bridge.pending.reset();
        bridge.authored.request_id=submitted_id;
        bridge.request_id=submitted_id;
        bridge.request_status=EnvironmentMessage::ACCEPTED;
      }
      bridge.publishEnvironment(environment_->verticalRatio());
      if (reason) reason->clear();
      return true;
    }

    bool mask_enabled=true;
    double radial_step=0;
    node_->get_parameter("manager/risk_terrain_mask_enable",mask_enabled);
    node_->get_parameter("manager/risk_mask_radial_step",radial_step);
    auto terrain=std::make_shared<FrozenProbabilityTerrain>(
        environment_->terrain(),scale,settings,mask_enabled,radial_step);
    auto candidate=bridge.store;
    const auto exposure=message.reference_length_configured
        ? std::optional<double>(message.reference_length_m) : std::nullopt;
    const auto committed=candidate.commit(request,true,scale,settings,exposure,terrain);
    if (!committed.accepted) throw std::invalid_argument(committed.reason);
    const auto snapshot=candidate.snapshot();
    terrain->build(snapshot->sources,full);

    std::vector<RiskZone> policy;
    policy.reserve(snapshot->sources.size());
    for (std::size_t i=0;i<snapshot->sources.size();++i) {
      const auto& p=snapshot->sources[i].center_m;
      const Eigen::Vector3d center(p.x/scale.xy_m,p.y/scale.xy_m,p.z/scale.z_m);
      require(center.allFinite(),"resolved policy center exceeds frame numeric range");
      policy.push_back({center,
                        full.sources[i].policy_reach_frame,1.0});
    }
    if (before_commit) before_commit();
    require(environment_->setResolvedRiskZones(policy),"policy geometry was rejected");
    bridge.store=std::move(candidate);
    bridge.authored=std::move(full);
    bridge.configured=true;
    bridge.valid=true;
    if (!bridge.refreshing_active) {
      // Only a successfully applied external request consumes/replaces the
      // waiting request. Rebuilding the accepted world for a DEM never does.
      bridge.pending.reset();
      bridge.request_id=submitted_id;
      bridge.request_status=EnvironmentMessage::ACCEPTED;
    }
    execution_->invalidatePending();
    bindRiskZonesToSearcher();
    bindRiskZonesToOptimizer();
    bridge.publishEnvironment(environment_->verticalRatio(),bridge.pending ? "awaiting DEM for pending request" : "");
    if (bridge.last_wire) bridge.invalidated_through_id=bridge.last_wire->trajectory_id;
    bridge.invalidateEvaluation("risk environment changed; waiting for a trajectory in this environment");
    if (execution_->current().duration>0) invalidateStoredTrajectoryForEnvChange();
    publishEffectiveRiskField();
    if (reason) reason->clear();
    return true;
  } catch (const std::exception& error) {
    const bool missing_dem=proposed && std::string(error.what()).find("AGL source")!=std::string::npos;
    if (!bridge.refreshing_active && missing_dem && !require_ready) {
      bridge.pending=std::move(proposed);
      bridge.request_id=submitted_id;
      bridge.request_status=EnvironmentMessage::AWAITING_DEM;
      const std::string waiting=std::string("Awaiting DEM: ")+error.what();
      bridge.publishEnvironment(environment_->verticalRatio(),waiting);
      if (reason) *reason=waiting;
      return true;
    }
    if (!bridge.refreshing_active) {
      bridge.request_id=submitted_id;
      bridge.request_status=EnvironmentMessage::REJECTED;
      bridge.publishEnvironment(environment_->verticalRatio(),std::string("Rejected update: ")+error.what());
    }
    if (reason) *reason=error.what();
    return false;
  }
}

void PathManager::refreshRiskProbabilityForTerrain() {
  if (!probabilityRiskMode()) return;
  auto& bridge=*probability_bridge_;
  if (bridge.pending) {
    const auto pending=*bridge.pending;
    std::string reason;
    // true can mean queued. Only disappearance of pending proves application.
    if (setRiskModelsRuntime(pending,&reason) && !bridge.pending) return;
  }
  if (!bridge.configured) {
    bridge.publishEnvironment(environment_->verticalRatio(),bridge.pending ?
        "awaiting DEM for pending request" : "probability environment not received");
    return;
  }
  auto request=bridge.authored;
  request.replace=true;
  std::string reason;
  bridge.refreshing_active=true;
  const bool rebuilt=setRiskModelsRuntime(request,&reason);
  bridge.refreshing_active=false;
  if (!rebuilt) {
    // Keep the waiting request and its DEM coverage even if the accepted
    // world's AGL centers cannot be reconstructed on this particular DEM.
    bridge.valid=false;
    if (bridge.last_wire) bridge.invalidated_through_id=bridge.last_wire->trajectory_id;
    bindRiskZonesToSearcher();
    bindRiskZonesToOptimizer();
    execution_->invalidatePending();
    if (execution_->current().duration>0) invalidateStoredTrajectoryForEnvChange();
    bridge.publishEnvironment(environment_->verticalRatio(),"DEM update invalidated accepted model: "+reason);
    bridge.invalidateEvaluation("DEM update invalidated model: "+reason);
  }
}

std::function<double(size_t,const Eigen::Vector3d&)>
PathManager::probabilityReferenceCallback() const {
  if (!probabilityRiskMode()) return {};
  const auto snapshot=riskProbabilitySnapshot();
  return [snapshot](size_t index,const Eigen::Vector3d& position) {
    if (!snapshot) throw std::runtime_error("probability environment is unavailable");
    return querySource(*snapshot,index,position,Eigen::Vector3d::Zero(),
        risk_model::FormulaMode::Planning,risk_model::RcsMode::Reference).risk.value;
  };
}

std::function<double(size_t,const Eigen::Vector3d&,const Eigen::Vector3d&,
                     Eigen::Vector3d*,Eigen::Vector3d*)>
PathManager::probabilityHeadingCallback() const {
  if (!probabilityRiskMode()) return {};
  const auto snapshot=riskProbabilitySnapshot();
  return [snapshot](size_t index,const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,
                    Eigen::Vector3d* gp,Eigen::Vector3d* gv) {
    if (!snapshot) throw std::runtime_error("probability environment is unavailable");
    const auto result=querySource(*snapshot,index,position,velocity,
        risk_model::FormulaMode::Planning,risk_model::RcsMode::Heading);
    if (gp) *gp=Eigen::Vector3d(result.risk.grad_p.x,result.risk.grad_p.y,result.risk.grad_p.z);
    if (gv) *gv=Eigen::Vector3d(result.risk.grad_v.x,result.risk.grad_v.y,result.risk.grad_v.z);
    return result.risk.value;
  };
}

void PathManager::evaluateProbabilityWire(const mmp_traj_msgs::msg::PolyTraj::ConstSharedPtr& message) {
  if (!probabilityRiskMode() || !message) return;
  auto& bridge=*probability_bridge_;
  if (message->drone_id!=bridge.drone_id || message->planner_session_id!=bridge.session) return;
  if (message->trajectory_id<=bridge.invalidated_through_id) return;
  if (bridge.last_wire && message->trajectory_id<bridge.last_wire->trajectory_id) return;
  bridge.last_wire=message;
  auto output=bridge.baseEvaluation();
  const auto snapshot=riskProbabilitySnapshot();
  if (!snapshot) {
    output.reason="probability environment unavailable";
    bridge.evaluation_pub->publish(output);
    return;
  }
  try {
    require(message->order>=1 && message->order<=12,"unsupported polynomial order");
    const std::size_t count=message->duration.size(), stride=message->order+1;
    require(count>0 && count<=100000,"empty or excessive trajectory piece count");
    require(message->coef_x.size()==count*stride && message->coef_y.size()==count*stride &&
            message->coef_z.size()==count*stride,"trajectory coefficient array sizes disagree");
    for (const auto* coefficients : {&message->coef_x,&message->coef_y,&message->coef_z})
      for (float value : *coefficients)
        require(std::isfinite(value),"non-finite trajectory coefficient");
    std::vector<risk_model::PolynomialPiece> pieces(count);
    const std::array<const std::vector<float>*,3> axes{{&message->coef_x,&message->coef_y,&message->coef_z}};
    for (std::size_t piece=0;piece<count;++piece) {
      const double duration=message->duration[piece];
      require(std::isfinite(duration) && duration>0,"invalid trajectory duration");
      pieces[piece].duration_s=duration;
      for (std::size_t axis=0;axis<3;++axis) {
        auto& coefficients=pieces[piece].coefficients_m[axis];
        coefficients.resize(stride);
        const double scale=axis==2?snapshot->scale.z_m:snapshot->scale.xy_m;
        for (std::size_t j=0;j<stride;++j)
          coefficients[stride-1-j]=(*axes[axis])[piece*stride+j]*scale;
      }
    }
    risk_model::IntegrationSettings settings;
    settings.reference_length_m=snapshot->reference_length_m.value_or(risk_model::unset);
    settings.maximum_time_step_s=bridge.maximum_dt;
    settings.absolute_tolerance=bridge.absolute_tolerance;
    settings.relative_tolerance=bridge.relative_tolerance;
    settings.maximum_evaluations=bridge.maximum_samples;
    const auto evaluated=risk_model::evaluatePolynomialTrajectory(*snapshot,pieces,settings);
    output.absolute_tolerance=settings.absolute_tolerance;
    output.relative_tolerance=settings.relative_tolerance;
    output.boundary_time_tolerance_s=settings.boundary_time_tolerance_s;
    output.hazard_error_estimate=evaluated.hazard_error_estimate;
    output.length_error_estimate_m=evaluated.length_error_estimate_m;
    output.boundary_count=evaluated.boundary_count;
    output.unresolved_boundary_count=evaluated.unresolved_boundary_count;
    output.boundaries_resolved=evaluated.boundaries_resolved;
    output.subdivision_limited=evaluated.subdivision_limited;
    output.partition_reason=evaluated.partition_reason;
    output.status=static_cast<std::uint8_t>(evaluated.accumulated.status);
    output.reason=evaluated.accumulated.reason;
    output.cumulative_probability=evaluated.accumulated.probability.value_or(
        std::numeric_limits<double>::quiet_NaN());
    output.hazard=evaluated.accumulated.hazard;
    output.length_m=evaluated.accumulated.length_m;
    output.sample_count=evaluated.sample_count;
    output.fallback_sample_count=evaluated.fallback_sample_count;
    output.visibility_assumed=evaluated.flags.visibility_assumed;
    output.coincident_source=evaluated.flags.coincident_source;
    for (const auto& interval : evaluated.intervals) {
      output.interval_lengths_m.push_back(interval.length_m);
      output.probabilities.push_back(interval.probability);
      output.interval_log_survivals.push_back(interval.log_survival);
      output.interval_time_begin_s.push_back(interval.time_begin_s);
      output.interval_time_end_s.push_back(interval.time_end_s);
      output.interval_arc_begin_m.push_back(interval.arc_begin_m);
      output.interval_arc_end_m.push_back(interval.arc_end_m);
    }
  } catch (const std::exception& error) {
    output.reason=error.what();
  }
  bridge.evaluation_pub->publish(output);
}

} // namespace path_manager
