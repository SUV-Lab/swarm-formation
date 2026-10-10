#include "path_manager/ros_planning_backend.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "path_manager/path_manager.h"
#include "path_manager/segment_chain_planner.h"
#include "path_manager/ros_terrain_grid.h"

namespace path_manager {
RosPlanningBackend::RosPlanningBackend(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<swarm_formation::LogManager> logger)
    : node_(std::move(node)), logger_(std::move(logger))
{
  if (!node_) throw std::invalid_argument("RosPlanningBackend requires a node");
}
RosPlanningBackend::RosPlanningBackend(
    RosPlanningOptions options,
    std::shared_ptr<swarm_formation::LogManager> logger)
    : logger_(std::move(logger)), pending_options_(std::move(options)) {}
RosPlanningBackend::~RosPlanningBackend() = default;

void RosPlanningBackend::initialize(PlanSettings settings, PlanningEnvironment environment)
{
  if (initialization_started_)
    throw std::logic_error("RosPlanningBackend initialization may be attempted only once");
  initialization_started_ = true;
  if (!pending_options_ && !settings.parameters.empty())
    throw std::invalid_argument(
        "bound-node RosPlanningBackend uses live node parameters; "
        "PlanSettings.parameters must be empty");
  if (settings.vehicle_model && !settings.vehicle_model->valid())
    throw std::invalid_argument("initial VehicleModel is invalid");
  grid_map_msgs::msg::GridMap::SharedPtr initial_terrain;
  if (environment.terrain) {
    std::string error;
    initial_terrain = encodeTerrainGrid(std::move(*environment.terrain), &error);
    if (!initial_terrain) throw std::invalid_argument(error);
  }
  if (environment.dynamic_obstacles) {
    for (const auto &obstacle : *environment.dynamic_obstacles) {
      std::visit([](const auto &value) {
        if (!value.center.allFinite())
          throw std::invalid_argument("initial obstacle center is not finite");
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, SphereObstacle>) {
          if (!std::isfinite(value.radius) || !(value.radius > 0.0))
            throw std::invalid_argument("initial sphere radius must be positive and finite");
        } else {
          if (!value.size.allFinite() || !(value.size.array() > 0.0).all())
            throw std::invalid_argument("initial box size must be positive and finite");
        }
      }, obstacle);
    }
  }
  if (pending_options_) {
    auto options = std::move(*pending_options_);
    if (!rclcpp::ok(options.node_options.context()))
      throw std::invalid_argument("RosPlanningBackend requires an initialized ROS context");
    if (options.node_options.automatically_declare_parameters_from_overrides())
      throw std::invalid_argument(
          "RosPlanningBackend requires deferred parameter declarations; "
          "automatically_declare_parameters_from_overrides must be false");
    auto overrides = options.node_options.parameter_overrides();
    for (const auto &entry : settings.parameters) {
      overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
          [&](const rclcpp::Parameter &parameter) {
            return parameter.get_name() == entry.first;
          }), overrides.end());
      overrides.push_back(std::visit([&](const auto &value) {
        return rclcpp::Parameter(entry.first, value);
      }, entry.second));
    }
    options.node_options.parameter_overrides(overrides);
    node_ = std::make_shared<rclcpp::Node>(options.node_name,
                                        options.node_namespace,
                                        options.node_options);
    pending_options_.reset();
  }
  if (!logger_)
    logger_ = std::make_shared<swarm_formation::LogManager>(
        node_->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);

  // The ROS adapter already declares these; a library caller need not do so.
  // declare_parameter honors NodeOptions/YAML overrides on first declaration.
  const auto declare_once = [&](const char *name, auto value) {
    if (!node_->has_parameter(name)) node_->declare_parameter(name, value);
  };
  declare_once("drone_id", 0);
  declare_once("enable_debug_logs", false);
  declare_once("enable_lbfgs_detail_logs", false);
  node_->get_parameter("drone_id", drone_id_);

  manager_ = std::make_shared<PathManager>(node_);
  declare_once("chain/enable", false);
  declare_once("chain/segments", 3);
  declare_once("chain/inherit_route", true);
  node_->get_parameter("chain/enable", chain_enabled_);
  node_->get_parameter("chain/segments", configured_segments_);
  bool inherit_route = true;
  node_->get_parameter("chain/inherit_route", inherit_route);
  chain_ = std::make_unique<SegmentChainPlanner>(
      node_, manager_, logger_.get(), configured_segments_, inherit_route);
  if (settings.vehicle_model) manager_->setVehicleModel(std::move(settings.vehicle_model));
  if (settings.route_search_backend)
    manager_->setRouteSearchBackend(std::move(settings.route_search_backend));
  // Preserve ingestion/grounding and deterministic obstacle spawn order.
  // Missing fields perform zero clears, retaining the existing YAML state.
  if (initial_terrain) manager_->setTerrainData(initial_terrain);
  if (environment.risk_zones && !manager_->setRiskZonesRuntime(*environment.risk_zones))
    throw std::invalid_argument("initial risk-zone set was rejected");
  if (environment.dynamic_obstacles) {
    manager_->clearDynamicObstacles();
    for (const auto &obstacle : *environment.dynamic_obstacles) {
      const int id = std::visit([&](const auto &value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, SphereObstacle>)
          return manager_->addDynamicSphere(value.center, value.radius, value.model);
        else
          return manager_->addDynamicBox(value.center, value.size, value.model);
      }, obstacle);
      // -2 is the existing successful deferral until the first SDF build.
      if (id == -1) throw std::invalid_argument("initial dynamic obstacle was rejected");
    }
  }
  initialized_ = true;
}

void RosPlanningBackend::requireInitialized() const
{
  if (!initialized_) throw std::logic_error("RosPlanningBackend is not initialized");
}
std::shared_ptr<PathManager> RosPlanningBackend::executionManager() const
{
  requireInitialized(); return manager_;
}
rclcpp::Node::SharedPtr RosPlanningBackend::node() const
{
  requireInitialized(); return node_;
}
bool RosPlanningBackend::chainEnabled() const
{
  requireInitialized(); return chain_enabled_;
}
int RosPlanningBackend::configuredSegments() const
{
  requireInitialized(); return configured_segments_;
}
void RosPlanningBackend::setVehicleModel(mmp_vehicle_dynamics::VehicleModel::Ptr model)
{
  requireInitialized(); manager_->setVehicleModel(std::move(model));
}
void RosPlanningBackend::setRouteSearchBackend(std::shared_ptr<RouteSearchBackend> backend)
{
  requireInitialized(); manager_->setRouteSearchBackend(std::move(backend));
}
bool RosPlanningBackend::setTerrain(TerrainGrid terrain, std::string *error)
{
  requireInitialized();
  auto message = encodeTerrainGrid(std::move(terrain), error);
  if (!message) return false;
  manager_->setTerrainData(message);
  return true;
}
void RosPlanningBackend::clearTerrain()
{
  requireInitialized();
  manager_->setTerrainData(std::make_shared<grid_map_msgs::msg::GridMap>());
}
void RosPlanningBackend::setTerrainData(
    const grid_map_msgs::msg::GridMap::SharedPtr &terrain)
{
  requireInitialized(); manager_->setTerrainData(terrain);
}
bool RosPlanningBackend::setRiskZones(const std::vector<RiskZone> &zones)
{
  requireInitialized(); return manager_->setRiskZonesRuntime(zones);
}
int RosPlanningBackend::addDynamicSphere(const Eigen::Vector3d &center,
                                        double radius, const std::string &model)
{
  requireInitialized(); return manager_->addDynamicSphere(center, radius, model);
}
int RosPlanningBackend::addDynamicBox(const Eigen::Vector3d &center,
                                     const Eigen::Vector3d &size,
                                     const std::string &model)
{
  requireInitialized(); return manager_->addDynamicBox(center, size, model);
}
void RosPlanningBackend::clearDynamicObstacles()
{
  requireInitialized(); manager_->clearDynamicObstacles();
}
std::unique_ptr<RosPlanningBackend> makeRosPlanningBackend(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<swarm_formation::LogManager> logger)
{
  return std::make_unique<RosPlanningBackend>(std::move(node), std::move(logger));
}
std::unique_ptr<RosPlanningBackend> makeRosPlanningBackend(
    RosPlanningOptions options,
    std::shared_ptr<swarm_formation::LogManager> logger)
{
  return std::make_unique<RosPlanningBackend>(std::move(options), std::move(logger));
}

MissionPlanResult RosPlanningBackend::plan(MissionPlanRequest request)
{
  PlanningComputation computation = compute(std::move(request));
  const ExecutionReceipt receipt =
      manager_->applyPlanningComputation(computation);
  MissionPlanResult result;
  result.drone_id = drone_id_;
  result.environment_generation = computation.environment_generation;
  if (!receipt.applied) {
    result.status = PlanResult::failed(
        "planning computation became stale before execution adoption");
    return result;
  }
  result.status = std::move(computation.status);
  // Failed computations can retain an intermediate product only for the
  // compatibility invalidation receipt. Never expose it as a new flight.
  if (!result.status.hasTrajectory()) return result;
  if (!computation.product) {
    result.status = PlanResult::failed("planning computation has no trajectory");
    return result;
  }
  auto &product = *computation.product;
  result.trajectory = std::move(product.trajectory);
  result.reference_trajectory = std::move(product.reference_trajectory);
  result.trajectory_id = receipt.trajectory_id;
  result.drone_id = receipt.drone_id;
  result.start_time = product.start_time;
  result.reference_start_time = product.reference_start_time;
  result.evaluation = std::move(computation.evaluation);
  result.phases = std::move(computation.phases);
  return result;
}

PlanningComputation RosPlanningBackend::compute(MissionPlanRequest request)
{
  requireInitialized();
  manager_->beginPlanningComputation();
  // A fresh result for every attempt. A refusal must never package the
  // previous execution slot or the chain's previous evaluation as this plan.
  const auto failed = [&](PlanResult status) {
    PlanningComputation result;
    result.status = std::move(status);
    return manager_->sealPlanningComputation(std::move(result));
  };
  if (request.head.src == StartStateSource::UNSPECIFIED)
    return failed(PlanResult::failedBecause(
        PlanReason::INITIAL_STATE_UNSPECIFIED,
        "start state has no source — the planner will not invent one"));
  if (!request.head.pos_u.allFinite() || !request.head.vel_u.allFinite() ||
      !request.head.acc_u.allFinite())
    return failed(PlanResult::failedBecause(
        PlanReason::INITIAL_STATE_MALFORMED, "start state is not finite"));
  if (request.waypoints.empty())
    return failed(PlanResult::failed("mission has no waypoints"));
  for (const auto &waypoint : request.waypoints)
    if (!waypoint.allFinite())
      return failed(PlanResult::failed("mission waypoint is not finite"));

  if (request.length_per_piece > 0.0)
    manager_->setLengthPerPiece(request.length_per_piece);
  if (request.obstacle_clearance > 0.0)
    manager_->setObstacleClearance(request.obstacle_clearance);

  // Exactly the existing commanded-start conversion. Reused requests remain
  // AGL at the caller because this function owns its working request copy.
  if (request.start_altitude == StartAltitude::COMMAND_AGL) {
    if (manager_->hasTerrainData()) {
      double base = 0.0;
      manager_->terrainElevation(request.head.pos_u.x(),
                                  request.head.pos_u.y(), &base);
      request.head.pos_u.z() =
          base + std::max(request.head.pos_u.z(), manager_->minGoalAgl());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "[START AGL] no terrain at first plan — commanded start "
                  "z=%.2f kept ABSOLUTE", request.head.pos_u.z());
      logger_->warnf("[START AGL] no terrain at first plan — commanded start "
                     "z=%.2f kept ABSOLUTE", request.head.pos_u.z());
    }
  }

  if (!manager_->isOptimizerInitialized()) {
    try {
      manager_->initOptimizer();
      manager_->deliverTrajToOptimizer();
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize optimizer: %s",
                   error.what());
      logger_->errorf("Failed to initialize optimizer: %s", error.what());
      return failed(PlanResult::failed(
          std::string("optimizer initialization failed: ") + error.what()));
    }
  }

  manager_->setFormationInfo(drone_id_, request.formation_type,
                             request.formation_pattern);
  PlanningComputation computation;
  if (chain_enabled_)
    computation = chain_->compute(request.head, request.waypoints, request.tail);
  else
    computation.status = planDirect(request.head, request.waypoints,
                                    request.tail, computation);
  if (!computation.status.hasTrajectory())
    return manager_->sealPlanningComputation(std::move(computation));

  // Preserve the existing ordering: formation reaches the optimizer after
  // this successful solve, not before it. Moving this earlier changes solves.
  manager_->setFormationToOptimizer(request.formation_pattern,
                                    request.formation_size);

  return manager_->sealPlanningComputation(std::move(computation));
}

PlanResult RosPlanningBackend::planDirect(
    const StartHead &head, const std::vector<Eigen::Vector3d> &waypoints,
    const ego_planner::TailBoundary &tail, PlanningComputation &computation)
{
  // The chain-disabled dispatch retains the FSM's boundary rules. In
  // particular it does not dispatch a transition or add a whole-flight gate.
  // Warning/error destinations remain console AND file irrespective of debug.
  const bool judge_pva = judgeFullPva(head.src, head.acc_prescribed);
  const std::string head_problem =
      !isOperatorInput(head.src)
          ? std::string{}
          : (judge_pva ? manager_->pvaEnvelopeProblem(
                             head.pos_u, head.vel_u, head.acc_u)
                       : manager_->stateEnvelopeProblem(head.vel_u));
  if (!head_problem.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ENVELOPE] commanded initial state REJECTED: "
                 "%s (INITIAL_MODE_UNSUPPORTED)", head_problem.c_str());
    logger_->errorf("[ENVELOPE] commanded initial state REJECTED: "
                    "%s (INITIAL_MODE_UNSUPPORTED)", head_problem.c_str());
    return PlanResult::failedBecause(
        PlanReason::INITIAL_MODE_UNSUPPORTED,
        "initial state unsupported: " + head_problem);
  }

  ego_planner::TailBoundary effective_tail = tail;
  bool relaxed = false;
  std::string relax_why;
  if (tail.anyPrescribed()) {
    std::string problem;
    if (!tail.allFinite()) {
      problem = "non-finite final boundary";
    } else if (tail.prescribe_vel) {
      const std::string envelope = manager_->stateEnvelopeProblem(tail.vel);
      if (!envelope.empty())
        problem = "final velocity outside the cruise validity region: " +
                  envelope;
    }
    if (!problem.empty()) {
      bool allow = false;
      if (!node_->has_parameter("planning/allow_final_boundary_relaxation"))
        node_->declare_parameter("planning/allow_final_boundary_relaxation",
                                  false);
      node_->get_parameter("planning/allow_final_boundary_relaxation", allow);
      if (!allow) {
        RCLCPP_ERROR(node_->get_logger(), "[PHASE] final boundary REJECTED: %s",
                     problem.c_str());
        logger_->errorf("[PHASE] final boundary REJECTED: %s", problem.c_str());
        return PlanResult::failed(problem);
      }
      RCLCPP_WARN(node_->get_logger(),
                  "[PHASE] final boundary relaxed (opt-in): %s",
                  problem.c_str());
      logger_->warnf("[PHASE] final boundary relaxed (opt-in): %s",
                     problem.c_str());
      effective_tail = ego_planner::TailBoundary{};
      relaxed = true;
      relax_why = problem + " — final boundary relaxed (opt-in)";
    }
  }

  auto product = manager_->computeGlobalTrajectory(head, waypoints,
                                                    effective_tail);
  const bool solved = product.has_value();
  if (product) computation.recordProduct(std::move(*product));
  PlanResult result = solved ? PlanResult::success()
                            : PlanResult::failed("single plan failed");
  if (relaxed)
    result.degrade(PlanReason::FINAL_BOUNDARY_RELAXED, relax_why);
  return result;
}

}  // namespace path_manager
