#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include "path_manager/mission_planning_backend.h"
#include "path_manager/planning_computation.h"

namespace swarm_formation { class LogManager; }
namespace path_manager {
class PathManager;
class SegmentChainPlanner;

struct RosPlanningOptions {
  std::string node_name{"mission_planner"};
  std::string node_namespace;
  rclcpp::NodeOptions node_options;
  // The selected context must already be initialized. This backend never
  // calls rclcpp::init/shutdown. A value-input caller need not create a Node.
  // automatically_declare_parameters_from_overrides must remain false so
  // legacy consumers retain their declaration order and exact value types.
};

// Explicit compatibility implementation retaining live ROS parameters,
// visualization and the existing manager/chain/optimizer computation pipeline.
// The generic API is ROS-free; this implementation deliberately is not.
class RosPlanningBackend final : public MissionPlanningBackend {
 public:
  explicit RosPlanningBackend(rclcpp::Node::SharedPtr node,
      std::shared_ptr<swarm_formation::LogManager> logger = nullptr);
  explicit RosPlanningBackend(RosPlanningOptions options,
      std::shared_ptr<swarm_formation::LogManager> logger = nullptr);
  ~RosPlanningBackend() override;
  void initialize(PlanSettings settings, PlanningEnvironment environment) override;
  // Calculates owned values without adopting a candidate into execution
  // storage. Environment preparation keeps its existing live revalidation
  // behavior. Calls and subsequent adoption must be serialized; the result's
  // context rejects reuse or adoption after another calculation, execution
  // update, environment update, or model/search replacement.
  // Do not change live ROS parameter values between calculation and adoption:
  // parameter updates are not a frozen settings snapshot or revision token.
  PlanningComputation compute(MissionPlanRequest request);
  // Compatibility entry: compute, explicitly apply execution effects, then
  // return owned values with their execution identity. No trajectory data is
  // reconstructed from the execution slot.
  MissionPlanResult plan(MissionPlanRequest request) override;
  void setVehicleModel(mmp_vehicle_dynamics::VehicleModel::Ptr model) override;
  void setRouteSearchBackend(std::shared_ptr<RouteSearchBackend> backend) override;
  bool setTerrain(TerrainGrid terrain, std::string *error = nullptr) override;
  void clearTerrain() override;
  bool setRiskZones(const std::vector<RiskZone> &zones) override;
  int addDynamicSphere(const Eigen::Vector3d &center, double radius,
                       const std::string &model = "") override;
  int addDynamicBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size,
                    const std::string &model = "") override;
  void clearDynamicObstacles() override;

  // Execution/callback adapter access only, absent from MissionPlanner's API.
  std::shared_ptr<PathManager> executionManager() const;
  rclcpp::Node::SharedPtr node() const;
  bool chainEnabled() const;
  int configuredSegments() const;
  void setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr &terrain);

 private:
  void requireInitialized() const;
  // Compute the direct-mode product without applying it to execution storage.
  // May declare the legacy final-boundary-relaxation ROS parameter on first use.
  PlanResult computeDirect(const StartHead &head,
                        const std::vector<Eigen::Vector3d> &waypoints,
                        const ego_planner::TailBoundary &tail,
                        PlanningComputation &computation);
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<swarm_formation::LogManager> logger_;
  std::shared_ptr<PathManager> manager_;
  // Destroy chain before the manager and logger its callbacks borrow.
  std::unique_ptr<SegmentChainPlanner> chain_;
  std::optional<RosPlanningOptions> pending_options_;
  bool initialization_started_{false};
  bool initialized_{false};
  bool chain_enabled_{false};
  int configured_segments_{3};
  int drone_id_{0};
};

// Bound-node mode keeps that node as the single live setting authority.
// initialize accepts empty PlanSettings.parameters only (injection handles are
// allowed). No capture/reapply, eager declarations, or frozen runtime values.
std::unique_ptr<RosPlanningBackend> makeRosPlanningBackend(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<swarm_formation::LogManager> logger = nullptr);

// Value-input mode merges explicit typed settings over NodeOptions overrides
// before constructing the node; existing ROS declaration order is retained.
std::unique_ptr<RosPlanningBackend> makeRosPlanningBackend(
    RosPlanningOptions options,
    std::shared_ptr<swarm_formation::LogManager> logger = nullptr);
}  // namespace path_manager
