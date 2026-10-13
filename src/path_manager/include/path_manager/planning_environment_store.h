#ifndef PATH_MANAGER_PLANNING_ENVIRONMENT_STORE_H
#define PATH_MANAGER_PLANNING_ENVIRONMENT_STORE_H

#include "path_manager/terrain_risk.h"
#include "path_planner/dyn_a_star.h"
#include "path_planner/sdf/sdf_manager.h"
#include "../../common/log_manager.hpp"
#include <grid_map_msgs/msg/grid_map.hpp>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace path_manager {

// Values already read and normalized by the ROS adapter. This owner does not
// declare parameters or keep another source of mission/optimizer defaults.
struct PlanningEnvironmentOptions {
  double sdf_voxel_size;
  double sdf_voxel_z;
  bool sdf_voxel_size_auto;
  double dynamic_influence_radius;
  bool obstacle_ground_snap;
  uint32_t yaw_seed;
  bool risk_zone_agl;
  double risk_vertical_ratio;
  bool risk_terrain_mask_enable;
  double risk_mask_radial_step;
  double risk_mask_softness;
  bool risk_grounded;
};

struct EnvironmentObstacle {
  int id;
  Eigen::Vector3d center;
  Eigen::Vector3d size;
  bool is_box;
  std::string model;
  double yaw;
};

struct PendingEnvironmentObstacle {
  bool is_box;
  Eigen::Vector3d center;
  Eigen::Vector3d size;
  double radius;
  std::string model;
};

enum class TerrainUpdateKind { REJECTED, CLEARED, REPLACED };
struct TerrainUpdate {
  TerrainUpdateKind kind{TerrainUpdateKind::REJECTED};
  bool had_terrain{false};
  bool obstacles_cleared{false};
  bool accepted() const { return kind != TerrainUpdateKind::REJECTED; }
};

// Owns the live terrain, SDF, risk field and obstacle inputs. Serialized
// mutation belongs to the adapter; concurrent queries are allowed only while
// the owner and its inputs remain unchanged. It owns no execution trajectory,
// search policy, ROS publishers or callbacks into an execution owner.
//
// Terrain ingestion, deferred-obstacle installation and risk refresh are
// separate stages so the adapter can preserve its existing revalidation and
// publication order. Complete those stages before starting a new query/solve.
// Rebuild borrowed solver bindings after changing this owner.
class PlanningEnvironmentStore {
 public:
  PlanningEnvironmentStore(PlanningEnvironmentOptions options,
      std::shared_ptr<swarm_formation::LogManager> logger,
      std::vector<RiskZone> initial_zones = {});
  PlanningEnvironmentStore(const PlanningEnvironmentStore &) = delete;
  PlanningEnvironmentStore &operator=(const PlanningEnvironmentStore &) = delete;

  TerrainUpdate ingestTerrain(const grid_map_msgs::msg::GridMap::SharedPtr &msg);
  bool ensureTerrainDistanceField();
  bool ensureDistanceField(const Eigen::Vector3d &mission_lower,
                           const Eigen::Vector3d &mission_upper);
  bool terrainBounds(Eigen::Vector3d *lower, Eigen::Vector3d *upper);
  // Successful replacement refreshes effective geometry and masks. Malformed
  // zones leave the entire previous field and generation unchanged.
  bool setRiskZones(const std::vector<RiskZone> &zones);
  void refreshRisk();
  // Preserve the adapter's explicit search binding points. PathSearcher
  // retains the address of this owned vector between searches.
  void refreshSearchRiskZones();

  // >=0 installed, -1 failed, -2 queued until grid ready. Nonfinite centers,
  // nonpositive/nonfinite radius or extents are rejected before any mutation.
  int addSphere(const Eigen::Vector3d &center, double radius,
                const std::string &model = "");
  int addBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size,
             const std::string &model = "");
  void clearObstacles();
  std::vector<PendingEnvironmentObstacle> takePendingObstacles();

  const TerrainData &terrain() const { return terrain_data_; }
  const path_planner::sdf::SDFManager &distanceField() const { return sdf_manager_; }
  double voxelSize() const { return sdf_voxel_size_; }
  double voxelZ() const { return options_.sdf_voxel_z; }
  double verticalRatio() const { return options_.risk_vertical_ratio; }
  double maskSoftness() const { return options_.risk_mask_softness; }
  bool groundedRisk() const { return options_.risk_grounded; }
  const std::vector<RiskZone> &riskZones() const { return risk_zones_; }
  const std::vector<path_planner::search::RiskZoneLite> &searchRiskZones() const {
    return astar_risks_;
  }
  const std::vector<EnvironmentObstacle> &obstacles() const { return obstacles_; }
  uint64_t zoneGeneration() const { return zone_policy_generation_; }
  uint64_t revision() const {
    return zone_policy_generation_ + terrain_data_.generation + sdf_manager_.revision();
  }
  uint64_t riskZoneFingerprint() const;

  // Grounding clearance is a call input. A local optimizer profile must not
  // rewrite the shared environment or another worker's risk callbacks.
  double visibility(size_t zone, const Eigen::Vector3d &pos, double clearance,
                    Eigen::Vector3d *gradient = nullptr) const;
  double visibilityValue(size_t zone, const Eigen::Vector3d &pos,
                         double clearance) const;
  double groundedZ(const Eigen::Vector3d &pos, double clearance,
                   double *derivative = nullptr) const;
  double shadowCeiling(size_t zone, const Eigen::Vector3d &pos) const;
  double ellipsoidRadius(const RiskZone &zone, const Eigen::Vector3d &pos) const;
  double riskValue(size_t zone, const Eigen::Vector3d &pos, double clearance) const;

 private:
  terrain_risk::Settings riskSettings(double clearance) const;
  const terrain_risk::Mask *riskMask(size_t zone) const;
  void refreshEffectiveRiskZones();
  void rebuildTerrainRiskMasks();
  bool buildSDFForBounds(const Eigen::Vector3d &lower, const Eigen::Vector3d &upper);
  Eigen::Vector3d groundedCenter(const Eigen::Vector3d &center, double half_height) const;

  const PlanningEnvironmentOptions options_;
  double sdf_voxel_size_;
  bool sdf_voxel_size_auto_;
  std::shared_ptr<swarm_formation::LogManager> log_manager_;
  TerrainData terrain_data_;
  path_planner::sdf::SDFManager sdf_manager_;
  bool sdf_built_{false};
  bool terrain_bbox_computed_{false};
  Eigen::Vector3d terrain_bbox_lo_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d terrain_bbox_hi_{Eigen::Vector3d::Zero()};
  std::vector<RiskZone> risk_zones_raw_;
  std::vector<RiskZone> risk_zones_;
  uint64_t zone_policy_generation_{0};
  std::vector<path_planner::search::RiskZoneLite> astar_risks_;
  std::vector<terrain_risk::Mask> terrain_risk_masks_;
  std::vector<EnvironmentObstacle> obstacles_;
  std::vector<PendingEnvironmentObstacle> pending_obstacles_;
  std::mt19937 yaw_rng_;
};

}  // namespace path_manager
#endif  // PATH_MANAGER_PLANNING_ENVIRONMENT_STORE_H
