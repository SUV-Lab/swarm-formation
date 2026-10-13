#include "path_manager/planning_environment_store.h"
#include "path_planner/risk_shape.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <malloc.h>
#include <stdexcept>
#include <utility>

namespace path_manager {

PlanningEnvironmentStore::PlanningEnvironmentStore(
    PlanningEnvironmentOptions options,
    std::shared_ptr<swarm_formation::LogManager> logger,
    std::vector<RiskZone> initial_zones)
    : options_(std::move(options)), sdf_voxel_size_(options_.sdf_voxel_size),
      sdf_voxel_size_auto_(options_.sdf_voxel_size_auto), log_manager_(std::move(logger)),
      risk_zones_raw_(std::move(initial_zones)), yaw_rng_(options_.yaw_seed)
{
    if (!log_manager_) throw std::invalid_argument("environment logger is null");
    if (options_.dynamic_influence_radius > sdf_manager_.influenceRadius())
        sdf_manager_.setInfluenceRadius(options_.dynamic_influence_radius);
    refreshRisk();
}

terrain_risk::Settings PlanningEnvironmentStore::riskSettings(double clearance) const
{
    return {options_.risk_terrain_mask_enable, options_.risk_grounded,
            options_.risk_mask_radial_step, options_.risk_mask_softness, clearance};
}

const terrain_risk::Mask *PlanningEnvironmentStore::riskMask(size_t zone) const
{
    return zone < terrain_risk_masks_.size() ? &terrain_risk_masks_[zone] : nullptr;
}

void PlanningEnvironmentStore::refreshRisk()
{
    refreshEffectiveRiskZones();
    rebuildTerrainRiskMasks();
}

void PlanningEnvironmentStore::refreshSearchRiskZones()
{
    astar_risks_.clear();
    astar_risks_.reserve(risk_zones_.size());
    for (const auto &zone : risk_zones_)
        astar_risks_.push_back({zone.center, zone.reach, zone.peak,
                              zone.reach * options_.risk_vertical_ratio});
}


void PlanningEnvironmentStore::refreshEffectiveRiskZones()
{
    ++zone_policy_generation_;  // [S13] any zone/terrain re-derivation
                                // invalidates every outstanding snapshot
    risk_zones_ = risk_zones_raw_;
    if (!options_.risk_zone_agl || risk_zones_.empty()) return;
    if (!terrain_data_.valid) {
        // No DEM yet: raw z is used as-is; setTerrainData() re-derives.
        if (log_manager_) {
            log_manager_->infof(
                "[RISK-ZONE] AGL grounding deferred until DEM arrives "
                "(%zu zones)", risk_zones_.size());
        }
        return;
    }
    for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
        auto &tz = risk_zones_[zi];
        const double z_agl = risk_zones_raw_[zi].center.z();
        // Same water/off-DEM rule as groundedCenter: sea level is 0.
        double base = 0.0;
        const float h = terrain_data_.getElevation(tz.center.x(),
                                                   tz.center.y());
        if (std::isfinite(h) && h > 0.0f) base = static_cast<double>(h);
        tz.center.z() = base + z_agl;
        if (log_manager_) {
            log_manager_->infof(
                "[RISK-ZONE] zone[%zu] grounded: DEM h=%.2f + source height "
                "%.2f -> source z=%.2f", zi, base, z_agl, tz.center.z());
            if (z_agl > 2.0) {
                log_manager_->warnf(
                    "[RISK-ZONE] zone[%zu] source height %.1f units = %.0f m "
                    "AGL — looks like an absolute altitude or a metres "
                    "value (frame is 1 unit = 100 m)", zi, z_agl,
                    100.0 * z_agl);
            }
        }
    }
}

double PlanningEnvironmentStore::ellipsoidRadius(const RiskZone &zone,
                                        const Eigen::Vector3d &pos) const
{
    if (!(zone.reach > 0.0) || !(options_.risk_vertical_ratio > 0.0))
        return std::numeric_limits<double>::infinity();
    // CONTRACT: rv (vertical semi-axis) MUST match the value the optimizer and
    // heatmap use — oz.vertical_reach = zone.reach * options_.risk_vertical_ratio. All
    // three recompute it independently today. If a per-zone vertical override
    // is ever added, thread zone.vertical_reach through HERE and the heatmap so
    // the rendered/penalized moat geometry cannot silently diverge.
    const double rv = zone.reach * options_.risk_vertical_ratio;
    const Eigen::Vector3d d = pos - zone.center;
    return std::sqrt(d.head<2>().squaredNorm() /
                         (zone.reach * zone.reach) +
                     d.z() * d.z() / (rv * rv));
}

void PlanningEnvironmentStore::rebuildTerrainRiskMasks()
{
    terrain_risk_masks_.assign(risk_zones_.size(), terrain_risk::Mask{});

    if (!options_.risk_terrain_mask_enable || !terrain_data_.valid ||
        risk_zones_.empty()) {
        if (log_manager_ && !risk_zones_.empty()) {
            log_manager_->infof(
                "[RISK-MASK] terrain masking %s; publishing ideal fallback (%zu zones)",
                options_.risk_terrain_mask_enable ? "waiting for DEM" : "disabled",
                risk_zones_.size());
        }
        return;
    }

    size_t total_samples = 0;
    const auto settings = riskSettings(0.0);
    for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
        const auto &zone = risk_zones_[zi];
        if (!(zone.reach > 0.0)) continue;

        terrain_risk::Mask &mask = terrain_risk_masks_[zi];
        auto built = terrain_risk::buildMask(
            zone, terrain_data_, settings, std::move(mask));
        mask = std::move(built.mask);
        if (built.source_at_or_below_ground && log_manager_) {
            log_manager_->warnf(
                "[RISK-MASK] zone[%zu] source z=%.2f is at/below DEM %.2f; "
                "center.z is used literally as the source height",
                zi, zone.center.z(), static_cast<double>(built.source_ground));
        }
        total_samples += mask.shadow_ceiling.size();
        if (log_manager_) {
            log_manager_->infof(
                "[RISK-MASK] zone[%zu] horizon ready: %d azimuth x %d range "
                "(dr=%.3f, R=%.1f)",
                zi, mask.angular_count, mask.radial_count,
                mask.radial_step, zone.reach);
        }
    }
    if (log_manager_) {
        log_manager_->infof(
            "[RISK-MASK] built %zu terrain-horizon samples for %zu zones "
            "(soft edge %.3f z-units)",
            total_samples, risk_zones_.size(), options_.risk_mask_softness);
    }
}

Eigen::Vector3d PlanningEnvironmentStore::groundedCenter(const Eigen::Vector3d& center,
                                            double half_height) const
{
    if (!options_.obstacle_ground_snap) return center;
    double base = 0.0;  // sea level
    if (terrain_data_.valid) {
        const float h = terrain_data_.getElevation(center.x(), center.y());
        if (std::isfinite(h) && h > 0.0f) base = static_cast<double>(h);
    }
    Eigen::Vector3d out = center;
    out.z() = base + half_height;
    return out;
}

uint64_t PlanningEnvironmentStore::riskZoneFingerprint() const
{
    // Order-sensitive on purpose: a reordered set is a different set to the
    // searcher, which indexes policy by position. Bit patterns rather than
    // rounded values, so a change too small to print is still a change.
    uint64_t h = 1469598103934665603ull;   // FNV-1a
    const auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
    const auto mixd = [&mix](double d) {
        uint64_t bits;
        static_assert(sizeof(bits) == sizeof(d), "double must be 64-bit");
        std::memcpy(&bits, &d, sizeof(bits));
        mix(bits);
    };
    mix(risk_zones_.size());
    for (const auto &z : risk_zones_) {
        mixd(z.center.x()); mixd(z.center.y()); mixd(z.center.z());
        mixd(z.reach); mixd(z.peak);
    }
    return h;
}

bool PlanningEnvironmentStore::buildSDFForBounds(const Eigen::Vector3d &lo,
                                    const Eigen::Vector3d &hi)
{
    const double res = sdf_voxel_size_;
    const double res_z = options_.sdf_voxel_z;
    Eigen::Vector3d ext = hi - lo;
    if ((ext.array() <= 0.0).any()) {
        log_manager_->warnf("buildSDFForBounds: invalid bounds");
        return false;
    }
    int nx = std::max(8, (int)std::ceil(ext.x() / res));
    int ny = std::max(8, (int)std::ceil(ext.y() / res));
    int nz = std::max(8, (int)std::ceil(ext.z() / res_z));

    // NOTE: no occupancy grid is materialized. Terrain and static geometry
    // are never voxelised here (see below), so the grid was PROVABLY all-zero
    // — yet vector-init + the emptiness scan committed nx*ny*nz bytes
    // (~5.8 GB on a 30 m corridor) just for SDFManager to notice it was
    // empty and discard it. buildEmpty() takes that exact path directly.

    // Terrain is NOT voxelised into the SDF any more. It is handled as a 2.5D
    // heightmap (terrain_data_.getElevation, EXACT z) directly by the front end
    // (checkOccupancy_esdf) and the optimizer (heightmap terrain term). Baking
    // the DEM into 3D voxels quantised terrain z to sdf_voxel_z (~10 m) so it
    // under-saw hills, AND double-counted terrain against the heightmap term
    // (SDF-terrain + heightmap both firing -> line-search blowup on tall maps).
    // So the static SDF now holds ONLY geometry obstacles (added as patches
    // after the build): small, box-only, and terrain-precise everywhere.
    if (!terrain_data_.valid) {
        log_manager_->infof("SDF: terrain_data_ INVALID (heightmap terrain unavailable)");
    }

    // NOTE: ground and virtual ceiling are NOT voxelised here. Folding them
    // into the SDF would drag the clearance band above/below the actual
    // plane, so the trajectory would be pushed off a `-0.1` floor by up to
    // `obstacle_clearance` metres. Instead they are enforced as hard
    // half-space constraints inside the optimizer (sdfGradCostP), which
    // applies a unit upward/downward gradient only when the query point
    // crosses the plane — no clearance band, no lateral contamination.

    const Eigen::Vector3d current_voxel = sdf_manager_.voxelSizes();
    if (!sdf_manager_.isInitialized() ||
        std::abs(current_voxel.x() - res) > 1e-12 ||
        std::abs(current_voxel.y() - res) > 1e-12 ||
        std::abs(current_voxel.z() - res_z) > 1e-12) {
        if (!sdf_manager_.initialize(res, res_z)) {
            log_manager_->warnf(
                "buildSDFForBounds: SDF initialization failed");
            return false;
        }
    }
    bool ok = sdf_manager_.buildEmpty(nx, ny, nz, lo);
    if (ok) {
        log_manager_->infof("SDF built: shape=(%d,%d,%d) voxel=(%.2f,%.2f,%.2f) blocks=%zu",
            nx, ny, nz, res, res, res_z, sdf_manager_.numAllocatedBlocks());
    }
    return ok;
}

bool PlanningEnvironmentStore::terrainBounds(Eigen::Vector3d* lo, Eigen::Vector3d* hi)
{
    if (!terrain_data_.valid) return false;
    if (terrain_bbox_computed_) {
        *lo = terrain_bbox_lo_;
        *hi = terrain_bbox_hi_;
        return true;
    }

    // XY: walk the 4 corners of the terrain grid through terrainToWorld to
    //     get the correct world-frame bounds after the X-mirror / rotation.
    const int cols = terrain_data_.cols;
    const int rows = terrain_data_.rows;
    if (cols <= 0 || rows <= 0) return false;

    std::vector<Eigen::Vector3d> corners = {
        terrain_data_.terrainToWorld(0,        0,        0.0f),
        terrain_data_.terrainToWorld(cols - 1, 0,        0.0f),
        terrain_data_.terrainToWorld(0,        rows - 1, 0.0f),
        terrain_data_.terrainToWorld(cols - 1, rows - 1, 0.0f),
    };
    double min_x = corners[0].x(), max_x = corners[0].x();
    double min_y = corners[0].y(), max_y = corners[0].y();
    for (const auto& c : corners) {
        min_x = std::min(min_x, c.x()); max_x = std::max(max_x, c.x());
        min_y = std::min(min_y, c.y()); max_y = std::max(max_y, c.y());
    }
    // terrainToWorld returns CELL CENTRES, so the corner walk gives a bbox
    // inset by half a cell on every side vs the true published extent
    // [origin, origin+length]. Expand by half a cell so the SDF/obstacle
    // bounds cover the full map footprint — a box or risk zone in the outer
    // half-cell ring was otherwise clipped out of the built grid.
    const double half = 0.5 * terrain_data_.resolution;
    min_x -= half; max_x += half;
    min_y -= half; max_y += half;

    // Z: scan all elevation samples for the actual peak, add flight headroom.
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (float e : terrain_data_.elevation) {
        if (!std::isfinite(e)) continue;
        min_z = std::min(min_z, (double)e);
        max_z = std::max(max_z, (double)e);
    }
    if (!std::isfinite(min_z) || !std::isfinite(max_z)) {
        min_z = 0.0; max_z = 0.0;
    }
    const double z_headroom = 30.0;  // vertical margin above peak for traj room

    terrain_bbox_lo_ = Eigen::Vector3d(min_x, min_y, std::min(0.0, min_z));
    terrain_bbox_hi_ = Eigen::Vector3d(max_x, max_y, max_z + z_headroom);
    terrain_bbox_computed_ = true;
    *lo = terrain_bbox_lo_;
    *hi = terrain_bbox_hi_;
    return true;
}

double PlanningEnvironmentStore::shadowCeiling(size_t zone, const Eigen::Vector3d &pos) const
{
    return terrain_risk::shadowCeiling(riskMask(zone), options_.risk_terrain_mask_enable, pos);
}
double PlanningEnvironmentStore::groundedZ(const Eigen::Vector3d &pos,
                                            double clearance, double *derivative) const
{
    return terrain_risk::groundedZ(terrain_data_, riskSettings(clearance), pos, derivative);
}
double PlanningEnvironmentStore::visibilityValue(size_t zone, const Eigen::Vector3d &pos,
                                                   double clearance) const
{
    return terrain_risk::visibilityValue(riskMask(zone), terrain_data_, riskSettings(clearance), pos);
}
double PlanningEnvironmentStore::visibility(size_t zone, const Eigen::Vector3d &pos,
                                              double clearance, Eigen::Vector3d *gradient) const
{
    return terrain_risk::visibility(riskMask(zone), terrain_data_, riskSettings(clearance), pos, gradient);
}
double PlanningEnvironmentStore::riskValue(size_t zone_index, const Eigen::Vector3d &pos,
                                            double clearance) const
{
    if (zone_index >= risk_zones_.size()) return 0.0;
    const auto &zone = risk_zones_[zone_index];
    const double q = ellipsoidRadius(zone, pos);
    if (!(q < 1.0)) return 0.0;
    return std::min(zone.peak * mmp::risk::shape(q) *
                        visibilityValue(zone_index, pos, clearance),
                    mmp::risk::kMoatCap);
}


TerrainUpdate PlanningEnvironmentStore::ingestTerrain(const grid_map_msgs::msg::GridMap::SharedPtr &msg) {
    TerrainUpdate update;
    if (!msg) {
        log_manager_->warnf("Received null terrain GridMap");
        return update;
    }

    // world=none is published as an empty GridMap. It is an explicit state
    // transition, not a malformed update: keeping the previous DEM here while
    // RViz clears it would make the displayed terrain/risk field disagree with
    // the one used for planning.
    if (msg->layers.empty() && msg->data.empty()) {
        update.had_terrain = terrain_data_.valid;
        const uint64_t next_generation = terrain_data_.generation + 1;
        terrain_data_ = TerrainData{};
        terrain_data_.generation = next_generation;
        terrain_bbox_computed_ = false;
        sdf_built_ = false;
        // Every obstacle patch was tied to the old map/grid (including
        // terrain-grounded z and infinite-column bounds), so it cannot safely
        // survive an explicit world clear.
        if (sdf_manager_.numActiveObstacles() > 0 ||
            !obstacles_.empty() || !pending_obstacles_.empty()) {
            clearObstacles();
            update.obstacles_cleared = true;
        }
        // Invalidate the old grid itself as well as PathManager's latch.
        // Otherwise hasData() stays true and an obstacle added while no world
        // is loaded is immediately grounded at sea level, then survives the
        // next DEM load instead of being deferred and re-grounded.
        if (sdf_manager_.hasData() &&
            !sdf_manager_.initialize(sdf_voxel_size_, options_.sdf_voxel_z)) {
            log_manager_->warnf(
                "[TERRAIN] failed to invalidate SDF grid after world clear");
        }

        update.kind = TerrainUpdateKind::CLEARED;
        return update;
    }

    // A non-empty but malformed message is not an instruction to discard a
    // valid map. Reject it and retain the last accepted DEM on both planner
    // and panel sides.
    if (!(msg->info.resolution > 0.0) || msg->data.empty()) {
        log_manager_->warnf("Invalid non-empty terrain GridMap");
        return update;
    }
    if (sdf_voxel_size_auto_ && msg->info.resolution > 1e-6) {
        sdf_voxel_size_ = msg->info.resolution;
    } else if (sdf_voxel_size_ <= 0.0 && msg->info.resolution > 1e-6) {
        sdf_voxel_size_ = msg->info.resolution;
        sdf_voxel_size_auto_ = true;    // keep tracking the DEM cell on re-crop
        log_manager_->infof("[SDF] xy voxel auto-set to DEM cell: %.3f units",
                            sdf_voxel_size_);
    }

    // Find elevation layer
    int elev_idx = -1;
    for (size_t i = 0; i < msg->layers.size(); ++i) {
        if (msg->layers[i] == "elevation") {
            elev_idx = static_cast<int>(i);
            break;
        }
    }
    if (elev_idx < 0) {
        log_manager_->warnf("Elevation layer not found in terrain GridMap");
        return update;
    }

    if (static_cast<size_t>(elev_idx) >= msg->data.size()) {
        log_manager_->warnf("Terrain GridMap elevation payload is missing");
        return update;
    }
    const auto& elev_data = msg->data[elev_idx];
    if (elev_data.layout.dim.size() < 2) {
        log_manager_->warnf("Invalid terrain GridMap data layout");
        return update;
    }
    const size_t cols = elev_data.layout.dim[0].size;
    const size_t rows = elev_data.layout.dim[1].size;
    if (cols == 0 || rows == 0 ||
        cols > std::numeric_limits<size_t>::max() / rows ||
        elev_data.data.size() < cols * rows) {
        log_manager_->warnf("Invalid terrain GridMap dimensions/data size");
        return update;
    }

    // Geometry change (new world / corridor re-crop) invalidates everything
    // derived from the OLD map: the boxes-only SDF grid and the cached
    // terrain bbox were one-shot latches sized to the first map ever seen,
    // so a corridor loaded after a nationwide map (or vice versa) kept
    // planning against stale bounds. Reset the latches; the blocks below
    // rebuild them from this message.
    {
        const double new_origin_x = msg->info.pose.position.x - msg->info.length_x / 2.0;
        const double new_origin_y = msg->info.pose.position.y - msg->info.length_y / 2.0;
        const bool geometry_changed = terrain_data_.valid &&
            (std::abs(terrain_data_.resolution - msg->info.resolution) > 1e-9 ||
             std::abs(terrain_data_.length_x - msg->info.length_x) > 1e-6 ||
             std::abs(terrain_data_.length_y - msg->info.length_y) > 1e-6 ||
             std::abs(terrain_data_.origin_x - new_origin_x) > 1e-6 ||
             std::abs(terrain_data_.origin_y - new_origin_y) > 1e-6);
        if (geometry_changed) {
            log_manager_->warnf(
                "[TERRAIN] map geometry changed (res %.4f->%.4f, origin (%.1f,%.1f)->(%.1f,%.1f)) "
                "— rebuilding SDF grid and terrain bbox",
                terrain_data_.resolution, msg->info.resolution,
                terrain_data_.origin_x, terrain_data_.origin_y,
                new_origin_x, new_origin_y);
            sdf_built_ = false;
            terrain_bbox_computed_ = false;
            if (sdf_voxel_size_auto_) {
                sdf_voxel_size_ = msg->info.resolution;   // re-track DEM cell
            }
            // Clear ALL patches: every patch was placed on the OLD grid/DEM —
            // they baked groundedCenter() on the old ground, so they cannot
            // survive a geometry change.
            if (sdf_manager_.numActiveObstacles() > 0 ||
                !obstacles_.empty()) {
                clearObstacles();
                update.obstacles_cleared = true;
            }
        }
    }

    terrain_data_.cols = elev_data.layout.dim[0].size;
    terrain_data_.rows = elev_data.layout.dim[1].size;
    terrain_data_.resolution = msg->info.resolution;
    terrain_data_.length_x = msg->info.length_x;
    terrain_data_.length_y = msg->info.length_y;
    terrain_data_.origin_x = msg->info.pose.position.x - msg->info.length_x / 2.0;
    terrain_data_.origin_y = msg->info.pose.position.y - msg->info.length_y / 2.0;
    terrain_data_.elevation = elev_data.data;
    terrain_data_.valid = true;
    // Invalidate the ELEV-MEMO: a same-geometry re-crop reuses the buffer, so
    // the memo (keyed on generation) must see a new value or it serves stale
    // samples from the previous crop.
    ++terrain_data_.generation;

    // Return freed arenas to the OS after a map swap. Alternating small/large
    // corridor crops re-allocate every big grid (elevation, FM2 fields) at a
    // different size each epoch; glibc keeps the fragmented arenas and
    // manager RSS ratcheted ~+100 MB per revisit (measured; same-size
    // re-crops stay flat). One trim per terrain message is microseconds.
    malloc_trim(0);

    log_manager_->infof("Terrain data loaded: %dx%d, resolution=%.3f, origin=(%.2f,%.2f), center=(%.2f,%.2f)",
        terrain_data_.cols, terrain_data_.rows, terrain_data_.resolution,
        terrain_data_.origin_x, terrain_data_.origin_y,
        msg->info.pose.position.x, msg->info.pose.position.y);

    // ALIGNMENT SELF-CHECK: terrainToWorld uses the cell-CENTRE convention,
    // which provably matches the grid_map_rviz_plugin mesh (wx = L-(row+.5)res).
    // Sampling the peak cell's world position back through getElevation must
    // therefore return the peak value exactly (bilinear at an exact node).
    // Before the half-cell fix this read the average of the 4 shifted
    // neighbours instead (delta up to the full cell-to-cell relief, ~114 m
    // horizontal displacement) — a nonzero delta here means the sampled
    // surface is offset from the rendered terrain.
    {
        int pc = -1, pr = -1;
        float peak = -std::numeric_limits<float>::infinity();
        for (int c = 0; c < terrain_data_.cols; ++c) {
            for (int r = 0; r < terrain_data_.rows; ++r) {
                const float e = terrain_data_.elevation[c * terrain_data_.rows + r];
                if (std::isfinite(e) && e > peak) { peak = e; pc = c; pr = r; }
            }
        }
        if (pc >= 0) {
            const Eigen::Vector3d w = terrain_data_.terrainToWorld(pc, pr, peak);
            const float back = terrain_data_.getElevation(w.x(), w.y());
            log_manager_->infof(
                "[TERRAIN-ALIGN] peak cell (c=%d,r=%d) h=%.4f -> world (%.2f,%.2f) "
                "-> getElevation=%.4f delta=%.4f %s",
                pc, pr, peak, w.x(), w.y(), back, back - peak,
                std::abs(back - peak) < 1e-3 ? "(ALIGNED)" : "(MISALIGNED!)");
        }
    }

    update.kind = TerrainUpdateKind::REPLACED;
    return update;
}

bool PlanningEnvironmentStore::ensureTerrainDistanceField()
{
    if (sdf_built_) return false;
    Eigen::Vector3d lo, hi;
    if (!terrainBounds(&lo, &hi) || !buildSDFForBounds(lo, hi)) return false;
    sdf_built_ = true;
    return true;
}

bool PlanningEnvironmentStore::ensureDistanceField(
    const Eigen::Vector3d &mission_lower, const Eigen::Vector3d &mission_upper)
{
    if (sdf_built_) return true;
    Eigen::Vector3d lo = mission_lower, hi = mission_upper;
    Eigen::Vector3d terrain_lo, terrain_hi;
    const bool full_world = terrain_data_.valid && terrainBounds(&terrain_lo, &terrain_hi);
    if (full_world) { lo = terrain_lo; hi = terrain_hi; }
    if (!buildSDFForBounds(lo, hi)) return false;
    if (full_world) sdf_built_ = true;
    return true;
}

std::vector<PendingEnvironmentObstacle> PlanningEnvironmentStore::takePendingObstacles()
{
    std::vector<PendingEnvironmentObstacle> pending;
    if (sdf_manager_.hasData()) pending.swap(pending_obstacles_);
    return pending;
}

void PlanningEnvironmentStore::clearObstacles()
{
    sdf_manager_.clearObstacles();
    obstacles_.clear();
    pending_obstacles_.clear();
    log_manager_->infof("Dynamic obstacles cleared");
}


int PlanningEnvironmentStore::addSphere(const Eigen::Vector3d& center, double radius,
                                  const std::string& model)
{
    if (!sdf_manager_.hasData()) {
        // Defer: the SDF (ESDF) is built lazily on the first plan. Without a
        // cache it is not ready yet, so queue and add once it exists.
        pending_obstacles_.push_back({false, center,
            Eigen::Vector3d(radius, radius, radius), radius, model});
        log_manager_->infof("addDynamicSphere: SDF not ready, deferred "
                            "(center=%.2f,%.2f,%.2f r=%.2f)",
                            center.x(), center.y(), center.z(), radius);
        return -2;  // deferred (not an error)
    }
    const Eigen::Vector3d gcenter = groundedCenter(center, radius);
    path_planner::sdf::PrimitiveSpec spec;
    spec.kind = path_planner::sdf::PrimitiveKind::kSphere;
    spec.center = gcenter;
    const double d = 2.0 * radius;
    spec.size = Eigen::Vector3d(d, d, d);

    int id = sdf_manager_.addObstacle(spec);
    if (id < 0) {
        log_manager_->warnf("addDynamicSphere: addObstacle failed "
                            "(center=%.2f,%.2f,%.2f r=%.2f)",
                            center.x(), center.y(), center.z(), radius);
        return -1;
    }
    const double yaw =
        std::uniform_real_distribution<double>(0.0, 6.283185307179586)(yaw_rng_);
    obstacles_.push_back({id, gcenter, Eigen::Vector3d(d, d, d), false, model, yaw});
    log_manager_->infof("Dynamic sphere added: id=%d center=(%.2f,%.2f,%.2f) r=%.2f "
                        "(request z=%.2f), total=%zu",
                        id, gcenter.x(), gcenter.y(), gcenter.z(), radius,
                        center.z(), sdf_manager_.numActiveObstacles());
    return id;
}

int PlanningEnvironmentStore::addBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                               const std::string& model)
{
    if (!sdf_manager_.hasData()) {
        // Defer until the SDF is built (see addDynamicSphere).
        pending_obstacles_.push_back({true, center, size, 0.0, model});
        log_manager_->infof("addDynamicBox: SDF not ready, deferred "
                            "(center=%.2f,%.2f,%.2f)",
                            center.x(), center.y(), center.z());
        return -2;  // deferred (not an error)
    }
    // One heading per spawn, shared by BOTH the collision primitive and the
    // RViz model marker. It used to be visual-only (collision stayed an
    // axis-aligned box), so a rotated hull stuck out of its own collision
    // volume and trajectories legally clipped the bow/stern.
    const double yaw =
        std::uniform_real_distribution<double>(0.0, 6.283185307179586)(yaw_rng_);
    const Eigen::Vector3d gcenter = groundedCenter(center, 0.5 * size.z());
    path_planner::sdf::PrimitiveSpec spec;
    spec.kind = path_planner::sdf::PrimitiveKind::kCube;
    spec.center = gcenter;
    spec.size = size;  // full extents (sx, sy, sz)
    spec.yaw = yaw;

    int id = sdf_manager_.addObstacle(spec);
    if (id < 0) {
        log_manager_->warnf("addDynamicBox: addObstacle failed "
                            "(center=%.2f,%.2f,%.2f size=%.2f,%.2f,%.2f)",
                            gcenter.x(), gcenter.y(), gcenter.z(),
                            size.x(), size.y(), size.z());
        return -1;
    }
    obstacles_.push_back({id, gcenter, size, true, model, yaw});
    log_manager_->infof("Dynamic box added: id=%d center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) "
                        "yaw=%.2f (request z=%.2f), total=%zu",
                        id, gcenter.x(), gcenter.y(), gcenter.z(),
                        size.x(), size.y(), size.z(), yaw,
                        center.z(), sdf_manager_.numActiveObstacles());
    return id;
}

bool PlanningEnvironmentStore::setRiskZones(const std::vector<RiskZone>& zones)
{
    // [PA-7] ATOMIC REJECTION: refuse the SET, not the offending member.
    // Keeping the survivors would delete a declared hazard from the field and
    // the planner would route through it with nothing on record. Refusing
    // leaves the previous known-good set in force and says why.
    size_t bad = 0;
    for (size_t i = 0; i < zones.size(); ++i) {
        const auto &z = zones[i];
        const bool geom_ok = std::isfinite(z.center.x()) &&
                             std::isfinite(z.center.y()) &&
                             std::isfinite(z.center.z()) &&
                             std::isfinite(z.reach) && z.reach > 0.0;
        if (!geom_ok || !mmp::risk::isValidPeak(z.peak)) {
            ++bad;
            if (log_manager_) {
                log_manager_->errorf(
                    "setRiskZonesRuntime: zone #%zu REJECTED "
                    "center=(%.3f,%.3f,%.3f) reach=%.3f peak=%.3f — peak must "
                    "be finite and in (0, 1], reach finite and > 0",
                    i, z.center.x(), z.center.y(), z.center.z(), z.reach,
                    z.peak);
            }
        }
    }
    if (bad > 0) {
        if (log_manager_) {
            log_manager_->errorf(
                "setRiskZonesRuntime: REJECTING the whole update — %zu of %zu "
                "zones invalid. Active set unchanged at %zu.",
                bad, zones.size(), risk_zones_.size());
        }
        return false;
    }

    // The adapter rebinds search/optimizer consumers after this replacement.
    risk_zones_raw_ = zones;
    refreshRisk();
    if (log_manager_) {
        log_manager_->infof(
            "[risk_zones] runtime update: %zu zones now active",
            risk_zones_.size());
        for (size_t i = 0; i < risk_zones_.size(); ++i) {
            const auto& tz = risk_zones_[i];
            log_manager_->infof(
                "  zone[%zu] center=(%.1f,%.1f,%.1f) reach=%.1f peak=%.2f",
                i, tz.center.x(), tz.center.y(), tz.center.z(),
                tz.reach, tz.peak);
        }
    }
    return true;
}

}  // namespace path_manager
