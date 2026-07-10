#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <cmath>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>
#include "path_planner/sdf/distance_field.h"
#include "../../common/log_manager.hpp"
#include <queue>
#include <vector>
#include <algorithm>
#include <limits>

constexpr double inf = 1e20;

namespace path_planner { namespace search {

struct RiskZoneLite {
    Eigen::Vector3d center; // frame units (1 unit = 100 m)
    double reach;   // FRAME UNITS (not metres!); vertical-cylinder radius,
                    // risk exactly zero outside (typical scenario ~90 = 9 km)
    double peak;    // dimensionless in (0, 1]; >1 saturates to a flat disc
};

struct GridNode
{
    enum enum_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    int rounds{0};
    int state{UNDEFINED};
    double gScore{inf};
    // SMHA* keeps two f-scores per node: anchor (admissible) and
    // inadmissible (risk-inflated). g is shared.
    double fAnchor{inf};
    double fInadmis{inf};
    int cameFromFlat{-1};
};

class PathSearcher;

class NodeComparatorAnchor
{
public:
    NodeComparatorAnchor() = default;
    explicit NodeComparatorAnchor(const std::vector<GridNode> *pool) : pool_(pool) {}
    bool operator()(int a, int b) const
    {
        return (*pool_)[a].fAnchor > (*pool_)[b].fAnchor;
    }
private:
    const std::vector<GridNode> *pool_ = nullptr;
};

class NodeComparatorInadmis
{
public:
    NodeComparatorInadmis() = default;
    explicit NodeComparatorInadmis(const std::vector<GridNode> *pool) : pool_(pool) {}
    bool operator()(int a, int b) const
    {
        return (*pool_)[a].fInadmis > (*pool_)[b].fInadmis;
    }
private:
    const std::vector<GridNode> *pool_ = nullptr;
};

class PathSearcher
{
public:
    // Front-end search selector (yaml manager/front_end).
    enum class FrontEnd { ASTAR, FM2 };

private:
    // SDF query backend. We do not need a separate occupancy map: a voxel is
    // considered blocked when sdf_distance < obstacle_margin_.
    const path_planner::sdf::IDistanceField *sdf_ = nullptr;
    // 2.5D terrain heightmap (frame units; -inf over water/invalid). Authoritative
    // terrain-collision source for the front end (see checkOccupancy_esdf) — the
    // SDF's voxel terrain is coarse/z-quantised.
    std::function<float(double, double)> terrain_height_;
    const std::vector<RiskZoneLite> *risk_zones_ = nullptr;
    double obstacle_margin_ = 0.5;  // frame units (1 unit = 100 m)
    double dyn_obstacle_margin_ = 0.0;  // dynamic-obstacle berth, frame units; 0 = off
    // When true, the A* graph expansion ignores obstacles (every voxel is
    // traversable); shortcut / downstream checks still use obstacle_margin_.
    bool search_ignores_obstacles_ = false;
    // Hard ground / ceiling for A* expansion. Cells at or below
    // ground_height_ (and at or above virtual_ceil_height_) are rejected
    // just like SDF-occupied voxels. Sentinel: ≤ -0.5 disables the plane.
    double ground_height_ = -1.0;
    double virtual_ceil_height_ = -1.0;
    double risk_alpha_ = 1.0;
    double risk_barrier_ = 0.0;   // finite "hard wall" K added inside non-exempt zones
    std::vector<char> zone_no_barrier_;  // per-zone: 1 = exempt (contains start/goal)
    // SMHA* (Aine et al., IJRR 2016) shared-g, dual-heuristic A*, run
    // unconditionally for every query — NO binary mode switch.
    //
    // - ANCHOR queue (admissible): h = euclidean (Diag) tie-broken.
    //   Bounds suboptimality (SMHA* 2-expand theorem) so detour quality
    //   is preserved.
    // - INADMIS queue: h = coarse risk-aware cost-to-go (see below).
    //   Dispatch: pop inadmis while INADMIS.top.f <= smha_w_ * ANCHOR.top.f.
    //
    // The inadmissible heuristic is a COARSE-GRID risk-aware value-to-go,
    // computed by Dijkstra from the goal on a K-times-downsampled grid
    // using the SAME edge cost  dist*(1 + alpha*risk).  Per Wilt & Ruml
    // (SoCS 2012, "When does Weighted A* Fail?"): greedy/weighted search
    // is fast iff the heuristic is correlated with true cost-to-go. A
    // reweighted Euclidean heuristic is *anti*-correlated inside a risk
    // depression (goal gets closer geometrically but more expensive),
    // which is exactly why every distance-based inadmissible heuristic we
    // tried stalled on goal-in-zone or bulldozed mid-path detours. A
    // coarse cost-to-go encodes the depression structure exactly, so the
    // SAME single heuristic:
    //   * routes around a mid-path zone (coarse value says detour is
    //     cheaper),
    //   * cuts straight through when the goal is inside a zone (coarse
    //     value says transit is the cheapest available),
    //   * never bulldozes a detour that is actually cheaper.
    // (Holte hierarchical A*, Felner additive PDB: an abstract search's
    // exact cost-to-go is a valid heuristic for the fine search.)
    double smha_w_ = 1.0;

    // Coarse value field for the inadmissible heuristic.
    int coarse_k_ = 8;                  // downsample factor (fine->coarse)
    int cnx_ = 0, cny_ = 0, cnz_ = 0;   // coarse grid dims
    std::vector<double> coarse_g_;      // cost-to-go from goal; inf if unreachable
    bool coarse_valid_ = false;
    Eigen::Vector3i coarse_goal_idx_{-1, -1, -1};

    inline int coarseFlat(int ci, int cj, int ck) const {
        return ci + cnx_ * (cj + cny_ * ck);
    }
    // World position -> coarse cost-to-go (inf-safe). Returns -1 if the
    // coarse field is not valid so the caller can fall back to euclidean.
    double coarseCostToGo(const Eigen::Vector3d &world) const;
    // (Re)compute the coarse Dijkstra value field rooted at `goal`.
    // Cheap (coarse grid has ~ fine/K^3 cells); called once per query.
    void buildCoarseValueField(const Eigen::Vector3d &goal_world);

    // ----- FM2 (Fast Marching Square) front-end -----
    // Heuristic-free Eikonal planner. Solves |∇T|·F = 1 from the goal on
    // a coarse risk-weighted speed map, then extracts the geodesic by
    // gradient descent. No local minima (Valero-Gomez et al.) so it has
    // none of the Wilt&Ruml depression-explosion of the A* family.
    // (FrontEnd enum is public — see below.)
    FrontEnd front_end_ = FrontEnd::ASTAR;
    int   fm2_coarse_k_ = 4;            // Eikonal grid downsample factor
    size_t fm2_max_cells_ = 8'000'000; // FMM grid cell-count cap (OOM/hang guard)
    bool  fm2_star_ = true;             // FM2*: cost-to-go heuristic on
                                        // the FMM queue (same trajectory)
    int   fcnx_ = 0, fcny_ = 0, fcnz_ = 0;
    std::vector<float> fm2_T_;          // arrival time / cost-to-go
    std::vector<float> fm2_F_;          // speed map in (0, 1]
    bool  fm2_valid_ = false;
    // Altitude-band penalty: slow the wave outside the mission altitude band
    // so the geodesic holds altitude and leaves the band only where terrain
    // blocks it. Without it T(z) is nearly flat (free space has no altitude
    // cost) and the over-the-terrain wave is a hair faster than the
    // valley-weaving one, so gradient descent integrates that tiny bias into
    // a ~1000 m climb (and with only an upper penalty it dives to the floor).
    // alt_cost = w * dist_outside_band(z) / zscale, added to risk_cost.
    // Band = [min(start,goal)z - cell, max(start,goal)z + cell]. w=0 disables.
    // Asymmetric: the up-side stays gentle so the wall-climb gradient at a
    // terrain ridge still wins (a steep up-side cancels it and the descent
    // deadlocks at the wall foot); the down-side is stiff since nothing ever
    // requires diving below the mission altitude.
    double fm2_alt_w_ = 2.0;            // penalty weight (per zscale outside)
    double fm2_alt_zscale_ = 10.0;      // up-side ramp (z-units per w)
    double fm2_alt_zscale_dn_ = 2.0;    // down-side ramp (stiff: no diving)
    double fm2_alt_zlo_ = -1.0;         // set per-search from start/goal z
    double fm2_alt_zhi_ = -1.0;

    inline int fm2Flat(int i, int j, int k) const {
        return i + fcnx_ * (j + fcny_ * k);
    }
    // Build speed map (ESDF + OR-moat risk) on the coarse grid.
    void fm2BuildSpeedMap();
    // Solve the Eikonal equation rooted at goal_world; fills fm2_T_.
    void fm2SolveEikonal(const Eigen::Vector3d &goal_world,
                         const Eigen::Vector3d &start_world);
    // Extract the geodesic start->goal by descending -∇T. World coords.
    std::vector<Eigen::Vector3d> fm2ExtractGeodesic(
        const Eigen::Vector3d &start_world,
        const Eigen::Vector3d &goal_world);
    // Trilinear sample of fm2_T_ at a world point; +inf if outside/blocked.
    double fm2SampleT(const Eigen::Vector3d &world) const;
    // Debug toggle: when true, astarSearchAndGetSimplePath returns the raw
    // 1-voxel-step A* path without shortcut/visibility-thinning. Used to
    // verify front-end behavior independent of the shortcut filter.
    bool bypass_shortcut_ = false;
    double map_resolution_ = 1.0;
    double map_resolution_z_ = 1.0;  // vertical grid spacing (anisotropic)
    Eigen::Vector3d map_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_size_ = Eigen::Vector3d::Zero();

    swarm_formation::LogManager::Ptr log_manager_;

    double getDiagHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);
    // Admissible anchor heuristic (pure Euclidean diag with tie breaker).
    inline double getHeuAnchor(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);
    // Inadmissible heuristic — same shape as anchor, scaled by (1 + alpha*risk(n))
    // evaluated at i1. Helps SMHA* escape depression regions.
    inline double getHeuInadmis(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);

    bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
    inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

    // Collision / risk queries backed by SDF.
    inline bool checkOccupancy_esdf(const Eigen::Vector3d &pos) {
        // 2.5D TERRAIN via the DEM heightmap (exact z). The SDF's voxelised
        // terrain is z-quantised (~10 m) and under-sees it, so the FM2 speed
        // map / A* / shortcut (all route through here) cut through hills the
        // heightmap catches. Authoritative terrain check; blocked within
        // obstacle_margin_ of the surface. -inf = water/invalid -> skip (SDF
        // below then handles box obstacles).
        if (terrain_height_) {
            const float h = terrain_height_(pos.x(), pos.y());
            if (std::isfinite(h) &&
                pos.z() - static_cast<double>(h) < obstacle_margin_) return true;
        }
        if (!sdf_ || !sdf_->hasData()) return false;
        float d = sdf_->getDistance(pos);
        if (!std::isfinite(d)) return true;  // outside map = blocked
        if (d < obstacle_margin_) return true;
        // Dynamic obstacles (cars/buildings spawned at runtime) get a more
        // generous berth than terrain: blocked out to dyn_obstacle_margin_.
        // Applies to FM2, A* and the shortcut consistently (they all come
        // through here). Capped in range by the SDF patch influence radius.
        if (dyn_obstacle_margin_ > obstacle_margin_ &&
            sdf_->getDynamicDistance(pos) < dyn_obstacle_margin_) return true;
        return false;
    }
    inline bool checkOccupancy(const Eigen::Vector3d &pos) {
        return checkOccupancy_esdf(pos);
    }

    // V3: quadratic moat + probabilistic-OR composition.
    // Per zone: moat_i(x) = peak_i * (1 - d/reach_i)^2 for d < reach_i.
    // Composed: risk(x) = 1 - prod_i (1 - moat_i(x)),  bounded in [0, 1].
    // Returns risk_alpha_ * risk(x). Compact support; AABB pre-filter
    // skips the sqrt for far zones.
    // Normalized OR-moat risk in [0, 1] (no alpha scaling). Used by the
    // inadmissible heuristic so its inflation factor stays dimensionless.
    inline double getRiskNorm(const Eigen::Vector3d &pos) const {
        if (!risk_zones_ || risk_zones_->empty()) return 0.0;
        double survival = 1.0;
        for (const auto &tz : *risk_zones_) {
            // Vertical-cylinder threat: moat decays with HORIZONTAL distance and
            // is z-flat within |dz| < reach (the ceiling), so climbing buys no
            // risk reduction; above the ceiling risk is 0.
            const double dz = pos.z() - tz.center.z();
            if (std::abs(dz) >= tz.reach) continue;
            const double dx = pos.x() - tz.center.x();
            if (std::abs(dx) >= tz.reach) continue;
            const double dy = pos.y() - tz.center.y();
            if (std::abs(dy) >= tz.reach) continue;
            const double d = std::sqrt(dx*dx + dy*dy);   // horizontal only
            if (d >= tz.reach) continue;
            const double u = 1.0 - d / tz.reach;
            const double moat = tz.peak * u * u;
            constexpr double kMoatCap = 1.0 - 1e-3;
            survival *= (1.0 - std::min(moat, kMoatCap));
        }
        return 1.0 - survival;
    }

    // Mark zones containing the start or goal as barrier-exempt (must enter).
    // Call once per search before evaluating risk costs.
    void prepareBarrier(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) {
        zone_no_barrier_.clear();
        if (!risk_zones_) return;
        zone_no_barrier_.assign(risk_zones_->size(), 0);
        // Same vertical-cylinder membership as getRiskNorm/insideBarrierZone.
        // The old 3D-sphere test disagreed with them: an endpoint inside the
        // cylinder but outside the sphere kept the barrier on a zone the
        // route MUST enter, walling off its own start/goal.
        auto in_zone = [](const Eigen::Vector3d &p, const RiskZoneLite &tz) {
            if (std::abs(p.z() - tz.center.z()) >= tz.reach) return false;
            const double dx = p.x() - tz.center.x();
            const double dy = p.y() - tz.center.y();
            return dx * dx + dy * dy < tz.reach * tz.reach;
        };
        for (size_t i = 0; i < risk_zones_->size(); ++i) {
            const auto &tz = (*risk_zones_)[i];
            if (in_zone(start, tz) || in_zone(goal, tz))
                zone_no_barrier_[i] = 1;
        }
    }

    // Inside a barrier-applied zone? Zones containing the start/goal are exempt
    // (zone_no_barrier_) — they must be entered, so they stay soft (moat only).
    inline bool insideBarrierZone(const Eigen::Vector3d &pos) const {
        if (!risk_zones_ || risk_barrier_ <= 0.0) return false;
        for (size_t i = 0; i < risk_zones_->size(); ++i) {
            if (i < zone_no_barrier_.size() && zone_no_barrier_[i]) continue;
            const auto &tz = (*risk_zones_)[i];
            // Vertical cylinder, consistent with getRiskNorm.
            const double dz = pos.z() - tz.center.z();
            if (std::abs(dz) >= tz.reach) continue;
            const double dx = pos.x() - tz.center.x();
            const double dy = pos.y() - tz.center.y();
            if (dx*dx + dy*dy < tz.reach * tz.reach) return true;
        }
        return false;
    }

    // Altitude-band cost shared by the FM2 speed map and the shortcut cost
    // metric (zlo/zhi set per-search from the mission endpoints).
    inline double altBandCost(double z) const {
        if (fm2_alt_w_ <= 0.0 || fm2_alt_zhi_ < 0.0) return 0.0;
        if (z > fm2_alt_zhi_)
            return fm2_alt_w_ * (z - fm2_alt_zhi_) / fm2_alt_zscale_;
        if (z < fm2_alt_zlo_)
            return fm2_alt_w_ * (fm2_alt_zlo_ - z) / fm2_alt_zscale_dn_;
        return 0.0;
    }

    inline double getRiskCost(const Eigen::Vector3d &pos) const {
        // alpha*moat (all zones) + finite barrier K on non-exempt zones. K is
        // large but finite, so the graph never disconnects: detour when a route
        // exists, else cross at the least-moat point.
        double cost = risk_alpha_ * getRiskNorm(pos);
        if (insideBarrierZone(pos)) cost += risk_barrier_;
        return cost;
    }

    std::vector<int> retrievePath(int current_flat);

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;
    Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
    const double tie_breaker_ = 1.0 + 1.0 / 10000;
    const int max_iterations_ = 50000;

    std::vector<int> gridPath_;

    std::vector<GridNode> pool_;
    int nx_{0}, ny_{0}, nz_{0};
    std::priority_queue<int, std::vector<int>, NodeComparatorAnchor> openSet_anchor_;
    std::priority_queue<int, std::vector<int>, NodeComparatorInadmis> openSet_inadmis_;

    // Flat 1D index helpers. Row-major: i fastest, k slowest.
    inline int flatIdx(int i, int j, int k) const {
        return i + nx_ * (j + ny_ * k);
    }
    inline int flatIdx(const Eigen::Vector3i &idx) const {
        return idx(0) + nx_ * (idx(1) + ny_ * idx(2));
    }
    inline Eigen::Vector3i flatToIdx(int flat) const {
        int k = flat / (nx_ * ny_);
        int r = flat - k * (nx_ * ny_);
        int j = r / nx_;
        int i = r - j * nx_;
        return Eigen::Vector3i(i, j, k);
    }
    int rounds_{0};

public:
    typedef std::shared_ptr<PathSearcher> Ptr;

    PathSearcher(){};
    ~PathSearcher();

    void setLogManager(swarm_formation::LogManager::Ptr log_manager) { log_manager_ = log_manager; }

    // resolution_z <= 0 keeps the legacy isotropic behaviour (z = xy). A
    // finer z lets the FM2 grid resolve altitude at real-terrain scale
    // instead of quantizing every climb to one xy-sized cell.
    void setSDF(const path_planner::sdf::IDistanceField *sdf,
                const Eigen::Vector3d &origin,
                const Eigen::Vector3d &size,
                double resolution,
                double resolution_z = -1.0) {
        sdf_ = sdf;
        map_origin_ = origin;
        map_size_ = size;
        map_resolution_ = resolution;
        map_resolution_z_ = (resolution_z > 0.0) ? resolution_z : resolution;
    }
    void setRiskZones(const std::vector<RiskZoneLite> *zones) { risk_zones_ = zones; }
    void setObstacleMargin(double m) { obstacle_margin_ = m; }
    // Extra berth around dynamic obstacles only (<= obstacle_margin_ disables).
    void setDynObstacleMargin(double m) { dyn_obstacle_margin_ = m; }
    void setSearchIgnoresObstacles(bool b) { search_ignores_obstacles_ = b; }
    void setGroundHeight(double h)      { ground_height_ = h; }
    void setTerrainHeightmap(std::function<float(double, double)> f) { terrain_height_ = std::move(f); }
    void setVirtualCeilHeight(double h) { virtual_ceil_height_ = h; }
    void setRiskAlpha(double a) { risk_alpha_ = a; }
    void setRiskBarrier(double k) { risk_barrier_ = (k > 0.0 ? k : 0.0); }
    void setSmhaW(double w) { smha_w_ = w; }
    void setFrontEnd(FrontEnd fe) { front_end_ = fe; }
    void setFm2CoarseK(int k) { fm2_coarse_k_ = (k >= 1 ? k : 1); }
    // Clamped to INT_MAX: all FM2 flat indices (fm2Flat, the FMM queue
    // payload, the i/j/k decode) are int, so a larger cap would let flat
    // indexing wrap negative (UB) instead of failing cleanly at the cap.
    void setFm2MaxCells(size_t n) {
        if (n > 0)
            fm2_max_cells_ = std::min<size_t>(
                n, static_cast<size_t>(std::numeric_limits<int>::max()));
    }
    void setFm2Star(bool on) { fm2_star_ = on; }
    void setFm2AltPenalty(double w, double zscale, double zscale_dn = 2.0) {
        fm2_alt_w_ = (w >= 0.0 ? w : 0.0);
        if (zscale > 1e-6) fm2_alt_zscale_ = zscale;
        if (zscale_dn > 1e-6) fm2_alt_zscale_dn_ = zscale_dn;
    }
    void setBypassShortcut(bool b) { bypass_shortcut_ = b; }

    // ----- Visualization dump accessors (read-only, post-search) -----
    const std::vector<float>&  getFm2T() const { return fm2_T_; }
    const std::vector<float>&  getFm2F() const { return fm2_F_; }
    const std::vector<double>& getCoarseG() const { return coarse_g_; }
    Eigen::Vector3i getFm2Dims() const { return {fcnx_, fcny_, fcnz_}; }
    Eigen::Vector3i getCoarseDims() const { return {cnx_, cny_, cnz_}; }
    int getFm2CoarseK() const { return fm2_coarse_k_; }
    int getCoarseK() const { return coarse_k_; }

    // Fine pool gScore (A* per-cell accumulated cost) z-slice dump.
    // Returns a flat (nx * ny) vector of gScore for the given z layer,
    // with INFs left as +inf. Used for visualization only.
    Eigen::Vector3i getPoolSize() const { return POOL_SIZE_; }
    std::vector<double> getFineGScoreSlice(int z) const {
        std::vector<double> out;
        if (z < 0 || z >= POOL_SIZE_(2)) return out;
        out.reserve(static_cast<size_t>(POOL_SIZE_(0)) * POOL_SIZE_(1));
        for (int x = 0; x < POOL_SIZE_(0); ++x)
            for (int y = 0; y < POOL_SIZE_(1); ++y) {
                // pool_ is x-fastest (see flatIdx); the old z-fastest formula
                // here read a scrambled slice.
                out.push_back(pool_[flatIdx(x, y, z)].gScore);
            }
        return out;
    }

    void initGridMap(const Eigen::Vector3i &pool_size);
    // Free the existing pool (if any) and allocate a new one. Use when the
    // map span changes between queries.
    void resizePool(const Eigen::Vector3i &pool_size);

    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check);

    std::vector<Eigen::Vector3d> getPath();

    std::vector<Eigen::Vector3d> astarSearchAndGetSimplePath(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, int drone_id);

    Eigen::Vector3d getOrigin() const { return map_origin_; }
    Eigen::Vector3d getMapSize() const { return map_size_; }

    // Lowest free altitude in the FM2 speed-field column containing world
    // (wx, wy): the bottom edge of the first non-blocked coarse z-layer.
    // Read straight from fm2_F_, so it captures EVERY hard blocker exactly
    // as the front-end saw it — terrain + obstacle_margin, box obstacles
    // (+ dyn margin), the ground plane — with no re-derivation. This is the
    // floor the geodesic could not go below in that column. NaN when the fm2
    // grid is empty/stale, xy is outside its span, or the whole column is
    // free from layer 0 (no blocker: nothing to explain).
    float fm2ColumnFloor(double wx, double wy) const {
        const size_t N = (size_t)fcnx_ * fcny_ * fcnz_;
        if (fcnx_ <= 0 || fcny_ <= 0 || fcnz_ <= 0 || fm2_F_.size() != N)
            return std::numeric_limits<float>::quiet_NaN();
        const double fine = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
        const double fine_z = map_resolution_z_ > 1e-6 ? map_resolution_z_ : fine;
        const double cres = fine * static_cast<double>(fm2_coarse_k_);
        const double cres_z = fine_z * static_cast<double>(fm2_coarse_k_);
        const int i = static_cast<int>(std::floor((wx - map_origin_.x()) / cres));
        const int j = static_cast<int>(std::floor((wy - map_origin_.y()) / cres));
        if (i < 0 || i >= fcnx_ || j < 0 || j >= fcny_)
            return std::numeric_limits<float>::quiet_NaN();
        // Blocked cells hold kFMin (1e-3); free cells 1/(1+risk) which stays
        // well above it except inside barrier zones — which ARE walls.
        for (int k = 0; k < fcnz_; ++k) {
            if (fm2_F_[fm2Flat(i, j, k)] > 2e-3f) {
                if (k == 0) return std::numeric_limits<float>::quiet_NaN();
                return static_cast<float>(map_origin_.z() + k * cres_z);
            }
        }
        return std::numeric_limits<float>::quiet_NaN();  // fully blocked column
    }

    // Max column floor within lateral radius `halfwidth` (world units) of
    // (wx, wy) — the corridor/swath view. A successful lateral dodge leaves
    // the causal column NEXT TO the path, never under it, so an underfoot
    // profile systematically hides exactly the constraints that shaped the
    // route; the swath max puts them back. Same exact fm2_F_ source.
    float fm2SwathFloor(double wx, double wy, double halfwidth) const {
        const size_t N = (size_t)fcnx_ * fcny_ * fcnz_;
        if (fcnx_ <= 0 || fcny_ <= 0 || fcnz_ <= 0 || fm2_F_.size() != N)
            return std::numeric_limits<float>::quiet_NaN();
        const double fine = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
        const double fine_z = map_resolution_z_ > 1e-6 ? map_resolution_z_ : fine;
        const double cres = fine * static_cast<double>(fm2_coarse_k_);
        const double cres_z = fine_z * static_cast<double>(fm2_coarse_k_);
        const int r = std::max(0, (int)std::floor(halfwidth / cres));
        const int ic = (int)std::floor((wx - map_origin_.x()) / cres);
        const int jc = (int)std::floor((wy - map_origin_.y()) / cres);
        const double r2 = (halfwidth / cres) * (halfwidth / cres);
        float best = std::numeric_limits<float>::quiet_NaN();
        for (int dj = -r; dj <= r; ++dj) {
            for (int di = -r; di <= r; ++di) {
                if ((double)(di * di + dj * dj) > r2) continue;
                const int i = ic + di, j = jc + dj;
                if (i < 0 || i >= fcnx_ || j < 0 || j >= fcny_) continue;
                for (int k = 0; k < fcnz_; ++k) {
                    if (fm2_F_[fm2Flat(i, j, k)] > 2e-3f) {
                        if (k > 0) {
                            const float f = (float)(map_origin_.z() + k * cres_z);
                            if (std::isnan(best) || f > best) best = f;
                        }
                        break;
                    }
                }
            }
        }
        return best;
    }
};

inline double PathSearcher::getHeuAnchor(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    return tie_breaker_ * getDiagHeu(i1, i2);
}

inline double PathSearcher::getHeuInadmis(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    // Coarse risk-aware cost-to-go (see buildCoarseValueField). This
    // encodes the depression structure exactly, so the same heuristic
    // detours around mid-path zones AND cuts through goal-in-zone.
    if (coarse_valid_) {
        const Eigen::Vector3d w = Index2Coord(i1);
        const double c = coarseCostToGo(w);
        if (c >= 0.0) return c;            // valid coarse value
    }
    // Fallback (coarse field unavailable / cell unreachable): inflated
    // euclidean so the inadmis queue still makes progress.
    return tie_breaker_ * smha_w_ * getDiagHeu(i1, i2);
}

// KNOWN LIMITATION (astar ablation/fallback path only): the A* grid is
// ISOTROPIC — step_size_ applies to z too, so with step_size 1.0 vertical
// moves quantize to ~100 m, far coarser than the 30-38 m terrain berths.
// The FM2 front end (the production path) uses per-axis resolutions and is
// unaffected. Fixing this means per-axis steps here, in Coord2Index and in
// the neighbour costs — deliberate scope, not an oversight.
inline Eigen::Vector3d PathSearcher::Index2Coord(const Eigen::Vector3i &index) const
{
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool PathSearcher::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
    // Use round() instead of "+0.5, cast<int>()" because the latter truncates
    // toward zero for negative values, which biases cells on the negative side
    // of center_ by one step and leaves path[0] up to ~1 step farther from
    // start_pt than the true sqrt(3)/2 quantization bound.
    Eigen::Vector3d rel = (pt - center_) * inv_step_size_;
    idx = Eigen::Vector3i(std::lround(rel(0)), std::lround(rel(1)), std::lround(rel(2))) + CENTER_IDX_;

    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
    {
        return false;
    }

    return true;
};

}} // namespace path_planner::astar

#endif
