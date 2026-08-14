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
    double reach;            // horizontal reach, frame units
    double peak;             // dimensionless peak
    double vertical_reach{0.0}; // z semi-axis; <=0 falls back to reach
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
    // Per-zone terrain visibility supplied by PathManager's precomputed
    // radial-horizon field. 1 = direct line of sight (full risk),
    // 0 = terrain-occluded
    // behind terrain. Keeping the callback per zone preserves the existing
    // probabilistic-OR composition and start/goal barrier exemptions.
    std::function<double(size_t, const Eigen::Vector3d &)> risk_visibility_;
    // [ROUGH] (H2 terrain-roughness routing) optional slope field for the
    // shared cost: bilinear h + dh/dxy from PathManager's TerrainData — the
    // same surface the optimizer's terrain terms use. Pure read with a
    // thread_local memo, so the OpenMP speed-map build may call it freely.
    std::function<bool(double, double, float *, float *, float *)> terrain_hgrad_;
    double rough_weight_ = 0.0;   // cost per slope unit above s0 (own currency,
                                  // NOT risk_alpha_-scaled); 0 = off
    double rough_slope0_ = 0.20;  // slope deadband (tan; ~11 deg): plains free
    double obstacle_margin_ = 0.5;  // frame units (1 unit = 100 m)
    // Terrain sampling pitch for chord feasibility scans: min(0.5, half a DEM
    // cell). See setTerrainHeightmap.
    double terrain_stride_floor_ = 0.5;
    double dyn_obstacle_margin_ = 0.0;  // dynamic-obstacle berth, frame units; 0 = off
    // Hard ground / ceiling for A* expansion. Cells at or below
    // ground_height_ (and at or above virtual_ceil_height_) are rejected
    // just like SDF-occupied voxels. Sentinel: ≤ -0.5 disables the plane.
    double ground_height_ = -1.0;
    double virtual_ceil_height_ = -1.0;
    double risk_alpha_ = 1.0;
    double risk_barrier_ = 0.0;   // finite "hard wall" K added inside non-exempt zones
    std::vector<char> zone_no_barrier_;  // per-zone: 1 = exempt (contains start/goal)
    // [GNRON] Goal-radius risk taper (Ge & Cui 2000 transplanted to the
    // eikonal field): a zone the mission endpoint sits INSIDE cannot be
    // avoided, yet its moat kept pricing the approach — the cost-optimal
    // entry then degenerates to hover-high + vertical plunge (the moat
    // ellipsoid is 0.35-flat, so crossing it vertically is cheapest), which
    // the fixed-wing back-end cannot fly. Fading that zone's moat to zero
    // within taper_radius of the endpoint restores the endpoint as the
    // field's minimum, so the geodesic descends on a flyable slant instead.
    // PLANNING cost only — display/metrics risk stays untapered.
    double risk_goal_taper_radius_ = 0.0;  // frame units; <=0 = off
    Eigen::Vector3d taper_start_{0.0, 0.0, 0.0};
    Eigen::Vector3d taper_goal_{0.0, 0.0, 0.0};
    std::vector<unsigned char> zone_taper_;  // bit0: start inside, bit1: goal inside
    // Max |dz|/|dxy| the geodesic extraction may follow (tan of the shared
    // model's flight-path-angle limit). <=0 = off. Applied only when a
    // horizontal direction exists — a pure-vertical gradient is left alone
    // so the descent can never stall (the taper removes that regime anyway).
    double geo_slope_tan_max_ = 0.0;
    // [ZONE-AVOID] Lexicographic zone policy (2026-07-24): avoidance is not
    // traded against detour length. Pass 1 DISCONNECTS (F=0) every
    // non-exempt zone's visible volume; only if the goal is then unreachable
    // does the soft finite-K field run, and pass 3 re-hardens the zones the
    // soft route did not need. zone_soft_override_[i]=1 keeps zone i soft in
    // hard mode (the pass-2 crossed set).
    bool zone_avoid_lexico_ = true;
    bool zone_hard_mode_ = false;
    // True after a search whose final field was a HARD pass (1 or 3): the
    // shortcut phase must then VETO chords touching the hard volume, or its
    // soft cost margins re-enter zones the wave was forbidden to cross.
    bool zone_lexico_hard_ = false;
    int zone_avoid_pass_ = 0;
    // Set by fm2ExtractGeodesic: did the descent actually reach the goal?
    // (Aborted descents append the goal anyway — the [ZONE-AVOID] passes
    // must not accept a truncated wall-chord as a "route".)
    bool fm2_geo_reached_goal_ = false;
    std::vector<char> zone_soft_override_;
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
    // [FM2-FINAL-CLEAR] the terrain sweep's outcome, read by the whole-path
    // gate so a refusal can say whether the repair ran out of budget.
    int dbg_lifted_ = 0;
    int dbg_lift_cap_ = 0;
    bool dbg_sweep_clean_ = false;
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
 public:
    // The ROUTE-VALIDATION contract, public so it can be regressed as the
    // pure predicate it is. The searcher's own fixtures can only reach these
    // through a full FM2 plan, which pins the call site but not the rule --
    // a mutation that deleted the obstacle half of polylineClear passed the
    // whole variant sweep. Everything below is a query: no state changes.
    // How far the route may stay inside the terrain margin at its start
    // before that becomes a refusal. Expressed in the margin itself rather
    // than as a new tuning knob: the aircraft has to clear obstacle_margin_
    // of ground, and it is given a few multiples of that distance to do it.
    // Small on purpose — this is a takeoff allowance, not a corridor.
    double startTerrainReliefArc() const { return 5.0 * obstacle_margin_; }

    // Obstacles ONLY. This is what a RAW FM2 geodesic may be judged on: it
    // is a seed, not a route — the simplification and the terrain-lift sweep
    // that follow it exist precisely to raise vertices that sit too close to
    // the ground. Refusing the seed for terrain proximity throws away a path
    // the pipeline was about to fix, and on a real 40 m corridor that is
    // every descent (measured: the r5 probe stopped planning entirely).
    // Geometry is different — a box or a sphere cannot be lifted out of, so
    // a seed that goes through one is refused here and now.
    bool polylineObstacleClear(const std::vector<Eigen::Vector3d> &pts,
                               Eigen::Vector3d *hit = nullptr) {
        if (pts.size() < 2) return false;
        const double pitch =
            std::max(0.02, 0.5 * std::min(map_resolution_, map_resolution_z_));
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const Eigen::Vector3d &a = pts[i], &b = pts[i + 1];
            const double len = (b - a).norm();
            const int n = std::max(1, static_cast<int>(std::ceil(len / pitch)));
            for (int k = 0; k <= n; ++k) {
                const Eigen::Vector3d p = a + (b - a) * (double(k) / n);
                if (obstacleBlocked(p)) {
                    if (hit) *hit = p;
                    return false;
                }
            }
        }
        return true;
    }

    bool polylineClear(const std::vector<Eigen::Vector3d> &pts,
                       Eigen::Vector3d *hit = nullptr,
                       double start_relief_arc = 0.0) {
        if (pts.size() < 2) return false;
        // Half the finest step the occupancy predicate can resolve, so a thin
        // slab between two widely spaced vertices cannot be stepped over.
        const double pitch =
            std::max(0.02, 0.5 * std::min(map_resolution_, map_resolution_z_));
        double arc = 0.0;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const Eigen::Vector3d &a = pts[i], &b = pts[i + 1];
            const double len = (b - a).norm();
            const int n = std::max(1, static_cast<int>(std::ceil(len / pitch)));
            for (int k = 0; k <= n; ++k) {
                const double f = double(k) / n;
                const Eigen::Vector3d p = a + (b - a) * f;
                const double arc_here = arc + len * f;
                // Obstacles: never exempt, anywhere.
                if (obstacleBlocked(p)) {
                    if (hit) *hit = p;
                    return false;
                }
                // Terrain: exempt only inside the start relief arc.
                if (terrainClearanceShort(p) &&
                    arc_here > start_relief_arc) {
                    if (hit) *hit = p;
                    return false;
                }
            }
            arc += len;
        }
        return true;
    }

    // The two halves of checkOccupancy_esdf, separated so a caller can
    // tolerate one without tolerating the other. Their disjunction is exactly
    // checkOccupancy_esdf.
    inline bool terrainClearanceShort(const Eigen::Vector3d &pos) {
        if (!terrain_height_) return false;
        const float h = terrain_height_(pos.x(), pos.y());
        return std::isfinite(h) &&
               pos.z() - static_cast<double>(h) < obstacle_margin_;
    }
    inline bool obstacleBlocked(const Eigen::Vector3d &pos) {
        if (!sdf_ || !sdf_->hasData()) return false;
        float d = sdf_->getDistance(pos);
        if (!std::isfinite(d)) return true;  // outside map = blocked
        if (d < obstacle_margin_) return true;
        if (dyn_obstacle_margin_ > obstacle_margin_ &&
            sdf_->getDynamicDistance(pos) < dyn_obstacle_margin_) return true;
        return false;
    }

 private:
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
    // Bulk twin of checkOccupancy_esdf for HOT loops: identical logic but
    // uses the lock-free *Bulk SDF queries. Caller MUST hold an
    // sdf_->bulkReadGuard() for the whole loop (see fm2BuildSpeedMap — the
    // per-cell shared_lock there cost ~60 s of rwlock traffic on a 686M-cell
    // grid). Keep in sync with checkOccupancy_esdf.
    inline bool checkOccupancyBulk_esdf(const Eigen::Vector3d &pos) {
        return checkOccupancyBulkCol_esdf(pos, terrainColumn(pos.x(), pos.y()));
    }

    // Both DEM queries (terrain_height_, terrain_hgrad_) are std::function
    // indirections into a heightmap: they depend on (x, y) only, so their
    // results are constant down a z column. A caller that walks a whole
    // column — fm2BuildSpeedMap, ~110 cells deep — resolves the column ONCE
    // through terrainColumn/roughColumn and passes it to the *Col twins
    // below. Same values in the same order, so F is bit-identical either way.
    struct TerrainCol {
        float occ_h = std::numeric_limits<float>::quiet_NaN();  // surface (occupancy)
        float rough_h = 0.f;    // surface as reported by the gradient query
        double slope = 0.0;     // |dh/dxy| at (x, y)
        bool rough_ok = false;  // gradient query succeeded AND roughness is on
    };
    // Gradient query only — the A* cost path needs roughness but not the
    // occupancy height, and must not pay for a lookup it will not read.
    inline TerrainCol roughColumn(double x, double y) const {
        TerrainCol c;
        if (rough_weight_ <= 0.0 || !terrain_hgrad_) return c;
        float h = 0.f, gx = 0.f, gy = 0.f;
        if (!terrain_hgrad_(x, y, &h, &gx, &gy)) return c;
        c.rough_ok = true;
        c.rough_h = h;
        c.slope = std::hypot(static_cast<double>(gx), static_cast<double>(gy));
        return c;
    }
    inline TerrainCol terrainColumn(double x, double y) const {
        TerrainCol c = roughColumn(x, y);
        if (terrain_height_) c.occ_h = terrain_height_(x, y);
        return c;
    }
    inline bool checkOccupancyBulkCol_esdf(const Eigen::Vector3d &pos,
                                           const TerrainCol &col) {
        // occ_h is NaN when no heightmap is installed, which fails the
        // isfinite test exactly as the absent-callback branch used to.
        if (std::isfinite(col.occ_h) &&
            pos.z() - static_cast<double>(col.occ_h) < obstacle_margin_)
            return true;
        if (!sdf_ || !sdf_->hasData()) return false;
        float d = sdf_->getDistanceBulk(pos);
        if (!std::isfinite(d)) return true;  // outside map = blocked
        if (d < obstacle_margin_) return true;
        if (dyn_obstacle_margin_ > obstacle_margin_ &&
            sdf_->getDynamicDistanceBulk(pos) < dyn_obstacle_margin_) return true;
        return false;
    }

    // V3: quadratic moat + probabilistic-OR composition.
    // Per zone: moat_i(x) = peak_i * (1 - d/reach_i)^2 for d < reach_i.
    // Composed: risk(x) = 1 - prod_i (1 - moat_i(x)),  bounded in [0, 1].
    // Returns risk_alpha_ * risk(x). Compact support; AABB pre-filter
    // skips the sqrt for far zones.
    // [ROUGH] terrain-roughness cost, ADDITIVE and in its own currency —
    // deliberately NOT composed into the OR-moat and NOT scaled by
    // risk_alpha_. The composed form was built first and measured: with
    // survival = (1-rough)(1-moat) a high roughness devalues the moat's
    // marginal weight, so the front-end started trading visibility cost
    // for gentle terrain (complex-field stress-case risk_cost 4x at similar
    // max_risk). The
    // additive form keeps the zone moat's gradient intact at any roughness.
    // It is added inside getRiskCost and the FM2 speed map, so every
    // consumer of the shared field — FM2 wave, A* edge cost, coarse
    // cost-to-go heuristic, shortcut acceptance and the corner-cut guard —
    // sees the same detour incentive and the shortcut pass cannot revert an
    // FM2 roughness detour. AGL fade: full effect below kRoughAglNear, zero
    // above kRoughAglFar — a high transit over a ridge is not "rough", only
    // terrain-following across it is.
    static constexpr double kRoughAglNear = 1.0;   // full effect below (units)
    static constexpr double kRoughAglFar  = 2.5;   // zero at/above (units)
    // Cost cap: bends routes as hard as a mid-strength zone moat
    // (~ alpha 30 * norm 0.33) but stays far below the barrier K, and a
    // DEM cliff's huge |dh/dxy| cannot act like a binary wall.
    static constexpr double kRoughCostCap = 10.0;
    // The roughness formula lives here alone; getRoughCost is the per-point
    // entry and fm2BuildSpeedMap reuses one column down its z axis.
    inline double getRoughCostCol(double z, const TerrainCol &col) const {
        if (!col.rough_ok) return 0.0;
        const double agl = z - static_cast<double>(col.rough_h);
        if (agl >= kRoughAglFar) return 0.0;
        if (col.slope <= rough_slope0_) return 0.0;
        double r = rough_weight_ * (col.slope - rough_slope0_);
        if (agl > kRoughAglNear)
            r *= (kRoughAglFar - agl) / (kRoughAglFar - kRoughAglNear);
        return std::min(r, kRoughCostCap);
    }
    inline double getRoughCost(const Eigen::Vector3d &pos) const {
        return getRoughCostCol(pos.z(), roughColumn(pos.x(), pos.y()));
    }

    // [GNRON] Per-zone endpoint taper factor in [0, 1]: 0 at a contained
    // endpoint, 1 at/beyond taper_radius (C1 smoothstep — same rationale as
    // the barrier ramp: kinked factors are what L-BFGS line searches die on;
    // the front-end shares the shape so both stages price one field).
    inline double endpointTaper(size_t zi, const Eigen::Vector3d &pos) const {
        if (risk_goal_taper_radius_ <= 0.0 || zi >= zone_taper_.size() ||
            zone_taper_[zi] == 0)
            return 1.0;
        double f = 1.0;
        for (int b = 0; b < 2; ++b) {
            if (!(zone_taper_[zi] & (1u << b))) continue;
            const Eigen::Vector3d &e = (b == 0) ? taper_start_ : taper_goal_;
            const double x = (pos - e).norm() / risk_goal_taper_radius_;
            if (x >= 1.0) continue;
            f *= x * x * (3.0 - 2.0 * x);
        }
        return f;
    }

    // Normalized OR-moat risk in [0, 1] (no alpha scaling). Used by the
    // inadmissible heuristic so its inflation factor stays dimensionless.
    inline double getRiskNorm(const Eigen::Vector3d &pos) const {
        if (!risk_zones_ || risk_zones_->empty()) return 0.0;
        double survival = 1.0;
        for (size_t zi = 0; zi < risk_zones_->size(); ++zi) {
            const auto &tz = (*risk_zones_)[zi];
            // Compact ellipsoidal risk envelope. Terrain visibility is a
            // separate multiplier, so terrain occlusion and geometric risk
            // coverage do not get conflated.
            const double dz = pos.z() - tz.center.z();
            const double rv = tz.vertical_reach > 0.0
                                  ? tz.vertical_reach : tz.reach;
            if (!(rv > 0.0) || std::abs(dz) >= rv) continue;
            const double dx = pos.x() - tz.center.x();
            if (std::abs(dx) >= tz.reach) continue;
            const double dy = pos.y() - tz.center.y();
            if (std::abs(dy) >= tz.reach) continue;
            const double q = std::sqrt(
                (dx * dx + dy * dy) / (tz.reach * tz.reach) +
                (dz * dz) / (rv * rv));
            if (q >= 1.0) continue;
            const double u = 1.0 - q;
            double visibility = 1.0;
            if (risk_visibility_) {
                visibility = std::clamp(risk_visibility_(zi, pos), 0.0, 1.0);
                if (visibility <= 0.0) continue;
            }
            const double moat =
                tz.peak * u * u * visibility * endpointTaper(zi, pos);
            constexpr double kMoatCap = 1.0 - 1e-3;
            survival *= (1.0 - std::min(moat, kMoatCap));
        }
        return 1.0 - survival;
    }

    // Mark zones containing the start or goal as barrier-exempt (must enter).
    // Call once per search before evaluating risk costs.
    void prepareBarrier(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) {
        zone_no_barrier_.clear();
        zone_taper_.clear();
        if (!risk_zones_) return;
        zone_no_barrier_.assign(risk_zones_->size(), 0);
        // Same ellipsoidal membership as getRiskNorm/insideBarrierZone.
        auto in_zone = [this](const Eigen::Vector3d &p,
                              const RiskZoneLite &tz, size_t zi) {
            const double rv = tz.vertical_reach > 0.0
                                  ? tz.vertical_reach : tz.reach;
            if (!(tz.reach > 0.0) || !(rv > 0.0)) return false;
            const Eigen::Vector3d d = p - tz.center;
            const double q2 = d.head<2>().squaredNorm() /
                                  (tz.reach * tz.reach) +
                              d.z() * d.z() /
                                  (rv * rv);
            if (q2 >= 1.0) return false;
            return !risk_visibility_ || risk_visibility_(zi, p) > 0.5;
        };
        taper_start_ = start;
        taper_goal_ = goal;
        zone_taper_.assign(risk_zones_->size(), 0);
        for (size_t i = 0; i < risk_zones_->size(); ++i) {
            const auto &tz = (*risk_zones_)[i];
            const bool s_in = in_zone(start, tz, i);
            const bool g_in = in_zone(goal, tz, i);
            if (s_in || g_in) zone_no_barrier_[i] = 1;
            // [GNRON] taper only zones exempted BY CONTAINMENT — zones later
            // exempted for other reasons (soft-override crossings) keep their
            // full moat: crossing them is priced, entering "home" is not.
            zone_taper_[i] = static_cast<unsigned char>(
                (s_in ? 1u : 0u) | (g_in ? 2u : 0u));
        }
    }

    // Inside a barrier-applied zone? Zones containing the start/goal are exempt
    // (zone_no_barrier_) — they must be entered, so they stay soft (moat only).
    // First non-exempt zone whose VISIBLE ellipsoid volume contains pos
    // (-1 = none). skip_soft_override skips zones the [ZONE-AVOID] pass-2
    // route needed (they stay finite-K in hard passes).
    inline int visibleBarrierZoneAt(const Eigen::Vector3d &pos,
                                    bool skip_soft_override) const {
        if (!risk_zones_ || risk_barrier_ <= 0.0) return -1;
        for (size_t i = 0; i < risk_zones_->size(); ++i) {
            if (i < zone_no_barrier_.size() && zone_no_barrier_[i]) continue;
            if (skip_soft_override && i < zone_soft_override_.size() &&
                zone_soft_override_[i]) continue;
            if (zoneVisibleVolumeContains(i, pos))
                return static_cast<int>(i);
        }
        return -1;
    }
    inline bool insideBarrierZone(const Eigen::Vector3d &pos) const {
        return visibleBarrierZoneAt(pos, false) >= 0;
    }
    // [ZONE-AVOID] hard-volume membership WITH standoff margin: ellipsoid
    // inflated 5% and the LOS contour lowered to 0.35 (vs the soft barrier's
    // 0.5 rim) so the zone-free geodesic stands OFF the visible rim instead
    // of hugging it — rim-hugging routes parked the back-end in the steepest
    // visibility gradients and lost the duck-below tug-of-war (measured:
    // pull 6-7x terrain restoring, terrain penetration, audit reject).
    inline bool insideHardZoneVol(const Eigen::Vector3d &pos) const {
        if (!risk_zones_ || risk_barrier_ <= 0.0) return false;
        for (size_t i = 0; i < risk_zones_->size(); ++i) {
            if (i < zone_no_barrier_.size() && zone_no_barrier_[i]) continue;
            if (i < zone_soft_override_.size() && zone_soft_override_[i])
                continue;
            if (zoneHardVolumeContains(i, pos)) return true;
        }
        return false;
    }
    // [ZONE-AVOID] volume to DISCONNECT in the current hard pass.
    inline bool insideHardZone(const Eigen::Vector3d &pos) const {
        return zone_hard_mode_ && insideHardZoneVol(pos);
    }
    // [ZONE-AVOID] Conservative CELL-scale hard test for the speed-map
    // rasterization. Center-point tests alias badly here: the shadow
    // sigmoid is ~0.1 u wide vs ~1 u coarse cells, so a cell whose CENTER
    // reads "shadowed" can be mostly exposed — the wave then threads
    // sub-cell "seams" that are not real corridors, and the extracted
    // route triggers the optimizer's crossing marker despite a pass=1
    // claim. Block a cell when the inflated ellipsoid intersects the cell
    // AABB (closest-point test, separable per axis) AND any of the
    // center/8-corner visibility probes clears the hard threshold. The
    // half-cell geometric growth also covers the T-sampler's sub-cell
    // corner-cuts during extraction.
    inline bool insideHardZoneCell(const Eigen::Vector3d &c,
                                   double hx, double hy, double hz) const {
        if (!risk_zones_ || risk_barrier_ <= 0.0) return false;
        constexpr double kInflate2 = kZoneHardInflate * kZoneHardInflate;
        constexpr double kHardVis = kZoneHardVis;
        for (size_t i = 0; i < risk_zones_->size(); ++i) {
            if (i < zone_no_barrier_.size() && zone_no_barrier_[i]) continue;
            if (i < zone_soft_override_.size() && zone_soft_override_[i])
                continue;
            const auto &tz = (*risk_zones_)[i];
            const double rv = tz.vertical_reach > 0.0
                                  ? tz.vertical_reach : tz.reach;
            if (!(tz.reach > 0.0) || !(rv > 0.0)) continue;
            const double dx =
                std::max(0.0, std::abs(tz.center.x() - c.x()) - hx);
            const double dy =
                std::max(0.0, std::abs(tz.center.y() - c.y()) - hy);
            const double dz =
                std::max(0.0, std::abs(tz.center.z() - c.z()) - hz);
            const double q2 = (dx * dx + dy * dy) / (tz.reach * tz.reach) +
                              dz * dz / (rv * rv);
            if (q2 >= kInflate2) continue;
            if (!risk_visibility_) return true;
            if (risk_visibility_(i, c) > kHardVis) return true;
            for (int sx = -1; sx <= 1; sx += 2)
              for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Eigen::Vector3d q(c.x() + sx * hx,
                                            c.y() + sy * hy,
                                            c.z() + sz * hz);
                    if (risk_visibility_(i, q) > kHardVis) return true;
                }
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
        // exists, else cross at the least-moat point. Terrain roughness adds
        // in its own currency (see getRoughCost — NOT alpha-coupled).
        double cost = risk_alpha_ * getRiskNorm(pos) + getRoughCost(pos);
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
    void setRiskVisibility(
        std::function<double(size_t, const Eigen::Vector3d &)> f) {
        risk_visibility_ = std::move(f);
    }
    void setObstacleMargin(double m) { obstacle_margin_ = m; }
    // Extra berth around dynamic obstacles only (<= obstacle_margin_ disables).
    void setDynObstacleMargin(double m) { dyn_obstacle_margin_ = m; }
    void setGroundHeight(double h)      { ground_height_ = h; }
    // cell_u > 0 = DEM cell size in frame units. The chord/inner-chord
    // samplers must out-resolve the DEM: the legacy 0.5 u pitch was sized for
    // the 250 m full_map grid and skips whole cells of the 30-40 m corridor
    // crops (a one-cell ridge between samples passes untested). Half-cell
    // pitch is sufficient — the bilinear surface has no sub-cell features.
    void setTerrainHeightmap(std::function<float(double, double)> f,
                             double cell_u = 0.0) {
        terrain_height_ = std::move(f);
        terrain_stride_floor_ =
            (cell_u > 0.0) ? std::min(0.5, 0.5 * cell_u) : 0.5;
    }
    void setVirtualCeilHeight(double h) { virtual_ceil_height_ = h; }
    void setRiskAlpha(double a) { risk_alpha_ = a; }
    void setTerrainHeightGrad(
        std::function<bool(double, double, float *, float *, float *)> f) {
        terrain_hgrad_ = std::move(f);
    }
    // [ROUGH] weight 0 disables; slope0 = slope deadband (tan units).
    void setRoughness(double w, double slope0) {
        rough_weight_ = std::max(0.0, w);
        rough_slope0_ = std::max(0.0, slope0);
    }
    void setRiskBarrier(double k) { risk_barrier_ = (k > 0.0 ? k : 0.0); }
    // [GNRON] radius (frame units) of the endpoint moat taper; <=0 = off.
    void setRiskGoalTaperRadius(double r) {
        risk_goal_taper_radius_ = (r > 0.0 ? r : 0.0);
    }
    // Geodesic-extraction slope limit, tan of the shared flight-path-angle
    // cap; <=0 = off.
    void setGeodesicSlopeTanMax(double t) {
        geo_slope_tan_max_ = (t > 0.0 ? t : 0.0);
    }
    void setZoneAvoidLexico(bool on) { zone_avoid_lexico_ = on; }
    // Last plan's [ZONE-AVOID] pass (0 = policy off / no zones,
    // 1 = zone-free route, 2 = soft fallback, 3 = re-hardened).
    int zoneAvoidPass() const { return zone_avoid_pass_; }
    // [S13] Read-only zone disposition state for the policy snapshot:
    // endpoint containment exemptions and the pass-2/3 soft-crossing set.
    const std::vector<char> &zoneNoBarrier() const { return zone_no_barrier_; }
    const std::vector<char> &zoneSoftOverride() const {
      return zone_soft_override_;
    }
    // [S13] GEOMETRY-ONLY NOMINAL-volume membership of ONE zone (1.0x
    // ellipsoid, visibility > 0.5 — the barrier wall / risk-positive rim),
    // WITHOUT exemption filtering. This identifies which zones a route
    // actually PASSES THROUGH (the pass-2 crossed set, the barrier wall
    // via visibleBarrierZoneAt) — it is NOT the contact gate; candidates
    // are gated on the larger zoneHardVolumeContains below.
    inline bool zoneVisibleVolumeContains(size_t i,
                                          const Eigen::Vector3d &pos) const {
        return zoneVolumeContains(i, pos, 1.0, 0.5);
    }
    // [S13] GEOMETRY-ONLY HARD-EXCLUSION volume of ONE zone — the exact
    // 1.05x-inflated ellipsoid + visibility>0.35 standoff volume the hard
    // passes exclude ROUTES from (insideHardZoneVol), WITHOUT the
    // exemption filtering: exemptions are POLICY and live in the
    // snapshot's dispositions. This is the shared contact primitive the
    // transition generator consumes through PathManager::zoneContact — a
    // candidate is judged by the same standard a route is (review find:
    // the nominal 1.0x/0.5 volume read CLEAR inside the standoff band,
    // where visibility in (0.35, 0.5] still carries positive risk).
    inline bool zoneHardVolumeContains(size_t i,
                                       const Eigen::Vector3d &pos) const {
        return zoneVolumeContains(i, pos, kZoneHardInflate, kZoneHardVis);
    }
    // [S13] The volume the MISSION AUTHORED: the ellipsoid as written in the
    // scenario, un-inflated, with the SAME visibility floor. The 1.05 above
    // is a route-seeding standoff (see insideHardZoneVol) and the two must
    // not be conflated — a flight refused for entering the standoff is
    // refused half a kilometre outside the volume anyone declared. Note the
    // floor is kZoneHardVis (0.35), NOT the 0.5 of
    // zoneVisibleVolumeContains: visibility in (0.35, 0.5] still carries
    // positive risk, so the stricter-looking helper is the wrong test here.
    inline bool zoneAuthoredVolumeContains(size_t i,
                                           const Eigen::Vector3d &pos) const {
        return zoneVolumeContains(i, pos, 1.0, kZoneHardVis);
    }
    // Shared single-zone volume test: scaled ellipsoid + LOS contour.
    inline bool zoneVolumeContains(size_t i, const Eigen::Vector3d &pos,
                                   double scale, double vis_floor) const {
        if (!risk_zones_ || i >= risk_zones_->size()) return false;
        const auto &tz = (*risk_zones_)[i];
        const double rv =
            tz.vertical_reach > 0.0 ? tz.vertical_reach : tz.reach;
        if (!(tz.reach > 0.0) || !(rv > 0.0)) return false;
        const Eigen::Vector3d d = pos - tz.center;
        const double q2 =
            d.head<2>().squaredNorm() / (tz.reach * tz.reach) +
            d.z() * d.z() / (rv * rv);
        if (q2 >= scale * scale) return false;
        return !risk_visibility_ || risk_visibility_(i, pos) > vis_floor;
    }
    // [ZONE-AVOID] standoff constants shared by insideHardZoneVol and the
    // transition contact gate (see insideHardZoneVol's rationale above).
    static constexpr double kZoneHardInflate = 1.05;
    static constexpr double kZoneHardVis = 0.35;
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

    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    std::vector<Eigen::Vector3d> getPath();

    // `is_takeoff_leg` says this search begins where the AIRCRAFT IS, not at
    // a waypoint the route reached on its own. Only then may the extracted
    // route start closer to the terrain than the clearance margin — the
    // mission pins the aircraft's position and it has to be allowed to climb
    // away from it. A multi-waypoint mission calls this once per leg, and
    // every leg after the first starts at a point the planner chose, which is
    // not a takeoff and gets no allowance.
    std::vector<Eigen::Vector3d> astarSearchAndGetSimplePath(
        const double step_size, Eigen::Vector3d start_pt,
        Eigen::Vector3d end_pt, int drone_id,
        bool is_takeoff_leg = false);


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
    // profile systematically omits exactly the constraints that shaped the
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
