#include "path_planner/dyn_a_star.h"
#ifdef PP_HAVE_CUDA
#include "path_planner/fm2_gpu.h"
#include "path_planner/eikonal_godunov.h"
#endif
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <utility>

using namespace std;
using namespace Eigen;

namespace path_planner { namespace search {

PathSearcher::~PathSearcher() = default;

void PathSearcher::initGridMap(const Eigen::Vector3i &pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;
    nx_ = pool_size(0);
    ny_ = pool_size(1);
    nz_ = pool_size(2);
    const size_t N = static_cast<size_t>(nx_) * ny_ * nz_;
    if (N > static_cast<size_t>(std::numeric_limits<int>::max())) {
        // flatIdx/flatToIdx and the queue payloads are int: a larger pool
        // overflows them (UB) before the self-check below could catch it.
        // Empty the pool instead — Coord2Index then fails every query and
        // AstarSearch returns false, so callers take their normal fallback.
        if (log_manager_)
            log_manager_->errorf("[A* INIT] pool %dx%dx%d exceeds INT_MAX cells - A* disabled for this map",
                                 nx_, ny_, nz_);
        POOL_SIZE_.setZero();
        nx_ = ny_ = nz_ = 0;
        pool_.clear();
        return;
    }
    pool_.assign(N, GridNode{});
    // Comparators bind to pool_ for f-score lookup (anchor/inadmis split).
    openSet_anchor_  = std::priority_queue<int, std::vector<int>, NodeComparatorAnchor>(
        NodeComparatorAnchor(&pool_));
    openSet_inadmis_ = std::priority_queue<int, std::vector<int>, NodeComparatorInadmis>(
        NodeComparatorInadmis(&pool_));

    // Self-check: flatIdx/flatToIdx round-trip on a sparse sample plus
    // boundary cells. Cheap (sub-ms) and only runs on init.
    {
        auto check = [&](int i, int j, int k) {
            const int f = flatIdx(i, j, k);
            const Eigen::Vector3i back = flatToIdx(f);
            if (back(0) != i || back(1) != j || back(2) != k) {
                if (log_manager_) {
                    log_manager_->errorf("[A* INIT] flatIdx self-check FAILED at (%d,%d,%d) -> %d -> (%d,%d,%d)",
                        i, j, k, f, back(0), back(1), back(2));
                }
                std::abort();
            }
        };
        const int sx = std::max(1, nx_ / 8);
        const int sy = std::max(1, ny_ / 8);
        const int sz = std::max(1, nz_ / 8);
        for (int i = 0; i < nx_; i += sx)
            for (int j = 0; j < ny_; j += sy)
                for (int k = 0; k < nz_; k += sz)
                    check(i, j, k);
        check(0, 0, 0);
        check(nx_ - 1, ny_ - 1, nz_ - 1);
        if (log_manager_) {
            log_manager_->infof("[A* INIT] flatIdx self-check passed (pool=%dx%dx%d, total=%zu)",
                nx_, ny_, nz_, pool_.size());
        }
    }
}

void PathSearcher::resizePool(const Eigen::Vector3i &pool_size)
{
    initGridMap(pool_size);
}

double PathSearcher::getDiagHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    double dx = abs(i1(0) - i2(0));
    double dy = abs(i1(1) - i2(1));
    double dz = abs(i1(2) - i2(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

vector<int> PathSearcher::retrievePath(int current_flat)
{
    vector<int> path;
    if (current_flat < 0) return path;

    int cur = current_flat;
    while (cur >= 0)
    {
        path.push_back(cur);
        cur = pool_[cur].cameFromFlat;
    }
    if (log_manager_ && risk_zones_ && !risk_zones_->empty()) {
        log_manager_->infof("[A* RETRIEVE] path length=%zu (listed goal->start)", path.size());
        int stride = std::max(1, (int)path.size() / 20);
        for (size_t i = 0; i < path.size(); i += stride) {
            const int f = path[i];
            const Eigen::Vector3i idx = flatToIdx(f);
            Eigen::Vector3d w = Index2Coord(idx);
            double tc = getRiskCost(w);
            log_manager_->infof("[A* RETRIEVE] i=%zu idx=(%d,%d,%d) world=(%.2f,%.2f,%.2f) g=%.3f risk_here=%.3f",
                i, idx(0), idx(1), idx(2),
                w.x(), w.y(), w.z(), pool_[f].gScore, tc);
        }
    }

    return path;
}

bool PathSearcher::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (log_manager_) {
        log_manager_->debugf("시작/끝점 변환 시도 - Start: (%.2f,%.2f,%.2f), End: (%.2f,%.2f,%.2f)", 
                           start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
    }
    
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx)) {
        if (log_manager_) {
            log_manager_->error("좌표를 인덱스로 변환 실패");
        }
        return false;
    }

    if (checkOccupancy_esdf(Index2Coord(start_idx)))
    {
        if (log_manager_) {
            log_manager_->warnf("시작점이 장애물 내부에 위치 - Idx: (%d,%d,%d), Coord: (%.2f,%.2f,%.2f)", 
                               start_idx(0), start_idx(1), start_idx(2), start_pt(0), start_pt(1), start_pt(2));
        }
        // Direction away from the goal; when start==goal normalized() would be
        // NaN (zero vector) and the march never terminates — climb instead.
        Eigen::Vector3d adj_dir = start_pt - end_pt;
        adj_dir = (adj_dir.norm() > 1e-9) ? Eigen::Vector3d(adj_dir / adj_dir.norm())
                                          : Eigen::Vector3d(0, 0, 1);
        do
        {
            start_pt += adj_dir * step_size_;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy_esdf(Index2Coord(start_idx)));
        if (log_manager_) {
            log_manager_->warnf("시작점 조정 완료 - 새로운 시작점: (%.2f,%.2f,%.2f)", start_pt(0), start_pt(1), start_pt(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New start point: (%f,%f,%f)", start_pt(0), start_pt(1), start_pt(2));
    }

    if (checkOccupancy_esdf(Index2Coord(end_idx)))
    {
        if (log_manager_) {
            log_manager_->warnf("도착점이 장애물 내부에 위치 - Coord: (%.2f,%.2f,%.2f)", end_pt(0), end_pt(1), end_pt(2));
        }
        Eigen::Vector3d adj_dir = end_pt - start_pt;
        adj_dir = (adj_dir.norm() > 1e-9) ? Eigen::Vector3d(adj_dir / adj_dir.norm())
                                          : Eigen::Vector3d(0, 0, 1);
        do
        {
            end_pt += adj_dir * step_size_;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy_esdf(Index2Coord(end_idx)));
        if (log_manager_) {
            log_manager_->warnf("도착점 조정 완료 - 새로운 도착점: (%.2f,%.2f,%.2f)", end_pt(0), end_pt(1), end_pt(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New END point: (%f,%f,%f)", end_pt(0), end_pt(1), end_pt(2));
    }

    return true;
}

bool PathSearcher::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    auto time_1 = rclcpp::Clock().now();
    ++rounds_;
    
    if (log_manager_) {
        log_manager_->infof("3D A* 검색 시작 - Round: %d, Step size: %.3f",
                           rounds_, step_size);
        log_manager_->infof("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)",
                           start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
        // DEBUG: risk binding at entry.
        size_t tz_n = risk_zones_ ? risk_zones_->size() : 0;
        log_manager_->infof("[A* DBG] risk_zones_ptr=%p size=%zu weight=%.3f",
                           (const void*)risk_zones_, tz_n, risk_alpha_);
        if (risk_zones_) {
            for (size_t i = 0; i < risk_zones_->size(); ++i) {
                const auto &tz = (*risk_zones_)[i];
                log_manager_->infof("[A* DBG]  tz[%zu] c=(%.2f,%.2f,%.2f) R=%.2f L=%.2f",
                    i, tz.center.x(), tz.center.y(), tz.center.z(),
                    tz.reach, tz.peak);
                // Direct probe: getRiskCost(center) should equal
                // peak * risk_alpha_ (moat at u=1, no other zones).
                double probe = getRiskCost(tz.center);
                log_manager_->infof("[A* DBG]  tz[%zu] probe@center cost=%.3f (expected~%.3f)",
                    i, probe, tz.peak * risk_alpha_);
                // Off-by-one checks: 1m step towards goal from center.
                Eigen::Vector3d off_pos = tz.center + Eigen::Vector3d(1.0, 0.0, 0.0);
                log_manager_->infof("[A* DBG]  tz[%zu] probe@center+1mX cost=%.3f",
                    i, getRiskCost(off_pos));
            }
        }
    }
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    if (log_manager_) {
        log_manager_->debugf("검색 중심점: (%.2f,%.2f,%.2f)", center_(0), center_(1), center_(2));
    }
    RCLCPP_INFO(rclcpp::get_logger("astar"), "CENTER: (%f,%f,%f)", center_(0), center_(1), center_(2));

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        if (log_manager_) {
            log_manager_->errorf("3D A* 검색 실패 - 시작/끝점 처리 불가");
            log_manager_->errorf("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)", 
                               start_pt.x(), start_pt.y(), start_pt.z(), end_pt.x(), end_pt.y(), end_pt.z());
            log_manager_->errorf("Pool 크기: (%d,%d,%d), 중심: (%d,%d,%d)", 
                               POOL_SIZE_(0), POOL_SIZE_(1), POOL_SIZE_(2), CENTER_IDX_(0), CENTER_IDX_(1), CENTER_IDX_(2));
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Unable to handle the initial or end point, force return!");
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Start: (%.2f,%.2f,%.2f) End: (%.2f,%.2f,%.2f)", 
                    start_pt.x(), start_pt.y(), start_pt.z(), end_pt.x(), end_pt.y(), end_pt.z());
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Pool size: (%d,%d,%d) Center: (%d,%d,%d)", 
                    POOL_SIZE_(0), POOL_SIZE_(1), POOL_SIZE_(2), CENTER_IDX_(0), CENTER_IDX_(1), CENTER_IDX_(2));
        return false;
    }

    int start_flat = flatIdx(start_idx);
    // end_idx is used directly for goal comparison (no need to flatten).

    std::priority_queue<int, std::vector<int>, NodeComparatorAnchor> empty_a{NodeComparatorAnchor(&pool_)};
    openSet_anchor_.swap(empty_a);
    std::priority_queue<int, std::vector<int>, NodeComparatorInadmis> empty_i{NodeComparatorInadmis(&pool_)};
    openSet_inadmis_.swap(empty_i);

    GridNode &startNode = pool_[start_flat];
    startNode.rounds = rounds_;
    startNode.gScore = 0;
    startNode.fAnchor  = getHeuAnchor (start_idx, end_idx);
    startNode.fInadmis = getHeuInadmis(start_idx, end_idx);
    startNode.state = GridNode::OPENSET;
    startNode.cameFromFlat = -1;
    openSet_anchor_.push(start_flat);
    if (smha_w_ > 1.0) openSet_inadmis_.push(start_flat);

    double tentative_gScore;

    // Risk-aware A* per-search summary counters.
    size_t risk_query_count = 0;
    size_t in_zone_expansions = 0;
    double max_risk_observed = 0.0;

    int num_iter = 0;
    int current_flat = -1;
    size_t expand_inadmis = 0;
    size_t expand_anchor  = 0;
    while (!openSet_anchor_.empty())
    {
        num_iter++;

        // SMHA* dispatch: prefer inadmissible queue if it stays inside the
        // w * f_anchor_min suboptimality envelope. Fall back to anchor.
        bool pop_inadmis = false;
        if (smha_w_ > 1.0 && !openSet_inadmis_.empty()) {
            const double f_inadmis_top = pool_[openSet_inadmis_.top()].fInadmis;
            const double f_anchor_top  = pool_[openSet_anchor_ .top()].fAnchor;
            if (f_inadmis_top <= smha_w_ * f_anchor_top) pop_inadmis = true;
        }

        if (pop_inadmis) {
            current_flat = openSet_inadmis_.top();
            openSet_inadmis_.pop();
            ++expand_inadmis;
        } else {
            current_flat = openSet_anchor_.top();
            openSet_anchor_.pop();
            ++expand_anchor;
        }
        GridNode &current = pool_[current_flat];
        if (current.state == GridNode::CLOSEDSET) continue;  // stale push

        const Eigen::Vector3i current_idx = flatToIdx(current_flat);
        if (current_idx(0) == end_idx(0) && current_idx(1) == end_idx(1) && current_idx(2) == end_idx(2))
        {
            auto time_2 = rclcpp::Clock().now();
            auto elapsed = time_2 - time_1;
            if (log_manager_) {
                log_manager_->infof(
                    "A* done: iter=%d time=%.1fms risk_q=%zu in_zone=%zu "
                    "max_risk=%.3f alpha=%.2f goal_g=%.3f "
                    "smha_w=%.2f exp_inadmis=%zu exp_anchor=%zu",
                    num_iter, elapsed.seconds()*1000.0,
                    risk_query_count, in_zone_expansions,
                    max_risk_observed, risk_alpha_,
                    current.gScore,
                    smha_w_, expand_inadmis, expand_anchor);
            }
            printf("\033[34mA star iter:%d, time:%.3f\033[0m\n", num_iter, elapsed.seconds()*1000);
            gridPath_ = retrievePath(current_flat);
            return true;
        }
        current.state = GridNode::CLOSEDSET;

        static const int neighbor_offsets[26][3] = {
            {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
            {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
            {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
            {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
            {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
            {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
        };
        static const double neighbor_costs_ordered[26] = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.732050807568877, 1.732050807568877, 1.732050807568877, 1.732050807568877,
            1.732050807568877, 1.732050807568877, 1.732050807568877, 1.732050807568877
        };

        for (int i = 0; i < 26; i++)
        {
            int dx = neighbor_offsets[i][0];
            int dy = neighbor_offsets[i][1];
            int dz = neighbor_offsets[i][2];

            Vector3i neighborIdx;
            neighborIdx(0) = current_idx(0) + dx;
            neighborIdx(1) = current_idx(1) + dy;
            neighborIdx(2) = current_idx(2) + dz;

            if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 ||
                neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 ||
                neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
            {
                continue;
            }

            const int neighbor_flat = flatIdx(neighborIdx);
            GridNode &neighbor = pool_[neighbor_flat];

            bool flag_explored = neighbor.rounds == rounds_;

            if (flag_explored && neighbor.state == GridNode::CLOSEDSET)
            {
                continue;
            }

            neighbor.rounds = rounds_;

            const Eigen::Vector3d neigh_world = Index2Coord(neighborIdx);

            // Hard ground / ceiling gate.
            if (ground_height_ > -0.5 && neigh_world.z() < ground_height_) continue;
            if (virtual_ceil_height_ > -0.5 && neigh_world.z() > virtual_ceil_height_) continue;

            if (checkOccupancy_esdf(neigh_world))
                continue;

            double static_cost = neighbor_costs_ordered[i];

            // Risk-aware edge cost: distance * (1 + risk). Multiplicative
            // form keeps shorter paths cheaper inside risk regions.
            double risk_cost = getRiskCost(neigh_world);
            ++risk_query_count;
            if (risk_cost > 0.0) {
                ++in_zone_expansions;
                if (risk_cost > max_risk_observed) max_risk_observed = risk_cost;
            }
            tentative_gScore = current.gScore + static_cost * (1.0 + risk_cost);

            // Compute both f-scores (g is shared); SMHA* pushes onto both
            // queues so the inadmissible dispatch can prefer this node.
            const double h_a = getHeuAnchor (neighborIdx, end_idx);
            const double h_i = getHeuInadmis(neighborIdx, end_idx);

            if (!flag_explored)
            {
                neighbor.state = GridNode::OPENSET;
                neighbor.cameFromFlat = current_flat;
                neighbor.gScore = tentative_gScore;
                neighbor.fAnchor  = tentative_gScore + h_a;
                neighbor.fInadmis = tentative_gScore + h_i;
                openSet_anchor_.push(neighbor_flat);
                if (smha_w_ > 1.0) openSet_inadmis_.push(neighbor_flat);
            }
            else if (tentative_gScore < neighbor.gScore)
            {
                neighbor.cameFromFlat = current_flat;
                neighbor.gScore = tentative_gScore;
                neighbor.fAnchor  = tentative_gScore + h_a;
                neighbor.fInadmis = tentative_gScore + h_i;
                // Re-open even if it was closed (g improved); cheap because
                // pool entries are POD and CLOSEDSET nodes get re-flagged.
                neighbor.state = GridNode::OPENSET;
                openSet_anchor_.push(neighbor_flat);
                if (smha_w_ > 1.0) openSet_inadmis_.push(neighbor_flat);
            }
        }

        auto time_2 = rclcpp::Clock().now();
        auto elapsed = time_2 - time_1;
        // 20 s cap: global (one-shot) planning can afford a long front-end
        // search; upstream 0.5 s was tuned for on-board real-time replan.
        if (elapsed.seconds() > 20.0)
        {
            if (log_manager_) {
                log_manager_->warnf("3D A* 검색 시간 초과 - %.3fms 경과, 반복: %d회", elapsed.seconds()*1000, num_iter);
                log_manager_->errorf("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)",
                                    start_pt.x(), start_pt.y(), start_pt.z(),
                                    end_pt.x(), end_pt.y(), end_pt.z());
            }
            RCLCPP_ERROR(rclcpp::get_logger("astar"),
                        "A* search timeout! Start: (%.2f,%.2f,%.2f), End: (%.2f,%.2f,%.2f), Iter: %d, Time: %.3fs",
                        start_pt.x(), start_pt.y(), start_pt.z(),
                        end_pt.x(), end_pt.y(), end_pt.z(),
                        num_iter, elapsed.seconds());
            return false;
        }
    }

    auto time_2 = rclcpp::Clock().now();
    auto elapsed_total = time_2 - time_1;

    if (log_manager_) {
        log_manager_->warnf("3D A* 검색 실패 - 전체 시간: %.3fms, 반복: %d회", elapsed_total.seconds()*1000, num_iter);
    }
    
    if (elapsed_total.seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Time consume in A star path finding is %.3fs, iter=%d", elapsed_total.seconds(), num_iter);

    return false;
}


vector<Vector3d> PathSearcher::getPath()
{
    vector<Vector3d> path;
    path.reserve(gridPath_.size());

    for (int flat : gridPath_)
        path.push_back(Index2Coord(flatToIdx(flat)));

    reverse(path.begin(), path.end());
    return path;
}

// The terrain-clearance repair, lifted out of astarSearchAndGetSimplePath so
// the production loop is what a test can drive. It used to be inline, so the
// only way to test it was to reimplement it — and a reimplementation cannot
// catch the stride drifting apart from the gate's, which is the defect this
// exists to prevent.
//
// Walks every kept segment at terrainCheckPitch(), inserts a lifted vertex at
// the worst clearance shortfall, and repeats to a fixpoint. Returns the number
// of lifts; sets dbg_sweep_pitch_/dbg_lifted_/dbg_sweep_clean_.
int PathSearcher::sweepTerrainClearance(std::vector<Vector3d> &simple_path) {
    if (!terrain_height_) return 0;
    // One stride for the whole terrain repair, taken from the same place
    // the final gate takes its own. Recorded so the log and the test can
    // show the two sides agreed rather than assume it.
    const double sweep_pitch = terrainCheckPitch();
    dbg_sweep_pitch_ = sweep_pitch;
    int lifted = 0;
    // Hard cap on total lift/insert operations: every full_route vertex is
    // preserved 1:1 into clean_path anchors (STEP 3 never merges), so an
    // unbounded sweep would inflate piece_num_/variable_num_ and the
    // optimizer's per-iteration cost. Real DEMs converge in a handful of
    // lifts (bilinear cells have no sub-cell features — measured 2 lifts
    // on a 44 km ridge-grazing terrain-following route); the cap only
    // guards pathological
    // geometry. A capped exit with work remaining is WARNed below — the
    // optimizer terrain term is the remaining guard.
    //
    // NO fixed round cap: termination is guaranteed by the lift cap alone.
    // In-place lifts are once-per-vertex (v.z is set to need+0.02 at FIXED
    // (x,y), so the same vertex can never re-trigger), and every other
    // change INSERTS a vertex, bounded by kLiftCap. The old 8-round cap
    // bound FIRST on guard-dense corner chains (observed: terrain-following
    // stress case exited at 8 rounds with only lifted=14, leaving a
    // terrain-overlapping seed that the optimizer then rode into a -0.543
    // goal-approach collision).
    // kRoundSafety is a pure backstop against an unforeseen cycle.
    constexpr int kLiftCap = 512;
    constexpr int kRoundSafety = 256;
    bool clean_exit = false;
    dbg_lift_cap_ = kLiftCap;
    for (int round = 0; round < kRoundSafety && lifted < kLiftCap; ++round) {
        bool changed = false;
        // (0) Lift KEPT interior vertices that themselves overlap terrain.
        // The
        // per-segment sweep below only samples interior points (s=1..n-1),
        // so a vertex retained by chordOk's b<=a+1 fast path — which returns
        // true WITHOUT calling chordOccRisk/checkOccupancy — can sit below
        // terrain and would otherwise never be repaired. Endpoints (start,
        // goal) are commanded positions and left untouched.
        for (size_t k = 1; k + 1 < simple_path.size(); ++k) {
            Vector3d &v = simple_path[k];
            const float h = terrain_height_(v.x(), v.y());
            if (!std::isfinite(h)) continue;
            const double need = double(h) + obstacle_margin_;
            if (v.z() < need) {
                v.z() = need + 0.02;
                ++lifted;
                changed = true;
            }
        }
        for (size_t k = 0; k + 1 < simple_path.size(); ++k) {
            const Vector3d &p = simple_path[k];
            const Vector3d &q = simple_path[k + 1];
            const double len = (q - p).norm();
            // Shared with the final terrain gate. Walking coarser than the
            // judge is what let this sweep report "clean" on a route the
            // judge then refused 30 cm short.
            const int n =
                std::max(1, (int)std::ceil(len / sweep_pitch));
            double worst_pen = 0.0;
            Vector3d worst_pt;
            for (int s = 1; s < n; ++s) {
                const Vector3d x = p + (double(s) / n) * (q - p);
                const float h = terrain_height_(x.x(), x.y());
                if (!std::isfinite(h)) continue;
                const double pen =
                    (double(h) + obstacle_margin_) - x.z();
                if (pen > worst_pen) {
                    worst_pen = pen;
                    worst_pt = x;
                    worst_pt.z() = double(h) + obstacle_margin_ + 0.02;
                }
            }
            if (worst_pen > 0.0) {
                simple_path.insert(simple_path.begin() + k + 1, worst_pt);
                ++lifted;
                ++k;  // the lifted vertex itself is clear; recheck halves next round
                changed = true;
            }
        }
        // (2) Inner-chord terrain check. The MINCO back-end smooths ACROSS
        // kept corners along the mid->mid "inner chord", which can dip
        // below a ridge even when both adjacent SEGMENTS are clear. The
        // corner-cut guard earlier ran on the pre-sweep polyline, so
        // sweep-inserted lift vertices never got this check. Lift the worst
        // inner-chord clearance violation in place (same fixpoint as above).
        for (size_t k = 1; k + 1 < simple_path.size(); ++k) {
            const Vector3d m0 = 0.5 * (simple_path[k - 1] + simple_path[k]);
            const Vector3d m1 = 0.5 * (simple_path[k] + simple_path[k + 1]);
            const double len = (m1 - m0).norm();
            const int n =
                std::max(1, (int)std::ceil(len / sweep_pitch));
            double worst_pen = 0.0;
            Vector3d worst_pt;
            for (int s = 0; s <= n; ++s) {
                const Vector3d x = m0 + (double(s) / n) * (m1 - m0);
                const float h = terrain_height_(x.x(), x.y());
                if (!std::isfinite(h)) continue;
                const double pen = (double(h) + obstacle_margin_) - x.z();
                if (pen > worst_pen) {
                    worst_pen = pen;
                    worst_pt = x;
                    worst_pt.z() = double(h) + obstacle_margin_ + 0.02;
                }
            }
            if (worst_pen > 0.0) {
                simple_path.insert(simple_path.begin() + k + 1, worst_pt);
                ++lifted;
                ++k;
                changed = true;
            }
        }
        if (!changed) { clean_exit = true; break; }
    }
    // [FM2-FINAL-CLEAR] carried to the whole-path gate below, which is the
    // only place that can say whether the sweep's outcome was enough.
    dbg_lifted_ = lifted;
    dbg_sweep_clean_ = clean_exit;
    RCLCPP_INFO(rclcpp::get_logger("astar"),
                "[FM2-SWEEP] lifted=%d cap=%d clean_exit=%s pts=%zu",
                 lifted, kLiftCap, clean_exit ? "true" : "false",
                 simple_path.size());
    if (log_manager_) {
        if (!clean_exit && lifted > 0) {
            // Exited via the round cap or kLiftCap while still finding
            // work — a residual sub-chord clearance violation may remain.
            // Distinct
            // from the clean-convergence info line so it is greppable.
            log_manager_->warnf(
                "[A* SHORTCUT] terrain sweep hit its cap (lifted=%d, "
                "cap=%d) with work remaining — residual clearance violation "
                "possible; optimizer terrain term is the remaining guard",
                lifted, kLiftCap);
        } else if (lifted > 0) {
            log_manager_->infof(
                "[A* SHORTCUT] terrain sweep lifted %d vertex(es) over "
                "sub-chord ridges (sweep_pitch %.3f u, dem_cell_floor %.3f u)",
                lifted, sweep_pitch, terrain_stride_floor_);
        }
    }
    return dbg_lifted_;
}

vector<Vector3d> PathSearcher::astarSearchAndGetSimplePath(const double step_size, Vector3d start_pt, Vector3d end_pt, int drone_id, bool is_takeoff_leg){

    if (log_manager_) {
        log_manager_->infof("드론 %d: 3D 경로 검색 및 단순화 시작", drone_id);
        log_manager_->debugf("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)",
                           start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
    }

    // Zones containing the start/goal are barrier-exempt (must enter them).
    prepareBarrier(start_pt, end_pt);

    // === FM2 front-end (heuristic-free Eikonal). Produces `path`, then
    //     joins the SAME simplification block the A* path uses. ===
    bool fm2_done = false;
    vector<Vector3d> fm2_path;
    // Mission altitude band, used by the FM2 speed map AND the shortcut cost
    // metric below (so chords that trade the altitude hold for a long
    // diagonal ramp are priced fairly against the in-band detour). No slack:
    // with coarse z-cells a one-cell allowance can park the whole bottom
    // layer inside the band and the path settles a cell too low.
    fm2_alt_zlo_ = std::min(start_pt.z(), end_pt.z());
    fm2_alt_zhi_ = std::max(start_pt.z(), end_pt.z());

    if (front_end_ == FrontEnd::FM2) {
        auto tf0 = rclcpp::Clock().now();
        // [ZONE-AVOID] Lexicographic zone policy: if ANY route that stays
        // out of every visible zone volume exists, take it — avoidance is
        // never traded against detour length ("cheap enough to cross" is
        // not a thing). Only when a zone wall truly blocks the mission does
        // the soft finite-K field run (decisive minimum-dwell crossing at
        // the least-exposure seam), and pass 3 re-hardens every zone the
        // soft route did not need so an unavoidable wall never softens the
        // avoidable zones around it.
        int zone_pass = 0;
        zone_avoid_pass_ = 0;
        zone_lexico_hard_ = false;
        const bool lexico = zone_avoid_lexico_ && risk_zones_ &&
                            !risk_zones_->empty() && risk_barrier_ > 0.0;
        if (lexico) {
            zone_soft_override_.assign(risk_zones_->size(), 0);
            zone_hard_mode_ = true;
            fm2BuildSpeedMap();
            fm2SolveEikonal(end_pt, start_pt);
            // Reachability = the probe geodesic actually reaches the goal.
            // A finite T(start) alone is NOT enough: the field can be
            // finite via routes the descent cannot follow (e.g. over the
            // ellipsoid top), and an aborted descent appends the goal as a
            // straight chord THROUGH the wall — accepting that as "pass 1"
            // published a zone-crossing route labelled zone-free.
            // A probe route only counts if the descent truly reached the
            // goal AND never went below the surface: terrain keeps kFMin
            // porosity even in hard mode (see fm2BuildSpeedMap), so the
            // "reachable" wave may be a burrow under a zone wall.
            auto probeRouteOk = [&](const std::vector<Eigen::Vector3d> &pr) {
                if (pr.size() < 2) return false;
                if (!fm2_geo_reached_goal_) {
                    // Near-goal aborts are fine: the descent regularly
                    // stalls a few cells short inside the (exempt) goal
                    // zone and the appended goal closes it with a short
                    // chord — reject only if the gap is long or the chord
                    // clips the hard volume. (A start-side abort, e.g. the
                    // seal test's 400 u gap, fails here and falls back.)
                    const Eigen::Vector3d &last = pr[pr.size() - 2];
                    const Eigen::Vector3d &gw = pr.back();
                    const double rem = (gw - last).norm();
                    if (rem > 25.0) return false;
                    const int n = std::max(1, (int)std::ceil(rem / 0.5));
                    for (int t = 0; t <= n; ++t) {
                        const Eigen::Vector3d q =
                            last + (double)t / n * (gw - last);
                        if (insideHardZoneVol(q)) return false;
                    }
                }
                // The WHOLE route body must stay out of every hard
                // volume — not just the appended goal chord. Hard mode
                // keeps kFMin porosity for terrain reasons, so on a
                // mission a zone RING makes truly unavoidable the wave
                // burrows THROUGH the wall at kFMin, the descent follows,
                // and (terrain-clean, goal reached) the old checks
                // accepted a zone-crossing route as "pass 1 zone-free" —
                // the exact mislabel this probe exists to prevent
                // (found by the zonewall ring fixture).
                for (size_t vi = 0; vi + 1 < pr.size(); ++vi) {
                    const Eigen::Vector3d &a = pr[vi];
                    const Eigen::Vector3d &b = pr[vi + 1];
                    const double seg = (b - a).norm();
                    const int n = std::max(1, (int)std::ceil(seg / 0.5));
                    for (int t = 0; t <= n; ++t) {
                        const Eigen::Vector3d q =
                            a + (double)t / n * (b - a);
                        if (insideHardZoneVol(q)) return false;
                    }
                }
                if (!terrain_height_) return true;
                for (const auto &q : pr) {
                    if (q.z() <
                        (double)terrain_height_(q.x(), q.y()) - 0.15)
                        return false;
                }
                return true;
            };
            bool hard_ok = fm2_valid_ && std::isfinite(fm2SampleT(start_pt));
            if (hard_ok) {
                const std::vector<Eigen::Vector3d> probe1 =
                    fm2ExtractGeodesic(start_pt, end_pt);
                hard_ok = probeRouteOk(probe1);
            }
            if (hard_ok) {
                zone_pass = 1;   // zone-free route exists
            } else {
                // Pass 2: legacy soft field — always connected.
                zone_hard_mode_ = false;
                fm2BuildSpeedMap();
                fm2SolveEikonal(end_pt, start_pt);
                std::vector<Eigen::Vector3d> probe;
                if (fm2_valid_) probe = fm2ExtractGeodesic(start_pt, end_pt);
                // Zones the soft route actually needs (visible-volume
                // hits) — EVERY containing zone, not the first hit: a
                // crossing through a two-zone overlap needs both members
                // soft, or pass 3 re-hardens a zone the route is already
                // inside and the search collapses to the all-soft pass 2
                // (observed on a 12-zone ring: one release of an
                // overlapping pair kept the annulus sealed).
                // Sampled along SEGMENTS (<=0.5 u, same step as
                // probeRouteOk), not vertices only: a zone entered
                // between two vertices would stay unmarked, be
                // re-hardened by pass 3, and collapse the search the
                // same way the first-hit marking did.
                std::vector<char> crossed(risk_zones_->size(), 0);
                auto mark = [&](const Eigen::Vector3d &q) {
                    for (size_t zi = 0; zi < risk_zones_->size(); ++zi) {
                        if (crossed[zi]) continue;
                        if (zi < zone_no_barrier_.size() &&
                            zone_no_barrier_[zi]) continue;
                        if (zoneVisibleVolumeContains(zi, q))
                            crossed[zi] = 1;
                    }
                };
                for (size_t vi = 0; vi + 1 < probe.size(); ++vi) {
                    const Eigen::Vector3d &a = probe[vi];
                    const Eigen::Vector3d &b = probe[vi + 1];
                    const int n = std::max(
                        1, (int)std::ceil((b - a).norm() / 0.5));
                    for (int t = 0; t < n; ++t)
                        mark(a + (double)t / n * (b - a));
                }
                if (!probe.empty()) mark(probe.back());
                zone_soft_override_ = crossed;
                zone_hard_mode_ = true;
                fm2BuildSpeedMap();
                fm2SolveEikonal(end_pt, start_pt);
                bool hard3_ok =
                    fm2_valid_ && std::isfinite(fm2SampleT(start_pt));
                if (hard3_ok) {
                    const std::vector<Eigen::Vector3d> probe3 =
                        fm2ExtractGeodesic(start_pt, end_pt);
                    hard3_ok = probeRouteOk(probe3);
                }
                if (hard3_ok) {
                    zone_pass = 3;   // needed zones soft, the rest hard
                } else {
                    // Rare: hardening the rest re-blocked the route (the
                    // crossed set shifted the topology). Rebuild the plain
                    // soft field so the extracted path and every downstream
                    // consumer of fm2_F_/fm2_T_ see one consistent field.
                    zone_hard_mode_ = false;
                    fm2BuildSpeedMap();
                    fm2SolveEikonal(end_pt, start_pt);
                    zone_pass = 2;
                }
            }
            // Chord veto in EVERY lexico pass, not just the hard ones. On
            // the pass-2 soft field the raw geodesic still avoids every
            // visible volume it can (it only crosses the zone_soft_override_
            // set), but without the veto the SHORTCUT re-litigates that
            // avoidance: its risk margins accept chords that clip a zone the
            // raw route went around (observed: 337 km chain probe crossed
            // 0/10 zones, simplified path clipped two). insideHardZoneVol
            // skips override zones, so committed crossings simplify freely.
            zone_lexico_hard_ = (zone_pass >= 1);
            zone_avoid_pass_ = zone_pass;
            int n_soft = 0;
            for (char c : zone_soft_override_) n_soft += (c != 0);
            if (log_manager_)
                log_manager_->infof(
                    "[ZONE-AVOID] pass=%d (%s), soft-crossings=%d/%zu",
                    zone_pass,
                    zone_pass == 1 ? "zone-free route"
                    : zone_pass == 3 ? "wall crossed, rest re-hardened"
                                     : "soft fallback",
                    n_soft, risk_zones_->size());
        } else {
            fm2BuildSpeedMap();
            fm2SolveEikonal(end_pt, start_pt);
        }
        auto tf1 = rclcpp::Clock().now();
        if (fm2_valid_) fm2_path = fm2ExtractGeodesic(start_pt, end_pt);
        zone_hard_mode_ = false;   // never leak into later consumers
        auto tf2 = rclcpp::Clock().now();
        double fm2_zmin = 1e9, fm2_zmax = -1e9;
        for (const auto &p : fm2_path) {
            fm2_zmin = std::min(fm2_zmin, p.z());
            fm2_zmax = std::max(fm2_zmax, p.z());
        }
        if (log_manager_) {
            log_manager_->infof(
                "[FM2] grid=%dx%dx%d valid=%s eikonal=%.1fms geodesic=%.1fms "
                "wp=%zu start_risk=%.3f goal_risk=%.3f alpha=%.2f barrier=%.1f "
                "geo_z=[%.2f,%.2f] start_z=%.2f end_z=%.2f",
                fcnx_, fcny_, fcnz_, fm2_valid_ ? "ok" : "FAIL",
                (tf1 - tf0).seconds()*1000.0, (tf2 - tf1).seconds()*1000.0,
                fm2_path.size(), getRiskNorm(start_pt), getRiskNorm(end_pt),
                risk_alpha_, risk_barrier_,
                fm2_zmin, fm2_zmax, start_pt.z(), end_pt.z());
        }
        if (fm2_path.size() < 2) {
            // NOT a direct line. A two-point straight segment is a path
            // nobody checked for obstacles, and the caller accepts any
            // result with >= 2 points as a successful search — so returning
            // one converts "the search failed" into "here is your route",
            // precisely when a straight line is least likely to be safe.
            // Empty means failed, and planFrontEnd refuses the plan.
            if (log_manager_)
                log_manager_->errorf(
                    "[FM2] geodesic extraction failed — no path (a straight "
                    "line through unchecked space is not an answer)");
            return {};
        }
        fm2_path.front() = start_pt;
        fm2_path.back()  = end_pt;
        // [FM2-OCCUPANCY] The wave is allowed to burrow (kFMin porosity, see
        // fm2BuildSpeedMap); the ROUTE is not. If what came out touches
        // anything, throw the whole geodesic away. What follows is the A*
        // block, which in FM2 mode has no pool — it fails, and now returns
        // NO path rather than a straight line, so the mission is refused.
        // That ordering matters: while the search still answered a failure
        // with { start, end }, discarding here turned a burrowed detour into
        // a straight segment through the same wall, which is worse.
        Eigen::Vector3d hit;
        // RAW SEED: obstacles only. This is the split commit 9895396 made and
        // that the abort merge (86b4eac) undid by accident — mmp_dev branched
        // on 08-13, the split landed on 08-14, so taking "their" side of this
        // hunk silently reverted a later fix. Restored here.
        //
        // Why the split: the raw geodesic is a SEED, not a route. The
        // simplifier (:860+) and the terrain-lift sweep (:1206-1338, which
        // raises interior vertices and inserts climb vertices) exist precisely
        // to recover AGL, and both run AFTER this point. Judging terrain here
        // throws away a path that would have been clean two stages later —
        // measured then as "the r5 probe stopped planning entirely".
        //
        // Geometry is different: no lift gets a route out of a solid volume,
        // so obstacles still refuse at the raw stage. Terrain is judged on the
        // FINAL path at :1443, with the takeoff relief arc.
        //
        // The refusal here is TERMINAL under the shipped front_end: fm2 —
        // path_manager.cpp:1424 skips the A* pool allocation in fm2 mode, so
        // the else-branch below cannot produce a route. That is why the
        // criterion has to be the one no later stage can fix.
        if (polylineObstacleClear(fm2_path, &hit)) {
            fm2_done = true;
        } else if (log_manager_) {
            log_manager_->errorf(
                "[FM2] extracted geodesic is OCCUPIED at (%.2f, %.2f, %.2f) "
                "— discarding it; the speed map is porous by design, the "
                "route may not be",
                hit.x(), hit.y(), hit.z());
        }
    }

    bool search_success = false;
    vector<Vector3d> path;

  if (fm2_done) {
    path = std::move(fm2_path);
    search_success = true;
  } else {
    // Single algorithm, no mode switch. Build the coarse risk-aware
    // cost-to-go field rooted at the goal; the SMHA* inadmissible queue
    // uses it as a depression-structure-aware heuristic (Wilt&Ruml
    // SoCS2012; Holte hierarchical A*). The admissible anchor bounds
    // suboptimality so detour quality is preserved.
    auto t_cv0 = rclcpp::Clock().now();
    buildCoarseValueField(end_pt);
    auto t_cv1 = rclcpp::Clock().now();
    if (log_manager_) {
        log_manager_->infof(
            "[A* SMHA] start_risk=%.3f goal_risk=%.3f smha_w=%.2f "
            "alpha=%.2f coarse_field=%s (%.1fms, %dx%dx%d)",
            getRiskNorm(start_pt), getRiskNorm(end_pt),
            smha_w_, risk_alpha_,
            coarse_valid_ ? "ok" : "FALLBACK",
            (t_cv1 - t_cv0).seconds() * 1000.0,
            cnx_, cny_, cnz_);
    }

    // 3D A* search with ESDF. The path may legitimately start away from
    // start_pt when the start cell was occupied and got adjusted inside
    // AstarSearch, so no distance gate on path[0] — rejecting those cascades
    // to a straight line through the obstacle.
    if (AstarSearch(step_size, start_pt, end_pt)) {
        path = getPath();
        if (path.size() > 1) {
            if (log_manager_) {
                log_manager_->infof("드론 %d: 3D A* 검색 성공 (ESDF 사용) - 경로 점 개수: %zu", drone_id, path.size());
            }
            RCLCPP_INFO(rclcpp::get_logger("astar"), "3D A* search successful with ESDF");
            search_success = true;
        }
    }
    // (The old "retry without ESDF" pass was removed: use_esdf_check only
    // changed the log line, so the retry was a byte-identical search that
    // could only fail again — up to tens of seconds wasted per replan.)

    // Fallback: Z축 상승 후 재시도
    if (!search_success) {
        Vector3d elevated_end = end_pt;
        elevated_end(2) += 5.0;  // 5 z-units (~220 m real) 상승

        if (log_manager_) {
            log_manager_->warnf("드론 %d: 3D A* 재시도 실패, 목표점 Z축 상승 시도 (%.2f → %.2fm)",
                               drone_id, end_pt(2), elevated_end(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"),
                    "3D A* failed again, trying with elevated end point Z: %.2f -> %.2f",
                    end_pt(2), elevated_end(2));

        if (AstarSearch(step_size, start_pt, elevated_end)) {
            path = getPath();
            if (path.size() > 1) {
                if (log_manager_) {
                    log_manager_->infof("드론 %d: Z축 상승 후 A* 검색 성공 - 경로 점 개수: %zu", drone_id, path.size());
                }
                RCLCPP_INFO(rclcpp::get_logger("astar"), "3D A* search successful with elevated end point");
                search_success = true;
            }
        }
    }

    if (!search_success) {
        // Same rule as the FM2 branch above: no path is the answer when no
        // path was found. This used to return { start_pt, end_pt } — a
        // straight segment through whatever the search had just failed to
        // get around — and because the caller treats any result with >= 2
        // points as success, the mission planned and flew on it. The failure
        // was reported only in a log line nobody downstream reads.
        if (log_manager_) {
            log_manager_->errorf(
                "드론 %d: 3D A* 검색 완전 실패 — 경로 없음 (검사되지 않은 "
                "직선은 답이 아니다)", drone_id);
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"),
                     "3D A* search completely failed, returning NO path");
        return {};
    }
  }  // end else (A* search branch)

    // Snap the end-points to the exact caller-provided coordinates so the
    // downstream MINCO boundary conditions match. A* cells can be up to
    // sqrt(3)/2 * step_size off in arbitrary directions.
    if (!path.empty()) {
        path.front() = start_pt;
        path.back() = end_pt;
    }

    // ===========================================
    // 3D Path Simplification (based on temp code)
    // ===========================================

    if (log_manager_) {
        double min_z = path[0](2), max_z = path[0](2);
        for (const auto& p : path) {
            min_z = std::min(min_z, p(2));
            max_z = std::max(max_z, p(2));
        }
        log_manager_->infof("드론 %d: Raw 경로 Z 범위: %.3f ~ %.3f (변화량: %.3f)",
                           drone_id, min_z, max_z, max_z - min_z);
    }

    int size = path.size();
    if (size <= 2) {
        if (log_manager_) {
            log_manager_->warnf("드론 %d: 경로가 2점만 가지고 있음 - 단순화 불필요", drone_id);
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Path only has two points, no simplification needed");
        return path;
    }

    // Debug bypass: return the raw 1-voxel A* path verbatim. Used to verify
    // front-end risk-avoidance behavior independent of the shortcut filter.
    if (bypass_shortcut_) {
        if (log_manager_) {
            log_manager_->infof(
                "[A* SHORTCUT BYPASS] returning raw path verbatim (%zu wp)",
                path.size());
        }
        return path;
    }

    const auto t_sc0 = rclcpp::Clock().now();
    // Risk-aware shortcut (ported from the pre-A* RRT* pipeline).
    // Precompute cumulative edge cost (straight-line distance + Gaussian
    // risk integral) along the raw path. When trying to collapse points
    // i..j into a single straight segment, accept only if the segment is
    // (a) occupancy-free and (b) costs no more than 5 % over the detour
    // cost the A* search actually paid. This way:
    //   - clear corridors: fully shortcut to a straight line
    //   - risk detour    : original avoidance is preserved
    //   - required zone crossing: detour_cost ≈ shortcut_cost, shortcut allowed
    const double kShortcutMargin = 1.05;
    const int    kRiskSamples  = 4;  // trapezoidal samples per segment

    // The per-sample max-cost envelope guards below (segmentMaxRisk /
    // chordOccRisk / chordOk / innerChordBlocked) protect an FM2/A* detour
    // from being collapsed by a chord that spikes cost somewhere the coarse
    // 4-sample average misses. They were originally gated on zone presence,
    // but getRiskCost = alpha*moat + getRoughCost and the H2 roughness term
    // is ZONE-INDEPENDENT — so on a zone-free rough-terrain,
    // low-altitude terrain-following mission the
    // envelope was fully disabled and a chord could cut straight across a
    // narrow rough ridge FM2 had detoured around (header invariant "the
    // shortcut pass cannot revert an FM2 roughness detour"). Gate on cost
    // presence, not zone presence: getRiskCost already returns pure roughness
    // when zones are empty, so seg_max/detour_max then carry the roughness
    // envelope and the V3 filter applies uniformly.
    const bool have_zones = risk_zones_ && !risk_zones_->empty();
    const bool need_cost  = have_zones || (rough_weight_ > 0.0 && terrain_hgrad_);

    auto segmentRiskCost = [&](const Vector3d &a, const Vector3d &b) {
        double d = (b - a).norm();
        if (d < 1e-6) return 0.0;
        double sum = 0.0;
        for (int si = 0; si <= kRiskSamples; ++si) {
            double t = (double)si / (double)kRiskSamples;
            Vector3d p = a + t * (b - a);
            double w = (si == 0 || si == kRiskSamples) ? 0.5 : 1.0;
            // (1 + risk) multiplier form matches the RRT* reference; the
            // altitude-band term prices a chord that leaves the mission
            // altitude (long diagonal ramp) against the in-band detour it
            // replaces, so shortcutting preserves the hold-then-climb shape.
            sum += w * (1.0 + getRiskCost(p) + altBandCost(p.z()));
        }
        return d * sum / (double)kRiskSamples;
    };

    // V3 shortcut risk filter: a shortcut is permitted iff its max
    // risk sample does not exceed the A*-chosen detour's max risk by
    // more than kShortcutRiskMargin. This is scale-invariant in α
    // and lets forced-transit homotopies (encircling band) keep their
    // shortcuts.
    constexpr double kShortcutRiskMargin = 1.10;  // 10% slack

    auto segmentMaxRisk = [&](const Vector3d &a, const Vector3d &b) {
        if (!need_cost) return 0.0;
        // Pitch MUST match chordOccRisk's (terrain_stride_floor_, 0.15 u on
        // 30 m corridor maps — was hard-coded 0.5): a coarser detour scan
        // under-reads detour_max across narrow zone cores, misclassifying a
        // grazing detour as zero-risk and then holding candidate chords to
        // the strict zero-risk rule they can never meet (over-rejection).
        int n = std::max(1, (int)std::ceil((b - a).norm() / terrain_stride_floor_));
        double mx = 0.0;
        for (int k = 0; k <= n; ++k) {
            double t = (double)k / (double)n;
            Vector3d p = a + t * (b - a);
            mx = std::max(mx, getRiskCost(p));
        }
        return mx;
    };

    // Per-segment max risk along the original A* polyline.
    std::vector<double> seg_max(path.size(), 0.0);
    for (size_t k = 1; k < path.size(); ++k) {
        seg_max[k] = segmentMaxRisk(path[k - 1], path[k]);
    }

    std::vector<double> cum_cost(path.size(), 0.0);
    for (size_t k = 1; k < path.size(); ++k) {
        cum_cost[k] = cum_cost[k - 1] + segmentRiskCost(path[k - 1], path[k]);
    }

    // Single-pass chord check: occupancy + max-risk together, sampled in
    // bisection order (stride L/2, L/4, ... down to <=0.5). A failing chord
    // bails after a handful of coarse samples instead of paying the full
    // 0.5-spacing scan; a passing chord ends up sampled at the same <=0.5
    // density the old two-pass check used.
    auto chordOccRisk = [&](const Vector3d &a, const Vector3d &b,
                            double *max_risk_out) -> bool {
        const bool need_risk = need_cost;
        if (checkOccupancy_esdf(a) || checkOccupancy_esdf(b)) return false;
        // [ZONE-AVOID] after a hard pass the shortcut must not re-enter the
        // hard volume: the soft risk margins below would happily accept a
        // brief zone clip the wave was forbidden to make (observed leak:
        // pass=1 route, then "front-end route crosses zone" via a chord).
        if (zone_lexico_hard_ &&
            (insideHardZoneVol(a) || insideHardZoneVol(b))) return false;
        double mx = need_risk ? std::max(getRiskCost(a), getRiskCost(b)) : 0.0;
        const double len = (b - a).norm();
        if (len > 1e-9) {
            for (double stride = 0.5 * len; ; stride *= 0.5) {
                for (double sa = stride; sa < len; sa += 2.0 * stride) {
                    const Vector3d p = a + (sa / len) * (b - a);
                    if (checkOccupancy_esdf(p)) return false;
                    if (zone_lexico_hard_ && insideHardZoneVol(p)) return false;
                    if (need_risk) mx = std::max(mx, getRiskCost(p));
                }
                if (stride <= terrain_stride_floor_) break;
            }
        }
        if (max_risk_out) *max_risk_out = mx;
        return true;
    };

    // Full chord acceptance: occupancy-free + V3 max-risk filter + cost
    // margin (identical criteria to the old scan, evaluated once per chord).
    auto chordOk = [&](size_t a, size_t b) -> bool {
        if (b <= a + 1) return true;  // original segment, feasible by construction
        double shortcut_max = 0.0;
        if (!chordOccRisk(path[a], path[b], &shortcut_max)) return false;
        if (need_cost) {
            double detour_max = 0.0;
            for (size_t k = a + 1; k <= b; ++k)
                detour_max = std::max(detour_max, seg_max[k]);
            // If the A*-chosen sub-polyline was risk-free, the chord must be too.
            if (detour_max <= 1e-6) {
                if (shortcut_max > 1e-6) return false;
            } else if (shortcut_max > detour_max * kShortcutRiskMargin) {
                return false;
            }
        }
        const double shortcut_cost = segmentRiskCost(path[a], path[b]);
        const double detour_cost   = cum_cost[b] - cum_cost[a];
        return shortcut_cost <= detour_cost * kShortcutMargin;
    };

    // Greedy farthest-feasible, found by exponential probe + binary search
    // on the feasibility frontier: O(log n) chord tests per kept waypoint
    // instead of the old end-backwards scan (O(n) full-length tests per
    // waypoint, quadratic on curved sections where long chords keep
    // failing). Feasibility is not strictly monotone in chord length, so
    // around concave corners this can keep a point or two the exhaustive
    // scan would have dropped — the MINCO back-end smooths those anyway.
    std::vector<size_t> kept;
    kept.push_back(0);
    size_t i = 0;
    const size_t last = path.size() - 1;
    while (i + 1 < path.size()) {
        size_t farthest;
        if (chordOk(i, last)) {
            farthest = last;  // straight-corridor fast path
        } else {
            size_t ok = i + 1, bad = last;
            size_t step = 2;
            while (i + step < last && chordOk(i, i + step)) {
                ok = i + step;
                step <<= 1;
            }
            if (i + step < last) bad = i + step;
            while (ok + 1 < bad) {
                const size_t mid = ok + (bad - ok) / 2;
                if (chordOk(i, mid)) ok = mid;
                else bad = mid;
            }
            farthest = ok;
        }
        kept.push_back(farthest);
        i = farthest;
    }

    // Polish: feasibility is not monotone in chord length, so the frontier
    // search can keep redundant corner points — a 17-unit segment wedged
    // between 1000-unit ones. Such extreme spacing imbalance makes the MINCO
    // time allocation loiter (looping knots at the short pieces), so try
    // dropping each interior point and repeat until stable.
    {
        bool dropped = true;
        while (dropped && kept.size() > 2) {
            dropped = false;
            for (size_t n = 1; n + 1 < kept.size(); ++n) {
                if (chordOk(kept[n - 1], kept[n + 1])) {
                    kept.erase(kept.begin() + n);
                    dropped = true;
                    --n;
                }
            }
        }
    }

    // Corner-cut guard: the MINCO back-end smooths ACROSS kept corners,
    // flying near the midpoint->midpoint "inner chord" of adjacent chords
    // rather than the polyline itself. Chords are occupancy-checked above,
    // but the corner region between them is not — a sub-piece islet the
    // polyline detours around can sit exactly there. Observed: FM2 rounded a
    // 96 m spike (geodesic max z 88 m), the shortcut collapsed that detour to
    // a single corner between km-long chords, and the smoothed trajectory
    // crossed the spike crest at ~0 clearance, ringing the whole goal
    // approach. If an inner chord crosses occupied space, re-insert the
    // original-path vertex at the middle of the longer adjacent span:
    // shorter chords hug the detour, and locally shorter chords also mean
    // locally shorter pieces (smaller corner-cut depth) after subdivision.
    {
        const size_t kept_before_guard = kept.size();
        const bool need_risk = need_cost;
        // Inner chord blocked if it crosses OCCUPIED space (original check) OR
        // exceeds the raw detour's risk envelope (NEW): the guard used to be
        // occupancy-only, so at a detour apex between two chords that each
        // grazed a zone edge within the 1.10 margin, the mid->mid inner chord
        // — which MINCO actually flies — could cut through the zone CORE
        // unchecked. Mirror chordOk's V3 filter exactly: zero-risk detours
        // demand a zero-risk inner chord; risky detours bound the inner chord
        // at detour_max * kShortcutRiskMargin.
        auto innerChordBlocked = [&](size_t a, size_t b, size_t c) -> bool {
            const Vector3d m0 = 0.5 * (path[a] + path[b]);
            const Vector3d m1 = 0.5 * (path[b] + path[c]);
            const double len = (m1 - m0).norm();
            const int n =
                std::max(1, (int)std::ceil(len / terrain_stride_floor_));
            double detour_max = 0.0;
            if (need_risk) {
                for (size_t k = a + 1; k <= c; ++k)
                    detour_max = std::max(detour_max, seg_max[k]);
            }
            double inner_max = 0.0;
            for (int s = 0; s <= n; ++s) {
                const Vector3d p = m0 + (double(s) / n) * (m1 - m0);
                if (checkOccupancy_esdf(p)) return true;
                if (need_risk) inner_max = std::max(inner_max, getRiskCost(p));
            }
            if (need_risk) {
                // NOTE(candidate, not enabled): raising this 1e-6 zero-risk
                // razor to a meaningful moat value (~0.01*alpha) measured
                // -45% optimizer iterations on the complex-field stress case
                // (boundary-hug
                // flicker churn), but chordOk keeps its own 1e-6 razor and
                // the two filters would disagree across (1e-6, eps] —
                // promote only together with a chordOk-symmetric change.
                if (detour_max <= 1e-6) {
                    if (inner_max > 1e-6) return true;
                } else if (inner_max > detour_max * kShortcutRiskMargin) {
                    return true;
                }
            }
            return false;
        };
        // Integrated fixpoint: (a) corner scan re-inserts a raw vertex where
        // the inner chord is blocked; (b) validation scan re-checks every
        // multi-segment chord — guard insertions form chords (b,m),(m,c) that
        // never went through chordOk (an index midpoint on a curved raw span
        // can sit far off the old chord) and subdivides on failure. Both run
        // in the SAME loop so each other's insertions are re-examined; index
        // halving converges to raw adjacency, which is feasible by
        // construction. Original chords re-pass validation trivially.
        bool guard_clean = true;
        bool refined = true;
        int rounds = 0;
        while (refined && rounds++ < 8) {
            refined = false;
            for (size_t n = 0; n + 2 < kept.size(); ++n) {
                if (!innerChordBlocked(kept[n], kept[n + 1], kept[n + 2]))
                    continue;
                const size_t a = kept[n], b = kept[n + 1], c = kept[n + 2];
                const bool can_l = (b > a + 1), can_r = (c > b + 1);
                if (!can_l && !can_r) {
                    guard_clean = false;  // raw-dense corner still blocked
                    continue;
                }
                if (can_r && (!can_l || (c - b) >= (b - a))) {
                    kept.insert(kept.begin() + n + 2, b + (c - b) / 2);
                } else {
                    kept.insert(kept.begin() + n + 1, a + (b - a) / 2);
                }
                refined = true;
            }
            for (size_t n = 0; n + 1 < kept.size(); ++n) {
                const size_t u = kept[n], v = kept[n + 1];
                if (v <= u + 1) continue;
                double mx = 0.0;
                if (chordOccRisk(path[u], path[v], &mx)) continue;
                kept.insert(kept.begin() + n + 1, u + (v - u) / 2);
                refined = true;
                ++n;
            }
        }
        if (refined) guard_clean = false;  // round cap hit with work remaining

        if (log_manager_) {
            if (kept.size() != kept_before_guard) {
                log_manager_->infof(
                    "[A* SHORTCUT] corner-cut guard re-inserted %zu vertex(es) "
                    "(inner chord occupied/risk-exceeded)",
                    kept.size() - kept_before_guard);
            }
            if (!guard_clean) {
                // Mirror the sweep's capped-exit WARN: a silently skipped
                // still-blocked corner reaches the optimizer otherwise unlogged.
                log_manager_->warnf(
                    "[A* SHORTCUT] corner-cut guard capped/dense with blocked "
                    "inner chord(s) remaining — optimizer terms are the "
                    "remaining guard");
            }
        }
    }

    vector<Vector3d> simple_path;
    simple_path.reserve(kept.size());
    for (size_t n : kept) simple_path.push_back(path[n]);

    // Near-point dedup BEFORE the terrain sweep (was after — ordering bug:
    // the 0.3 u filter deleted sweep-inserted terrain-lift vertices, silently
    // re-opening the exact clearance violation the sweep repaired; observed
    // as the "simple=32 -> route=30" count mismatch on the full-map corridor).
    // Here it only cleans guard/polish near-duplicates; the sweep then runs
    // on final geometry and NOTHING may delete its vertices afterwards
    // (the post-sweep pass below merges with max-z instead of deleting).
    // Endpoints are commanded positions: never erase index 0 or last.
    {
        bool near_flag;
        do {
            near_flag = false;
            if (simple_path.size() <= 2) break;
            for (size_t i = 0; i + 1 < simple_path.size(); ++i) {
                if ((simple_path[i + 1] - simple_path[i]).norm() >= 0.3)
                    continue;
                if (i + 1 == simple_path.size() - 1) {
                    if (i == 0) break;              // only start+goal left
                    simple_path.erase(simple_path.begin() + i);  // keep goal
                } else {
                    simple_path.erase(simple_path.begin() + i + 1);
                }
                near_flag = true;
                break;
            }
        } while (near_flag);
    }

    // Terrain validation sweep over the FINAL polyline. Two holes the chord
    // machinery above cannot close: (a) chordOk trusts adjacent raw pairs
    // (b <= a+1) as "feasible by construction", but FM2 vetted those only at
    // coarse cell centres (fm2_coarse_k * DEM cell apart) — the segment
    // between them is never terrain-tested; (b) even a failing adjacent pair
    // would have no repair path, since there is no raw vertex to re-insert
    // between a and a+1. So instead of rejecting, REPAIR: sample every kept
    // segment at the DEM-scaled pitch and lift the worst clearance-violation
    // point to
    // terrain + margin as a new climb vertex, repeating until clean (same
    // spirit as the corner-cut guard's re-insertion loop above).
    // Terrain repair, at the SAME stride the final gate will judge with.
    sweepTerrainClearance(simple_path);

    // Post-sweep near-pair MERGE (terrain-safe replacement of the old delete
    // filter that ran here). The sweep may insert a lift vertex within 0.3 u
    // of a neighbor; deleting either would re-open the repaired clearance
    // violation,
    // but keeping sub-0.3 u segments recreates the documented MINCO loiter
    // hazard. So MERGE instead: erase one vertex of a near pair and raise the
    // INTERIOR survivor's z to the pair max — z-monotone, so the sweep's
    // terrain invariant is preserved. If the survivor would be a commanded
    // endpoint (whose z must not move), keep the pair unless the erased
    // vertex adds no height (z <= endpoint z): a rare tiny segment at an
    // endpoint is safer than losing a terrain lift.
    {
        bool near_flag;
        do {
            near_flag = false;
            if (simple_path.size() <= 2) break;
            for (size_t i = 0; i + 1 < simple_path.size(); ++i) {
                if ((simple_path[i + 1] - simple_path[i]).norm() >= 0.3)
                    continue;
                const size_t last_i = simple_path.size() - 1;
                if (i == 0) {                       // survivor = start
                    if (simple_path[1].z() > simple_path[0].z() + 1e-6)
                        continue;                   // lift next to start: keep
                    simple_path.erase(simple_path.begin() + 1);
                } else if (i + 1 == last_i) {       // survivor = goal
                    if (simple_path[i].z() > simple_path[last_i].z() + 1e-6)
                        continue;                   // lift next to goal: keep
                    simple_path.erase(simple_path.begin() + i);
                } else {                            // both interior: z-max merge
                    simple_path[i].z() =
                        std::max(simple_path[i].z(), simple_path[i + 1].z());
                    simple_path.erase(simple_path.begin() + i + 1);
                }
                near_flag = true;
                break;
            }
        } while (near_flag);
    }

    // z-profile logger: 21 rows (5% steps) of z + terrain-under, so the raw
    // geodesic and the shortcut output can be compared directly from the log
    // ("does FM2 dip into the band over water, and does the shortcut cut the
    // dip away?").
    auto logZProfileS = [&](const char *tag, const std::vector<Eigen::Vector3d> &pp) {
        if (!log_manager_ || pp.size() < 2) return;
        for (int pct = 0; pct <= 100; pct += 5) {
            const size_t idx = static_cast<size_t>(pct) * (pp.size() - 1) / 100;
            const Eigen::Vector3d &q = pp[idx];
            float hh = std::numeric_limits<float>::quiet_NaN();
            if (terrain_height_) hh = terrain_height_(q.x(), q.y());
            if (std::isfinite(hh)) {
                log_manager_->infof("[%s] %3d%% xy=(%7.1f,%7.1f) z=%6.3f terrain=%.3f",
                                    tag, pct, q.x(), q.y(), q.z(), hh);
            } else {
                log_manager_->infof("[%s] %3d%% xy=(%7.1f,%7.1f) z=%6.3f terrain=water",
                                    tag, pct, q.x(), q.y(), q.z());
            }
        }
    };
    logZProfileS("SIMPLE-PROFILE", simple_path);
    // Summary AFTER every mutation so the logged count equals the emitted
    // route (the old order printed simple=N, then the near filter deleted
    // vertices -> "simple=32 vs route=30" confusion in the full-map logs).
    if (log_manager_) {
        double max_risk_simple = 0.0;
        for (size_t k = 1; k < simple_path.size(); ++k) {
            max_risk_simple = std::max(max_risk_simple,
                                       segmentMaxRisk(simple_path[k-1], simple_path[k]));
        }
        log_manager_->infof(
            "[A* SHORTCUT] raw=%zu → simple=%zu cost_margin=%.2f "
            "risk_margin=%.2f max_risk_simple=%.3f (%.1f ms)",
            path.size(), simple_path.size(),
            kShortcutMargin, kShortcutRiskMargin, max_risk_simple,
            (rclcpp::Clock().now() - t_sc0).seconds() * 1000.0);
    }

    if (log_manager_) {
        log_manager_->infof("드론 %d: 3D 경로 단순화 완료 - %zu점 -> %zu점",
                           drone_id, path.size(), simple_path.size());

        double min_z = simple_path[0](2), max_z = simple_path[0](2);
        for (const auto& p : simple_path) {
            min_z = std::min(min_z, p(2));
            max_z = std::max(max_z, p(2));
        }
        log_manager_->infof("드론 %d: 단순화된 경로 Z 범위: %.3f ~ %.3f (변화량: %.3f)",
                           drone_id, min_z, max_z, max_z - min_z);
    }

    // [FM2-OCCUPANCY] THE FINAL PATH, judged whole. Everything that could
    // repair it has run by now: the shortcut, and the terrain-lift sweep that
    // raises and inserts vertices to recover ground clearance. What is left
    // is what the optimizer will be seeded with and what the aircraft will
    // approximately fly, so this is where "is it clear" is a fair question.
    //
    // Both kinds are refused here — terrain proximity and obstacles — with
    // the takeoff allowance applying only to a search that begins where the
    // aircraft is. The sweep above can hit its own caps and give up with a
    // warning; this is what turns that warning into a refusal instead of a
    // route nobody checked.
    {
        Eigen::Vector3d bad;
        if (!polylineClear(simple_path, &bad,
                           is_takeoff_leg ? startTerrainReliefArc() : 0.0)) {
            // [FM2-FINAL-CLEAR] Quantified, and on a channel this environment
            // actually shows. "The final route is not clear" is not
            // actionable; which KIND, how far along, and by how much is.
            {
                const bool terr = terrainClearanceShort(bad);
                const bool obst = obstacleBlocked(bad);
                double arc = 0.0;
                for (size_t i = 0; i + 1 < simple_path.size(); ++i) {
                    const double L = (simple_path[i + 1] - simple_path[i]).norm();
                    if ((simple_path[i] - bad).norm() +
                            (simple_path[i + 1] - bad).norm() <= L + 1e-6) {
                        arc += (simple_path[i] - bad).norm();
                        break;
                    }
                    arc += L;
                }
                const float th = terrain_height_ ? terrain_height_(bad.x(), bad.y())
                                                 : -1e30f;
                const double agl = bad.z() - static_cast<double>(th);
                RCLCPP_ERROR(rclcpp::get_logger("astar"),
                    "[FM2-FINAL-CLEAR] kind=%s arc=%.3f relief_end=%.3f "
                    "point=(%.2f,%.2f,%.3f) terrain_z=%.3f agl=%.3f "
                    "required=%.3f deficit=%.3f "
                    "sweep_pitch=%.3f gate_pitch=%.3f "
                    "lifted=%d sweep_clean=%s pts=%zu",
                    terr ? "TERRAIN" : (obst ? "OBSTACLE" : "OUTSIDE"),
                    arc, is_takeoff_leg ? startTerrainReliefArc() : 0.0,
                    bad.x(), bad.y(), bad.z(), (double)th, agl,
                    obstacle_margin_, obstacle_margin_ - agl,
                    dbg_sweep_pitch_, dbg_gate_pitch_, dbg_lifted_, 
                    dbg_sweep_clean_ ? "true" : "false", simple_path.size());
            }
            if (log_manager_)
                log_manager_->errorf(
                    "[FM2] the FINAL route is not clear at (%.2f, %.2f, %.2f) "
                    "— terrain lift could not recover it, or it meets an "
                    "obstacle; returning NO path",
                    bad.x(), bad.y(), bad.z());
            return {};
        }
    }
    return simple_path;
}

// ---------------------------------------------------------------------------
// Coarse risk-aware value-to-go field (SMHA* inadmissible heuristic).
//
// Downsample the active map span by coarse_k_ in every axis, run a
// Dijkstra from the goal cell using the SAME edge cost the fine A* uses
// (dist * (1 + alpha*risk)), and store cost-to-go per coarse cell.
// Obstacles: a coarse cell is blocked if its centre is occupied.
// ---------------------------------------------------------------------------
void PathSearcher::buildCoarseValueField(const Eigen::Vector3d &goal_world)
{
    coarse_valid_ = false;
    if (map_size_.minCoeff() <= 0.0) return;

    const double fine_res = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
    const double cres = fine_res * static_cast<double>(coarse_k_);

    cnx_ = std::max(1, static_cast<int>(std::ceil(map_size_.x() / cres)));
    cny_ = std::max(1, static_cast<int>(std::ceil(map_size_.y() / cres)));
    cnz_ = std::max(1, static_cast<int>(std::ceil(map_size_.z() / cres)));

    const size_t N = static_cast<size_t>(cnx_) * cny_ * cnz_;
    // Guard against pathological sizes (shouldn't happen with K>=4).
    if (N == 0 || N > 5'000'000) return;

    auto coarseCenter = [&](int ci, int cj, int ck) -> Eigen::Vector3d {
        return map_origin_ + Eigen::Vector3d(
            (ci + 0.5) * cres, (cj + 0.5) * cres, (ck + 0.5) * cres);
    };
    auto worldToCoarse = [&](const Eigen::Vector3d &w, Eigen::Vector3i &c) -> bool {
        const Eigen::Vector3d rel = (w - map_origin_) / cres;
        c = Eigen::Vector3i(static_cast<int>(std::floor(rel.x())),
                            static_cast<int>(std::floor(rel.y())),
                            static_cast<int>(std::floor(rel.z())));
        return (c.x() >= 0 && c.x() < cnx_ && c.y() >= 0 && c.y() < cny_ &&
                c.z() >= 0 && c.z() < cnz_);
    };

    Eigen::Vector3i gidx;
    if (!worldToCoarse(goal_world, gidx)) return;
    coarse_goal_idx_ = gidx;

    coarse_g_.assign(N, inf);

    // Precompute blocked mask (centre-occupied). Cheap: N coarse cells.
    std::vector<uint8_t> blocked(N, 0);
    for (int ck = 0; ck < cnz_; ++ck)
      for (int cj = 0; cj < cny_; ++cj)
        for (int ci = 0; ci < cnx_; ++ci) {
            const Eigen::Vector3d w = coarseCenter(ci, cj, ck);
            if (ground_height_ > -0.5 && w.z() < ground_height_) {
                blocked[coarseFlat(ci, cj, ck)] = 1; continue;
            }
            if (virtual_ceil_height_ > -0.5 && w.z() > virtual_ceil_height_) {
                blocked[coarseFlat(ci, cj, ck)] = 1; continue;
            }
            if (checkOccupancy_esdf(w))
                blocked[coarseFlat(ci, cj, ck)] = 1;
        }

    const int gflat = coarseFlat(gidx.x(), gidx.y(), gidx.z());
    if (blocked[gflat]) {
        // Goal cell occupied at coarse resolution: clear it so transit
        // is still represented (fine grid will refine).
        blocked[gflat] = 0;
    }

    using QItem = std::pair<double, int>;   // (cost-to-go, flat)
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    coarse_g_[gflat] = 0.0;
    pq.push({0.0, gflat});

    static const int off[26][3] = {
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
        {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
        {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1},
        {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
        {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}};
    static const double ocost[26] = {
        1,1,1,1,1,1,
        1.41421356,1.41421356,1.41421356,1.41421356,
        1.41421356,1.41421356,1.41421356,1.41421356,
        1.41421356,1.41421356,1.41421356,1.41421356,
        1.73205081,1.73205081,1.73205081,1.73205081,
        1.73205081,1.73205081,1.73205081,1.73205081};

    while (!pq.empty()) {
        auto [g, flat] = pq.top();
        pq.pop();
        if (g > coarse_g_[flat]) continue;   // stale

        const int ck = flat / (cnx_ * cny_);
        const int r  = flat - ck * (cnx_ * cny_);
        const int cj = r / cnx_;
        const int ci = r - cj * cnx_;

        for (int n = 0; n < 26; ++n) {
            const int ni = ci + off[n][0];
            const int nj = cj + off[n][1];
            const int nk = ck + off[n][2];
            if (ni < 0 || ni >= cnx_ || nj < 0 || nj >= cny_ ||
                nk < 0 || nk >= cnz_) continue;
            const int nf = coarseFlat(ni, nj, nk);
            if (blocked[nf]) continue;
            // Edge cost = dist * (1 + alpha*risk(midpoint)), same form as
            // the fine A*. dist is in metres (ocost * cres).
            const Eigen::Vector3d wa = coarseCenter(ci, cj, ck);
            const Eigen::Vector3d wb = coarseCenter(ni, nj, nk);
            const Eigen::Vector3d mid = 0.5 * (wa + wb);
            const double d_m = ocost[n] * cres;
            const double step_cost = d_m * (1.0 + getRiskCost(mid));
            const double ng = g + step_cost;
            if (ng < coarse_g_[nf]) {
                coarse_g_[nf] = ng;
                pq.push({ng, nf});
            }
        }
    }
    coarse_valid_ = true;

    // Diagnostics: how much of the coarse grid is blocked / reachable.
    size_t n_blocked = 0, n_reached = 0;
    for (size_t i = 0; i < N; ++i) {
        if (blocked[i]) ++n_blocked;
        if (std::isfinite(coarse_g_[i])) ++n_reached;
    }
    std::cerr << "[coarse] grid=" << cnx_ << "x" << cny_ << "x" << cnz_
              << " N=" << N << " blocked=" << n_blocked
              << " reached=" << n_reached
              << " goal_cell=(" << gidx.x() << "," << gidx.y() << ","
              << gidx.z() << ") goal_g=" << coarse_g_[gflat] << "\n";
}

double PathSearcher::coarseCostToGo(const Eigen::Vector3d &world) const
{
    if (!coarse_valid_) return -1.0;
    const double fine_res = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
    const double cres = fine_res * static_cast<double>(coarse_k_);
    const Eigen::Vector3d rel = (world - map_origin_) / cres;
    const int ci = static_cast<int>(std::floor(rel.x()));
    const int cj = static_cast<int>(std::floor(rel.y()));
    const int ck = static_cast<int>(std::floor(rel.z()));
    if (ci < 0 || ci >= cnx_ || cj < 0 || cj >= cny_ ||
        ck < 0 || ck >= cnz_) return -1.0;
    const double v = coarse_g_[coarseFlat(ci, cj, ck)];
    if (!std::isfinite(v)) return -1.0;     // unreachable at coarse res
    // coarse_g_ is in METRES of risk-weighted cost. The fine A* gScore /
    // anchor heuristic are in VOXEL units (static_cost = 1/√2/√3 per
    // step). Convert metres -> voxels by dividing by step_size_ so the
    // SMHA* dispatch `f_inadmis <= w * f_anchor` compares like with like.
    const double s = (step_size_ > 1e-6) ? step_size_ : 1.0;
    return tie_breaker_ * (v / s);
}

// ===========================================================================
// FM2 (Fast Marching Square) — heuristic-free Eikonal front-end.
//
// 1. fm2BuildSpeedMap : F(x) = 1/(1+alpha*risk(x)) in free space,
//                       F = kFMin on obstacles/ground/ceiling.
// 2. fm2SolveEikonal  : FMM from the goal; |∇T|·F = 1, Godunov upwind.
// 3. fm2ExtractGeodesic: gradient descent on T from start to goal.
//
// Grid: the active map span downsampled by fm2_coarse_k_. cres metres.
// ===========================================================================
namespace {
constexpr float kFMin = 1e-3f;          // blocked-cell speed (never 0)
// (An ESDF prox term used to multiply free-space speed here. Its "disabled"
// setting actually clamped every free cell to 0.05, collapsing the intended
// free:blocked speed ratio from 1000:1 to 50:1 — the wave could tunnel
// through thin obstacles, barrier zones became slower than real walls, and
// the FM2* heuristic (designed for F_max = 1) ran 20x weaker. Removed.)
}

void PathSearcher::fm2BuildSpeedMap()
{
    fm2_valid_ = false;
    // Every early return must clear BOTH the dims and the buffer: leaving the
    // new (larger) dims with last query's smaller fm2_F_ makes fm2SolveEikonal
    // index out of bounds (heap OOB write at the goal cell, OOB cudaMemcpy on
    // the GPU path), and leaving both stale silently solves on the previous
    // map's speed field.
    fm2_F_.clear();
    if (map_size_.minCoeff() <= 0.0) { fcnx_ = fcny_ = fcnz_ = 0; return; }

    const double fine_res = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
    const double fine_res_z = map_resolution_z_ > 1e-6 ? map_resolution_z_ : fine_res;
    const double cres = fine_res * static_cast<double>(fm2_coarse_k_);
    const double cres_z = fine_res_z * static_cast<double>(fm2_coarse_k_);
    fcnx_ = std::max(1, (int)std::ceil(map_size_.x() / cres));
    fcny_ = std::max(1, (int)std::ceil(map_size_.y() / cres));
    fcnz_ = std::max(1, (int)std::ceil(map_size_.z() / cres_z));
    const size_t N = (size_t)fcnx_ * fcny_ * fcnz_;
    if (N == 0 || N > fm2_max_cells_) { fcnx_ = fcny_ = fcnz_ = 0; return; }

    fm2_F_.assign(N, 1.0f);
    // Per-cell independent (distinct fm2_F_ writes; getRiskNorm/getDistance are
    // read-only) -> parallelise. On the k=1 grid this is ~8 s single-threaded.
    //
    // BULK GUARD: hold the SDF's dynamic-patch read lock ONCE for the whole
    // build and use the lock-free Bulk queries inside. Per-cell locking here
    // (2 rwlock RMWs x ~7e8 cells across OpenMP threads, all bouncing one
    // cacheline) measured ~60 s of pure lock traffic — the "eikonal" phase
    // went 8 s -> 68 s on the full-map mission. Writers (runtime obstacle
    // callbacks) simply wait out the build (~seconds), which is the correct
    // semantic anyway: a mid-build obstacle change would tear the speed map.
    const auto sdf_bulk_guard = sdf_ ? sdf_->bulkReadGuard() : nullptr;

    // COLUMN HOIST: the two DEM queries are std::function indirections that
    // read the heightmap at (x, y), so their answers are constant down a z
    // column — and this grid is ~110 cells deep, so resolving them per cell
    // issued 36.4M lookups where only fcnx_*fcny_ distinct columns exist.
    // Resolve the columns once here. The cell loop below keeps its i-fastest
    // order (fm2Flat = i + fcnx_*(j + fcny_*k)) so the write stream stays
    // sequential; hoisting by restructuring to a k-innermost loop instead
    // would stride 1.3 MB per step and give the cache back what the DEM
    // saved. The column table is fcnx_*fcny_ entries, a rounding error
    // against fm2_F_ itself.
    std::vector<TerrainCol> cols((size_t)fcnx_ * (size_t)fcny_);
    #pragma omp parallel for collapse(2) schedule(static)
    for (int j = 0; j < fcny_; ++j)
      for (int i = 0; i < fcnx_; ++i)
        cols[(size_t)j * fcnx_ + i] =
            terrainColumn(map_origin_.x() + (i + 0.5) * cres,
                          map_origin_.y() + (j + 0.5) * cres);

    long hard_blocked = 0;
    #pragma omp parallel for collapse(2) schedule(static) reduction(+:hard_blocked)
    for (int k = 0; k < fcnz_; ++k)
      for (int j = 0; j < fcny_; ++j)
        for (int i = 0; i < fcnx_; ++i) {
            const TerrainCol &col = cols[(size_t)j * fcnx_ + i];
            const Eigen::Vector3d w = map_origin_ + Eigen::Vector3d(
                (i + 0.5) * cres, (j + 0.5) * cres, (k + 0.5) * cres_z);
            bool blocked = false;
            if (ground_height_ > -0.5 && w.z() < ground_height_) blocked = true;
            else if (virtual_ceil_height_ > -0.5 && w.z() > virtual_ceil_height_) blocked = true;
            else if (checkOccupancyBulkCol_esdf(w, col)) blocked = true;
            if (blocked) {
                // kFMin porosity on purpose — in BOTH modes. It models
                // low-clearance terrain-following seam corridors (terrain-to-shadow
                // ceiling) are often thinner than a coarse cell, so a true
                // terrain wall (F=0) disconnects them at this resolution
                // (measured: every terrain-following mission fell back to
                // pass=2). The price is that a hard-pass wave can BURROW
                // under a zone wall through underground cells (huge but
                // finite T) — that leak is caught after extraction by the
                // below-terrain probe check in the [ZONE-AVOID] passes,
                // not here.
                fm2_F_[fm2Flat(i, j, k)] = kFMin;
            } else if (zone_hard_mode_ &&
                       insideHardZoneCell(w, 0.5 * cres, 0.5 * cres,
                                          0.5 * cres_z)) {
                ++hard_blocked;
                // [ZONE-AVOID] hard pass: the visible zone volume is
                // DISCONNECTED (F = 0, not kFMin) so reachability of the
                // goal doubles as the "does a zone-free route exist" test.
                fm2_F_[fm2Flat(i, j, k)] = 0.0f;
            } else {
                // Free-space speed: risk slowdown (alpha*risk + finite barrier K
                // inside non-exempt zones), modulated by obstacle distance.
                // Terrain roughness adds in its own currency ([ROUGH]).
                const double r = getRiskNorm(w);
                double risk_cost = risk_alpha_ * r + getRoughCostCol(w.z(), col);
                if (insideBarrierZone(w)) risk_cost += risk_barrier_;
                // Altitude-band penalty (see altBandCost): keeps the
                // geodesic at mission altitude over open water; it leaves the
                // band only where the in-band route is blocked by terrain.
                risk_cost += altBandCost(w.z());
                fm2_F_[fm2Flat(i, j, k)] =
                    (float)(1.0 / (1.0 + risk_cost));
            }
        }
    if (zone_hard_mode_ && log_manager_)
        log_manager_->infof("[ZONE-AVOID] hard-blocked %ld coarse cells", hard_blocked);
}

void PathSearcher::fm2SolveEikonal(const Eigen::Vector3d &goal_world,
                            const Eigen::Vector3d &start_world)
{
    // Require the buffer to match the dims exactly — a stale buffer from a
    // previous query with new dims would index out of bounds below.
    const size_t N = (size_t)fcnx_ * fcny_ * fcnz_;
    if (fm2_F_.empty() || fm2_F_.size() != N) return;
    const double fine_res = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
    const double fine_res_z = map_resolution_z_ > 1e-6 ? map_resolution_z_ : fine_res;
    const double cres = fine_res * static_cast<double>(fm2_coarse_k_);
    const double cres_z = fine_res_z * static_cast<double>(fm2_coarse_k_);
    const Eigen::Vector3d cellv(cres, cres, cres_z);

    Eigen::Vector3d rel = (goal_world - map_origin_).cwiseQuotient(cellv);
    int gi = (int)std::floor(rel.x());
    int gj = (int)std::floor(rel.y());
    int gk = (int)std::floor(rel.z());
    if (gi < 0 || gi >= fcnx_ || gj < 0 || gj >= fcny_ ||
        gk < 0 || gk >= fcnz_) return;

#ifdef PP_HAVE_CUDA
    // GPU eikonal (Fast Iterative Method): same Godunov as the CPU FMM below,
    // orders of magnitude faster on large grids. Solves the whole field (no
    // FM2* early stop needed). Any failure / missing device falls through to
    // the CPU FMM. fm2EikonalGPU writes a large sentinel for unreached cells;
    // convert it back to +inf so the geodesic sampler treats them as holes.
    // Debug: PP_FM2_FORCE_CPU=1 skips the GPU solver to compare solutions.
    const char *force_cpu = std::getenv("PP_FM2_FORCE_CPU");
    if (!(force_cpu && std::string(force_cpu) == "1") && fm2CudaAvailable()) {
      fm2_T_.resize(N);
      const int gpu_gflat = fm2Flat(gi, gj, gk);
      // fm2_F_ is a PERSISTENT buffer reused across queries and read by the
      // column/swath floor viz. Save the goal-cell speed, force it traversable
      // only for THIS solve, and restore afterwards so a temporary 0.5 never
      // leaks into a later query or the floor visualization. (GPU reads F,
      // never writes it, so post-call restore is safe.)
      const float gpu_f_orig = fm2_F_[gpu_gflat];
      if (fm2_F_[gpu_gflat] <= kFMin) fm2_F_[gpu_gflat] = 0.5f;
      const bool gpu_ok =
          fm2EikonalGPU(fm2_F_.data(), fcnx_, fcny_, fcnz_,
                        static_cast<float>(cres), static_cast<float>(cres),
                        static_cast<float>(cres_z), gi, gj, gk, fm2_T_.data());
      fm2_F_[gpu_gflat] = gpu_f_orig;  // restore regardless of GPU success
      if (gpu_ok) {
        const float inf = std::numeric_limits<float>::infinity();
        for (float &t : fm2_T_) if (t >= 1e17f) t = inf;
        fm2_valid_ = true;
        return;
      }
    }
#endif

    const float INF = std::numeric_limits<float>::infinity();
    fm2_T_.assign(N, INF);
    std::vector<uint8_t> frozen(N, 0);

    const int gflat = fm2Flat(gi, gj, gk);
    // Goal cell might be ESDF-blocked at coarse res; force it traversable
    // so the wave can still originate (fine path refines it). Save & restore
    // (after the FMM) so this 0.5 never leaks into fm2_F_ — a persistent
    // buffer reused by later queries and the column/swath floor viz.
    const float cpu_f_orig = fm2_F_[gflat];
    if (fm2_F_[gflat] <= kFMin) fm2_F_[gflat] = 0.5f;
    fm2_T_[gflat] = 0.0f;

    // FM2* causal-domain restriction: the wave terminates once the
    // start cell is frozen. Only cells on the goal->start corridor are
    // computed, in correct causal order, so the geodesic is preserved
    // ("same path" — Valero-Gomez et al.). Without this early stop the
    // T+h freeze order would corrupt the full field (FMM is order-
    // dependent: solveQuad reads frozen neighbours). When fm2_star_ is
    // false (plain FM2) sflat is unused and the wave runs to completion.
    Eigen::Vector3d srel = (start_world - map_origin_).cwiseQuotient(cellv);
    int si = (int)std::floor(srel.x());
    int sj = (int)std::floor(srel.y());
    int sk = (int)std::floor(srel.z());
    const bool s_in = (si >= 0 && si < fcnx_ && sj >= 0 && sj < fcny_ &&
                       sk >= 0 && sk < fcnz_);
    const int sflat = s_in ? fm2Flat(si, sj, sk) : -1;

    // FM2*: admissible cost-to-go heuristic. True remaining cost from a
    // cell to the start is integral(1/F) ds >= straight-line distance
    // (F <= 1 in free space, F_max = 1 at risk = 0). The Euclidean
    // distance to the start is thus an admissible, consistent lower
    // bound. It guides the wave toward the start; combined with the
    // early stop above, the computed corridor T (hence the geodesic) is
    // the same as plain FM2 while far fewer cells are expanded.
    auto heur = [&](int flat) -> float {
        if (!fm2_star_) return 0.0f;
        const int k = flat / (fcnx_ * fcny_);
        const int r = flat - k * (fcnx_ * fcny_);
        const int j = r / fcnx_;
        const int i = r - j * fcnx_;
        const Eigen::Vector3d w = map_origin_ + Eigen::Vector3d(
            (i + 0.5) * cres, (j + 0.5) * cres, (k + 0.5) * cres_z);
        return (float)(w - start_world).norm();
    };

    using HItem = std::pair<float, int>;   // (priority = T + h, flat)
    std::priority_queue<HItem, std::vector<HItem>, std::greater<HItem>> pq;
    pq.push({heur(gflat), gflat});

    // Weighted Godunov upwind solve, shared with the GPU FIM
    // (eikonal_godunov.h): sum_i ((T - m_i)/h_i)^2 = (1/F)^2 with
    // anisotropic spacings (hx = hy = cres, hz = cres_z).
    auto solveQuad = [&](int i, int j, int k) -> float {
        const int flat = fm2Flat(i, j, k);
        const float Fv = fm2_F_[flat];
        const float slow = 1.0f / std::max(Fv, 1e-6f);
        float m[3] = {kEikInf, kEikInf, kEikInf};
        auto consider = [&](int ax, int ni, int nj, int nk) {
            if (ni < 0 || ni >= fcnx_ || nj < 0 || nj >= fcny_ ||
                nk < 0 || nk >= fcnz_) return;
            const int nf = fm2Flat(ni, nj, nk);
            if (frozen[nf]) m[ax] = std::min(m[ax], fm2_T_[nf]);
        };
        consider(0, i-1, j, k); consider(0, i+1, j, k);
        consider(1, i, j-1, k); consider(1, i, j+1, k);
        consider(2, i, j, k-1); consider(2, i, j, k+1);
        const float T = eikSolve(m[0], m[1], m[2], (float)cres, (float)cres,
                                 (float)cres_z, slow);
        return (T >= kEikInf) ? INF : T;
    };

    while (!pq.empty()) {
        auto [t, flat] = pq.top();
        pq.pop();
        if (frozen[flat]) continue;
        frozen[flat] = 1;
        if (fm2_star_ && flat == sflat) break;  // start reached: stop

        const int k = flat / (fcnx_ * fcny_);
        const int r = flat - k * (fcnx_ * fcny_);
        const int j = r / fcnx_;
        const int i = r - j * fcnx_;

        static const int off[6][3] =
            {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
        for (auto &o : off) {
            const int ni = i + o[0], nj = j + o[1], nk = k + o[2];
            if (ni < 0 || ni >= fcnx_ || nj < 0 || nj >= fcny_ ||
                nk < 0 || nk >= fcnz_) continue;
            const int nf = fm2Flat(ni, nj, nk);
            if (frozen[nf]) continue;
            // [ZONE-AVOID] F = 0 means DISCONNECTED (hard zone volume) —
            // match the GPU semantics (live = F > 0) instead of crawling
            // through at 1/max(F,1e-6) slowness.
            if (fm2_F_[nf] <= 0.0f) continue;
            const float nt = solveQuad(ni, nj, nk);
            if (nt < fm2_T_[nf]) {
                fm2_T_[nf] = nt;
                pq.push({nt + heur(nf), nf});
            }
        }
    }
    fm2_F_[gflat] = cpu_f_orig;  // restore persistent speed buffer
    fm2_valid_ = true;
}

// Trilinear sample of fm2_T_. Values sit at cell centres ((i+0.5)*cres), so we
// index relative to centres and blend the 8 surrounding cells. Out-of-range /
// INF corners (obstacle, unreached) are dropped and the weights renormalised,
// keeping T continuous near holes (a continuous T is what keeps the geodesic
// gradient smooth instead of quantising to whole cells).
double PathSearcher::fm2SampleT(const Eigen::Vector3d &world) const
{
    if (!fm2_valid_) return std::numeric_limits<double>::infinity();
    const double fine_res = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
    const double fine_res_z = map_resolution_z_ > 1e-6 ? map_resolution_z_ : fine_res;
    const double cres = fine_res * static_cast<double>(fm2_coarse_k_);
    const double cres_z = fine_res_z * static_cast<double>(fm2_coarse_k_);
    const Eigen::Vector3d c =
        (world - map_origin_).cwiseQuotient(Eigen::Vector3d(cres, cres, cres_z))
        - Eigen::Vector3d(0.5, 0.5, 0.5);
    const int i0 = (int)std::floor(c.x());
    const int j0 = (int)std::floor(c.y());
    const int k0 = (int)std::floor(c.z());
    const double fx = c.x() - i0, fy = c.y() - j0, fz = c.z() - k0;
    double acc = 0.0, wsum = 0.0;
    for (int dk = 0; dk < 2; ++dk)
      for (int dj = 0; dj < 2; ++dj)
        for (int di = 0; di < 2; ++di) {
            const int i = i0 + di, j = j0 + dj, k = k0 + dk;
            if (i < 0 || i >= fcnx_ || j < 0 || j >= fcny_ ||
                k < 0 || k >= fcnz_) continue;
            const float v = fm2_T_[fm2Flat(i, j, k)];
            if (!std::isfinite(v)) continue;
            const double w = (di ? fx : 1.0 - fx) *
                             (dj ? fy : 1.0 - fy) *
                             (dk ? fz : 1.0 - fz);
            acc += w * (double)v;
            wsum += w;
        }
    return wsum > 1e-9 ? acc / wsum : std::numeric_limits<double>::infinity();
}

std::vector<Eigen::Vector3d> PathSearcher::fm2ExtractGeodesic(
    const Eigen::Vector3d &start_world,
    const Eigen::Vector3d &goal_world)
{
    std::vector<Eigen::Vector3d> path;
    if (!fm2_valid_) return path;
    // NOTE on sampling: strict-INF sampling near zone-hard cells was tried
    // (repel the descent from F=0 walls) and REGRESSED every rim-hugging
    // route — shadow seams and slalom gaps run within half a cell of the
    // hard volume, so strict samples poisoned the seam and the descent
    // aborted (r6/gauntlet fell to pass=2). Keep the renorm sampler: it
    // slides along rims by design, and a descent that still cannot reach
    // the goal is caught by the [ZONE-AVOID] extraction-success gate.
    const double fine_res = map_resolution_ > 1e-6 ? map_resolution_ : 1.0;
    const double fine_res_z = map_resolution_z_ > 1e-6 ? map_resolution_z_ : fine_res;
    const double cres = fine_res * static_cast<double>(fm2_coarse_k_);
    const double cres_z = fine_res_z * static_cast<double>(fm2_coarse_k_);
    const double step = 0.6 * cres;        // geodesic step length
    const double goal_tol = 1.5 * cres;
    const int max_iter = (int)((map_size_.norm() / step) * 4.0) + 1000;

    auto gradT = [&](const Eigen::Vector3d &p, Eigen::Vector3d &g) -> bool {
        // Per-axis probe distance = one grid cell of THAT axis, so the fine
        // z-structure of an anisotropic grid is not blurred by xy-sized probes.
        const double ex = cres, ez = cres_z;
        double tx0 = fm2SampleT(p - Eigen::Vector3d(ex,0,0));
        double tx1 = fm2SampleT(p + Eigen::Vector3d(ex,0,0));
        double ty0 = fm2SampleT(p - Eigen::Vector3d(0,ex,0));
        double ty1 = fm2SampleT(p + Eigen::Vector3d(0,ex,0));
        double tz0 = fm2SampleT(p - Eigen::Vector3d(0,0,ez));
        double tz1 = fm2SampleT(p + Eigen::Vector3d(0,0,ez));
        double tc  = fm2SampleT(p);
        if (!std::isfinite(tc)) return false;
        auto fb = [&](double a, double b, double c, double e) {
            // one-sided fallback if a neighbour is unreachable
            if (std::isfinite(a) && std::isfinite(b)) return (b - a) / (2*e);
            if (std::isfinite(b)) return (b - c) / e;
            if (std::isfinite(a)) return (c - a) / e;
            return 0.0;
        };
        g = Eigen::Vector3d(fb(tx0,tx1,tc,ex), fb(ty0,ty1,tc,ex), fb(tz0,tz1,tc,ez));
        return g.norm() > 1e-9;
    };

    Eigen::Vector3d p = start_world;
    path.push_back(p);

    // Discrete steepest-descent: step toward the lowest-T neighbour cell. The
    // eikonal field has no local minima (Valero-Gomez et al.), so a strictly
    // lower neighbour exists until the goal — this ALWAYS lowers T (monotone)
    // and follows the field AROUND a zone, never across it. Used to recover
    // whenever the smooth interp-gradient step stalls or overshoots.
    auto discreteStep = [&](const Eigen::Vector3d &pp, bool &ok) -> Eigen::Vector3d {
        static const int OFF[6][3] =
            {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
        double bestT = fm2SampleT(pp);
        // Move to the probed lowest-T neighbour ITSELF — per-axis cell-sized
        // steps. The old `pp + step * bdir.normalized()` was written for
        // isotropic voxels where step (0.6*cres) ~ the probe distance; with
        // anisotropic z (cres_z << cres) a z-directed recovery step probed
        // +-cres_z (0.1) but then jumped 0.6*cres (~1.4 units, ~13 cells)
        // UNGUARDED past the probed cell — observed as the geodesic plunging
        // to z=-0.53 (50+ m under the sea) on over-water missions. Stepping to
        // the probed point keeps the monotone guarantee exactly.
        Eigen::Vector3d bq = pp;
        ok = false;
        for (auto &o : OFF) {
            const Eigen::Vector3d q =
                pp + Eigen::Vector3d(o[0]*cres, o[1]*cres, o[2]*cres_z);
            const double tq = fm2SampleT(q);
            if (std::isfinite(tq) && tq < bestT) {
                bestT = tq; bq = q; ok = true;
            }
        }
        return bq;
    };
    // Keep z inside the grid (outside, the trilinear sample clamps and the
    // z-gradient degenerates to 0, so the path could drift below the floor).
    auto clampZ = [&](Eigen::Vector3d q) {
        q.z() = std::clamp(q.z(), map_origin_.z() + 0.5 * cres_z,
                           map_origin_.z() + map_size_.z() - 0.5 * cres_z);
        return q;
    };

    int n_recover = 0;   // steps that fell back to monotone discrete descent
    int n_grad_fail = 0, n_mono_rej = 0;  // why smooth steps failed (diagnosis)
    // Per-axis-aware step length: -grad(T) carries a z-component comparable to
    // xy wherever flying above the mission band (T prices the eventual
    // descent), so a FIXED step of 0.6*cres (~1.4 units) moved ~1 unit in z —
    // 10x the z-cell — punching through the whole free band into the blocked
    // sea in ONE step. The monotone guard then rejected it, every time: 96%
    // of steps ran the discrete recovery, whose 6-neighbour argmin can never
    // prefer a 0.1-unit z gain over a 2.3-unit xy gain — so altitude only
    // ratcheted UP at walls and never came back down over open water. Capping
    // |dz| per step at 0.6*cres_z lets the smooth follower actually integrate
    // the small persistent descent component; no artificial down-force, the
    // field itself decides. (Twin of the discreteStep anisotropy fix.)
    auto anisoStep = [&](const Eigen::Vector3d &from,
                         const Eigen::Vector3d &dir_unit,
                         double max_len) {
        double slen = max_len;
        if (std::abs(dir_unit.z()) > 1e-12) {
            slen = std::min(slen, 0.6 * cres_z / std::abs(dir_unit.z()));
        }
        return clampZ(from - slen * dir_unit);
    };
    // Fixed-wing slope shear: the field is direction-blind, so wherever its
    // gradient asks for a climb/dive steeper than the shared model's
    // flight-path-angle cap, shear the step onto that cone (keep the xy
    // heading, cap |dz|). A near-pure-vertical gradient (no horizontal
    // component to follow) is left alone — shrinking it to nothing would
    // stall the descent into the straight-chord fallback, a worse seed than
    // a short steep segment. The [GNRON] taper removes the one field regime
    // (endpoint moat funnel) that produced those verticals in practice.
    auto slopeLimit = [&](Eigen::Vector3d d) {
        if (geo_slope_tan_max_ <= 0.0) return d;
        const double hxy = d.head<2>().norm();
        if (hxy > 1e-9 && std::abs(d.z()) > geo_slope_tan_max_ * hxy) {
            d.z() = std::copysign(geo_slope_tan_max_ * hxy, d.z());
            d.normalize();
        }
        return d;
    };
    // Nearest-cell occupancy on the speed map (blocked cells carry F = kFMin;
    // free cells sit orders above it). Only the last-resort nudge needs this:
    // every other step is vetted by the monotone T-guard, which blocked cells
    // fail automatically (their T is huge or INF), but the nudge bypasses
    // that guard entirely — unchecked, it can tunnel a wall segment into the
    // seed path, and the shortcut later trusts adjacent raw pairs as
    // feasible-by-construction (b <= a+1 fast path).
    auto cellBlocked = [&](const Eigen::Vector3d &q) -> bool {
        const Eigen::Vector3d c = (q - map_origin_)
            .cwiseQuotient(Eigen::Vector3d(cres, cres, cres_z));
        const int i = (int)std::floor(c.x());
        const int j = (int)std::floor(c.y());
        const int k = (int)std::floor(c.z());
        if (i < 0 || i >= fcnx_ || j < 0 || j >= fcny_ ||
            k < 0 || k >= fcnz_) return true;  // off-grid = not traversable
        return fm2_F_[fm2Flat(i, j, k)] <= 2.0f * kFMin;
    };
    for (int it = 0; it < max_iter; ++it) {
        if ((p - goal_world).norm() < goal_tol) break;
        const double tp = fm2SampleT(p);
        Eigen::Vector3d p_next, g1;
        bool ok = false;
        if (gradT(p, g1)) {
            // RK2 (midpoint) smooth descent on the trilinear field.
            Eigen::Vector3d g2;
            const Eigen::Vector3d pmid =
                anisoStep(p, slopeLimit(g1.normalized()), 0.5 * step);
            const Eigen::Vector3d g = gradT(pmid, g2) ? g2 : g1;
            p_next = anisoStep(p, slopeLimit(g.normalized()), step);
            // Monotonicity guard: T strictly decreases along a geodesic. A step
            // that does NOT lower T overshot a narrow valley — the interp-gradient
            // zigzag that otherwise burns the whole iteration budget and then
            // forces a straight line-to-goal THROUGH zones at the end. Reject it.
            const double tn = fm2SampleT(p_next);
            ok = std::isfinite(tn) && tn < tp - 1e-9;
            if (!ok) ++n_mono_rej;
        } else {
            ++n_grad_fail;
        }
        if (!ok) {
            bool dok;
            p_next = clampZ(discreteStep(p, dok));
            if (!dok) {
                // Point-sampled recovery found no lower neighbour. That can
                // be a trilinear artifact: on FIM block-noise plateaus the
                // renorm blend at the CURRENT point reads below every
                // neighbour sample, which stalled long extractions
                // mid-route (observed: 337 km chain aborted at ~1900 u and
                // shipped a 1600 u straight tail chord THROUGH two zones).
                // Rescue on the RAW GRID: cell-to-cell descent is exactly
                // the FMM's causal order, so a strictly lower neighbour
                // CELL exists wherever T(cell) is finite and nonzero —
                // immune to interpolation artifacts by construction.
                const Eigen::Vector3d rc = (p - map_origin_)
                    .cwiseQuotient(Eigen::Vector3d(cres, cres, cres_z));
                const int ci = (int)std::floor(rc.x());
                const int cj = (int)std::floor(rc.y());
                const int ck = (int)std::floor(rc.z());
                auto cellT = [&](int i, int j, int k) -> double {
                    if (i < 0 || i >= fcnx_ || j < 0 || j >= fcny_ ||
                        k < 0 || k >= fcnz_)
                        return std::numeric_limits<double>::infinity();
                    return (double)fm2_T_[fm2Flat(i, j, k)];
                };
                const double tc0 = cellT(ci, cj, ck);
                double bt = tc0;
                int bi = ci, bj = cj, bk = ck;
                static const int OFF6[6][3] =
                    {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
                for (auto &o : OFF6) {
                    const int ni = ci + o[0], nj = cj + o[1], nk = ck + o[2];
                    const double tn = cellT(ni, nj, nk);
                    if (tn < bt) { bt = tn; bi = ni; bj = nj; bk = nk; }
                }
                if (bt < tc0 || (!std::isfinite(tc0) &&
                                 std::isfinite(bt))) {
                    // Land at the cell's xy center but KEEP the current z
                    // (clamped into the target cell's z slab): the cell-T
                    // monotone argument holds anywhere inside the cell, and
                    // snapping z to the center pokes a shadow-hugging route
                    // above the LOS ceiling (sigmoid width ~0.1 u < half a
                    // z-cell) — observed as a visible rim graze at q2=0.83
                    // that re-armed the crossing marker.
                    const double zlo = map_origin_.z() + bk * cres_z;
                    p_next = clampZ(Eigen::Vector3d(
                        map_origin_.x() + (bi + 0.5) * cres,
                        map_origin_.y() + (bj + 0.5) * cres,
                        std::clamp(p.z(), zlo + 0.05 * cres_z,
                                   zlo + 0.95 * cres_z)));
                    ++n_recover;
                    p = p_next;
                    path.push_back(p);
                    continue;
                }
                // No lower neighbour anywhere (numerical corner): last-resort
                // nudge toward the goal — gated on occupancy. Zones stay
                // nudgeable (traversable by design); walls/terrain/unreached
                // pockets abort the descent instead of tunnelling; the
                // reached_goal=NO warning below keeps the failure loud.
                const Eigen::Vector3d d = goal_world - p;
                if (d.norm() < 1e-6) break;
                p_next = clampZ(p + step * d.normalized());
                if (cellBlocked(p_next) ||
                    !std::isfinite(fm2SampleT(p_next))) {
                    fprintf(stderr,
                            "[GEODESIC] nudge into blocked/unreached cell at "
                            "(%.1f,%.1f,%.2f) — aborting descent\n",
                            p_next.x(), p_next.y(), p_next.z());
                    break;
                }
            }
            ++n_recover;
        }
        p = p_next;
        path.push_back(p);
    }
    fm2_geo_reached_goal_ = (p - goal_world).norm() < goal_tol;
    fprintf(stderr,
            "[GEODESIC] points=%zu recover_steps=%d (grad_fail=%d mono_rej=%d) reached_goal=%s\n",
            path.size(), n_recover, n_grad_fail, n_mono_rej,
            fm2_geo_reached_goal_ ? "yes" : "NO(timeout!)");
    path.push_back(goal_world);

    // Moving-average smoothing of z ONLY (endpoints pinned). The descent
    // leaves a small cell-scale sawtooth in z; averaging removes it. xy stays
    // exactly on the descent so risk-zone detours keep their horizontal
    // clearance — a full 3D average can cut corners INTO a zone. (Zones do
    // have a z-cap at |z - center.z| >= reach, so z-smoothing is not strictly
    // zone-neutral; but the window only moves z by a fraction of a cell,
    // while the cap sits hundreds of metres above any flown path.)
    if (path.size() > 8) {
        // Half-window sized to ONE coarse cell (the sawtooth's actual scale):
        // W = 4 was ±4*0.6*cres = ±2.4 CELLS of horizontal averaging — at
        // fm2_coarse_k=1 that planed narrow ridge crests down ~30 m (observed
        // 7.73 -> 7.43), which the alt-cap headroom then had to absorb. W = 2
        // (±1.2 cells) removes cell-scale sawtooth without carving crests.
        const int W = 2;  // half-window, in samples (step = 0.6*cres)
        std::vector<double> zs(path.size());
        for (size_t n = 0; n < path.size(); ++n) zs[n] = path[n].z();
        for (size_t n = 1; n + 1 < path.size(); ++n) {
            const int lo = std::max<int>(0, (int)n - W);
            const int hi = std::min<int>((int)path.size() - 1, (int)n + W);
            double acc = 0.0;
            for (int m = lo; m <= hi; ++m) acc += zs[m];
            path[n].z() = acc / double(hi - lo + 1);
        }
    }

    // z-profile logger: 21 rows (5% steps) of z + terrain-under, so the raw
    // geodesic and the shortcut output can be compared directly from the log
    // ("does FM2 dip into the band over water, and does the shortcut cut the
    // dip away?").
    auto logZProfile = [&](const char *tag, const std::vector<Eigen::Vector3d> &pp) {
        if (!log_manager_ || pp.size() < 2) return;
        for (int pct = 0; pct <= 100; pct += 5) {
            const size_t idx = static_cast<size_t>(pct) * (pp.size() - 1) / 100;
            const Eigen::Vector3d &q = pp[idx];
            float hh = std::numeric_limits<float>::quiet_NaN();
            if (terrain_height_) hh = terrain_height_(q.x(), q.y());
            if (std::isfinite(hh)) {
                log_manager_->infof("[%s] %3d%% xy=(%7.1f,%7.1f) z=%6.3f terrain=%.3f",
                                    tag, pct, q.x(), q.y(), q.z(), hh);
            } else {
                log_manager_->infof("[%s] %3d%% xy=(%7.1f,%7.1f) z=%6.3f terrain=water",
                                    tag, pct, q.x(), q.y(), q.z());
            }
        }
    };
    logZProfile("GEO-PROFILE", path);
    return path;
}

}} // namespace path_planner::astar
