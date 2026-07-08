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

    if (checkOccupancy(Index2Coord(start_idx)))
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
        } while (checkOccupancy(Index2Coord(start_idx)));
        if (log_manager_) {
            log_manager_->warnf("시작점 조정 완료 - 새로운 시작점: (%.2f,%.2f,%.2f)", start_pt(0), start_pt(1), start_pt(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New start point: (%f,%f,%f)", start_pt(0), start_pt(1), start_pt(2));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
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
        } while (checkOccupancy(Index2Coord(end_idx)));
        if (log_manager_) {
            log_manager_->warnf("도착점 조정 완료 - 새로운 도착점: (%.2f,%.2f,%.2f)", end_pt(0), end_pt(1), end_pt(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New END point: (%f,%f,%f)", end_pt(0), end_pt(1), end_pt(2));
    }

    return true;
}

bool PathSearcher::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_esdf_check)
{
    auto time_1 = rclcpp::Clock().now();
    ++rounds_;
    
    if (log_manager_) {
        log_manager_->infof("3D A* 검색 시작 - Round: %d, Step size: %.3f, ESDF 사용: %s",
                           rounds_, step_size, use_esdf_check ? "Yes" : "No");
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

    if ( start_pt(0) > -1 && start_pt(0) < 0 )
        cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

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

            {
                const Eigen::Vector3d nw = Index2Coord(neighborIdx);
                // Hard ground / ceiling gate. Always applied, even when
                // search_ignores_obstacles_ is set.
                if (ground_height_ > -0.5 && nw.z() < ground_height_) continue;
                if (virtual_ceil_height_ > -0.5 && nw.z() > virtual_ceil_height_) continue;
            }

            if (!search_ignores_obstacles_) {
                if(use_esdf_check){
                    if (checkOccupancy_esdf(Index2Coord(neighborIdx)))
                        continue;
                } else {
                    if (checkOccupancy(Index2Coord(neighborIdx)))
                        continue;
                }
            }

            double static_cost = neighbor_costs_ordered[i];

            // Risk-aware edge cost: distance * (1 + risk). Multiplicative
            // form keeps shorter paths cheaper inside risk regions.
            Eigen::Vector3d neigh_world = Index2Coord(neighborIdx);
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

vector<Vector3d> PathSearcher::astarSearchAndGetSimplePath(const double step_size, Vector3d start_pt, Vector3d end_pt, int drone_id){

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
        fm2BuildSpeedMap();
        fm2SolveEikonal(end_pt, start_pt);
        auto tf1 = rclcpp::Clock().now();
        if (fm2_valid_) fm2_path = fm2ExtractGeodesic(start_pt, end_pt);
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
            if (log_manager_)
                log_manager_->errorf("[FM2] geodesic failed, returning direct");
            return { start_pt, end_pt };
        }
        fm2_path.front() = start_pt;
        fm2_path.back()  = end_pt;
        fm2_done = true;
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
    if (AstarSearch(step_size, start_pt, end_pt, true)) {
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

        if (AstarSearch(step_size, start_pt, elevated_end, true)) {
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
        if (log_manager_) {
            log_manager_->errorf("드론 %d: 3D A* 검색 완전 실패 - 직선 경로 반환", drone_id);
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "3D A* search completely failed, returning direct path");

        vector<Vector3d> fallback_path;
        fallback_path.push_back(start_pt);
        fallback_path.push_back(end_pt);
        return fallback_path;
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
    //   - forced breakthrough: detour_cost ≈ shortcut_cost, shortcut allowed
    const double kShortcutMargin = 1.05;
    const int    kRiskSamples  = 4;  // trapezoidal samples per segment

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
        if (!risk_zones_ || risk_zones_->empty()) return 0.0;
        int n = std::max(1, (int)std::ceil((b - a).norm() / 0.5));
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
        const bool need_risk = risk_zones_ && !risk_zones_->empty();
        if (checkOccupancy_esdf(a) || checkOccupancy_esdf(b)) return false;
        double mx = need_risk ? std::max(getRiskCost(a), getRiskCost(b)) : 0.0;
        const double len = (b - a).norm();
        if (len > 1e-9) {
            for (double stride = 0.5 * len; ; stride *= 0.5) {
                for (double sa = stride; sa < len; sa += 2.0 * stride) {
                    const Vector3d p = a + (sa / len) * (b - a);
                    if (checkOccupancy_esdf(p)) return false;
                    if (need_risk) mx = std::max(mx, getRiskCost(p));
                }
                if (stride <= 0.5) break;
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
        if (risk_zones_ && !risk_zones_->empty()) {
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

    vector<Vector3d> simple_path;
    simple_path.reserve(kept.size());
    for (size_t n : kept) simple_path.push_back(path[n]);
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

    // Remove near points (3D distance)
    bool near_flag;
    do {
        near_flag = false;
        if (simple_path.size() <= 2) {
            break;
        }

        int num_same_check = simple_path.size();
        for (int i = 0; i < num_same_check - 1; i++) {
            double len = (simple_path[i+1] - simple_path[i]).norm();  // Full 3D norm
            if (len < 0.3) {
                simple_path.erase(simple_path.begin() + i + 1);
                near_flag = true;
                break;
            }
        }
    } while (near_flag);

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
            if (!search_ignores_obstacles_ && checkOccupancy_esdf(w))
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
    #pragma omp parallel for collapse(2) schedule(static)
    for (int k = 0; k < fcnz_; ++k)
      for (int j = 0; j < fcny_; ++j)
        for (int i = 0; i < fcnx_; ++i) {
            const Eigen::Vector3d w = map_origin_ + Eigen::Vector3d(
                (i + 0.5) * cres, (j + 0.5) * cres, (k + 0.5) * cres_z);
            bool blocked = false;
            if (ground_height_ > -0.5 && w.z() < ground_height_) blocked = true;
            else if (virtual_ceil_height_ > -0.5 && w.z() > virtual_ceil_height_) blocked = true;
            else if (!search_ignores_obstacles_ && checkOccupancy_esdf(w)) blocked = true;
            if (blocked) {
                fm2_F_[fm2Flat(i, j, k)] = kFMin;
            } else {
                // Free-space speed: risk slowdown (alpha*risk + finite barrier K
                // inside non-exempt zones), modulated by obstacle distance.
                const double r = getRiskNorm(w);
                double risk_cost = risk_alpha_ * r;
                if (insideBarrierZone(w)) risk_cost += risk_barrier_;
                // Altitude-band penalty (see altBandCost): keeps the
                // geodesic at mission altitude over open water; it leaves the
                // band only where the in-band route is blocked by terrain.
                risk_cost += altBandCost(w.z());
                fm2_F_[fm2Flat(i, j, k)] =
                    (float)(1.0 / (1.0 + risk_cost));
            }
        }
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
      if (fm2_F_[gpu_gflat] <= kFMin) fm2_F_[gpu_gflat] = 0.5f;
      if (fm2EikonalGPU(fm2_F_.data(), fcnx_, fcny_, fcnz_,
                        static_cast<float>(cres), static_cast<float>(cres),
                        static_cast<float>(cres_z), gi, gj, gk, fm2_T_.data())) {
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
    // so the wave can still originate (fine path refines it).
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
            const float nt = solveQuad(ni, nj, nk);
            if (nt < fm2_T_[nf]) {
                fm2_T_[nf] = nt;
                pq.push({nt + heur(nf), nf});
            }
        }
    }
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
    for (int it = 0; it < max_iter; ++it) {
        if ((p - goal_world).norm() < goal_tol) break;
        const double tp = fm2SampleT(p);
        Eigen::Vector3d p_next, g1;
        bool ok = false;
        if (gradT(p, g1)) {
            // RK2 (midpoint) smooth descent on the trilinear field.
            Eigen::Vector3d g2;
            const Eigen::Vector3d pmid = p - 0.5 * step * g1.normalized();
            const Eigen::Vector3d g = gradT(pmid, g2) ? g2 : g1;
            p_next = clampZ(p - step * g.normalized());
            // Monotonicity guard: T strictly decreases along a geodesic. A step
            // that does NOT lower T overshot a narrow valley — the interp-gradient
            // zigzag that otherwise burns the whole iteration budget and then
            // forces a straight line-to-goal THROUGH zones at the end. Reject it.
            const double tn = fm2SampleT(p_next);
            ok = std::isfinite(tn) && tn < tp - 1e-9;
        }
        if (!ok) {
            bool dok;
            p_next = clampZ(discreteStep(p, dok));
            if (!dok) {
                // No lower neighbour anywhere (numerical corner): last-resort
                // nudge toward the goal.
                const Eigen::Vector3d d = goal_world - p;
                if (d.norm() < 1e-6) break;
                p_next = clampZ(p + step * d.normalized());
            }
            ++n_recover;
        }
        p = p_next;
        path.push_back(p);
    }
    fprintf(stderr, "[GEODESIC] points=%zu recover_steps=%d reached_goal=%s\n",
            path.size(), n_recover,
            ((p - goal_world).norm() < goal_tol ? "yes" : "NO(timeout!)"));
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
    return path;
}

}} // namespace path_planner::astar
