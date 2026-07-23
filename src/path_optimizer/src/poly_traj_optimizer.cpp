#include "path_optimizer/poly_traj_optimizer.h"
#include <algorithm>
#include <iomanip>
#include <ctime>
#include <sys/resource.h>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace ego_planner
{
  void PolyTrajOptimizer::setLogManager(swarm_formation::LogManager::Ptr log_manager)
  {
    log_manager_ = log_manager;
  }

  bool PolyTrajOptimizer::optimizeFromPath(std::vector<Eigen::Vector3d> &clean_path,
                                           const Eigen::Vector3d &start_pos,
                                           const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &start_acc,
                                           const std::vector<Eigen::Vector3d> &waypoints,
                                           double max_vel,
                                           poly_traj::Trajectory &out_global,
                                           poly_traj::Trajectory &out_local,
                                           const std::vector<double> &cap_ref_z)
  {
    // Barrier exemptions for this plan (same start/goal rule as the
    // front-end): must be recomputed per plan since zones and endpoints
    // both change at runtime. Zones the front-end route itself crosses are
    // exempt too — the crossing is a committed front-end decision.
    prepareRiskBarrier(start_pos, waypoints.back());
    markFrontEndCrossings(clean_path);

    // Arc-varying cap reference: mirror EVERY clean_path insertion below on
    // this copy so vertex<->reference index correspondence holds by
    // construction. A size mismatch on entry means the caller's pairing is
    // broken — fall back to the scalar cap loudly rather than mis-index.
    std::vector<double> cap_ref = cap_ref_z;
    if (!cap_ref.empty() && cap_ref.size() != clean_path.size()) {
      LOG_WARN("[ALT-CAP] cap_ref size %zu != clean_path size %zu — "
               "falling back to scalar cap",
               cap_ref.size(), clean_path.size());
      cap_ref.clear();
    }

    // === MINCO initial trajectory from clean_path ===
    // Each shortcut vertex becomes one MINCO piece boundary directly;
    // clean_path is already densified so pieces stay roughly equal length.

    // Degenerate single-segment path → insert a midpoint so MINCO has >= 2 pieces.
    if (static_cast<int>(clean_path.size()) < 3) {
      Eigen::Vector3d mid = 0.5 * (clean_path.front() + clean_path.back());
      clean_path.insert(clean_path.begin() + 1, mid);
      if (!cap_ref.empty()) {
        // max(neighbours): erring permissive keeps the cap an upper bound.
        cap_ref.insert(cap_ref.begin() + 1, std::max(cap_ref[0], cap_ref[1]));
      }
    }

    // --- Initial-velocity lead-in ---
    // When start_vel is non-zero (in-flight replan), the head piece must leave along
    // start_vel. The front-end path ignores velocity, so the first segment can be long
    // (e.g. ~172 m) and point far from start_vel; the min-jerk head then arcs wildly to
    // reconcile the two, overshooting into risk/obstacle space -> a bad guess L-BFGS
    // can't recover (time-stretching the head was a no-op: that segment already spanned
    // ~86 s). Plant a short lead-in point ALONG start_vel so the head coasts out in the
    // direction the drone is already moving; the redirect then happens over the next,
    // normal segment. The lead-in is an inner point, so L-BFGS may relax it freely.
    bool lead_in_inserted = false;
    if (lead_in_time_ > 0.0 && start_vel.norm() > 1e-3) {
      lead_in_inserted = true;
      Eigen::Vector3d v_hat = start_vel.normalized();
      double d_lead = start_vel.norm() * lead_in_time_;  // distance travelled while redirecting
      Eigen::Vector3d p_lead = clean_path.front() + v_hat * d_lead;
      clean_path.insert(clean_path.begin() + 1, p_lead);
      if (!cap_ref.empty()) {
        cap_ref.insert(cap_ref.begin() + 1, std::max(cap_ref[0], cap_ref[1]));
      }
      if (log_manager_) {
        log_manager_->infof("[LEAD-IN] start_vel=%.2f m/s -> lead point +%.2f m along (%.2f,%.2f,%.2f)",
                            start_vel.norm(), d_lead, v_hat.x(), v_hat.y(), v_hat.z());
      }
    }

    int piece_num = static_cast<int>(clean_path.size()) - 1;

    // [H4-COMMIT] decision layer as seed authority (default off): rewrite
    // the inner z profile from the vertex-lattice corridor DP BEFORE any
    // consumer reads it, so the cap build's AGL fade, the ride/rough table,
    // the time allocation and the MINCO inner points all see ONE consistent
    // committed z. The lead-in vertex (dynamics-motivated, along start_vel)
    // and both endpoints stay pinned. Soft terms keep final authority.
    if (h4_commit_ && piece_num >= 2) {
      commitH4Profile(clean_path, start_pos, waypoints.back(),
                      lead_in_inserted ? 2 : 1);
    }

    // === Arc-varying altitude cap (per-piece ceiling) ===
    // Restores the z trust-region lost with the SFC corridor: the committed
    // FE profile governs the ceiling LOCALLY instead of one global scalar
    // licensing 100 m of dead band over open water. Steps:
    //  1. slope-cone envelope over the reference profile (1-D grayscale
    //     dilation, exact in two passes): env[i] = max_j(ref[j] −
    //     slope * dist_xy(i, j)). High values bleed sideways at
    //     alt_cap_slope_, licensing the physically necessary climb/descent
    //     anticipation; the envelope never clips the profile itself
    //     (env >= ref by the j = i term).
    //  2. per-piece cap = max of the piece's two boundary envelopes +
    //     headroom. Constant per piece w.r.t. decision variables, so the
    //     cap block's gradients keep their exact form.
    // Inner points can drift along the route during optimization; the cone's
    // sideways bleed is what makes a piece-index correspondence tolerant to
    // that drift (a hard window would cliff).
    alt_zhi_pieces_.resize(0);
    time_relief_pieces_.resize(0);
    ride_rough_pieces_.resize(0);
    if (!cap_ref.empty() && piece_num >= 1 &&
        static_cast<int>(cap_ref.size()) == piece_num + 1) {
      std::vector<double> env = cap_ref;
      for (int i = 1; i <= piece_num; ++i) {   // forward pass
        const double d = (clean_path[i] - clean_path[i - 1]).head<2>().norm();
        env[i] = std::max(env[i], env[i - 1] - alt_cap_slope_ * d);
      }
      for (int i = piece_num - 1; i >= 0; --i) { // backward pass
        const double d = (clean_path[i + 1] - clean_path[i]).head<2>().norm();
        env[i] = std::max(env[i], env[i + 1] - alt_cap_slope_ * d);
      }
      // SWATH-TERRAIN FLOOR on the cap reference. The FE-profile envelope
      // alone under-caps where the back-end deviates LATERALLY from the FE
      // route (risk-zone moats bend the trajectory off the committed xy):
      // if the FE dodged a hill sideways, its z at that arc is LOW, and the
      // cap then pins the water pieces FLANKING the hill so hard that the
      // hill-crossing piece cannot arch between its low endpoints — observed
      // as a converged 0.6-0.8 m graze on the serpentine mission (terrain
      // gate protects points NEAR terrain, not flank pins over water).
      // Contract completion: never cap below what terrain within the
      // corridor's lateral slack demands — cap_ref floor = max terrain
      // within kSwathR of the piece chord + the clearance band. Computed
      // once from clean_path (decision-variable independent, gradients keep
      // their exact form); over open water it adds nothing, so the hump
      // suppression is untouched, and the lift is LOCAL (a tall islet no
      // longer raises the whole route's ceiling — only its own +-kSwathR).
      const double kSwathR = 25.0;      // ~ one piece length of deviation slack
      // Sampling pitch = one DEM cell (2.3 u matched the 250 m korea grid;
      // corridor crops are 30-40 m, and a coarser-than-cell stride can step
      // clean over a one-cell ridge — the exact failure this floor guards).
      // Runs once per plan on a decision-variable-independent quantity, so
      // the finer pitch costs ~0.1-0.2 s worst case, not per-iteration time.
      const double kStepMax = 2.3;
      const double step = (terrain_cell_u_ > 0.0)
                              ? std::min(kStepMax, terrain_cell_u_)
                              : kStepMax;
      const double kStepS = step, kStepL = step;
      auto swath_terr = [&](const Eigen::Vector3d &a,
                            const Eigen::Vector3d &b) -> double {
        double hmax = -1e30;
        const Eigen::Vector2d ab(b.x() - a.x(), b.y() - a.y());
        const double L = ab.norm();
        const Eigen::Vector2d u = (L > 1e-9) ? Eigen::Vector2d(ab / L)
                                             : Eigen::Vector2d(1.0, 0.0);
        const Eigen::Vector2d n(-u.y(), u.x());
        for (double s = 0.0; s <= L + 1e-9; s += kStepS) {
          for (double l = -kSwathR; l <= kSwathR + 1e-9; l += kStepL) {
            const double x = a.x() + u.x() * s + n.x() * l;
            const double y = a.y() + u.y() * s + n.y() * l;
            if (terrain_hgrad_) {
              float h, gx, gy;
              if (terrain_hgrad_(x, y, &h, &gx, &gy) && h > hmax) hmax = h;
            } else if (terrain_height_) {
              const float h = terrain_height_(x, y);
              if (std::isfinite(h) && h > hmax) hmax = h;
            }
          }
        }
        return hmax;   // -1e30 over pure water
      };

      // [ZONE-RELAX] (z-redesign Stage 3) zone-proximity cap relaxation.
      // The altitude campaign quantified a global weight_altitude tradeoff:
      // tighter band tracking cuts ridge altitude_cost -81% but squeezes the
      // zone-crossing climb (kzone surplus margin 0.848 -> 0.535), because the
      // cap presses the arch back toward the committed profile exactly where
      // the risk field needs vertical freedom. The freedom is only needed NEAR
      // zones, so grant extra headroom there and nowhere else: relax * infl,
      // infl = max over zones of a linear fade of the piece chord's xy
      // distance to the zone center (1 inside the rim, 0 beyond 1.5x reach).
      // Computed once from clean_path — decision-variable independent, cap
      // gradients keep their exact form; zone-free (ridge NOE) missions are
      // untouched by construction. Default 0 = off (legacy cap).
      auto zone_infl = [&](const Eigen::Vector3d &a,
                           const Eigen::Vector3d &b) -> double {
        if (!use_risk_zones_ || alt_cap_zone_relax_ <= 0.0) return 0.0;
        double infl = 0.0;
        const Eigen::Vector2d p0 = a.head<2>(), p1 = b.head<2>();
        const Eigen::Vector2d ab = p1 - p0;
        const double L2 = ab.squaredNorm();
        for (const auto &tz : risk_zones_) {
          if (!(tz.reach > 0.0)) continue;
          const Eigen::Vector2d c = tz.center.head<2>();
          double t = (L2 > 1e-12) ? (c - p0).dot(ab) / L2 : 0.0;
          t = std::max(0.0, std::min(1.0, t));
          const double d = (p0 + t * ab - c).norm();
          const double s = 1.0 - (d - tz.reach) / (0.5 * tz.reach);
          infl = std::max(infl, std::max(0.0, std::min(1.0, s)));
        }
        return infl;
      };

      // [SHADOW-CAP] (H3) exposure-owned cap: sample the same swath the
      // terrain floor uses; at each point take the min over zones IN XY REACH
      // of the LOS shadow ceiling (PathManager's per-zone radial-horizon
      // table). Below (min - margin) the piece is hidden from every zone that
      // could reach it, so the band's downward pressure owns nothing there
      // and the cap may rise ("only low where visible"). A sample with an
      // in-reach zone but no finite ceiling (mask off, bare over-water ray,
      // at-center query) kills the min — a piece with ANY unshadowed in-reach
      // sample never relaxes, and pieces no zone can reach keep the legacy
      // band (its hump-suppression role is untouched). The swath, not just
      // the chord, keeps the license consistent with the lateral drift the
      // cap floor already budgets for: the trajectory may deviate kSwathR
      // sideways, so the relax must hold in the lit seams it could drift
      // into. Computed once from clean_path — decision-variable independent,
      // cap gradients keep their exact form.
      auto shadow_ceil = [&](const Eigen::Vector3d &a,
                             const Eigen::Vector3d &b) -> double {
        const double kNegInf = -std::numeric_limits<double>::infinity();
        if (!use_risk_zones_ || alt_cap_shadow_margin_ <= 0.0 ||
            !risk_shadow_ceiling_)
          return kNegInf;
        const Eigen::Vector2d p0 = a.head<2>(), p1 = b.head<2>();
        const Eigen::Vector2d ab = p1 - p0;
        const double L = ab.norm(), L2 = ab.squaredNorm();
        std::vector<size_t> cand;
        for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
          const auto &tz = risk_zones_[zi];
          if (!(tz.reach > 0.0)) continue;
          const Eigen::Vector2d c = tz.center.head<2>();
          double t = (L2 > 1e-12) ? (c - p0).dot(ab) / L2 : 0.0;
          t = std::max(0.0, std::min(1.0, t));
          if ((p0 + t * ab - c).norm() <= tz.reach + kSwathR) cand.push_back(zi);
        }
        if (cand.empty()) return kNegInf;
        const Eigen::Vector2d u = (L > 1e-9) ? Eigen::Vector2d(ab / L)
                                             : Eigen::Vector2d(1.0, 0.0);
        const Eigen::Vector2d n(-u.y(), u.x());
        double ceil_min = std::numeric_limits<double>::infinity();
        bool reached = false;
        for (double s = 0.0; s <= L + 1e-9; s += kStepS) {
          for (double l = -kSwathR; l <= kSwathR + 1e-9; l += kStepL) {
            const Eigen::Vector3d p(p0.x() + u.x() * s + n.x() * l,
                                    p0.y() + u.y() * s + n.y() * l, 0.0);
            for (size_t zi : cand) {
              const auto &tz = risk_zones_[zi];
              if ((p.head<2>() - tz.center.head<2>()).norm() > tz.reach)
                continue;
              reached = true;
              ceil_min = std::min(ceil_min, risk_shadow_ceiling_(zi, p));
              if (!(ceil_min > kNegInf)) return kNegInf;  // unshadowed sample
            }
          }
        }
        return reached ? ceil_min : kNegInf;
      };

      alt_zhi_pieces_.resize(piece_num);
      double cap_min = 1e30, cap_max = -1e30;
      int relaxed_pieces = 0;
      int shadow_pieces = 0;
      double shadow_lift_max = 0.0;
      for (int i = 0; i < piece_num; ++i) {
        double ref = std::max(env[i], env[i + 1]);
        const double terr = swath_terr(clean_path[i], clean_path[i + 1]);
        if (terr > -1e29) {
          ref = std::max(ref, terr + obstacle_clearance_);
        }
        const double zr =
            alt_cap_zone_relax_ * zone_infl(clean_path[i], clean_path[i + 1]);
        if (zr > 1e-9) ++relaxed_pieces;
        alt_zhi_pieces_(i) = ref + alt_cap_headroom_opt_ + zr;
        const double sc = shadow_ceil(clean_path[i], clean_path[i + 1]);
        if (std::isfinite(sc)) {
          const double cand_cap = sc - alt_cap_shadow_margin_;
          if (cand_cap > alt_zhi_pieces_(i)) {
            shadow_lift_max =
                std::max(shadow_lift_max, cand_cap - alt_zhi_pieces_(i));
            alt_zhi_pieces_(i) = cand_cap;
            ++shadow_pieces;
          }
        }
        cap_min = std::min(cap_min, alt_zhi_pieces_(i));
        cap_max = std::max(cap_max, alt_zhi_pieces_(i));
      }
      LOG_INFO("[ALT-CAP] arc-varying cap active: %d pieces, cap=[%.3f, %.3f] "
               "(slope=%.2f, headroom=%.2f, zone-relax=%.2f on %d pieces; "
               "scalar fallback %.3f)",
               piece_num, cap_min, cap_max, alt_cap_slope_,
               alt_cap_headroom_opt_, alt_cap_zone_relax_, relaxed_pieces,
               alt_zhi_);
      if (alt_cap_shadow_margin_ > 0.0 && use_risk_zones_) {
        LOG_INFO("[SHADOW-CAP] exposure-owned cap: %d/%d pieces lifted above "
                 "the band to LOS shadow - %.2f margin (max lift %.3f)",
                 shadow_pieces, piece_num, alt_cap_shadow_margin_,
                 shadow_lift_max);
      }
    } else if (!cap_ref.empty()) {
      LOG_WARN("[ALT-CAP] cap_ref/piece mismatch after insertions (%zu vs %d)"
               " — scalar cap only", cap_ref.size(), piece_num + 1);
    }

    // [H4] z-corridor decision-layer diagnostic (prototype, logging only —
    // the solve below is untouched). Stash cleared every plan so a skipped
    // diagnostic can never leave stale stations for the post-solve pass.
    h4_x_.clear(); h4_y_.clear(); h4_s_km_.clear(); h4_dp_z_.clear();
    h4_floor_.clear(); h4_floor_c_.clear(); h4_ceil_.clear();
    h4_status_.clear();
    if (h4_corridor_diag_ && log_manager_ && piece_num >= 2) {
      logH4ZCorridor(clean_path, start_pos, waypoints.back());
    }

    Eigen::MatrixXd innerPts(3, piece_num - 1);
    for (int i = 0; i < piece_num - 1; ++i) {
      innerPts.col(i) = clean_path[i + 1];
    }

    // [RIDE] (H1) per-piece time-weight relief over rough terrain. The time
    // cost is otherwise piece-uniform (wei_time * sum T_i), so the optimizer
    // has ZERO incentive to slow down locally — 200 m/s terrain-following
    // over a ridge demands the same time price as cruising over water. Relief
    // makes time spent on rough pieces cheaper: factor_i = 1 - relief *
    // rough_i with rough_i in [0, 1] from the mean slope excess along the
    // piece chord (deadband tan 0.2 ~ 11 deg, full at 0.8 ~ 39 deg — same
    // deadband as the FE [ROUGH] field). Frozen per solve from clean_path
    // (decision-variable independent; VirtualTGradCost keeps its exact
    // gradient form), consumed by BOTH the seed allocation below and the
    // running time cost. Default 0 = off (legacy uniform time price).
    if ((time_rough_relief_ > 0.0 || wei_ride_ > 0.0) &&
        (terrain_hgrad_ || terrain_height_)) {
      constexpr double kSlope0 = 0.2, kSlopeFull = 0.8;
      const double rstep = (terrain_cell_u_ > 0.0)
                               ? std::min(2.3, terrain_cell_u_)
                               : 2.3;
      ride_rough_pieces_.resize(piece_num);
      if (time_rough_relief_ > 0.0) time_relief_pieces_.resize(piece_num);
      int relieved = 0;
      double factor_min = 1.0;
      for (int i = 0; i < piece_num; ++i) {
        const Eigen::Vector3d &a = clean_path[i], &b = clean_path[i + 1];
        const double L = (b - a).head<2>().norm();
        double excess_sum = 0.0;
        int n = 0;
        for (double s = 0.0; s <= L + 1e-9; s += rstep) {
          const double t = (L > 1e-9) ? s / L : 0.0;
          const double x = a.x() + (b.x() - a.x()) * t;
          const double y = a.y() + (b.y() - a.y()) * t;
          float h = 0.f, gx = 0.f, gy = 0.f;
          if (terrain_hgrad_ && terrain_hgrad_(x, y, &h, &gx, &gy)) {
            const double slope = std::hypot(static_cast<double>(gx),
                                            static_cast<double>(gy));
            // AGL fade (same 1.0 -> 2.5 u band as the FE [ROUGH] field,
            // evaluated on the COMMITTED profile z — frozen per solve): a
            // high transit over a ridge is not terrain-following and gets
            // no relief; only the NOE regime does.
            constexpr double kAglNear = 1.0, kAglFar = 2.5;
            const double z = a.z() + (b.z() - a.z()) * t;
            const double agl = z - static_cast<double>(h);
            double fade = 1.0;
            if (agl >= kAglFar) fade = 0.0;
            else if (agl > kAglNear)
              fade = (kAglFar - agl) / (kAglFar - kAglNear);
            excess_sum += fade * std::max(0.0, slope - kSlope0);
          }
          ++n;
        }
        const double rough = std::min(
            1.0, (n > 0 ? excess_sum / n : 0.0) / (kSlopeFull - kSlope0));
        ride_rough_pieces_(i) = rough;
        if (rough > 1e-9) ++relieved;
        if (time_rough_relief_ > 0.0) {
          const double f = 1.0 - time_rough_relief_ * rough;
          time_relief_pieces_(i) = f;
          factor_min = std::min(factor_min, f);
        }
      }
      LOG_INFO("[RIDE] rough table: %d/%d pieces rough "
               "(relief=%.2f min factor %.3f, wei_ride=%.1f)",
               relieved, piece_num, time_rough_relief_, factor_min, wei_ride_);
    }

    const double des_vel = max_vel;
    Eigen::VectorXd time_vec(piece_num);
    for (int i = 0; i < piece_num; ++i) {
      double seg_len = (clean_path[i + 1] - clean_path[i]).norm();
      // Seed allocation matches the relieved time price: rough pieces start
      // slower instead of making L-BFGS discover the stretch by itself.
      const double f = (time_relief_pieces_.size() == piece_num)
                           ? time_relief_pieces_(i)
                           : 1.0;
      time_vec(i) = std::max(0.05, seg_len / (des_vel * std::max(0.2, f)));
    }

    // Arrival contract: full cruise speed at the goal (a separate control
    // planner takes over there) — but LEVEL. Pinning the tail's velocity to
    // the last chord's 3D direction gave it a DESCENT component whenever the
    // goal sits below a just-cleared spike, so the quintic had to thread an
    // S-curve at full speed and rang through the final kilometres. The
    // horizontal projection keeps the speed contract, hands the next planner
    // a level entry state, and lets the tail flare out of the final descent.
    Eigen::Vector3d approach_dir =
        clean_path.back() - clean_path[clean_path.size() - 2];
    approach_dir.z() = 0.0;
    if (approach_dir.norm() < 1e-6) {
        // Degenerate (near-vertical final chord): keep the 3D direction.
        approach_dir = clean_path.back() - clean_path[clean_path.size() - 2];
    }
    if (approach_dir.norm() < 1e-9) {
        // Coincident final points (e.g. start == goal): normalize() on a
        // zero vector would send NaN into the MINCO tail state silently.
        // Any level heading serves a zero-length approach.
        approach_dir = Eigen::Vector3d::UnitX();
    }
    approach_dir.normalize();
    Eigen::Vector3d traj_end_vel = approach_dir * max_vel;
    Eigen::Vector3d traj_end_acc = Eigen::Vector3d::Zero();

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), traj_end_vel, traj_end_acc;
    globalMJO.reset(headState, tailState, piece_num);
    globalMJO.generate(innerPts, time_vec);

    out_global = globalMJO.getTraj();

    // === L-BFGS optimization with SDF gradient penalty ===
    poly_traj::Trajectory initTraj = globalMJO.getTraj();

    // [FEASIBILITY] Pre-optimization z-feasibility advisory — the risk-effective
    // ceiling, run on the committed route BEFORE the solve. In the route's
    // terrain-following (NOE) stretches, weigh the risk visibility down-pull
    // against the terrain up-push at the committed height. Where risk wins, the
    // optimizer is forced to trade clearance for concealment (duck-below) — the
    // risk-driven -1004/graze class the geometric cap-floor corridor is blind to
    // (validated: risk 4x -> -1004 with a 20 m breach yet the geometric gap
    // stays open; see [VDIAG-ZDOF-RISK]). Advisory ONLY: it logs a reason and a
    // recommendation and changes nothing. Cheap (one route scan per plan). Not
    // diag-gated — this is operator-facing pre-flight information, always on.
    if (log_manager_ && use_risk_zones_ && (terrain_hgrad_ || terrain_height_)) {
      const double T = initTraj.getTotalDuration();
      const double m_xy = (dyn_unit_xy_m_ > 0.0) ? dyn_unit_xy_m_ : 100.0;
      // The terrain's full in-band restoring capacity: the largest up-force it
      // can raise before the surface (clearance 0), = wei_obs*3*clearance^2.
      // A risk pull beyond this cannot be held above terrain -> penetration.
      const double terr_cap =
          3.0 * wei_obs_ * obstacle_clearance_ * obstacle_clearance_;
      const int NS = 400;
      double s_km = 0.0;
      Eigen::Vector3d prev = initTraj.getPos(0.0);
      // Evaluate the risk pull at the SAFE band height (terrain + clearance),
      // not the seed's committed z: the seed has not ducked yet, so its own z
      // hides the conflict. The question is normative — "if the vehicle flew at
      // its safe clearance here, would the risk field let it stay?" The pull is
      // near its peak at the band (the detection sigmoid is steepest at the
      // grounded ceiling ~ terrain + band). risk_dn / terr_cap is then the
      // fraction of the terrain's entire restoring capacity the risk demands;
      // >= 1 means even the surface cannot hold it (penetration).
      double worst_ratio = 0.0, worst_s = 0.0, worst_risk = 0.0, worst_clr = 0.0;
      double ratio_sum = 0.0;
      int conflict = 0, severe = 0;
      for (int k = 0; k <= NS; ++k) {
        const double tt = std::min(k * (T / NS), T - 1e-6);
        const Eigen::Vector3d p = initTraj.getPos(tt);
        const Eigen::Vector3d vel = initTraj.getVel(tt);
        const double dx = p.x() - prev.x(), dy = p.y() - prev.y();
        s_km += std::sqrt(dx * dx + dy * dy) * m_xy / 1000.0;
        prev = p;
        double h = 0.0;
        bool land = false;
        if (terrain_hgrad_) {
          float hh = 0.f, gx = 0.f, gy = 0.f;
          land = terrain_hgrad_(p.x(), p.y(), &hh, &gx, &gy);
          if (land) h = hh;
        } else {
          const float hv = terrain_height_(p.x(), p.y());
          if (std::isfinite(hv)) { h = hv; land = true; }
        }
        if (!land) continue;
        const double clr_here = p.z() - h;
        // Only where the committed route is low enough that ducking is a real
        // option; a high cruise over a zone is never forced down to the band.
        if (clr_here > 3.0 * obstacle_clearance_) continue;
        // Risk down-pull evaluated at the safe band height.
        const Eigen::Vector3d p_band(p.x(), p.y(),
                                     h + obstacle_clearance_);
        Eigen::Vector3d gp, gv;
        double cp, risk_dn = 0.0;
        // gp.z() = d(risk cost)/dz >= 0 for the visibility down-pull (higher is
        // more visible); the downward FORCE magnitude is that positive gradient.
        // (VDIAG stores fz_risk = -gp.z() and re-negates; here gp is raw.) The
        // risk cost is an ARC-LENGTH integral, so its position gradient scales
        // with speed (gradp = |v|*...); pass the committed cruise velocity, not
        // zero, or the pull reads a spurious 0 (matches VDIAG, which uses vel).
        if (RiskGradCostP(0, p_band, vel, gp, gv, cp))
          risk_dn = std::max(0.0, gp.z());
        if (risk_dn <= 1e-6 || terr_cap <= 0.0) continue;
        // ratio = the risk down-pull at the SAFE band height / the terrain's
        // largest restoring force (at the surface). > 1 means the terrain
        // cannot hold the band against the pull's PEAK, so the route ducks.
        // This detects the duck-below robustly; it does NOT predict the settled
        // depth (the pull eases as the route sinks into shadow, so the solve
        // grazes rather than penetrates unless the pull is far above capacity —
        // calibrated: ~5x => graze/converge, ~20x => terrain breach).
        const double ratio = risk_dn / terr_cap;
        ratio_sum += ratio;
        if (ratio > 0.25) ++conflict;    // a meaningful pull at the safe height
        if (ratio > worst_ratio) {
          worst_ratio = ratio; worst_s = s_km;
          worst_risk = risk_dn; worst_clr = clr_here;
        }
      }
      if (conflict > 0) {
        const bool severe = worst_ratio > 8.0;
        LOG_WARN(
            "[FEASIBILITY] DUCK-BELOW EXPECTED (%s): the risk field pulls the "
            "route off its %.0f m clearance band in %d NOE sample(s) — worst "
            "@s=%.1fkm, peak pull %.1fx the terrain's restoring capacity. The "
            "optimizer will trade clearance for concealment here%s. Recommend: "
            "%s",
            severe ? "severe" : "moderate", obstacle_clearance_ * m_xy, conflict,
            worst_s, worst_ratio,
            severe ? " (the pull far exceeds what terrain can restore — expect a "
                     "clearance breach approaching penetration)"
                   : " (grazing the band, not penetration)",
            severe ? "re-route around these zones or cut their coverage/peak — "
                     "clearance cannot be held through the forced crossing"
                   : "accept the reduced clearance here, or shift the "
                     "clearance-vs-risk balance (weight_Risk down), or re-route");
      } else {
        LOG_INFO("[FEASIBILITY] FEASIBLE: terrain holds the clearance band along "
                 "the route (no risk-driven duck-below predicted).");
      }
    }

    // [INIT-PROFILE]: z of the INITIAL MINCO guess (pre-L-BFGS), same 5%-step
    // format as GEO/SIMPLE/TERRAIN-PROFILE. Splits head/tail transients on
    // sight: a hump already HERE was born in the min-jerk boundary/time
    // allocation (at-rest head + cruise-assumed seg_len/v times); a hump only
    // in the final trajectory was made by the optimizer's cost terms. Head
    // rows get finer 1% steps — the intermittent start hump lives in the
    // first few pieces and 5% (~20 km) steps straddle it.
    if (log_manager_) {
      const double T = initTraj.getTotalDuration();
      auto row = [&](double frac) {
        const double tt = std::min(frac * T, T - 1e-6);
        const Eigen::Vector3d p = initTraj.getPos(tt);
        const Eigen::Vector3d v = initTraj.getVel(tt);
        log_manager_->infof(
            "[INIT-PROFILE] %5.1f%% t=%7.1f xy=(%7.1f,%7.1f) z=%6.3f vz=%+.4f",
            100.0 * frac, tt, p.x(), p.y(), p.z(), v.z());
      };
      for (int pc = 0; pc < 5; ++pc) row(0.01 * pc);   // head, 1% steps
      for (int pc = 1; pc <= 20; ++pc) row(0.05 * pc); // rest, 5% steps
    }
    Eigen::MatrixXd cps = globalMJO.getInitConstrainPoints(cps_num_prePiece_);
    setControlPoints(cps);

    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd optInnerPts = all_pos.block(0, 1, 3, PN - 1);

    Eigen::MatrixXd optimal_points;
    bool use_formation = true;
    bool opt_success = OptimizeTrajectory_lbfgs(
        headState, tailState, optInnerPts, initTraj.getDurations(),
        optimal_points, use_formation);
    if (!opt_success) {
      return false;
    }

    out_local = jerkOpt_.getTraj();

    // [H4] post-solve leg of the corridor diagnostic: compare the flown z
    // against the pre-solve corridor and DP profile at the same stations.
    if (h4_corridor_diag_ && log_manager_ && !h4_x_.empty()) {
      logH4PostSolve(out_local);
    }

    return true;
  }

  void PolyTrajOptimizer::setupTerrainTaper(
      const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
  {
    const Eigen::Vector3d ends[2] = {start, goal};
    const double span = (goal.head<2>() - start.head<2>()).norm();
    // Default: full radius. The span clamp is applied AFTER the arming loop
    // below, once we know how many endpoints actually taper (the in-loop log
    // lines therefore print the BASE radius; a clamp emits its own log).
    terr_taper_len_eff_ = terrain_taper_len_;
    for (int e = 0; e < 2; ++e) {
      terr_taper_on_[e] = false;
      floor_taper_on_[e] = false;
      terr_taper_xy_[e] = ends[e].head<2>();

      // terrain-band taper (land endpoints pinned inside the clearance band)
      if (terrain_hgrad_ || terrain_height_) {
        float h = 0.f, gx = 0.f, gy = 0.f;
        bool land = false;
        if (terrain_hgrad_) {
          land = terrain_hgrad_(ends[e].x(), ends[e].y(), &h, &gx, &gy);
        } else {
          const float hv = terrain_height_(ends[e].x(), ends[e].y());
          if (std::isfinite(hv)) { h = hv; land = true; }
        }
        if (!land) {
          // Off-DEM endpoint: the accessor has no data there (returns false
          // since the off-map/water split). For taper ARMING ONLY, fall back
          // to sea level (h=0): an off-DEM pinned goal (min_goal_agl 0.15 <
          // band 0.30 makes this reachable) would otherwise get no taper at
          // all — the in-map coastal approach then demands the FULL band right
          // up to the DEM boundary while the terrain term vanishes outside
          // (cost cliff). Arming with h=0 fades the demanded clearance to the
          // commanded AGL across the approach, closing most of that cliff.
          // The terrain TERM itself still prices nothing off-DEM.
          h = 0.f;
          land = true;
        }
        if (land) {
          const double agl = ends[e].z() - static_cast<double>(h);
          if (agl < obstacle_clearance_) {
            terr_taper_on_[e] = true;
            terr_taper_agl_[e] = std::max(0.0, agl);  // underground request -> 0
            if (log_manager_) {
              log_manager_->infof(
                  "[TERRAIN-TAPER] %s pinned at %.3f u AGL < band %.2f u — "
                  "demanded clearance tapers to the commanded AGL within %.0f u",
                  e == 0 ? "start" : "goal", agl, obstacle_clearance_,
                  terr_taper_len_eff_);
            }
          }
        }
      }

      // altitude-floor taper (endpoint pinned below the scalar floor/cushion)
      if (wei_alt_ > 0.0 && ends[e].z() < alt_zlo_) {
        floor_taper_on_[e] = true;
        floor_taper_z_[e] = ends[e].z();
        if (log_manager_) {
          log_manager_->infof(
              "[TERRAIN-TAPER] %s pinned at z %.3f < altitude floor %.3f — "
              "floor tapers to the pin within %.0f u",
              e == 0 ? "start" : "goal", ends[e].z(), alt_zlo_,
              terr_taper_len_eff_);
        }
      }
    }

    // Span clamp — applied AFTER arming so it can key on how many endpoints
    // actually taper. Overlap into mid-span (the penetration risk the clamp
    // exists for) needs BOTH zones, so frac=0.25 there; a single armed zone
    // cannot overlap anything and may reach half the span (frac=0.5). Floor
    // the radius at 2.0 u so a short-span mission cannot collapse the zone to
    // ~nothing: taperedTarget's peak gradient is 1.5*(high-low)/len <=
    // 1.5*0.30/2.0 = 0.225/u at the floor (tame), and a couple of constraint
    // points always land inside the ramp. Long legs (span >> 4*len): no-op.
    const bool armed0 = terr_taper_on_[0] || floor_taper_on_[0];
    const bool armed1 = terr_taper_on_[1] || floor_taper_on_[1];
    if ((armed0 || armed1) && span > 1e-6) {
      const double frac = (armed0 && armed1) ? 0.25 : 0.5;
      const double clamped =
          std::min(terrain_taper_len_, std::max(2.0, frac * span));
      if (clamped < terr_taper_len_eff_) {
        terr_taper_len_eff_ = clamped;
        if (log_manager_) {
          log_manager_->infof(
              "[TERRAIN-TAPER] radius clamped %.0f -> %.1f u "
              "(span %.1f u, %s taper)",
              terrain_taper_len_, terr_taper_len_eff_, span,
              (armed0 && armed1) ? "both-end" : "single-end");
        }
      }
    }
  }

  // Shared taper ramp: low at the endpoint, high beyond `len`, smoothstep in
  // between. COST AND GRADIENT FROM THE SAME SURFACE (project hard rule — a
  // frozen approximation of a spatially-varying target de-syncs cost from
  // gradient and the line search dies, -1005/-1008): the analytic
  // d(target)/dxy is returned alongside. smoothstep has zero slope at BOTH
  // ends, so the taper adds no kinks of its own.
  static double taperedTarget(
      const Eigen::Vector2d &pos_xy, const Eigen::Vector2d &end_xy,
      double low, double high, double len, Eigen::Vector2d *grad_xy)
  {
    const Eigen::Vector2d dv = pos_xy - end_xy;
    const double d = dv.norm();
    if (d >= len) { grad_xy->setZero(); return high; }
    const double t = d / len;
    const double s = t * t * (3.0 - 2.0 * t);
    const double dsdt = 6.0 * t * (1.0 - t);
    if (d > 1e-9) {
      *grad_xy = (high - low) * dsdt / len * (dv / d);
    } else {
      grad_xy->setZero();
    }
    return low + (high - low) * s;
  }

  double PolyTrajOptimizer::altitudeFloorTarget(
      const Eigen::Vector3d &pos, Eigen::Vector2d *grad_xy) const
  {
    double floor_t = alt_zlo_;
    grad_xy->setZero();
    for (int e = 0; e < 2; ++e) {
      if (!floor_taper_on_[e]) continue;
      Eigen::Vector2d g;
      const double v = taperedTarget(pos.head<2>(), terr_taper_xy_[e],
                                     floor_taper_z_[e] - 0.02, alt_zlo_,
                                     terr_taper_len_eff_, &g);
      if (v < floor_t) { floor_t = v; *grad_xy = g; }
    }
    return floor_t;
  }

  double PolyTrajOptimizer::terrainClearanceTarget(
      const Eigen::Vector3d &pos, Eigen::Vector2d *grad_xy) const
  {
    double target = obstacle_clearance_;
    grad_xy->setZero();
    for (int e = 0; e < 2; ++e) {
      if (!terr_taper_on_[e]) continue;
      Eigen::Vector2d g;
      const double v = taperedTarget(pos.head<2>(), terr_taper_xy_[e],
                                     terr_taper_agl_[e], obstacle_clearance_,
                                     terr_taper_len_eff_, &g);
      if (v < target) { target = v; *grad_xy = g; }
    }
    return target;
  }

  // [H5] Freeze the per-inner-point z box from the SEED junctions. Must run
  // AFTER setupTerrainTaper (taper state arms the floor targets) and after the
  // cap build (alt_zhi_pieces_). lo = floor (max of terrain+clearance and the
  // mission anchor), hi = the looser of the two adjoining piece caps; box
  // collapse (floor>cap) raises hi only. All from seed xy — never a decision
  // variable. Sets h5_active_.
  void PolyTrajOptimizer::buildH5Bounds(const Eigen::MatrixXd &initInnerPts)
  {
    h5_active_ = false;
    h5_lo_.resize(0);
    h5_hi_.resize(0);
    if (!h5_bounded_z_) return;
    const int M = static_cast<int>(initInnerPts.cols());  // = piece_num_ - 1
    if (M < 1) return;
    const bool have_cap =
        (alt_zhi_pieces_.size() == piece_num_);
    Eigen::VectorXd lo(M), hi(M);
    int collapsed = 0;
    for (int i = 0; i < M; ++i) {
      const Eigen::Vector3d p = initInnerPts.col(i);
      // floor: terrain+clearance and/or mission anchor (mirror UNIFIED FLOOR)
      double terr_floor = -1e30;
      if (enable_obstacles_ && (terrain_hgrad_ || terrain_height_)) {
        float h = 0.f, gx = 0.f, gy = 0.f;
        bool ok = false;
        if (terrain_hgrad_) ok = terrain_hgrad_(p.x(), p.y(), &h, &gx, &gy);
        else { const float hv = terrain_height_(p.x(), p.y());
               if (std::isfinite(hv)) { h = hv; ok = true; } }
        if (ok) { Eigen::Vector2d g; terr_floor = static_cast<double>(h) +
                                     terrainClearanceTarget(p, &g); }
      }
      Eigen::Vector2d fg;
      const double anchor_floor =
          (wei_alt_ > 0.0) ? altitudeFloorTarget(p, &fg) : -1e30;
      double lo_i = std::max(terr_floor, anchor_floor);
      // ceiling: looser of the two adjoining piece caps (outer backstop; min
      // would erase a neighbour's shadow-lift / headroom license)
      double hi_i = have_cap ? std::max(alt_zhi_pieces_(i), alt_zhi_pieces_(i + 1))
                             : (alt_zhi_ >= 0.0 ? alt_zhi_ : 1e30);
      if (!(hi_i < 1e29)) { // no ceiling source -> cannot bound this solve
        LOG_WARN("[H5] no cap at inner point %d; H5 inactive this solve", i);
        return;
      }
      if (lo_i < -1e29) { // no floor source -> synthesize a wide lower bound
        lo_i = (ground_height_ > -0.5) ? ground_height_ : hi_i - h5_max_width_;
      }
      lo_i -= h5_floor_slack_;
      if (hi_i < lo_i + h5_min_width_) { hi_i = lo_i + h5_min_width_; ++collapsed; }
      lo(i) = lo_i;
      hi(i) = hi_i;
    }
    h5_lo_ = lo;
    h5_hi_ = hi;
    h5_D_.resize(M);
    h5_active_ = true;
    LOG_INFO("[H5] bounded-z active: %d junctions, %d box-collapsed (floor>cap, "
             "hi raised), min_width=%.2f floor_slack=%.2f",
             M, collapsed, h5_min_width_, h5_floor_slack_);
  }

  // [H4] z-corridor decision layer — DIAGNOSTIC ONLY (prototype).
  //
  // The eventual layer would sit between the FE (H2 routes around rough
  // terrain in xy) and the BE (H5 hard-boxes z): from the committed route it
  // derives the vertical corridor
  //   floor(s) = what safety demands (swath terrain + tapered clearance,
  //              altitude anchor / ground over water),
  //   ceil(s)  = what concealment allows (min over zones in xy reach of the
  //              LOS shadow ceiling − hidden margin),
  // and commits a climb-feasible z profile into clean_path before MINCO ever
  // sees it. This prototype MEASURES that layer without wiring it in — the
  // commit path (clean_path z) has a wide blast radius (taper arming,
  // fe_raw_max, the seed time allocation and the MINCO boundary states all
  // key on it), so instrumentation comes first.
  //
  // Three-state ceiling: the H3 [SHADOW-CAP] lambda collapses "no zone
  // reaches this swath" (FREE) and "an in-reach sample with no finite LOS
  // ceiling" (EXPOSED) into one −inf return — fine for a cap that only ever
  // relaxes, but the corridor must tell them apart: FREE transfers no
  // ceiling at all, EXPOSED means concealment is impossible at ANY altitude
  // there (such stations are counted, not depth-scored — with no finite
  // ceiling there is nothing to measure a depth against; the DP prices them
  // by altitude alone).
  //
  // Sentinel caveat (measured on k3): a ray with NO terrain blocker at all
  // keeps the mask's finite clear_ceiling fill (center.z − 2·max_range −
  // 20·softness − 1, ≈ −1e3 u; path_manager rebuildTerrainRiskMasks), not
  // −inf — those stations read as 'C' with a ceiling far below the chord
  // floor. Physically that is ground-visible, i.e. EXPOSED for flight
  // purposes, and any per-station depth SUM they enter is dominated by the
  // fill's magnitude. Where RAW-CLOSED depths reach kilometer scale, read
  // the exposure COUNTS, not the sums.
  //
  // Climb-rate closure: the h4_climb_slope two-pass dilation (the same
  // machinery as the alt_cap_slope cap envelope, at the vehicle's grade) IS
  // the one-shot feasibility pass for the "fly as low as safely possible"
  // objective —
  // F*(s) = max_j(floor(j) − slope·d(s,j)) is the lowest slope-feasible
  // profile and the matching erosion C*(s) = min_j(ceil(j) + slope·d(s,j))
  // the highest all-hidden one; F* > C* at a station means no z there is
  // both safe and hidden under the climb limit, whatever the rest of the
  // profile does. The discretized 1-D DP (z-grid, altitude + exposure +
  // climb costs, slope-limited transitions, endpoints pinned to the
  // committed z) generalizes the envelope where hidden and exposed
  // stretches trade off; DP-vs-F* divergence measures what the one-shot
  // pass leaves on the table.
  //
  // Calls setupTerrainTaper with the same endpoint args the solve passes
  // later (idempotent — the solve re-arms identical state; only the
  // [TERRAIN-TAPER] log lines repeat while the diagnostic is on).
  void PolyTrajOptimizer::logH4ZCorridor(
      const std::vector<Eigen::Vector3d> &clean_path,
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
  {
    const int N = static_cast<int>(clean_path.size()) - 1; // station per piece
    if (N < 2) return;
    setupTerrainTaper(start_pos, goal_pos);

    const double m_xy = (dyn_unit_xy_m_ > 0.0) ? dyn_unit_xy_m_ : 100.0;
    const double kInf = std::numeric_limits<double>::infinity();
    // Same swath geometry as the cap floor / [SHADOW-CAP]: the corridor must
    // hold under the lateral drift the cap already budgets for.
    const double kSwathR = 25.0;
    const double step = (terrain_cell_u_ > 0.0)
                            ? std::min(2.3, terrain_cell_u_)
                            : 2.3;

    std::vector<double> floor_v(N), floor_c(N), ceil_v(N, kInf), fe_z(N),
        st_s(N);
    std::vector<double> cap_v(N, kInf);
    std::vector<char> stat(N, 'F');
    int n_free = 0, n_ceil = 0, n_exp = 0;
    Eigen::Vector2d prev_mid(0.0, 0.0);
    h4_x_.reserve(N); h4_y_.reserve(N); h4_s_km_.reserve(N);
    h4_floor_.reserve(N); h4_ceil_.reserve(N);
    for (int i = 0; i < N; ++i) {
      const Eigen::Vector3d &a = clean_path[i], &b = clean_path[i + 1];
      const Eigen::Vector3d mid = 0.5 * (a + b);
      fe_z[i] = mid.z();
      st_s[i] = (i == 0) ? 0.0
                         : st_s[i - 1] + (mid.head<2>() - prev_mid).norm();
      prev_mid = mid.head<2>();

      const Eigen::Vector2d p0 = a.head<2>(), p1 = b.head<2>();
      const Eigen::Vector2d ab = p1 - p0;
      const double L = ab.norm(), L2 = ab.squaredNorm();
      const Eigen::Vector2d u = (L > 1e-9) ? Eigen::Vector2d(ab / L)
                                           : Eigen::Vector2d(1.0, 0.0);
      const Eigen::Vector2d nrm(-u.y(), u.x());

      // zones whose reach can touch the swath (chord distance, as SHADOW-CAP)
      std::vector<size_t> cand;
      if (use_risk_zones_ && risk_shadow_ceiling_) {
        for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
          const auto &tz = risk_zones_[zi];
          if (!(tz.reach > 0.0)) continue;
          const Eigen::Vector2d c = tz.center.head<2>();
          double t = (L2 > 1e-12) ? (c - p0).dot(ab) / L2 : 0.0;
          t = std::max(0.0, std::min(1.0, t));
          if ((p0 + t * ab - c).norm() <= tz.reach + kSwathR)
            cand.push_back(zi);
        }
      }

      // One swath walk: terrain max (floor) + LOS ceiling status. An
      // EXPOSED verdict stops further shadow queries but NEVER the terrain
      // walk — the floor must come from the complete swath.
      double hmax = -1e30, ceil_min = kInf;
      bool reached = false, exposed = false;
      for (double s = 0.0; s <= L + 1e-9; s += step) {
        for (double l = -kSwathR; l <= kSwathR + 1e-9; l += step) {
          const double x = p0.x() + u.x() * s + nrm.x() * l;
          const double y = p0.y() + u.y() * s + nrm.y() * l;
          if (terrain_hgrad_) {
            float h, gx, gy;
            if (terrain_hgrad_(x, y, &h, &gx, &gy) && h > hmax) hmax = h;
          } else if (terrain_height_) {
            const float h = terrain_height_(x, y);
            if (std::isfinite(h) && h > hmax) hmax = h;
          }
          if (!exposed && !cand.empty()) {
            const Eigen::Vector3d p(x, y, 0.0);
            for (size_t zi : cand) {
              const auto &tz = risk_zones_[zi];
              if ((p.head<2>() - tz.center.head<2>()).norm() > tz.reach)
                continue;
              reached = true;
              const double v = risk_shadow_ceiling_(zi, p);
              if (!(v > -kInf)) { exposed = true; break; } // unshadowed
              ceil_min = std::min(ceil_min, v);
            }
          }
        }
      }

      // Chord-only terrain max (l = 0): the flown path tracks LOCAL terrain,
      // so safety comparisons against the flown z use this floor; the swath
      // max above is the drift-budget COMMIT floor (over ridge NOE the two
      // differ by several units — conflating them misread "flown below the
      // swath floor" as a safety breach on first measurement).
      double hmax_c = -1e30;
      for (double s = 0.0; s <= L + 1e-9; s += step) {
        const double x = p0.x() + u.x() * s;
        const double y = p0.y() + u.y() * s;
        if (terrain_hgrad_) {
          float h, gx, gy;
          if (terrain_hgrad_(x, y, &h, &gx, &gy) && h > hmax_c) hmax_c = h;
        } else if (terrain_height_) {
          const float h = terrain_height_(x, y);
          if (std::isfinite(h) && h > hmax_c) hmax_c = h;
        }
      }

      Eigen::Vector2d gdum;
      double fl = -1e30, flc = -1e30;
      if (hmax > -1e29) fl = hmax + terrainClearanceTarget(mid, &gdum);
      if (hmax_c > -1e29) flc = hmax_c + terrainClearanceTarget(mid, &gdum);
      if (wei_alt_ > 0.0) {
        const double af = altitudeFloorTarget(mid, &gdum);
        fl = std::max(fl, af);
        flc = std::max(flc, af);
      }
      const double fallback = (ground_height_ > -0.5) ? ground_height_ : 0.0;
      if (fl < -1e29) fl = fallback;
      if (flc < -1e29) flc = fallback;
      floor_v[i] = fl;
      floor_c[i] = flc;

      if (exposed)      { stat[i] = 'E'; ceil_v[i] = -kInf; ++n_exp; }
      else if (reached) { stat[i] = 'C';
                          ceil_v[i] = ceil_min - h4_shadow_margin_; ++n_ceil; }
      else              { stat[i] = 'F'; ceil_v[i] = kInf; ++n_free; }

      cap_v[i] = (alt_zhi_pieces_.size() == N)
                     ? alt_zhi_pieces_(i)
                     : (alt_zhi_ >= 0.0 ? alt_zhi_ : kInf);

      h4_x_.push_back(mid.x());
      h4_y_.push_back(mid.y());
      h4_s_km_.push_back(st_s[i] * m_xy / 1000.0);
      h4_floor_.push_back(floor_v[i]);
      h4_floor_c_.push_back(floor_c[i]);
      h4_ceil_.push_back(ceil_v[i]);
    }
    h4_status_.assign(stat.begin(), stat.end());

    // Climb-rate envelopes (two-pass dilation/erosion, exact). FREE and
    // EXPOSED stations transfer no ceiling into the erosion: FREE has none,
    // and EXPOSED cannot be hidden at any z, so a −inf there must not poison
    // its hidable neighbours.
    const double smax = std::max(1e-6, h4_climb_slope_);
    std::vector<double> F(floor_v), C(N);
    for (int i = 0; i < N; ++i) C[i] = (stat[i] == 'C') ? ceil_v[i] : kInf;
    for (int i = 1; i < N; ++i) {
      const double d = st_s[i] - st_s[i - 1];
      F[i] = std::max(F[i], F[i - 1] - smax * d);
      C[i] = std::min(C[i], C[i - 1] + smax * d);
    }
    for (int i = N - 2; i >= 0; --i) {
      const double d = st_s[i + 1] - st_s[i];
      F[i] = std::max(F[i], F[i + 1] - smax * d);
      C[i] = std::min(C[i], C[i + 1] + smax * d);
    }

    int n_raw = 0, n_dil = 0, n_finC = 0;
    double raw_depth = 0.0, raw_s = 0.0, dil_depth = 0.0, dil_s = 0.0;
    double gap_min = kInf, gap_min_s = 0.0;
    for (int i = 0; i < N; ++i) {
      if (stat[i] == 'C' && floor_v[i] > ceil_v[i]) {
        ++n_raw;
        if (floor_v[i] - ceil_v[i] > raw_depth) {
          raw_depth = floor_v[i] - ceil_v[i]; raw_s = st_s[i];
        }
      }
      // Dilated-closure accounting only at stations that actually carry a
      // hiding requirement ('C'): the erosion legitimately propagates a
      // ceiling THROUGH a FREE/EXPOSED station, but the all-hidden profile
      // z=F* only has to satisfy z<=ceil at 'C' stations, so counting an
      // eroded C* elsewhere would over-report the closure.
      if (stat[i] == 'C') {
        ++n_finC;
        const double gap = C[i] - F[i];
        if (gap < gap_min) { gap_min = gap; gap_min_s = st_s[i]; }
        if (gap < 0.0) {
          ++n_dil;
          if (-gap > dil_depth) { dil_depth = -gap; dil_s = st_s[i]; }
        }
      }
    }

    // 1-D DP over a shared z-grid: altitude above floor + exposure above the
    // ceiling + climb effort, transitions limited to the climb cone,
    // endpoints pinned to the committed z (start/goal are not free).
    constexpr int NZ = 64;
    constexpr double W_ALT = 1.0, W_EXP = 10.0, W_CLIMB = 0.1;
    const double kBig = 1e18;
    // Grid range from floors and the committed profile ONLY. Ceilings are
    // deliberately excluded: a ground-visible forward slope carries a finite
    // LOS ceiling far below terrain (hundreds of units on the first k3
    // measurement), and letting it stretch the grid destroyed the DP's
    // resolution (dz jumped to 18 u). Exposure depth (z − ceil) needs no
    // grid coverage — it is monotone in z either way.
    double zlo = kInf, zhi = -kInf;
    for (int i = 0; i < N; ++i) {
      zlo = std::min(zlo, std::min(floor_v[i], fe_z[i]));
      zhi = std::max(zhi, std::max(floor_v[i], fe_z[i]));
    }
    zlo -= 0.5; zhi += 1.5;
    const double dz = (zhi - zlo) / (NZ - 1);
    auto site_cost = [&](int i, int k) -> double {
      const double z = zlo + k * dz;
      const bool anchor = (i == 0 || i == N - 1);
      if (!anchor && z < floor_v[i] - 0.5 * dz) return kBig; // below floor
      double c = W_ALT * std::max(0.0, z - floor_v[i]);
      if (stat[i] == 'C' && z > ceil_v[i]) c += W_EXP * (z - ceil_v[i]);
      return c;
    };
    auto near_k = [&](double z) {
      return std::max(0, std::min(NZ - 1,
                 static_cast<int>(std::lround((z - zlo) / dz))));
    };
    const int k0 = near_k(fe_z[0]), kN = near_k(fe_z[N - 1]);
    std::vector<double> dp_prev(NZ, kBig), dp_cur(NZ, kBig);
    std::vector<int> par(static_cast<size_t>(N) * NZ, -1);
    dp_prev[k0] = 0.0;
    int dead_at = -1;
    for (int i = 1; i < N; ++i) {
      const double d = std::max(1e-9, st_s[i] - st_s[i - 1]);
      const int win = static_cast<int>((smax * d) / dz + 0.5) + 1;
      std::fill(dp_cur.begin(), dp_cur.end(), kBig);
      bool alive = false;
      for (int k = 0; k < NZ; ++k) {
        if (i == N - 1 && k != kN) continue; // tail anchor
        const double sc = site_cost(i, k);
        if (sc >= kBig) continue;
        double best = kBig; int bj = -1;
        for (int j = std::max(0, k - win);
             j <= std::min(NZ - 1, k + win); ++j) {
          if (dp_prev[j] >= kBig) continue;
          const double tc = dp_prev[j] + W_CLIMB * std::abs(k - j) * dz;
          if (tc < best) { best = tc; bj = j; }
        }
        if (bj < 0) continue;
        dp_cur[k] = best + sc;
        par[static_cast<size_t>(i) * NZ + k] = bj;
        alive = true;
      }
      if (!alive) { dead_at = i; break; }
      dp_prev.swap(dp_cur);
    }
    std::vector<double> dpz(N, std::numeric_limits<double>::quiet_NaN());
    bool dp_ok = (dead_at < 0) && dp_prev[kN] < kBig;
    if (dp_ok) {
      int k = kN;
      for (int i = N - 1; i >= 1 && dp_ok; --i) {
        dpz[i] = zlo + k * dz;
        k = par[static_cast<size_t>(i) * NZ + k];
        if (k < 0) dp_ok = false;
      }
      if (dp_ok) dpz[0] = zlo + k * dz;
    }
    h4_dp_z_.assign(dpz.begin(), dpz.end());

    // Metrics: DP vs envelope / committed FE profile.
    int dp_above_F = 0, dp_exposed = 0, fe_exposed = 0;
    int fe_below_chord = 0, fe_below_swath = 0;
    double dp_exp_sum = 0.0, fe_exp_sum = 0.0;
    double dfe_mean = 0.0, dfe_max = 0.0, dfe_max_s = 0.0;
    for (int i = 0; i < N; ++i) {
      if (stat[i] == 'C' && fe_z[i] > ceil_v[i] + 1e-9) {
        ++fe_exposed; fe_exp_sum += fe_z[i] - ceil_v[i];
      }
      if (fe_z[i] < floor_c[i] - 0.02) ++fe_below_chord;
      if (fe_z[i] < floor_v[i] - 0.02) ++fe_below_swath;
      if (!dp_ok) continue;
      if (dpz[i] > F[i] + dz) ++dp_above_F;
      if (stat[i] == 'C' && dpz[i] > ceil_v[i] + 1e-9) {
        ++dp_exposed; dp_exp_sum += dpz[i] - ceil_v[i];
      }
      const double dd = std::abs(dpz[i] - fe_z[i]);
      dfe_mean += dd;
      if (dd > dfe_max) { dfe_max = dd; dfe_max_s = st_s[i]; }
    }
    if (dp_ok) dfe_mean /= N;

    LOG_INFO("[H4-CORRIDOR] stations=%d zones=%zu | status: free=%d ceil=%d "
             "exposed=%d | margin=%.2f climb=%.2f (cap-slope %.2f) "
             "swath=%.0f step=%.2f",
             N, risk_zones_.size(), n_free, n_ceil, n_exp,
             h4_shadow_margin_, smax, alt_cap_slope_, kSwathR, step);
    if (n_finC > 0) {
      LOG_INFO("[H4-CORRIDOR] raw collapse (floor>ceil): %d, max depth %.3f "
               "@s=%.1fkm | climb-dilated (F*>C*): %d, max depth %.3f "
               "@s=%.1fkm | min gap C*-F* = %.3f @s=%.1fkm (%d ceiled)",
               n_raw, raw_depth, raw_s * m_xy / 1000.0, n_dil, dil_depth,
               dil_s * m_xy / 1000.0, gap_min, gap_min_s * m_xy / 1000.0,
               n_finC);
    } else {
      LOG_INFO("[H4-CORRIDOR] no concealment ceiling anywhere on the route "
               "(all stations FREE or EXPOSED) — corridor is floor-only");
    }
    if (dp_ok) {
      LOG_INFO("[H4-CORRIDOR] DP(grid %d x %.3fu, W alt/exp/climb "
               "%.0f/%.0f/%.1f): dp>F*+dz at %d | dp exposed %d (per-stn "
               "depth sum %.2f) vs fe exposed %d (sum %.2f) | fe below "
               "floor: chord %d / swath %d",
               NZ, dz, W_ALT, W_EXP, W_CLIMB, dp_above_F, dp_exposed,
               dp_exp_sum, fe_exposed, fe_exp_sum, fe_below_chord,
               fe_below_swath);
      LOG_INFO("[H4-CORRIDOR] DP-vs-FE z: mean|dz|=%.3f max=%.3f @s=%.1fkm",
               dfe_mean, dfe_max, dfe_max_s * m_xy / 1000.0);
    } else if (dead_at >= 0) {
      LOG_WARN("[H4-CORRIDOR] DP infeasible: no reachable level at station "
               "%d (s=%.1fkm floor=%.3f F*=%.3f anchor0 z=%.3f, climb=%.2f) "
               "— corridor stats above remain valid",
               dead_at, st_s[dead_at] * m_xy / 1000.0, floor_v[dead_at],
               F[dead_at], fe_z[0], smax);
    } else {
      LOG_WARN("[H4-CORRIDOR] DP infeasible under endpoint anchors / climb "
               "window (corridor stats above remain valid)");
    }

    // Decimated profile + every flagged station (capped against log storms).
    auto f7 = [](double v) -> std::string {
      if (!std::isfinite(v)) return v > 0.0 ? "    inf" : "   -inf";
      char b[32]; snprintf(b, sizeof b, "%7.3f", v);
      return std::string(b);
    };
    const int stride = std::max(1, N / 40);
    int rows = 0;
    for (int i = 0; i < N && rows < 120; ++i) {
      const bool fraw = (stat[i] == 'C' && floor_v[i] > ceil_v[i]);
      const bool fdil = (stat[i] == 'C' && F[i] > C[i]);
      const bool fdfe = dp_ok && std::abs(dpz[i] - fe_z[i]) > 0.5;
      if (i % stride != 0 && !fraw && !fdil && stat[i] != 'E' && !fdfe)
        continue;
      const std::string cs = (stat[i] == 'F') ? "   FREE"
                             : (stat[i] == 'E') ? "    EXP"
                                                : f7(ceil_v[i]);
      LOG_INFO("[H4-PROF] i=%4d s=%6.1fkm floor=%7.3f fc=%7.3f ceil=%s "
               "F*=%7.3f C*=%s fe=%7.3f dp=%s cap=%s%s%s%s",
               i, st_s[i] * m_xy / 1000.0, floor_v[i], floor_c[i],
               cs.c_str(), F[i], f7(C[i]).c_str(), fe_z[i],
               (dp_ok ? f7(dpz[i]) : std::string("    n/a")).c_str(),
               f7(cap_v[i]).c_str(),
               fraw ? " RAW-CLOSED" : "", fdil ? " DIL-CLOSED" : "",
               fdfe ? " DP!=FE" : "");
      ++rows;
    }
  }

  // [H4] post-solve leg: sample the flown trajectory, map each corridor
  // station to its nearest-xy trajectory sample (robust to arc-vs-time
  // reparameterization; the worst xy match distance is reported so a
  // self-crossing route's mismatch is visible), and score the flown z
  // against the corridor and the DP profile.
  void PolyTrajOptimizer::logH4PostSolve(const poly_traj::Trajectory &traj)
  {
    const int N = static_cast<int>(h4_x_.size());
    if (N < 2) return;
    const double T = traj.getTotalDuration();
    if (!(T > 1e-6)) return;
    const int M = std::max(64, 4 * N);
    std::vector<double> sx(M + 1), sy(M + 1), sz(M + 1);
    for (int k = 0; k <= M; ++k) {
      const double tt = std::min(k * (T / M), T - 1e-9);
      const Eigen::Vector3d p = traj.getPos(tt);
      sx[k] = p.x(); sy[k] = p.y(); sz[k] = p.z();
    }
    double match_max = 0.0, below_max = 0.0, below_s = 0.0;
    double fin_exp_sum = 0.0, ddp_mean = 0.0, ddp_max = 0.0, ddp_max_s = 0.0;
    int fin_below = 0, fin_below_swath = 0, fin_exposed = 0, ddp_n = 0,
        rows = 0;
    // Stations advance along the route, so the matched sample index may
    // only slip back by a small slack: on a self-crossing / switchback
    // route an unconstrained nearest-xy search happily grabs the OTHER
    // arm's z (and match_max would not show it — the wrong arm is close in
    // xy by definition).
    const int back_slack = std::max(1, M / N);
    int bk_prev = 0;
    for (int i = 0; i < N; ++i) {
      double best = 1e30; int bk = bk_prev;
      for (int k = std::max(0, bk_prev - back_slack); k <= M; ++k) {
        const double dx = sx[k] - h4_x_[i], dy = sy[k] - h4_y_[i];
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; bk = k; }
      }
      bk_prev = bk;
      match_max = std::max(match_max, std::sqrt(best));
      const double zf = sz[bk];
      // Safety comparison against the CHORD floor (the flown path follows
      // local terrain; the swath floor is the drift-budget commit surface
      // and reads several units high over ridge NOE by construction).
      const bool below = zf < h4_floor_c_[i] - 0.02;
      if (zf < h4_floor_[i] - 0.02) ++fin_below_swath;
      if (below) {
        ++fin_below;
        if (h4_floor_c_[i] - zf > below_max) {
          below_max = h4_floor_c_[i] - zf; below_s = h4_s_km_[i];
        }
      }
      const bool expo = (h4_status_[i] == 'C' && zf > h4_ceil_[i] + 1e-9);
      if (expo) { ++fin_exposed; fin_exp_sum += zf - h4_ceil_[i]; }
      double dd = -1.0;
      if (std::isfinite(h4_dp_z_[i])) {
        dd = std::abs(zf - h4_dp_z_[i]);
        ddp_mean += dd; ++ddp_n;
        if (dd > ddp_max) { ddp_max = dd; ddp_max_s = h4_s_km_[i]; }
      }
      if ((below || dd > 0.5) && rows < 40) {
        LOG_INFO("[H4-POST-ROW] i=%4d s=%6.1fkm flown=%7.3f chord-floor="
                 "%7.3f swath-floor=%7.3f dp=%7.3f%s%s",
                 i, h4_s_km_[i], zf, h4_floor_c_[i], h4_floor_[i],
                 std::isfinite(h4_dp_z_[i]) ? h4_dp_z_[i] : -999.0,
                 below ? " BELOW-FLOOR" : "", expo ? " EXPOSED" : "");
        ++rows;
      }
    }
    if (ddp_n) ddp_mean /= ddp_n;
    LOG_INFO("[H4-POST] flown-vs-corridor: below chord-floor %d (max %.3f "
             "@s=%.1fkm), below swath-floor %d | exposed %d (per-stn depth "
             "sum %.2f) | worst xy match %.2fu over %d stations",
             fin_below, below_max, below_s, fin_below_swath, fin_exposed,
             fin_exp_sum, match_max, N);
    LOG_INFO("[H4-POST] flown-vs-DP z: mean|dz|=%.3f max=%.3f @s=%.1fkm "
             "(%d stations)",
             ddp_mean, ddp_max, ddp_max_s, ddp_n);
  }

  // [H4-COMMIT] Vertex-lattice corridor DP -> committed inner z (see header
  // for the design rationale). Returns false (clean_path untouched) whenever
  // the corridor DP is infeasible under the anchors/climb window — the
  // legacy seed then flies unchanged, so the gate can never lose a plan.
  // Same taper idempotency argument as the diagnostic.
  bool PolyTrajOptimizer::commitH4Profile(
      std::vector<Eigen::Vector3d> &clean_path,
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
      int first_free)
  {
    const int NV = static_cast<int>(clean_path.size());
    if (NV < 3 || first_free >= NV - 1) return false;
    setupTerrainTaper(start_pos, goal_pos);

    const double m_xy = (dyn_unit_xy_m_ > 0.0) ? dyn_unit_xy_m_ : 100.0;
    const double kInf = std::numeric_limits<double>::infinity();
    const double rc = std::max(0.5, h4_commit_swath_);
    const double step = (terrain_cell_u_ > 0.0)
                            ? std::min(2.3, terrain_cell_u_)
                            : 2.3;

    std::vector<double> floor_v(NV), ceil_v(NV, kInf), zs(NV), s_v(NV);
    std::vector<char> stat(NV, 'F'), anch(NV, 0);
    int n_ceil = 0, n_exp = 0, n_closed = 0;
    for (int i = 0; i < NV; ++i) {
      const Eigen::Vector3d &p = clean_path[i];
      zs[i] = p.z();
      s_v[i] = (i == 0) ? 0.0
                        : s_v[i - 1] + (p.head<2>() -
                                        clean_path[i - 1].head<2>()).norm();
      anch[i] = (i == 0 || i == NV - 1 || i < first_free) ? 1 : 0;

      // floor: disc max terrain around the vertex + tapered clearance
      double hmax = -1e30;
      for (double dx = -rc; dx <= rc + 1e-9; dx += step) {
        for (double dy = -rc; dy <= rc + 1e-9; dy += step) {
          if (dx * dx + dy * dy > rc * rc + 1e-9) continue;
          const double x = p.x() + dx, y = p.y() + dy;
          if (terrain_hgrad_) {
            float h, gx, gy;
            if (terrain_hgrad_(x, y, &h, &gx, &gy) && h > hmax) hmax = h;
          } else if (terrain_height_) {
            const float h = terrain_height_(x, y);
            if (std::isfinite(h) && h > hmax) hmax = h;
          }
        }
      }
      Eigen::Vector2d gdum;
      double fl = -1e30;
      if (hmax > -1e29) fl = hmax + terrainClearanceTarget(p, &gdum);
      if (wei_alt_ > 0.0) fl = std::max(fl, altitudeFloorTarget(p, &gdum));
      if (fl < -1e29) fl = (ground_height_ > -0.5) ? ground_height_ : 0.0;
      floor_v[i] = fl;

      // concealment ceiling at the vertex disc (3-state, as the diagnostic)
      if (use_risk_zones_ && risk_shadow_ceiling_) {
        bool reached = false, exposed = false;
        double cmin = kInf;
        for (size_t zi = 0; zi < risk_zones_.size() && !exposed; ++zi) {
          const auto &tz = risk_zones_[zi];
          if (!(tz.reach > 0.0)) continue;
          if ((p.head<2>() - tz.center.head<2>()).norm() > tz.reach + rc)
            continue;
          for (double dx = -rc; dx <= rc + 1e-9 && !exposed; dx += step) {
            for (double dy = -rc; dy <= rc + 1e-9; dy += step) {
              if (dx * dx + dy * dy > rc * rc + 1e-9) continue;
              const Eigen::Vector3d q(p.x() + dx, p.y() + dy, 0.0);
              if ((q.head<2>() - tz.center.head<2>()).norm() > tz.reach)
                continue;
              reached = true;
              const double v = risk_shadow_ceiling_(zi, q);
              if (!(v > -kInf)) { exposed = true; break; }
              cmin = std::min(cmin, v);
            }
          }
        }
        if (exposed) { stat[i] = 'E'; ceil_v[i] = -kInf; ++n_exp; }
        else if (reached) {
          stat[i] = 'C'; ceil_v[i] = cmin - h4_shadow_margin_; ++n_ceil;
          if (floor_v[i] > ceil_v[i]) ++n_closed;
        }
      }
    }

    // 1-D DP, anchor-aware. NOT the diagnostic's min-altitude objective:
    // an A/B measured that seeding "as low as safely possible" everywhere
    // shifts the solve's HOMOTOPY — from a floor-hugging seed the soft
    // equilibrium settles into a band-grazing local optimum even where
    // nothing asks for lowness (kwaypt clearance 1.862 -> 0.267, kzone1
    // 3.929 -> 2.800, k3 3.085 -> 2.237; only the already-grazing gauntlet
    // improved). The commit therefore TRACKS the committed profile
    // (W_TRACK * |z - fe|) and departs from it only for the three
    // corrections the layer exists for: floor violations (hard), climb
    // infeasibility (window), and exposure above the concealment ceiling
    // (W_EXP >> W_TRACK pulls under the ceiling / to the floor exactly in
    // lit stretches — "only low where visible").
    const double smax = std::max(1e-6, h4_climb_slope_);
    constexpr int NZ = 64;
    constexpr double W_TRACK = 1.0, W_EXP = 10.0, W_CLIMB = 0.1;
    const double kBig = 1e18;
    double zlo = kInf, zhi = -kInf;
    for (int i = 0; i < NV; ++i) {
      zlo = std::min(zlo, std::min(floor_v[i], zs[i]));
      zhi = std::max(zhi, std::max(floor_v[i], zs[i]));
    }
    zlo -= 0.5; zhi += 1.5;
    const double dz = (zhi - zlo) / (NZ - 1);
    auto near_k = [&](double z) {
      return std::max(0, std::min(NZ - 1,
                 static_cast<int>(std::lround((z - zlo) / dz))));
    };
    auto site_cost = [&](int i, int k) -> double {
      if (anch[i]) return (k == near_k(zs[i])) ? 0.0 : kBig;
      const double z = zlo + k * dz;
      if (z < floor_v[i] - 0.5 * dz) return kBig;
      double c = W_TRACK * std::abs(z - zs[i]);
      if (stat[i] == 'C' && z > ceil_v[i]) c += W_EXP * (z - ceil_v[i]);
      return c;
    };
    std::vector<double> dp_prev(NZ, kBig), dp_cur(NZ, kBig);
    std::vector<int> par(static_cast<size_t>(NV) * NZ, -1);
    dp_prev[near_k(zs[0])] = 0.0;
    int dead_at = -1;
    for (int i = 1; i < NV; ++i) {
      const double d = std::max(1e-9, s_v[i] - s_v[i - 1]);
      const int win = static_cast<int>((smax * d) / dz + 0.5) + 1;
      std::fill(dp_cur.begin(), dp_cur.end(), kBig);
      bool alive = false;
      for (int k = 0; k < NZ; ++k) {
        const double sc = site_cost(i, k);
        if (sc >= kBig) continue;
        double best = kBig; int bj = -1;
        for (int j = std::max(0, k - win);
             j <= std::min(NZ - 1, k + win); ++j) {
          if (dp_prev[j] >= kBig) continue;
          const double tc = dp_prev[j] + W_CLIMB * std::abs(k - j) * dz;
          if (tc < best) { best = tc; bj = j; }
        }
        if (bj < 0) continue;
        dp_cur[k] = best + sc;
        par[static_cast<size_t>(i) * NZ + k] = bj;
        alive = true;
      }
      if (!alive) { dead_at = i; break; }
      dp_prev.swap(dp_cur);
    }
    const int kN = near_k(zs[NV - 1]);
    if (dead_at >= 0 || dp_prev[kN] >= kBig) {
      LOG_WARN("[H4-COMMIT] corridor DP infeasible (%s station %d) — commit "
               "skipped, legacy seed flies unchanged",
               dead_at >= 0 ? "dead at" : "tail unreachable,",
               dead_at >= 0 ? dead_at : NV - 1);
      return false;
    }
    std::vector<double> dpz(NV, 0.0);
    {
      int k = kN;
      for (int i = NV - 1; i >= 1; --i) {
        dpz[i] = zlo + k * dz;
        k = par[static_cast<size_t>(i) * NZ + k];
        if (k < 0) {
          LOG_WARN("[H4-COMMIT] backtrace broke at station %d — commit "
                   "skipped", i);
          return false;
        }
      }
      dpz[0] = zlo + k * dz;
    }

    int changed = 0;
    double dmean = 0.0, dmax = 0.0, dmax_s = 0.0;
    int n_free_v = 0;
    for (int i = first_free; i < NV - 1; ++i) {
      if (anch[i]) continue;
      ++n_free_v;
      const double d = std::abs(dpz[i] - clean_path[i].z());
      dmean += d;
      if (d > dmax) { dmax = d; dmax_s = s_v[i]; }
      // Within one grid cell the DP is just tracking the committed z — the
      // difference is quantization, not a correction; committing it would
      // sprinkle +-dz/2 snap noise over the whole profile. Rewrite only
      // where a real correction (floor / climb / exposure) moved the DP.
      if (d > dz) { clean_path[i].z() = dpz[i]; ++changed; }
    }
    if (n_free_v > 0) dmean /= n_free_v;
    LOG_INFO("[H4-COMMIT] committed z at %d/%d free vertices (swath %.1f, "
             "grid %.3fu, climb %.2f): mean|dz|=%.3f max=%.3f @s=%.1fkm | "
             "corridor: ceil %d exposed %d closed %d of %d",
             changed, n_free_v, rc, dz, smax, dmean, dmax,
             dmax_s * m_xy / 1000.0, n_ceil, n_exp, n_closed, NV);
    return true;
  }

  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, const bool use_formation)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      return false;
    }

    t_now_ = node_->get_clock()->now().seconds();
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);

    Eigen::Vector3d start_pos = iniState.col(0);

    // [TERRAIN-TAPER] arm the per-endpoint clearance taper (see header).
    setupTerrainTaper(iniState.col(0), finState.col(0));

    // [H5] freeze the bounded-z box now that the taper (floor targets) and the
    // per-piece cap are both armed. No-op / byte-identical when off.
    buildH5Bounds(initInnerPts);

    double final_cost;

    auto t0 = node_->get_clock()->now();
    auto t1 = node_->get_clock()->now();
    auto t2 = node_->get_clock()->now();

    // L-BFGS params scaled up from the Swarm-Formation local-replan reference
    // (mem_size 16 / max_iter 60) for global planning with hundreds of vars.
    // mem_size capped at 64: 256 caused -1005 line-search failures.
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size       = lb_mem_size_; // ref 16 → 64 (global scale); 256 caused -1005
    // 0.1 (2026-07-21 sweep, 100+ runs): -34% iterations across the suite
    // with the same solutions (clearance deltas <= 0.021 u), and the
    // zone-saturated gauntlet flips -1004 -> converged. Robust across
    // 0.08-0.12 and mission seeds (no knife-edge). History: 0.05->0.005
    // (2.7x iters) left trajectories unchanged, so precision above 0.1 buys
    // nothing; g_epsilon=0 (pure GCOPTER plateau) quintuples iterations —
    // the dual gate (g_epsilon 0.1 + past/delta 3/1e-6) beats both extremes.
    lbfgs_params.g_epsilon      = lb_g_epsilon_;
    // [PAST-DELTA] GCOPTER-style cost-plateau convergence (gcopter.hpp:821:
    // g_epsilon=0, past=3, delta=relCostTol — their lbfgs header explicitly
    // warns g_epsilon is wrong for nonsmooth objectives). Our penalty terrain
    // keeps ||g|| above g_epsilon long after the cost has plateaued (opposing
    // stiff terms leave a large residual gradient), so hard instances idled
    // thousands of iterations to the cap and exited -1004 with the SAME
    // trajectory a plateau test would have blessed as converged. past=3 /
    // delta=1e-6 is conservative: three consecutive iterations must improve
    // the cost by <1e-6 relative before we call it converged; g_epsilon is
    // kept as a secondary (smooth-case) exit.
    lbfgs_params.past           = lb_past_;
    lbfgs_params.delta          = lb_delta_;
    lbfgs_params.min_step       = 1e-32;
    // [LBFGS-TUNE] optional overrides; <=0 keeps the library default.
    if (lb_max_linesearch_ > 0) lbfgs_params.max_linesearch = lb_max_linesearch_;
    if (lb_f_dec_  > 0.0)       lbfgs_params.f_dec_coeff    = lb_f_dec_;
    if (lb_s_curv_ > 0.0)       lbfgs_params.s_curv_coeff   = lb_s_curv_;
    // 300 consistently ended at -1004 while risk/altitude terms were still
    // polishing (~0.02%/iter). One-shot global plan on an idle desktop:
    // 300 iters ≈ 0.3 s, so thousands are nothing — let g_epsilon decide.
    // Budget scales with problem size: a ~90-piece (300 km) mission was still
    // actively descending at a flat 3000 (-1004) while small missions
    // converge in 300-1500; ~1 ms/iter, so even the ceiling stays ~12 s —
    // proportionate to the ~10 s eikonal on those same maps.
    // Per-piece budget 60 -> 100: the zone-saturated gauntlet converged via
    // g_epsilon using ~all of the old 60*piece ceiling (101 pieces -> 6060,
    // used ~6060), so it sat ON the budget edge — any gradient perturbation
    // (even a behaviour-preserving refactor) pushed g_epsilon-convergence past
    // the limit and flipped it to -1004. The extra budget is margin, not a new
    // optimum: cases that already converge are untouched (small missions finish
    // in 300-1500; the clean big_terrain baseline stays min-clr 4.903 byte for
    // byte); only budget-edge cases gain room. Genuine non-convergence is still
    // cut early by the past/delta plateau test, so the ceiling is rarely
    // reached. ~1 ms/iter keeps the worst case ~10-12 s.
    // Ceiling raised 12000 -> 20000 (2026-07-22): the roughness-routed
    // gauntlet plans 139 pieces, where 100/piece = 13900 was silently
    // truncated and the solve converged 3% under the old ceiling. Converged
    // runs stop at g_epsilon regardless, so only would-be -1004 truncations
    // are affected (same argument as the 60 -> 100 budget fix).
    lbfgs_params.max_iterations =
        std::min(20000, std::max(3000, 100 * piece_num_));

    if (!use_formation)
    {
      use_formation_ = false;
    }

    iter_num_ = 0;

    int result;

    // Direct 3D coordinate optimization with SDF-based obstacle penalty.
    variable_num_ = 4 * (piece_num_ - 1) + 1;
    std::vector<double> q(variable_num_);
    memcpy(q.data(), initInnerPts.data(), initInnerPts.size() * sizeof(double));
    // [H5] transform seed inner z into raw warp coordinates (inverse warp,
    // seed-clamped): q now holds r, not z, for the z rows.
    if (h5_active_) {
      int clamped = 0;
      for (int i = 0; i < piece_num_ - 1; ++i) {
        const double z = q[3 * i + 2];
        const double w = 0.5 * (h5_hi_(i) - h5_lo_(i));
        const double c = 0.5 * (h5_hi_(i) + h5_lo_(i));
        if (std::abs((z - c) / w) > 1.0 - h5_seed_margin_) ++clamped;
        q[3 * i + 2] = h5InvWarp(i, z);
      }
      if (clamped > 0)
        LOG_INFO("[H5] %d/%d seeds clamped into the box", clamped, piece_num_ - 1);
    }
    Eigen::Map<Eigen::VectorXd> Vt(q.data() + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    // [H5-FD] central-difference gradient probe on a few coords (z_raw of the
    // first/mid/last junction, one x, the last virtual time). Proves the warp
    // chain rule is exact before trusting the solve. Gate: h5_fd_check.
    if (h5_active_ && h5_fd_check_ && variable_num_ > 4) {
      std::vector<double> g(variable_num_, 0.0);
      const double c0 = costFunctionCallback(this, q.data(), g.data(), variable_num_);
      (void)c0;
      const int mid = 3 * ((piece_num_ - 1) / 2) + 2;
      const int ks[5] = {2, mid, 3 * (piece_num_ - 2) + 2, 0, variable_num_ - 1};
      double worst = 0.0;
      for (int idx = 0; idx < 5; ++idx) {
        const int k = ks[idx];
        if (k < 0 || k >= variable_num_) continue;
        const double h = 1e-6 * std::max(1.0, std::abs(q[k]));
        std::vector<double> qp = q, qm = q, gd(variable_num_, 0.0);
        qp[k] += h; qm[k] -= h;
        const double cp = costFunctionCallback(this, qp.data(), gd.data(), variable_num_);
        const double cm = costFunctionCallback(this, qm.data(), gd.data(), variable_num_);
        const double fd = (cp - cm) / (2.0 * h);
        const double rel = std::abs(fd - g[k]) / std::max(1.0, std::abs(fd));
        worst = std::max(worst, rel);
        LOG_INFO("[H5-FD] k=%d analytic=%.6e fd=%.6e rel=%.2e", k, g[k], fd, rel);
      }
      LOG_INFO("[H5-FD] worst relative error = %.2e (%s)", worst,
               worst < 1e-4 ? "PASS" : "CHECK");
      iter_num_ = 0;  // probe calls must not pollute the iteration count
    }

    t1 = node_->get_clock()->now();

    // Line-search stalls (-1005) usually mean the accumulated curvature
    // memory has gone inconsistent near a stiff feature (clearance band /
    // barrier ramp / z-corridor tug-of-war), not that the point is optimal:
    // q still holds the best accepted iterate, so re-entering from it with
    // FRESH memory (first step = steepest descent) routinely makes progress
    // again. Bounded retries keep the worst case cheap.
    int restarts = 0;
    for (;;) {
        result = lbfgs::lbfgs_optimize(
            variable_num_,
            q.data(),
            &final_cost,
            PolyTrajOptimizer::costFunctionCallback,
            NULL,
            // No progress callback: a non-null slot makes the line search
            // compute two full-vector norms per evaluation just to discard them.
            NULL,
            this,
            &lbfgs_params);
        // 4 restarts: observed hard instances were still DESCENDING fast
        // (risk 9.5M -> 7.9M and accelerating) when 2 restarts ran out, and
        // the published mid-iterate carried needle-spike artifacts.
        //
        // Retry ANY recoverable line-search failure, not just -1005: a stiff
        // feature can also trip ROUNDING_ERROR (-1008) or MINIMUMSTEP from a
        // still-descending iterate, and re-entering from the best accepted q
        // with fresh curvature memory recovers the same way. Only re-restart
        // from a FINITE iterate — a non-finite final_cost cannot recover and
        // would just burn the restart budget on garbage.
        const bool recoverable =
            result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
            result == lbfgs::LBFGSERR_ROUNDING_ERROR ||
            result == lbfgs::LBFGSERR_MINIMUMSTEP;
        // lbfgs_optimize reverts x(=q)/g to the best ACCEPTED iterate on a
        // line-search failure but does NOT revert fx — so final_cost can hold
        // the REJECTED trial's cost. The most common -1008 trigger is a
        // NaN/Inf trial cost (lbfgs.hpp isnan/isinf check), exactly where
        // final_cost is non-finite while q is a perfectly good finite iterate
        // (verified with a standalone probe: status=-1008, fx=nan, x reverted
        // finite). Recompute the cost AT q before gating, so the NaN-trial
        // subclass still restarts; the gate then only blocks a genuinely
        // non-finite accepted state.
        if (recoverable && !std::isfinite(final_cost)) {
            std::vector<double> gtmp(static_cast<size_t>(variable_num_));
            final_cost =
                costFunctionCallback(this, q.data(), gtmp.data(), variable_num_);
        }
        if (!recoverable || restarts >= 4 || !std::isfinite(final_cost)) {
            break;
        }
        ++restarts;
        LOG_WARN("[L-BFGS] recoverable line-search failure %d (%s) at cost=%.1f — "
                 "restart %d/4 from current iterate with fresh curvature memory",
                 result, lbfgs::lbfgs_strerror(result), final_cost, restarts);
    }

    // lbfgs_optimize reverts x=q to the best ACCEPTED iterate on a terminal
    // line-search failure (-1005 after the restart cap, or -1008 rounding),
    // but jerkOpt_/cps_ still describe the last REJECTED trial from the final
    // proc_evaluate. Regenerate them from q so the trajectory that is audited,
    // returned, executed and broadcast is the best iterate — the rejected
    // -1008 trial can be terrain-penetrating or non-finite (observed:
    // [COLLISION] clearance=-1.178 audited right after a -1008 exit). Harmless
    // on convergence/max-iter (q equals the last evaluated point already).
    {
        std::vector<double> regen_grad(static_cast<size_t>(variable_num_));
        costFunctionCallback(this, q.data(), regen_grad.data(), variable_num_);
    }

    if (log_manager_ && enable_debug_logs_) {
        const char* result_str = lbfgs::lbfgs_strerror(result);
        log_manager_->infof("L-BFGS Result: %d (%s), restarts=%d", result, result_str, restarts);
        log_manager_->infof("Iteration info: costFunction calls=%d, max_iterations=%d", iter_num_, lbfgs_params.max_iterations);
    }

    // Run the final trajectory audit unconditionally. Besides collision and
    // terrain checks it now contains the shared flight-envelope audit, which
    // must still run in obstacle-free ablations.
    //
    // [REJECT] (optimization/collision_reject, default ON) Ancestor safety
    // net: Swarm-Formation's checkCollision() fed OptimizeTrajectory's return
    // value so a penetrating trajectory was DISCARDED and replanned; MMP had
    // demoted the verdict to logs — a -1004 exit with a 35 m terrain breach
    // was published as "planning successful". With the gate on, a collision
    // audit failure fails the whole optimize call (manager's opt_success
    // false -> nothing published). Envelope violations still only log.
    {
        const bool audit_collided = checkCollision();
        if (collision_reject_ && audit_collided) {
            if (log_manager_)
                log_manager_->warnf(
                    "[REJECT] collision audit failed — trajectory DISCARDED "
                    "(optimization/collision_reject, nothing published)");
            return false;
        }
    }

    // Per-term vertical-force attribution on the converged trajectory —
    // the ground-truth answer to "what lifts the path over open water".
    if (diag_vertical_) logVerticalAttribution();

    t2 = node_->get_clock()->now();
    double time_ms = (t2 - t1).seconds() * 1000;
    double total_time_ms = (t2 - t0).seconds() * 1000;

    // Final result logging
    if (log_manager_) {
        log_manager_->infof("Optimization completed: iter=%d, use_formation_param=%d, use_formation_internal=%d, time(ms)=%f",
                            iter_num_, use_formation, use_formation_, time_ms);

        // Calculate and log jerk metrics
        poly_traj::Trajectory final_traj = jerkOpt_.getTraj();
        double total_jerk = computeTotalJerk(final_traj);
        double max_jerk = computeMaxJerk(final_traj);
        log_manager_->infof("[JERK METRICS] total_jerk=%.6f, max_jerk=%.6f units/s³ (x100 m/s³), duration=%.3f s",
                            total_jerk, max_jerk, final_traj.getTotalDuration());

        log_manager_->infof("[COST] formation_cost=%f (wei_formation=%f, similarity=%f)", dbg_cost_formation_, wei_formation_, debug_similarity_);

        // Additional debugging info
        if (enable_debug_logs_) {
            const char* result_str = lbfgs::lbfgs_strerror(result);
            log_manager_->debugf("L-BFGS Final Result: %d (%s)", result, result_str);
            log_manager_->debugf("Final iteration info: costFunction calls=%d, max_iterations=%d, Final cost=%f",
              iter_num_, lbfgs_params.max_iterations, final_cost);
        }
    } else {
        // Fallback to macro logger if log_manager is not available
        LOG_INFO("id=%d, iter=%d, use_formation=%d, time(ms)=%.3f",
                 drone_id_, iter_num_, use_formation, time_ms);
        LOG_INFO("[COST] formation_cost=%.6f (wei_formation=%.3f, similarity=%.6f)",
                 dbg_cost_formation_, wei_formation_, debug_similarity_);
    }
    optimal_points = cps_.points;

    // Collision verdicts are reported via logs only (checkCollision above);
    // failing the plan here would need an FSM-consequences decision first.
    return true;
  }
  bool PolyTrajOptimizer::checkCollision(void)
  {
    poly_traj::Trajectory traj = jerkOpt_.getTraj();
    // Sweep the FULL duration. The inherited ego-planner heuristic audited
    // only the first 2/3 (idx = k/3*2) — sensible for a rolling local replan
    // that never flies its tail, but on a single-shot global plan it left the
    // final third UNAUDITED: the boxes mission logged "[TERRAIN] min
    // clearance +0.087" while the RViz panel showed a -12.5 m dip in the
    // unswept goal-approach segment (a coverage hole, not a measurement
    // disagreement). Nothing consumes the 2/3 semantic: the verdict is
    // debug-only (the old time-end getter had no callers and is gone).
    const double T_end = traj.getDurations().sum();

    bool occ = false;
    double dt = 0.01;
    int i_end = std::max(1, (int)floor(T_end / dt));
    double t = 0.0;

    // [DYNAMICS] full fixed-wing flight-envelope audit. This is the same
    // inverse model used by the optimizer term and the dynamics simulator.
    {
      const Eigen::Vector3d S(dyn_unit_xy_m_, dyn_unit_xy_m_, dyn_unit_z_m_);
      // DUAL-DOMAIN stats. The inverse point-mass model is only meaningful
      // for steady flight (v >= speed_min): below stall the required CL/T
      // explode by construction (a rest-start mission BEGINS below stall),
      // and a peak taken over the spin-up ramp reads 1700%+ while the cruise
      // portion is clean — a misleading headline. So: CRUISE domain
      // (v >= speed_min) is the headline; the sub-stall ramp is reported
      // separately as a duration + its own peak, never mixed in.
      double util_peak = 0.0, util_peak_t = 0.0, util_sum = 0.0;
      double ramp_peak = 0.0, ramp_s = 0.0;
      mmp_vehicle_dynamics::EnvelopeLimit peak_limit =
          mmp_vehicle_dynamics::EnvelopeLimit::None;
      mmp_vehicle_dynamics::Evaluation peak_eval;
      int n_samp = 0, n_viol = 0;
      std::vector<double> utilization_all;
      utilization_all.reserve(static_cast<size_t>(T_end / 0.01) + 2);
      // dt matches the metrics panel's dynamics-violation row (100 Hz) so the
      // two report identical peaks.
      for (double tt = 0.0; tt < T_end; tt += 0.01) {
        const Eigen::Vector3d pm = S.cwiseProduct(traj.getPos(tt));
        const Eigen::Vector3d vm = S.cwiseProduct(traj.getVel(tt));
        const Eigen::Vector3d am = S.cwiseProduct(traj.getAcc(tt));
        const auto eval = mmp_vehicle_dynamics::evaluateInverseDynamics(
            dynamics_params_, pm, vm, am);
        if (!eval.valid) continue;
        // Below activation speed the model outputs are still finite (q-floor)
        // but not meaningful as envelope demands — count the TIME into the
        // ramp so the reported spin-up duration covers 0 -> speed_min, not
        // just activation -> speed_min (the old skip under-reported a
        // rest-start ramp by the 0-40 m/s third), while keeping such samples
        // out of ramp_peak/utilization.
        if (eval.speed_mps < dynamics_params_.model_activation_speed_mps) {
          ramp_s += 0.01;
          continue;
        }
        mmp_vehicle_dynamics::EnvelopeLimit limit;
        const double util = mmp_vehicle_dynamics::envelopeUtilization(
            dynamics_params_, eval, &limit);
        if (eval.speed_mps < dynamics_params_.speed_min_mps) {
          ramp_s += 0.01;
          if (util > ramp_peak) ramp_peak = util;
          continue;
        }
        ++n_samp;
        util_sum += util;
        utilization_all.push_back(util);
        if (!mmp_vehicle_dynamics::isWithinEnvelope(dynamics_params_, eval)) {
          ++n_viol;
        }
        if (util > util_peak) {
          util_peak = util;
          util_peak_t = tt;
          peak_limit = limit;
          peak_eval = eval;
        }
      }
      if (n_samp > 0) {
        const size_t p95_index = (utilization_all.size() * 95) / 100;
        std::nth_element(utilization_all.begin(),
                         utilization_all.begin() + p95_index,
                         utilization_all.end());
        const double util_p95 = utilization_all[p95_index];
        LOG_INFO("[DYNAMICS] envelope(cruise) peak=%.1f%% (%s) mean=%.1f%% "
                 "p95=%.1f%% at t=%.1f; state V=%.1f m/s n=%.2f CL=%.2f "
                 "T=%.0f N q=%.0f Pa bank=%.1f deg; violations %d/%d; "
                 "sub-stall ramp %.1f s (peak %.0f%%) (%s)",
                 100.0 * util_peak,
                 mmp_vehicle_dynamics::envelopeLimitName(peak_limit),
                 100.0 * util_sum / n_samp, 100.0 * util_p95,
                 util_peak_t, peak_eval.speed_mps, peak_eval.load_factor,
                 peak_eval.signed_lift_coefficient, peak_eval.thrust_required_n,
                 peak_eval.dynamic_pressure_pa,
                 peak_eval.bank_angle_rad * 180.0 / M_PI,
                 n_viol, n_samp, ramp_s, 100.0 * ramp_peak,
                 (dynamics_enable_ && wei_dynamics_ > 0.0) ? "term on" : "term OFF");
      }
    }

    // Terrain sweep via the heightmap: terrain is no longer voxelised into the
    // SDF, so the SDF pass below is boxes-only and terrain-BLIND — without this
    // a converged-but-penetrating optimum logged a misleading "no collision".
    // Reports the worst clearance and WHERE, so dips are locatable.
    if (terrain_hgrad_) {
      double worst = std::numeric_limits<double>::infinity();
      double wt = 0.0;
      Eigen::Vector3d wp = Eigen::Vector3d::Zero();
      for (double tt = 0.0; tt < T_end; tt += dt) {
        const Eigen::Vector3d pos = traj.getPos(tt);
        float h, gx, gy;
        if (terrain_hgrad_(pos.x(), pos.y(), &h, &gx, &gy)) {
          const double c = pos.z() - static_cast<double>(h);
          if (c < worst) { worst = c; wt = tt; wp = pos; }
        }
      }
      if (std::isfinite(worst)) {
        if (worst < 0.0) {
          LOG_WARN("[COLLISION] TERRAIN t=%.3f pos=(%.3f, %.3f, %.3f) clearance=%.3f (below surface)",
                   wt, wp.x(), wp.y(), wp.z(), worst);
          occ = true;
        } else {
          LOG_INFO("[TERRAIN] min clearance %.3f at t=%.3f pos=(%.3f, %.3f, %.3f)",
                   worst, wt, wp.x(), wp.y(), wp.z());
        }
      }
      // Terrain profile under the path, 5% steps: answers "what is it
      // climbing over?" directly from the log — the altitude panel cannot
      // show sub-pixel islets or the front-end berth, so climbs over
      // "open water" kept looking unmotivated.
      for (int pct = 0; pct <= 100; pct += 5) {
        const double tt = T_end * pct / 100.0;
        const Eigen::Vector3d pos = traj.getPos(std::min(tt, T_end - 1e-6));
        float hh, gx, gy;
        const bool land = terrain_hgrad_(pos.x(), pos.y(), &hh, &gx, &gy);
        // Lateral column: max terrain within +-4 units (400 m) of the sample —
        // tests the "underfoot is water but the NEIGHBOURING terrain forces
        // the climb" hypothesis (bilinear coastal blend spreads island height
        // ~1 DEM cell sideways; the FM2 coarse grid blocks whole 229 m cells
        // by their centre). If climbs coincide with rows where terrain=water
        // but near>0, the lateral halo is confirmed as the driver.
        float hnear = std::numeric_limits<float>::quiet_NaN();
        for (int dx = -4; dx <= 4; ++dx) {
          for (int dy = -4; dy <= 4; ++dy) {
            float hn, gnx, gny;
            if (terrain_hgrad_(pos.x() + dx, pos.y() + dy, &hn, &gnx, &gny)) {
              if (std::isnan(hnear) || hn > hnear) hnear = hn;
            }
          }
        }
        LOG_INFO("[TERRAIN-PROFILE] %3d%% t=%7.1f xy=(%7.1f,%7.1f) z=%6.3f terrain=%s near4=%s clr=%s",
                 pct, tt, pos.x(), pos.y(), pos.z(),
                 land ? std::to_string(hh).substr(0, 6).c_str() : "water",
                 std::isnan(hnear) ? "water" : std::to_string(hnear).substr(0, 6).c_str(),
                 land ? std::to_string(pos.z() - hh).substr(0, 6).c_str() : "-");
      }
    }

    if (sdf_manager_ && sdf_manager_->hasData())
    {
      for (int i = 0; i < i_end; i++)
      {
        Eigen::Vector3d pos = traj.getPos(t);
        float d = sdf_manager_->getDistance(pos);
        if (std::isfinite(d) && d < 0.0f) {
          LOG_WARN("[COLLISION] t=%.3f pos=(%.3f, %.3f, %.3f) d=%.3f inside obstacle",
                   t, pos.x(), pos.y(), pos.z(), d);
          occ = true;
          break;
        }
        t += dt;
      }
    }

    return occ;
  }

  // ==========================================================================
  //  Per-term VERTICAL-force attribution on the FINAL trajectory.
  //
  //  Motivation: the altitude panel shows the path climbing 40-80 m over what
  //  reads as open water (no terrain fill, no FE floor). The panel can only
  //  draw terrain; it CANNOT show which cost term (if any) pushed z up, nor
  //  can it show a min-jerk/MINCO smoothing overshoot (which has no spatial
  //  cause to draw). This sweep closes that gap directly from the log:
  //
  //   - For every sample along the converged trajectory it recomputes the
  //     VERTICAL component of each z-affecting cost term's force, using the
  //     EXACT same code paths / formulas as addPVAGradCost2CT (sdfGradCostP
  //     and RiskGradCostP are reused verbatim; the terrain / alt-cap / alt-
  //     floor blocks are replicated with "MUST MATCH" markers).
  //   - Sign convention: fz = -d(cost)/dz. fz > 0 pushes altitude UP,
  //     fz < 0 pushes it DOWN. So a lifting term shows fz > 0; the cap shows
  //     fz < 0; risk shows fz == 0 by construction (horizontal-only gradient).
  //   - It then auto-detects z-humps (local maxima over horizontal distance)
  //     and classifies each: if any spatial term exerts a real up-force on the
  //     ascending flank it is TERM-DRIVEN (and names the term); if every
  //     spatial fz is ~0 the hump is INTRINSIC — pure smoothness/variance
  //     min-jerk overshoot, which is exactly the "climb over nothing" case.
  //
  //  Opt-in (optimization/diag_vertical) — one-shot, but verbose.
  // ==========================================================================
  void PolyTrajOptimizer::logVerticalAttribution(void)
  {
    poly_traj::Trajectory traj = jerkOpt_.getTraj();
    const double T_end = traj.getDurations().sum();
    if (T_end <= 1e-6) { LOG_WARN("[VDIAG] empty trajectory, skipping"); return; }

    const int N = 500;                                    // one-shot: fine is ok
    const double dt = T_end / N;
    const double m_xy = (dyn_unit_xy_m_ > 0.0) ? dyn_unit_xy_m_ : 100.0;  // unit->m
    // Noise gate: a term "exerts vertical force" only above a small fraction of
    // a 1-unit violation's force (push terms scale with wei_* and a lever^2).
    const double eps_obs = 1e-3 * std::max(1.0, wei_obs_);
    const double eps_alt = 1e-3 * std::max(1.0, wei_alt_);

    // ---- Pass 1: sample the trajectory, recompute every term's fz ----------
    std::vector<double> S, Z, VZ, AZ;                      // s(km), z, vz, az
    std::vector<double> Fobs, Fterr, Frisk, Fcap, Ffloor;  // vertical forces
    std::vector<double> Hh, Clr, Near;                     // terrain h / clr / halo
    std::vector<char>   Land;
    std::vector<Eigen::Vector2d> XY;
    S.reserve(N + 1); Z.reserve(N + 1);

    // [ZDOF] Stage-0 z-corridor instrumentation (behavior-neutral, summarised
    // after the passes). The design plan predicts gauntlet -1004 from a CLOSED
    // z-corridor: where floor_surface (terrain+clearance / sea+cushion) exceeds
    // ceiling_surface (the per-piece cap), the vehicle has no feasible z and
    // the soft penalties fight to a non-converged minimum. Measuring the gap
    // gives an early warning the later stages gate on.
    double zdof_min_gap = 1e30;   // min(ceil_surface - floor_surface) over samples
    int    zdof_infeasible = 0;   // sample count with floor > ceiling
    double zdof_gap_at_worst_clr = std::numeric_limits<double>::quiet_NaN();
    double worst_clr_seen = 1e30;

    double s_units = 0.0;
    Eigen::Vector3d prev = traj.getPos(0.0);

    // Piece lookup for the ARC-VARYING cap (MUST MATCH addPVAGradCost2CT):
    // cumulative piece end-times; tt increases monotonically so a walking
    // index suffices.
    const Eigen::VectorXd durs = traj.getDurations();
    int piece_idx = 0;
    double piece_end = (durs.size() > 0) ? durs(0) : T_end;

    for (int k = 0; k <= N; ++k) {
      const double tt = std::min(k * dt, T_end - 1e-6);
      while (tt > piece_end && piece_idx + 1 < durs.size()) {
        ++piece_idx;
        piece_end += durs(piece_idx);
      }
      // Same per-piece-or-scalar selection as the optimization loop.
      const double zhi_i = (alt_zhi_pieces_.size() == durs.size())
                               ? alt_zhi_pieces_(piece_idx)
                               : alt_zhi_;
      const Eigen::Vector3d pos = traj.getPos(tt);
      const Eigen::Vector3d vel = traj.getVel(tt);
      const Eigen::Vector3d acc = traj.getAcc(tt);

      // cumulative HORIZONTAL arc-length — matches the panel's x-axis (km).
      const double dxs = pos.x() - prev.x(), dys = pos.y() - prev.y();
      s_units += std::sqrt(dxs * dxs + dys * dys);
      prev = pos;

      // terrain height / clearance under the foot (exact bilinear, as the term)
      double h = 0.0; bool land = false;
      float hh = 0.f, gx = 0.f, gy = 0.f;
      if (terrain_hgrad_) {
        land = terrain_hgrad_(pos.x(), pos.y(), &hh, &gx, &gy);
        if (land) h = static_cast<double>(hh);
      } else if (terrain_height_) {
        const float hv = terrain_height_(pos.x(), pos.y());
        if (std::isfinite(hv)) { h = hv; land = true; }
      }
      // clearance is over the surface if land, else over sea level (z itself).
      const double clr = land ? (pos.z() - h) : pos.z();

      // [ZDOF] floor/ceiling surfaces the optimizer actually enforces here.
      {
        const double floor_surf =
            land ? (h + obstacle_clearance_)
                 : (std::max(0.0, ground_height_) + obstacle_clearance_);
        const double gap = zhi_i - floor_surf;  // <0 => z-corridor closed
        if (gap < zdof_min_gap) zdof_min_gap = gap;
        if (gap < 0.0) ++zdof_infeasible;
        if (clr < worst_clr_seen) { worst_clr_seen = clr; zdof_gap_at_worst_clr = gap; }
      }

      // lateral terrain halo: max terrain within +-4 units (like TERRAIN-PROFILE)
      // — proves whether NEIGHBOURING land (not underfoot) could be the driver.
      double near = std::numeric_limits<double>::quiet_NaN();
      if (terrain_hgrad_) {
        for (int dx = -4; dx <= 4; ++dx)
          for (int dy = -4; dy <= 4; ++dy) {
            float hn, gnx, gny;
            if (terrain_hgrad_(pos.x() + dx, pos.y() + dy, &hn, &gnx, &gny))
              if (std::isnan(near) || hn > near) near = hn;
          }
      }

      // ---- fz per term (fz>0 pushes altitude UP; fz = -d(cost)/dz) --------
      // OBSTACLE (SDF boxes + hard ground/ceiling half-spaces): reuse verbatim,
      // with the same use_sdf gating as addPVAGradCost2CT (half-spaces always).
      double fz_obs = 0.0;
      {
        Eigen::Vector3d gradp; double costp;
        const bool use_sdf =
            enable_obstacles_ && sdf_manager_ && sdf_manager_->hasData();
        if (sdfGradCostP(0, pos, gradp, costp, use_sdf)) fz_obs = -gradp.z();
      }
      // UNIFIED FLOOR (MUST MATCH addPVAGradCost2CT): one floor surface
      // F = max(terrain+clearance, mission-altitude anchor); only the GOVERNING
      // branch fires. TERRAIN branch (slot 8, cubic wei_obs) when it is the
      // higher floor; ANCHOR branch (slot 6, quad wei_alt) otherwise. fz > 0
      // (floors lift). fz_terr feeds the [VDIAG-ZDOF-RISK] margin, so it must
      // read the governing terrain force, not a shadow of the retired term.
      double fz_terr = 0.0, fz_floor = 0.0;
      double vfloor_terr = -1e30;
      if (enable_obstacles_ && (terrain_hgrad_ || terrain_height_) && land) {
        Eigen::Vector2d tg_xy;   // fz only needs the z-derivative
        vfloor_terr = h + terrainClearanceTarget(pos, &tg_xy);
      }
      Eigen::Vector2d fg_xy0;
      const double vfloor_anchor =
          (wei_alt_ > 0.0) ? altitudeFloorTarget(pos, &fg_xy0) : -1e30;
      const bool terr_gov = (vfloor_terr > -1e29) && vfloor_terr >= vfloor_anchor;
      const double Fv = terr_gov ? vfloor_terr : vfloor_anchor;
      if (Fv > -1e29 && pos.z() < Fv) {
        const double viol = Fv - pos.z();
        if (terr_gov) fz_terr = wei_obs_ * 3.0 * viol * viol;
        else          fz_floor = wei_alt_ * 2.0 * viol;
      }
      // RISK (moat/barrier arc-length integral). The oblate-ellipsoid moat and
      // the z-visibility sigmoid both have a real z-derivative, so fz_risk is
      // NONZERO near zone skirts and shadow edges — it is a genuine 3D force,
      // not identically 0. Logged so its vertical share is attributable, NOT to
      // prove the zone term is horizontal-only. MUST MATCH RiskGradCostP.
      double fz_risk = 0.0;
      if (use_risk_zones_) {
        Eigen::Vector3d gradp, gradv; double costp;
        if (RiskGradCostP(0, pos, vel, gradp, gradv, costp)) fz_risk = -gradp.z();
      }
      // ALT-CAP (quadratic down-force above the band, terrain-aware gate;
      // arc-varying zhi_i, see piece lookup above).
      // MUST MATCH the alt-cap block incl. the smoothstep gate. fz < 0 (down).
      double fz_cap = 0.0; bool cap_on = false;
      if (wei_alt_ > 0.0 && zhi_i >= 0.0 && pos.z() > zhi_i) {
        // dgz = ∂gate/∂z: heightmap keys on (z − h) so dgz = dgate; the SDF
        // fallback keys on d(pos) so dgz = dgate·∇d.z. MUST MATCH the cap
        // block's gate_grad.z.
        double gate = 1.0, dgz = 0.0;
        const double glo = obstacle_clearance_, ghi = 2.0 * obstacle_clearance_;
        bool gated = false;
        if (terrain_hgrad_) {
          float hg = 0.f, hx = 0.f, hy = 0.f;
          if (terrain_hgrad_(pos.x(), pos.y(), &hg, &hx, &hy)) {
            const double tc = pos.z() - static_cast<double>(hg);
            double t = (ghi > glo) ? (tc - glo) / (ghi - glo) : 1.0;
            t = std::max(0.0, std::min(1.0, t));
            gate = t * t * (3.0 - 2.0 * t);
            if (t > 0.0 && t < 1.0) dgz = 6.0 * t * (1.0 - t) / (ghi - glo);
            gated = true;
          }
        }
        if (!gated && sdf_manager_ && sdf_manager_->hasData()) {
          float d = 0.f;
          Eigen::Vector3d grad_d = Eigen::Vector3d::Zero();
          if (sdf_manager_->getDistanceAndGradient(pos, &d, &grad_d) &&
              std::isfinite(d)) {
            double t = (ghi > glo) ? (static_cast<double>(d) - glo) / (ghi - glo) : 1.0;
            t = std::max(0.0, std::min(1.0, t));
            gate = t * t * (3.0 - 2.0 * t);
            if (t > 0.0 && t < 1.0)
              dgz = 6.0 * t * (1.0 - t) / (ghi - glo) * grad_d.z();
          }
        }
        if (gate > 0.0) {
          const double ua = pos.z() - zhi_i;
          const double gg = wei_alt_ * ua * ua * dgz;
          fz_cap = -(wei_alt_ * 2.0 * ua * gate + gg);  // d(cost)/dz>0 -> fz<0
          cap_on = true;
        }
      }
      // ALT-FLOOR is the anchor branch of the UNIFIED FLOOR above — fz_floor is
      // already set there (governance-selected), so nothing to recompute here.
      const bool floor_on = (fz_floor != 0.0);

      S.push_back(s_units * m_xy / 1000.0);
      Z.push_back(pos.z()); VZ.push_back(vel.z()); AZ.push_back(acc.z());
      Fobs.push_back(fz_obs); Fterr.push_back(fz_terr); Frisk.push_back(fz_risk);
      Fcap.push_back(fz_cap); Ffloor.push_back(fz_floor);
      Hh.push_back(h); Clr.push_back(clr); Near.push_back(near);
      Land.push_back(land ? 1 : 0);
      XY.emplace_back(pos.x(), pos.y());
      (void)cap_on; (void)floor_on;   // computed for clarity; fz carries the sign
    }

    const int M = static_cast<int>(Z.size());

    // ---- Pass 2: detect z-humps (local maxima over horizontal distance) ----
    // Window ~2% of the samples; a hump must rise PROM units above its flanks
    // and clear ALT_MIN (skip sea-level ripple). Tuned to catch the 40-80 m
    // (~0.4-0.8 unit) climbs the panel shows.
    const int W = std::max(3, M / 50);
    const double PROM = 0.06;      // ~6 m prominence
    const double ALT_MIN = 0.12;   // ~12 m above baseline
    std::vector<int> peaks;
    for (int i = W; i < M - W; ++i) {
      if (Z[i] < ALT_MIN) continue;
      bool is_max = true; double lo = Z[i];
      for (int j = i - W; j <= i + W; ++j) {
        if (Z[j] > Z[i]) { is_max = false; break; }
        if (Z[j] < lo) lo = Z[j];
      }
      if (is_max && (Z[i] - lo) >= PROM) {
        // de-dup flat tops: keep only if strictly the first of a plateau
        if (peaks.empty() || (i - peaks.back()) > W) peaks.push_back(i);
      }
    }

    // ---- Emit: legend + coarse table -------------------------------------
    LOG_INFO("[VDIAG] ===== vertical-force attribution on final trajectory "
             "(fz>0 => term pushes altitude UP; fz = -d(cost)/dz) =====");
    LOG_INFO("[VDIAG] band[alt_zlo=%.4f alt_zhi=%.4f] wei_alt=%.1f | "
             "wei_obs=%.1f clearance=%.4f | risk_zones=%s | unit_xy=%.1fm | "
             "samples=%d humps=%d",
             alt_zlo_, alt_zhi_, wei_alt_, wei_obs_, obstacle_clearance_,
             use_risk_zones_ ? "on" : "off", m_xy, M, (int)peaks.size());
    LOG_INFO("[VDIAG] cols: idx s_km xy z | land h clr near | "
             "fz[obs terr risk cap floor] | vz az");
    const int stride = std::max(1, M / 60);   // ~60 baseline rows
    for (int i = 0; i < M; i += stride) {
      LOG_INFO("[VDIAG] %3d %7.2f (%8.1f,%8.1f) z=%6.3f | l=%d h=%6.3f clr=%6.3f "
               "near=%s | %+8.1f %+8.1f %+6.2f %+8.1f %+8.1f | vz=%+.4f az=%+.4f",
               i, S[i], XY[i].x(), XY[i].y(), Z[i], (int)Land[i], Hh[i], Clr[i],
               std::isnan(Near[i]) ? "water" : std::to_string(Near[i]).substr(0, 6).c_str(),
               Fobs[i], Fterr[i], Frisk[i], Fcap[i], Ffloor[i], VZ[i], AZ[i]);
    }

    // ---- Emit: per-hump fine window + classification ----------------------
    for (size_t p = 0; p < peaks.size(); ++p) {
      const int i = peaks[p];
      const int a = std::max(0, i - W), b = std::min(M - 1, i + W);
      // dominant spatial up-force over the ASCENDING flank [a, i].
      double up_terr = 0, up_floor = 0, up_obs = 0, dn_cap = 0, mx_risk = 0;
      double near_max = std::numeric_limits<double>::quiet_NaN();
      for (int j = a; j <= i; ++j) {
        up_terr  = std::max(up_terr,  Fterr[j]);
        up_floor = std::max(up_floor, Ffloor[j]);
        up_obs   = std::max(up_obs,   Fobs[j]);       // >0 = ground/obstacle push
        dn_cap   = std::min(dn_cap,   Fcap[j]);       // <0
        mx_risk  = std::max(mx_risk,  std::abs(Frisk[j]));
        if (!std::isnan(Near[j]) && (std::isnan(near_max) || Near[j] > near_max))
          near_max = Near[j];
      }
      // classify
      const char *verdict; const char *driver;
      double drive_mag = 0.0;
      if (up_terr > eps_obs && up_terr >= up_floor && up_terr >= up_obs) {
        verdict = "TERM-DRIVEN"; driver = "terrain(8)"; drive_mag = up_terr;
      } else if (up_obs > eps_obs && up_obs >= up_floor) {
        verdict = "TERM-DRIVEN"; driver = "obstacle/ground(0)"; drive_mag = up_obs;
      } else if (up_floor > eps_alt) {
        verdict = "TERM-DRIVEN"; driver = "alt-floor cushion(6)"; drive_mag = up_floor;
      } else {
        verdict = "INTRINSIC";   driver = "smoothness/variance min-jerk overshoot";
        drive_mag = 0.0;
      }
      LOG_INFO("[VDIAG-HUMP] #%zu @ s=%.2fkm idx=%d z=%.3f (rose from flank) | "
               "underfoot: land=%d h=%.3f clr=%.3f | halo_max=%s | "
               "flank up-force[terr=%.1f floor=%.1f obs=%.1f] down[cap=%.1f] "
               "risk=%.3f | peak vz=%+.4f az=%+.4f",
               p, S[i], i, Z[i], (int)Land[i], Hh[i], Clr[i],
               std::isnan(near_max) ? "water" : std::to_string(near_max).substr(0, 6).c_str(),
               up_terr, up_floor, up_obs, dn_cap, mx_risk, VZ[i], AZ[i]);
      LOG_INFO("[VDIAG-HUMP] #%zu VERDICT: %s -- driver=%s (mag=%.1f). %s",
               p, verdict, driver, drive_mag,
               (std::string(verdict) == "INTRINSIC")
                 ? "No spatial term lifts this hump: it is the quintic relaxing "
                   "(min-jerk overshoot / variance smoothing), NOT terrain. "
                   "Panel is truthful; fix is boundary/smoothing, not viz."
                 : "A spatial cost term lifts this hump; check that term's "
                   "radius/gain/band if the climb is unwanted.");
      // fine window around the peak for eyeball confirmation
      for (int j = a; j <= b; ++j) {
        LOG_INFO("[VDIAG-FINE] #%zu %3d s=%7.2f z=%6.3f | fz[obs=%+7.1f "
                 "terr=%+7.1f risk=%+5.2f cap=%+7.1f flr=%+7.1f] vz=%+.4f az=%+.4f",
                 p, j, S[j], Z[j], Fobs[j], Fterr[j], Frisk[j], Fcap[j],
                 Ffloor[j], VZ[j], AZ[j]);
      }
    }
    if (peaks.empty())
      LOG_INFO("[VDIAG-HUMP] no z-humps above prominence %.2f / min-alt %.2f "
               "detected — trajectory is flat within tolerance.", PROM, ALT_MIN);

    // [ZDOF] Stage-0 summary — GEOMETRIC half: floor(terrain+clr) vs the cap
    // ceiling. min_gap < 0 (infeasible > 0) means the floor DEMAND structurally
    // exceeds the cap somewhere — a real -1004 (proven: clearance band raised
    // above the mission AGL closes it, min_gap -1.73 @ -1004). SUFFICIENT but
    // NOT necessary: the cap gates OFF within the clearance band, and the risk
    // down-pull is a FORCE, not a geometric surface, so the realistic
    // risk-driven -1004 keeps this gap OPEN (risk 4x -> -1004 with a 20 m
    // terrain breach yet min_gap +0.55). That class is the [VDIAG-ZDOF-RISK]
    // line below; the two together are the z-feasibility verdict.
    LOG_INFO("[VDIAG-ZDOF] z-corridor(geometric): min_gap=%.4f infeasible_samples=%d/%d "
             "gap@worst_clr=%.4f (worst_clr=%.4f) — %s",
             (zdof_min_gap > 1e29) ? 0.0 : zdof_min_gap, zdof_infeasible, N + 1,
             std::isnan(zdof_gap_at_worst_clr) ? 0.0 : zdof_gap_at_worst_clr,
             (worst_clr_seen > 1e29) ? 0.0 : worst_clr_seen,
             (zdof_infeasible > 0)
                 ? "CLOSED somewhere (geometric floor>cap -1004)"
                 : "open everywhere");

    // [ZDOF-RISK] Stage-0 summary — FORCE half: the risk-effective ceiling the
    // geometric gap is blind to. The gauntlet -1004 is not floor>cap; it is the
    // risk visibility down-pull (dv/dz >= 0) overpowering the terrain up-push
    // INSIDE the clearance band, dragging the vehicle below its safe clearance
    // while the cap-floor corridor stays open. Both fz are already recomputed
    // per sample above; scan where the terrain penalty is ACTIVE (Fterr>0 ==
    // genuinely in-band, taper-correct) and ask whether the risk down-force
    // wins there. margin = terr_up - risk_down; margin < 0 == risk-closed
    // (duck-below / clearance-guarantee breach). This is the seed of the
    // pre-flight feasibility check ("can this mission hold clearance under the
    // risk field, or must it duck?").
    double zdof_risk_margin_min = 1e30;   // min(terr_up - risk_down), in-band land
    int    zdof_risk_closed     = 0;      // in-band samples where risk_down > terr_up
    double zdof_risk_down_peak  = 0.0;    // strongest downward risk pull in-band
    int    zdof_inband          = 0;      // land samples with the terrain penalty active
    for (int k = 0; k < M; ++k) {
      if (!Land[k] || Fterr[k] <= 0.0) continue;         // terrain active == in-band
      ++zdof_inband;
      const double terr_up = Fterr[k];                    // >= 0 (terrain lifts)
      const double risk_dn = std::max(0.0, -Frisk[k]);    // downward risk only
      const double margin  = terr_up - risk_dn;
      if (margin < zdof_risk_margin_min) zdof_risk_margin_min = margin;
      if (risk_dn > terr_up) ++zdof_risk_closed;
      if (risk_dn > zdof_risk_down_peak) zdof_risk_down_peak = risk_dn;
    }
    LOG_INFO("[VDIAG-ZDOF-RISK] z-corridor(force): risk_margin_min=%.1f "
             "risk_closed_samples=%d/%d risk_down_peak=%.1f — %s",
             (zdof_risk_margin_min > 1e29) ? 0.0 : zdof_risk_margin_min,
             zdof_risk_closed, zdof_inband, zdof_risk_down_peak,
             (zdof_risk_closed > 0)
                 ? "RISK OVERPOWERS TERRAIN in-band (duck-below / clearance breach)"
                 : "terrain holds the band");
  }

  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);


    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    // Member scratch: this callback runs thousands of times per solve;
    // resize() is a no-op after the first call of a solve.
    Eigen::VectorXd &T = opt->cb_T_;
    T.resize(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd &gradT = opt->cb_gradT_;
    gradT.resize(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    // Slots: 0 obstacle(SDF), 1 swarm, 2 formation, 3 risk (moat+barrier),
    //        4 feasibility (v/a envelope ONLY), 5 sqrvariance, 6 altitude,
    //        7 dynamics, 8 terrain(heightmap 2.5D), 9 ride (H1 speed-over-rough).
    Eigen::VectorXd &obs_swarm_feas_qvar_costs = opt->cb_costs_;
    obs_swarm_feas_qvar_costs.resize(10);
    obs_swarm_feas_qvar_costs.setZero();

    // High-performance timing for debugging (similar to con code)
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t1 = t_start, t2 = t_start, t3 = t_start, t4 = t_start, t5 = t_start;

    // 1. Trajectory generation
    t1 = std::chrono::high_resolution_clock::now();
    // [H5] warp the raw z rows into the box before generate; stash dz/dr from
    // the SAME warped z that generate consumes (single-surface discipline).
    if (opt->h5_active_) {
      Eigen::MatrixXd Pw = P;
      for (int i = 0; i < opt->piece_num_ - 1; ++i) {
        const double zw = opt->h5Warp(i, P(2, i));
        Pw(2, i) = zw;
        opt->h5_D_(i) = opt->h5Deriv(i, zw);
      }
      opt->jerkOpt_.generate(Pw, T);
    } else {
      opt->jerkOpt_.generate(P, T);
    }
    double traj_gen_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t1).count();

    // 2. Smoothness cost
    t2 = std::chrono::high_resolution_clock::now();
    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost
    double smoothness_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t2).count();

    // 3. Obstacle/Swarm/Feasibility cost (most complex part)
    t3 = std::chrono::high_resolution_clock::now();
    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost
    double pva_cost_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t3).count();

    // 4. Gradient calculation
    t4 = std::chrono::high_resolution_clock::now();
    opt->jerkOpt_.getGrad2TP(gradT, gradP);
    // [H5] chain rule: gradP.row(2) after getGrad2TP is exactly ∂C/∂z_warped
    // (addPropCtoP assigns, no accumulation); ∂C/∂r = D·∂C/∂z. Rows 0,1 (xy)
    // and gradt (times) are untouched — the frozen bounds add no cross-terms
    // and z(r) has no T dependence.
    if (opt->h5_active_)
      gradP.row(2).array() *= opt->h5_D_.transpose().array();
    double grad_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t4).count();

    // 5. Time cost
    t5 = std::chrono::high_resolution_clock::now();
    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);
    double time_cost_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t5).count();

    opt->iter_num_ += 1;

    // Debug output for performance monitoring
    if (opt->iter_num_ % 50 == 0 && opt->log_manager_ && opt->enable_debug_logs_) {
        double total_callback_time = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t_start).count();
        opt->log_manager_->debugf("CostFunction iter=%d: Traj=%fms, Smooth=%fms, PVA=%fms, Grad=%fms, Time=%fms, Total=%fms", 
          opt->iter_num_, traj_gen_time, smoothness_time, pva_cost_time, grad_time, time_cost_time, total_callback_time);
    }

    // Detailed L-BFGS cost debugging
    if (opt->enable_lbfgs_detail_logs_ && opt->log_manager_ && opt->iter_num_ % 10 == 0) {
        double total_cost = smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
        opt->log_manager_->infof("[L-BFGS DETAIL] iter=%d, total_cost=%.6f", opt->iter_num_, total_cost);
        opt->log_manager_->infof("  smoothness_cost=%.6f (weight=%.1f)", smoo_cost, opt->wei_smooth_);
        opt->log_manager_->infof("  obstacle_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(0), opt->wei_obs_);
        opt->log_manager_->infof("  swarm_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(1), opt->wei_swarm_);
        opt->log_manager_->infof("  formation_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(2), opt->wei_formation_);
        opt->log_manager_->infof("  risk_cost=%.6f (weight=%.3f, barrier=%.3f)", obs_swarm_feas_qvar_costs(3), opt->wei_risk_, opt->wei_risk_barrier_);
        opt->log_manager_->infof("  altitude_cost=%.6f (weight=%.3f, band=[%.2f, %.2f])", obs_swarm_feas_qvar_costs(6), opt->wei_alt_, opt->alt_zlo_, opt->alt_zhi_);
        opt->log_manager_->infof("  feasibility_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(4), opt->wei_feas_);
        if (opt->wei_ride_ > 0.0)
            opt->log_manager_->infof("  ride_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(9), opt->wei_ride_);
        opt->log_manager_->infof("  dynamics_cost=%.6f (weight=%.3f, V=[%.0f,%.0f] m/s, n<=%.2f, %s)", obs_swarm_feas_qvar_costs(7), opt->wei_dynamics_, opt->dynamics_params_.speed_min_mps, opt->dynamics_params_.speed_max_mps, opt->dynamics_params_.load_factor_max,
                                 (opt->dynamics_enable_ && opt->wei_dynamics_ > 0.0) ? "on" : "off");
        opt->log_manager_->infof("  terrain_cost=%.6f (weight=%.3f, heightmap 2.5D, %s)", obs_swarm_feas_qvar_costs(8), opt->wei_obs_, opt->terrain_height_ ? "on" : "off");
        opt->log_manager_->infof("  time_cost=%.6f (weight=%.3f)", time_cost, opt->wei_time_);

        // Store formation cost for final logging
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    } else if (!opt->enable_lbfgs_detail_logs_ && opt->use_formation_) {
        // Store formation cost for final logging even when detail logs are disabled
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    }

    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      if (VT(i) > 0.0) {
        RT(i) = ((0.5 * VT(i) + 1.0) * VT(i) + 1.0);
      } else {
        double denom = ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
        // Protect against near-zero denominator
        if (std::abs(denom) < 1e-10) {
          if (log_manager_) {
            log_manager_->errorf("VirtualT2RealT: Near-zero denominator at i=%d, VT=%f, denom=%e",
                                i, VT(i), denom);
          }
          RT(i) = 1.0;  // Fallback to minimum time
        } else {
          RT(i) = 1.0 / denom;
        }
      }
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    // [RIDE] per-piece time price: wei_time * relief factor (1.0 when the
    // relief table is absent or piece counts mismatch — scalar legacy path).
    const bool per_piece_time =
        (time_relief_pieces_.size() == VT.size());
    costT = 0.0;
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }
      const double wt = per_piece_time ? wei_time_ * time_relief_pieces_(i)
                                       : wei_time_;
      gdVT(i) = (gdRT(i) + wt) * gdVT2Rt;
      if (per_piece_time) costT += RT(i) * wt;
    }
    // Scalar path keeps the original expression (identical rounding).
    if (!per_piece_time) costT = RT.sum() * wei_time_;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
    // Explicit smoothness weight (see header). Safe to scale gdC here:
    // initGradCost just seeded it with ONLY the jerk-energy gradients;
    // every other term accumulates afterwards in addPVAGradCost2CT.
    if (wei_smooth_ != 1.0) {
      cost *= wei_smooth_;
      gdT *= wei_smooth_;
      jerkOpt_.get_gdC() *= wei_smooth_;
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {

    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0;

    // Loop-invariant SDF gate (hasData() cannot change mid-solve): shared by
    // the precompute and the serial obstacle call below.
    const bool use_sdf =
        enable_obstacles_ && sdf_manager_ && sdf_manager_->hasData();

    // [RISK-PAR] Precompute every PURE per-sample cost term in parallel (one
    // task per piece — pieces are independent; within a piece the exact
    // s1 += step accumulation and beta/pos/vel/acc expressions of the main
    // loop are replayed, so every stored record is bit-identical to what the
    // inline call below would produce). The main loop then consumes the
    // records in its ORIGINAL order: accumulation order is untouched, so the
    // result is bitwise the same for any thread count. Covered terms: risk
    // (LOS/visibility — the dominant one), obstacle SDF + half-spaces,
    // unified floor, altitude cap, fixed-wing dynamics. swarm/formation stay
    // inline (they write members). All covered callees are const-pure and
    // the terrain memo is thread_local (audited); SDF reads take the shared
    // lock.
    const bool sample_par = risk_parallel_threads_ != 1;
    if (sample_par) {
      const int S = N * (K + 1);
      if (static_cast<int>(risk_par_found_.size()) < S) {
        risk_par_found_.resize(S);
        risk_par_gradp_.resize(3, S);
        risk_par_gradv_.resize(3, S);
        risk_par_costp_.resize(S);
        par_obs_found_.resize(S);
        par_floor_found_.resize(S);
        par_cap_found_.resize(S);
        par_dyn_found_.resize(S);
        par_floor_slot_.resize(S);
        par_obs_grad_.resize(3, S);
        par_floor_grad_.resize(3, S);
        par_cap_grad_.resize(3, S);
        par_dyn_gp_.resize(3, S);
        par_dyn_gv_.resize(3, S);
        par_dyn_ga_.resize(3, S);
        par_obs_cost_.resize(S);
        par_floor_cost_.resize(S);
        par_cap_cost_.resize(S);
        par_dyn_cost_.resize(S);
      }
      const bool dyn_on = dynamics_enable_ && wei_dynamics_ > 0.0;
#ifdef _OPENMP
      // auto (0) = half the hardware threads (~physical cores), capped at 16:
      // measured on a 32-thread host with all terms precomputed — 16 threads
      // 8.8 s vs 8 threads 9.8 s vs serial 26.7 s, while ALL 32 was
      // pathological (HT siblings spin-starve the serial remainder of the
      // loop, >2x slower than serial). Half-of-HW generalizes "never run on
      // every hyperthread" to smaller machines.
      const int risk_par_nthreads =
          risk_parallel_threads_ > 0
              ? risk_parallel_threads_
              : std::max(1, std::min(16, omp_get_max_threads() / 2));
#pragma omp parallel for schedule(dynamic, 1) num_threads(risk_par_nthreads)
#endif
      for (int pi = 0; pi < N; ++pi) {
        const Eigen::Matrix<double, 6, 3> &pc =
            jerkOpt_.get_b().block<6, 3>(pi * 6, 0);
        const double pstep = jerkOpt_.get_T1()(pi) / K;
        const double pzhi =
            (alt_zhi_pieces_.size() == N) ? alt_zhi_pieces_(pi) : alt_zhi_;
        double ps1 = 0.0;
        Eigen::Matrix<double, 6, 1> pb0, pb1, pb2;
        for (int pj = 0; pj <= K; ++pj) {
          const double ps2 = ps1 * ps1;
          const double ps3 = ps2 * ps1;
          const double ps4 = ps2 * ps2;
          const double ps5 = ps4 * ps1;
          pb0 << 1.0, ps1, ps2, ps3, ps4, ps5;
          pb1 << 0.0, 1.0, 2.0 * ps1, 3.0 * ps2, 4.0 * ps3, 5.0 * ps4;
          pb2 << 0.0, 0.0, 2.0, 6.0 * ps1, 12.0 * ps2, 20.0 * ps3;
          const Eigen::Vector3d ppos = pc.transpose() * pb0;
          const Eigen::Vector3d pvel = pc.transpose() * pb1;
          const Eigen::Vector3d pacc = pc.transpose() * pb2;
          const int s = pi * (K + 1) + pj;

          Eigen::Vector3d pg;
          double pcst;

          par_obs_found_[s] = sdfGradCostP(s, ppos, pg, pcst, use_sdf) ? 1 : 0;
          par_obs_grad_.col(s) = pg;
          par_obs_cost_(s) = pcst;

          int pslot = 6;
          par_floor_found_[s] =
              unifiedFloorTermP(ppos, &pg, &pcst, &pslot) ? 1 : 0;
          par_floor_grad_.col(s) = pg;
          par_floor_cost_(s) = pcst;
          par_floor_slot_[s] = pslot;

          if (use_risk_zones_) {
            Eigen::Vector3d rgp, rgv;
            double rcp;
            risk_par_found_[s] =
                RiskGradCostP(s, ppos, pvel, rgp, rgv, rcp) ? 1 : 0;
            risk_par_gradp_.col(s) = rgp;
            risk_par_gradv_.col(s) = rgv;
            risk_par_costp_(s) = rcp;
          } else {
            risk_par_found_[s] = 0;
          }

          par_cap_found_[s] = altCapTermP(ppos, pzhi, &pg, &pcst) ? 1 : 0;
          par_cap_grad_.col(s) = pg;
          par_cap_cost_(s) = pcst;

          par_dyn_found_[s] = 0;
          if (dyn_on) {
            Eigen::Vector3d dgp, dgv, dga;
            double dcst;
            // Outputs are written only on success — store only then.
            if (fixedWingDynamicsGradCostPVA(ppos, pvel, pacc, dgp, dgv, dga,
                                             dcst)) {
              par_dyn_found_[s] = 1;
              par_dyn_gp_.col(s) = dgp;
              par_dyn_gv_.col(s) = dgv;
              par_dyn_ga_.col(s) = dga;
              par_dyn_cost_(s) = dcst;
            }
          }

          ps1 += pstep;
        }
      }
    }

    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // SDF-based obstacle penalty. The ground crash-plane / virtual-ceiling
        // half-spaces are evaluated INSIDE sdfGradCostP before its SDF lookup;
        // they are a safety guarantee, not an obstacle term, so the call is no
        // longer gated on SDF presence / enable_obstacles_ — only the SDF box
        // penalty is (via use_sdf). [RISK-PAR]: precomputed record when active.
        {
            bool obs_found;
            if (sample_par) {
                const int s = i * (K + 1) + j;
                obs_found = par_obs_found_[s] != 0;
                if (obs_found) {
                    gradp = par_obs_grad_.col(s);
                    costp = par_obs_cost_(s);
                }
            } else {
                obs_found = sdfGradCostP(i_dp, pos, gradp, costp, use_sdf);
            }
            if (obs_found) {
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradp.transpose() * vel;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                costs(0) += omg * step * costp;
            }
        }

        // UNIFIED FLOOR (z-redesign Stage 2): one floor surface
        //   F(x,y) = max(terrain + clearance, mission-altitude anchor)
        // is the single source of truth for how low z may go, replacing the
        // separate 2.5D-terrain (slot 8) and alt-floor (slot 6) penalties. Only
        // the GOVERNING floor is penalised: where the terrain-clearance floor is
        // higher it rules (cubic wei_obs — the safety floor that MUST hold; the
        // heightmap sees the fine-DEM penetration the 10 m SDF misses; grad is
        // surface-normal so the path skirts slopes rather than spiking over
        // them); elsewhere the anchor rules (quadratic wei_alt — soft anti-sag,
        // nothing terrain-forced ever dives below mission altitude). Firing only
        // the governing term drops the old overlap double-count (both pushed
        // below the anchor over terrain) and hands the FE/panel/later stages ONE
        // floor to agree on. The crash-plane half-space (ground barrier,
        // sdfGradCostP) stays SEPARATE — a hard barrier of a different class.
        // enable_obstacles_ (the "ignore obstacles" debug flag) silences only
        // the terrain floor (have_terr=false -> anchor governs), never the
        // anchor.
        // (math lives in unifiedFloorTermP — shared with the [RISK-PAR]
        // precompute; records are bit-identical to the inline call.)
        {
            bool floor_found;
            Eigen::Vector3d grad3;
            double costf = 0.0;
            int slot = 6;
            if (sample_par) {
                const int s = i * (K + 1) + j;
                floor_found = par_floor_found_[s] != 0;
                if (floor_found) {
                    grad3 = par_floor_grad_.col(s);
                    costf = par_floor_cost_(s);
                    slot = par_floor_slot_[s];
                }
            } else {
                floor_found = unifiedFloorTermP(pos, &grad3, &costf, &slot);
            }
            if (floor_found) {
                gradViolaPc = beta0 * grad3.transpose();
                gradViolaPt = alpha * grad3.transpose() * vel;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costf / K + step * gradViolaPt);
                costs(slot) += omg * step * costf;
            }
        }

        double gradt, grad_prev_t;

        // Swarm collision cost calculation - now computed for every point for maximum accuracy
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)) {
            gradViolaPc = beta0 * gradp.transpose();
            gradViolaPt = alpha * gradt;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
            gdT(i) += omg * (costp / K + step * gradViolaPt);
            if (i > 0) {
                gdT.head(i).array() += omg * step * grad_prev_t;
            }
            costs(1) += omg * step * costp;
        }

        // Formation cost calculation - now computed for every point for maximum accuracy
        if (use_formation_) {
            if (swarmGraphGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)) {
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradt;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                if (i > 0) {
                    gdT.head(i).array() += omg * step * grad_prev_t;
                }
                costs(2) += omg * step * costp;
            }
        }

        // Risk zone cost (FM2-shared OR-moat field, arc-length integral
        // r(p)*||v||; see RiskGradCostP). Position AND velocity couple in.
        // [RISK-PAR] with the parallel precompute active the (bit-identical)
        // stored record replaces the inline call; accumulation is unchanged.
        {
            bool risk_found = false;
            if (sample_par) {
                const int s = i * (K + 1) + j;
                risk_found = risk_par_found_[s] != 0;
                if (risk_found) {
                    gradp = risk_par_gradp_.col(s);
                    gradv = risk_par_gradv_.col(s);
                    costp = risk_par_costp_(s);
                }
            } else if (use_risk_zones_) {
                risk_found = RiskGradCostP(i_dp, pos, vel, gradp, gradv, costp);
            }
            if (risk_found) {
                gradViolaPc = beta0 * gradp.transpose() + beta1 * gradv.transpose();
                gradViolaPt = alpha * (gradp.dot(vel) + gradv.dot(acc));
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                costs(3) += omg * step * costp;
            }
        }

        // Altitude-band cap. Own slot (6) — it used to share the risk slot
        // and made zone-risk diagnosis impossible when a terrain-forced climb
        // (front-end leaves the band only where terrain demands it) was the
        // real contributor. Down-side is covered by ground/obstacle terms.
        // ARC-VARYING: per-piece ceiling when alt_zhi_pieces_ is populated
        // (see optimizeFromPath), scalar alt_zhi_ otherwise. zhi_i is constant
        // w.r.t. the decision variables, so the gradients below are exact
        // either way. MUST MATCH logVerticalAttribution's cap recompute.
        // TERRAIN-AWARE GATE — root fix for cap-vs-terrain penetration.
        // alt_zhi_ is a single SCALAR (front-end geodesic max z + headroom),
        // but the sparse-piece back-end corner-cuts across terrain HIGHER
        // than that scalar. There the cap ("come down to alt_zhi_") and the
        // terrain penalty ("stay clear of terrain") are physically
        // unsatisfiable: L-BFGS stalls (-1004/-1005) and the crest is pressed
        // into the DEM. Gate the cap by TERRAIN CLEARANCE so the two are
        // mutually exclusive by construction: within obstacle_clearance_ of
        // the surface the cap is OFF (terrain rules; the terrain term lifts
        // the crest); a smoothstep hands control back to the cap once the
        // point is safely clear (>= 2*clearance), where its only job —
        // stopping the quintic ballooning above the committed profile —
        // applies unchanged. KEYED ON THE HEIGHTMAP (z - h), not the SDF
        // (terrain is not voxelised into the SDF; an SDF-keyed gate pinned to
        // 1 and pressed the crest -15.7 m into the DEM); SDF keying remains
        // only as a fallback when no heightmap is wired.
        // (gate + cost math lives in altCapTermP — shared with the
        // [RISK-PAR] precompute; records are bit-identical to inline.)
        {
            const double zhi_i =
                (alt_zhi_pieces_.size() == N) ? alt_zhi_pieces_(i) : alt_zhi_;
            bool cap_found;
            Eigen::Vector3d grad_a;
            double costa_z = 0.0;
            if (sample_par) {
                const int s = i * (K + 1) + j;
                cap_found = par_cap_found_[s] != 0;
                if (cap_found) {
                    grad_a = par_cap_grad_.col(s);
                    costa_z = par_cap_cost_(s);
                }
            } else {
                cap_found = altCapTermP(pos, zhi_i, &grad_a, &costa_z);
            }
            if (cap_found) {
                gradViolaPc = beta0 * grad_a.transpose();
                gradViolaPt = alpha * grad_a.transpose() * vel;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costa_z / K + step * gradViolaPt);
                costs(6) += omg * step * costa_z;
            }
        }

        // Altitude-band floor (soft anti-sag) is now the anchor branch of the
        // UNIFIED FLOOR above (slot 6, quadratic wei_alt) — folded in so there is
        // one floor surface and no terrain/anchor overlap double-count.

        // Feasibility cost calculation
        if (feasibilityGradCostV(vel, gradv, costv)) {
            gradViolaVc = beta1 * gradv.transpose();
            gradViolaVt = alpha * gradv.transpose() * acc;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
            gdT(i) += omg * (costv / K + step * gradViolaVt);
            costs(4) += omg * step * costv;
        }
        if (feasibilityGradCostA(acc, grada, costa)) {
            gradViolaAc = beta2 * grada.transpose();
            gradViolaAt = alpha * grada.transpose() * jer;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
            gdT(i) += omg * (costa / K + step * gradViolaAt);
            costs(4) += omg * step * costa;
        }

        // [RIDE] speed-over-roughness energy: wei_ride * rough_i * |v|^2 with
        // rough_i FROZEN per solve from the committed chord (slope excess,
        // AGL-faded) — the gradient touches only v, no terrain Hessian. This
        // is the direct local slow-down price the uniform time cost cannot
        // express: the time-relief variant measured durations flat while
        // clearance slipped, because making time cheap does not make speed
        // expensive. Piece-indexed like the cap (i is the piece index here).
        if (wei_ride_ > 0.0 &&
            static_cast<int>(ride_rough_pieces_.size()) == N) {
            const double r = ride_rough_pieces_(i);
            if (r > 1e-9) {
                // HORIZONTAL speed only: the full |v|^2 form measured knoe
                // clearance -4~-10 m — it taxed the climb/descent rate that
                // terrain-following IS, flattening z over crests. Ground
                // speed is the ride driver; vertical agility stays free.
                const double v2xy = vel.x() * vel.x() + vel.y() * vel.y();
                const double costr = wei_ride_ * r * v2xy;
                const Eigen::Vector3d gradvr(2.0 * wei_ride_ * r * vel.x(),
                                             2.0 * wei_ride_ * r * vel.y(),
                                             0.0);
                gradViolaVc = beta1 * gradvr.transpose();
                gradViolaVt = alpha * gradvr.transpose() * acc;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) +=
                    omg * step * gradViolaVc;
                gdT(i) += omg * (costr / K + step * gradViolaVt);
                costs(9) += omg * step * costr;   // slot 9 = ride (own slot,
                                                  // not the feasibility slot)
            }
        }

        // Fixed-wing inverse dynamics: couples position (density), velocity,
        // and acceleration through lift/drag/thrust and the flight envelope.
        // [RISK-PAR]: precomputed record when active.
        {
            double costdyn = 0.0;
            bool dyn_found;
            if (sample_par) {
                const int s = i * (K + 1) + j;
                dyn_found = par_dyn_found_[s] != 0;
                if (dyn_found) {
                    gradp = par_dyn_gp_.col(s);
                    gradv = par_dyn_gv_.col(s);
                    grada = par_dyn_ga_.col(s);
                    costdyn = par_dyn_cost_(s);
                }
            } else {
                dyn_found = dynamics_enable_ && wei_dynamics_ > 0.0 &&
                    fixedWingDynamicsGradCostPVA(pos, vel, acc, gradp, gradv,
                                                 grada, costdyn);
            }
            if (dyn_found) {
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaVc = beta1 * gradv.transpose();
                gradViolaAc = beta2 * grada.transpose();
                gradViolaVt = alpha *
                    (gradp.dot(vel) + gradv.dot(acc) + grada.dot(jer));
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) +=
                    omg * step * (gradViolaPc + gradViolaVc + gradViolaAc);
                gdT(i) += omg * (costdyn / K + step * gradViolaVt);
                costs(7) += omg * step * costdyn;
            }
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1)) {
            ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    // Distance variance cost (spreads inner points evenly).
    {
      Eigen::MatrixXd &gdp = sqrvar_gdp_;
      double var;
      distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

      // (A pass-1 beta/vel cache was tried here and REVERTED: forcing beta0
      // to memory every sample in the main pass disturbed its vectorization
      // and cost ~3% net on the gauntlet A/B — recomputing is cheaper.)
      i_dp = 0;
      for (int i = 0; i < N; ++i) {
        step = jerkOpt_.get_T1()(i) / K;
        s1 = 0.0;
        for (int j = 0; j <= K; ++j) {
          s2 = s1 * s1;
          s3 = s2 * s1;
          s4 = s2 * s2;
          s5 = s4 * s1;
          beta0 << 1.0, s1, s2, s3, s4, s5;
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
          alpha = 1.0 / K * j;
          vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;
          omg = (j == 0 || j == K) ? 0.5 : 1.0;
          gradViolaPc = beta0 * gdp.col(i_dp).transpose();
          gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
          gdT(i) += omg * (gradViolaPt);
          s1 += step;
          if (j != K || (j == K && i == N - 1)) {
              ++i_dp;
          }
        }
      }
      costs(5) += var;
    }

    dbg_cost_formation_ = costs(2);

  }

  bool PolyTrajOptimizer::swarmGraphGradCostP(const int i_dp,
                                              const double t,
                                              const Eigen::Vector3d &p,
                                              const Eigen::Vector3d &v,
                                              Eigen::Vector3d &gradp,
                                              double &gradt,
                                              double &grad_prev_t,
                                              double &costp)
  {
    (void)i_dp;  // guard removed: consider all control points.
    if (!swarm_trajs_) {
      return false;
    }

    // The buffer is indexed by drone id and the LAST drone's own slot is
    // absent by construction, so it needs one entry fewer than everyone else.
    // (The old "force size = formation_size_" shortcut bypassed the guard and
    // made at(id) throw at startup before all peers had broadcast.)
    const int have = static_cast<int>(swarm_trajs_->size());
    const int need = (drone_id_ == formation_size_ - 1) ? formation_size_ - 1
                                                        : formation_size_;
    if (have < need)
      return false;

    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    double pt_time = t_now_ + t;
    std::vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;

    // Bound by formation_size_, NOT the raw buffer size: a buffer with extra
    // (non-formation) drones overran the formation-sized vectors above.
    for (int id = 0; id < formation_size_; id++)
    {
      if (id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    swarm_graph_->updateGraph(swarm_graph_pos);

    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);

    debug_similarity_ = similarity_error;

    if (similarity_error > 0)
    {
      ret = true;

      costp = wei_formation_ * similarity_error;
      std::vector<Eigen::Vector3d> swarm_grad;
      swarm_graph_->getGrad(swarm_grad);

      gradp = wei_formation_ * swarm_grad[drone_id_];

      for (int id = 0; id < formation_size_; id++)
      {
        gradt += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
        if (id != drone_id_)
          grad_prev_t += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
      }
    }

    return ret;
  }

  // SDF-based obstacle penalty.
  // Cubic penalty (matches the main-branch obstacleGradCostP that was already
  // tuned against wei_obs_ ~ 1e4..5e4). smoothedL1 was inherited from GCOPTER
  // and produced a flat, oversized gradient (~wei_obs_) for any violation
  // > smoothing_eps, which whipsawed L-BFGS line search and produced
  // loop-shaped trajectories at the start of the plan.
  bool PolyTrajOptimizer::sdfGradCostP(const int i_dp,
                                        const Eigen::Vector3d &p,
                                        Eigen::Vector3d &gradp,
                                        double &costp,
                                        bool use_sdf)
  {
    (void)i_dp;
    gradp.setZero();
    costp = 0;

    float d = 0.0f;
    Eigen::Vector3d grad_d = Eigen::Vector3d::Zero();

    // Ground / ceiling hard half-spaces. Above the plane they contribute
    // NOTHING (sea-skim missions may legally fly at any z > ground), so the
    // penalty below MUST rise from ZERO at the plane — the earlier form
    // jumped straight to (clearance + depth)³ (≈270 at depth 0⁺), a cost
    // DISCONTINUITY that line searches die on the moment a low-altitude
    // iterate grazes the surface (-1005; ordinary missions never flew within
    // the band so it stayed latent). viol = 3·depth keeps the push steep and
    // strictly deeper-is-worse (no cheap dive-escapes) while the cubic makes
    // the crossing C²-continuous.
    if (ground_height_ > -0.5 && p.z() < ground_height_) {
      // BARRIER-grade weight, not wei_obs_: the surface is a crash plane, so
      // no soft term may buy its way below it. At wei_obs_ (10000) the
      // sub-stall recovery dive of an envelope-infeasible start state
      // (0,0,+50 m/s commanded) out-pushed this half-space and converged at
      // z = -0.093 (9 m SUBMERGED); the dynamics min-speed hinge scales with
      // wei_dynamics_*penalty_speed = 50000, so the plane must sit above
      // every purchasable gradient. Same C² cubic form (line-search safe).
      const double viol = 3.0 * (ground_height_ - p.z());
      costp = wei_ground_barrier_ * viol * viol * viol;
      gradp = Eigen::Vector3d(0.0, 0.0, -wei_ground_barrier_ * 9.0 * viol * viol);
      return true;
    }
    if (virtual_ceil_height_ > -0.5 && p.z() > virtual_ceil_height_) {
      const double viol = 3.0 * (p.z() - virtual_ceil_height_);
      costp = wei_obs_ * viol * viol * viol;
      gradp = Eigen::Vector3d(0.0, 0.0, wei_obs_ * 9.0 * viol * viol);
      return true;
    }
    if (!use_sdf || !sdf_manager_ || !sdf_manager_->hasData()) return false;
    if (!sdf_manager_->getDistanceAndGradient(p, &d, &grad_d)) return false;
    if (!std::isfinite(d)) return false;

    const double violation = obstacle_clearance_ - static_cast<double>(d);
    if (violation <= 0.0) return false;

    costp = wei_obs_ * violation * violation * violation;
    gradp = -wei_obs_ * 3.0 * violation * violation * grad_d;
    return true;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    (void)i_dp;  // guard removed: consider all control points.
    // Check for nullptr before accessing swarm_trajs_
    if (!swarm_trajs_) {
      return false;
    }

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
      {
        continue;
      }

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }

      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 +
                            (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;
        costp += wei_swarm_ * dist2_err3;
        Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) *
                                    Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }
    }
    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    // Cubic penalty (main-branch style). smoothedL1 saturated the gradient at
    // ~wei_feas_ once violation exceeded smoothing_eps, letting L-BFGS inflate
    // duration to satisfy max-vel instead of deforming the path.
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6.0 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    // Legacy frame-unit minimum-speed floor, normally disabled. The shared
    // physical dynamics model now owns the steady-flight speed envelope and
    // introduces it smoothly above its activation speed.
    // Gradient note: d/dv (m^2 - |v|^2)^3 = -6 (m^2 - |v|^2)^2 v — the push
    // vanishes AT v = 0 (saddle); acceptable because the floor is only
    // enabled on missions that start at speed.
    if (min_vel_ > 0.0)
    {
      const double lpen = min_vel_ * min_vel_ - v.squaredNorm();
      if (lpen > 0)
      {
        gradv = -wei_feas_ * 6.0 * lpen * lpen * v;
        costv = wei_feas_ * lpen * lpen * lpen;
        return true;
      }
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    // Cubic penalty (main-branch style).
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6.0 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  // UNIFIED FLOOR (z-redesign Stage 2) — pure per-sample computation. The
  // governing-floor doctrine and slot semantics are documented at the call
  // site in addPVAGradCost2CT; math moved here verbatim so the [RISK-PAR]
  // precompute and the serial inline path share one implementation.
  bool PolyTrajOptimizer::unifiedFloorTermP(const Eigen::Vector3d &pos,
                                            Eigen::Vector3d *grad3,
                                            double *cost, int *slot)
  {
    double terr_floor = -1e30;
    float h = 0.f, dhx = 0.f, dhy = 0.f;
    Eigen::Vector2d tgrad(0.0, 0.0);
    bool have_terr = false;
    if (enable_obstacles_ && (terrain_hgrad_ || terrain_height_)) {
        if (terrain_hgrad_) {
            // Value + ANALYTIC slope of the SAME bilinear surface (a
            // smoothed central-diff slope disagreed near DEM-cell edges
            // and killed the line search on cliff cells, -1008).
            have_terr = terrain_hgrad_(pos.x(), pos.y(), &h, &dhx, &dhy);
        } else {
            const float hv = terrain_height_(pos.x(), pos.y());
            if (std::isfinite(hv)) { h = hv; have_terr = true; }  // value-only push
        }
        if (have_terr)
            terr_floor = static_cast<double>(h) +
                         terrainClearanceTarget(pos, &tgrad);
    }
    // [TERRAIN-TAPER] both floors relax toward a below-band pinned
    // endpoint; their analytic d(target)/dxy joins the gradient below.
    Eigen::Vector2d fgrad(0.0, 0.0);
    const double anchor_floor =
        (wei_alt_ > 0.0) ? altitudeFloorTarget(pos, &fgrad) : -1e30;

    const bool terrain_governs = have_terr && terr_floor >= anchor_floor;
    const double F = terrain_governs ? terr_floor : anchor_floor;
    if (!(F > -1e29 && pos.z() < F)) return false;

    const double viol = F - pos.z();
    if (terrain_governs) {
        *cost = wei_obs_ * viol * viol * viol;
        const double dcoef = wei_obs_ * 3.0 * viol * viol;
        *grad3 = Eigen::Vector3d(dcoef * (dhx + tgrad.x()),
                                 dcoef * (dhy + tgrad.y()), -dcoef);
        *slot = 8;
    } else {
        *cost = wei_alt_ * viol * viol;
        const double dua = wei_alt_ * 2.0 * viol;
        *grad3 = Eigen::Vector3d(dua * fgrad.x(), dua * fgrad.y(), -dua);
        *slot = 6;
    }
    return true;
  }

  // ARC-VARYING altitude cap with the terrain-aware smoothstep gate — pure
  // per-sample computation; doctrine documented at the call site. Moved here
  // verbatim (same sharing rationale as unifiedFloorTermP).
  bool PolyTrajOptimizer::altCapTermP(const Eigen::Vector3d &pos, double zhi_i,
                                      Eigen::Vector3d *grad, double *cost)
  {
    if (!(wei_alt_ > 0.0 && zhi_i >= 0.0 && pos.z() > zhi_i)) return false;

    double gate = 1.0;
    Eigen::Vector3d gate_grad = Eigen::Vector3d::Zero();
    const double glo = obstacle_clearance_;        // cap OFF at/below clearance
    const double ghi = 2.0 * obstacle_clearance_;  // cap fully ON above
    bool gated = false;
    if (terrain_hgrad_) {
        float hg = 0.f, hx = 0.f, hy = 0.f;
        if (terrain_hgrad_(pos.x(), pos.y(), &hg, &hx, &hy)) {
            const double tc = pos.z() - static_cast<double>(hg);
            double t = (ghi > glo) ? (tc - glo) / (ghi - glo) : 1.0;
            t = std::max(0.0, std::min(1.0, t));
            gate = t * t * (3.0 - 2.0 * t);            // smoothstep, C1
            // CONSISTENT gradient: the cost is wei*ua^2*gate(z - h(x,y)),
            // so the gradient MUST carry d(gate) with the ANALYTIC slope of
            // the same surface (frozen/smoothed slopes died in the band).
            if (t > 0.0 && t < 1.0) {
                const double dgate = 6.0 * t * (1.0 - t) / (ghi - glo);
                gate_grad = dgate * Eigen::Vector3d(-hx, -hy, 1.0);
            }
            gated = true;
        }
    }
    if (!gated && sdf_manager_ && sdf_manager_->hasData()) {
        // Same cost/gradient-pair rule: gate(d(pos)) carries dgate*grad_d.
        float d = 0.f;
        Eigen::Vector3d grad_d = Eigen::Vector3d::Zero();
        if (sdf_manager_->getDistanceAndGradient(pos, &d, &grad_d) &&
            std::isfinite(d)) {
            double t = (ghi > glo) ? (static_cast<double>(d) - glo) / (ghi - glo) : 1.0;
            t = std::max(0.0, std::min(1.0, t));
            gate = t * t * (3.0 - 2.0 * t);            // smoothstep, C1
            if (t > 0.0 && t < 1.0) {
                const double dgate = 6.0 * t * (1.0 - t) / (ghi - glo);
                gate_grad = dgate * grad_d;
            }
        }
    }
    if (gate <= 0.0) return false;

    // QUADRATIC, not cubic: see call-site doctrine (cap must never out-press
    // the clearance wall).
    const double ua = pos.z() - zhi_i;
    *cost = wei_alt_ * ua * ua * gate;
    // Full gradient of wei*ua^2*gate(·):
    //   d/dpos = wei*ua^2*gate_grad + (0,0, wei*2*ua*gate).
    *grad = wei_alt_ * ua * ua * gate_grad +
            Eigen::Vector3d(0.0, 0.0, wei_alt_ * 2.0 * ua * gate);
    return true;
  }

  bool PolyTrajOptimizer::fixedWingDynamicsGradCostPVA(
      const Eigen::Vector3d &p,
      const Eigen::Vector3d &v,
      const Eigen::Vector3d &a,
      Eigen::Vector3d &gradp,
      Eigen::Vector3d &gradv,
      Eigen::Vector3d &grada,
      double &cost)
  {
    const Eigen::Vector3d S(dyn_unit_xy_m_, dyn_unit_xy_m_, dyn_unit_z_m_);
    const Eigen::Vector3d pm = S.cwiseProduct(p);
    const Eigen::Vector3d vm = S.cwiseProduct(v);
    const Eigen::Vector3d am = S.cwiseProduct(a);
    const auto result = mmp_vehicle_dynamics::flightEnvelopePenalty(
        dynamics_params_, pm, vm, am, wei_dynamics_);
    if (result.cost <= 0.0) return false;

    cost = result.cost;
    gradp = S.cwiseProduct(result.position);
    gradv = S.cwiseProduct(result.velocity);
    grada = S.cwiseProduct(result.acceleration);
    return true;
  }

  // Same must-enter rule as the front-end's prepareBarrier (dyn_a_star.h):
  // a zone containing the plan start or goal cannot be avoided, so its
  // barrier is dropped and only the shared moat prices the crossing.
  void PolyTrajOptimizer::prepareRiskBarrier(const Eigen::Vector3d &start,
                                             const Eigen::Vector3d &goal)
  {
    zone_barrier_exempt_.assign(risk_zones_.size(), 0);
    auto in_zone = [this](const Eigen::Vector3d &p, const RiskZone &tz,
                          size_t zi) {
      const double rv = tz.vertical_reach > 0.0
                            ? tz.vertical_reach : tz.reach;
      if (!(tz.reach > 0.0) || !(rv > 0.0)) return false;
      const Eigen::Vector3d d = p - tz.center;
      const double q2 = d.head<2>().squaredNorm() /
                            (tz.reach * tz.reach) +
                        d.z() * d.z() / (rv * rv);
      if (q2 >= 1.0) return false;
      return !risk_visibility_ || risk_visibility_(zi, p, nullptr) > 0.5;
    };
    for (size_t i = 0; i < risk_zones_.size(); ++i) {
      if (in_zone(start, risk_zones_[i], i) ||
          in_zone(goal, risk_zones_[i], i)) {
        zone_barrier_exempt_[i] = 1;
        LOG_INFO("[RISK] zone %zu contains start/goal -> barrier exempt (moat only)", i);
      }
    }
  }

  void PolyTrajOptimizer::markFrontEndCrossings(
      const std::vector<Eigen::Vector3d> &path)
  {
    if (zone_barrier_exempt_.size() != risk_zones_.size()) return;
    for (size_t i = 0; i < risk_zones_.size(); ++i) {
      if (zone_barrier_exempt_[i]) continue;
      const auto &tz = risk_zones_[i];
      const double rv = tz.vertical_reach > 0.0
                            ? tz.vertical_reach : tz.reach;
      if (!(tz.reach > 0.0) || !(rv > 0.0)) continue;
      for (const auto &p : path) {
        const Eigen::Vector3d d = p - tz.center;
        const double q2 = d.head<2>().squaredNorm() /
                              (tz.reach * tz.reach) +
                          d.z() * d.z() / (rv * rv);
        if (q2 < 1.0) {
          if (risk_visibility_ && risk_visibility_(i, p, nullptr) <= 0.5)
            continue;
          zone_barrier_exempt_[i] = 1;
          LOG_INFO("[RISK] front-end route crosses zone %zu -> barrier exempt (moat only)",
                   i);
          break;
        }
      }
    }
  }

  // Risk cost: consumes the SAME continuous risk field as the FM2 front-end
  // (dyn_a_star.h getRiskNorm), so both layers price risk on one shared field:
  // per zone a terrain-masked ellipsoidal quadratic moat
  // m_i = visibility_i(p) * peak_i*(1 - q_i)^2,
  // q_i^2 = rho_i^2/Rh_i^2 + dz_i^2/Rv_i^2,
  // OR-composed  r(p) = 1 - prod_i(1 - min(m_i, 1-1e-3))  in [0, 1].
  //
  // Consumption is the ARC-LENGTH integral  ∫ (wei_risk * r + wei_barrier * b)
  // ||v|| dt — the trajectory-level analogue of the front-end metric
  // ds*(1 + alpha*risk + K*inside) (FM2 speed map F = 1/(1+cost), shortcut
  // edge cost and coarse Dijkstra all integrate risk over LENGTH). Geometric
  // like FM2's, so it cannot be gamed by flying through a zone faster; on a
  // saturated plateau the surviving gradient (via ||v||) shortens the in-zone
  // chord — the same "cross at the cheapest transit" the front-end picks.
  //
  // b is the SMOOTHED BARRIER: the front-end adds a flat K inside every
  // non-exempt ellipsoid, which makes its geodesic keep a hard standoff at the
  // rim (observed margins of only metres). A moat-only back-end re-litigates
  // that standoff: near the rim the moat is ~peak*u^2 with ZERO contact
  // slope, so cutting the skirt is net-profitable against the time cost
  // until tens of metres deep. The barrier indicator is therefore shared
  // too (a step has no usable gradient, so it is ramped), OR-composed like
  // the moat. The ramp sits OUTSIDE the rim — full K at d <= reach, fading
  // over the outer kBarrierRampFrac band — mirroring how the front-end's
  // coarse grid bleeds its +K one cell past the rim (any cell whose center
  // is inside slows the whole cell). The penetration equilibrium therefore
  // lands OUTSIDE the true ellipsoid and the sensing volume stays untouched,
  // instead of the ~1-2 m designed clip an inside ramp allowed. Zones
  // holding the plan start/goal are exempt — same must-enter rule as
  // prepareBarrier — so the back-end never fights a committed crossing.
  bool PolyTrajOptimizer::RiskGradCostP(const int i_dp,
                                           const Eigen::Vector3d &p,
                                           const Eigen::Vector3d &v,
                                           Eigen::Vector3d &gradp,
                                           Eigen::Vector3d &gradv,
                                           double &costp)
  {
    (void)i_dp;  // consider all constraint points
    gradp.setZero();
    gradv.setZero();
    costp = 0.0;

    constexpr double kMoatCap = 1.0 - 1e-3;  // identical to getRiskNorm
    // Ramp fraction ~ the front-end's coarse-cell bleed (cres ~ metres) at
    // typical zone sizes; steep enough that the approach equilibrium sits
    // outside the true rim. Shared with the setRiskZones precomputation.
    constexpr double kBarrierRampFrac = kRiskBarrierRampFrac;

    double Sm = 1.0;                               // moat survival prod(1-m_i)
    Eigen::Vector3d Gm = Eigen::Vector3d::Zero();  // sum grad(m_i)/(1-m_i)
    double Sb = 1.0;                               // barrier survival
    Eigen::Vector3d Gb = Eigen::Vector3d::Zero();
    for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
      const auto &tz = risk_zones_[zi];
      // AABB pre-filter on the ENLARGED support (barrier ramp lives outside
      // the rim); the moat keeps the exact getRiskNorm support d < reach.
      // The normalized ramp expands every ellipsoid semi-axis equally.
      // Geometry-only constants come precomputed from setRiskZones — this
      // loop runs per constraint point per iteration, and for far zones the
      // constants WERE the whole cost.
      const RiskZonePrep &zp = risk_zone_prep_[zi];
      if (!zp.valid) continue;
      const double reach_b = zp.reach_b;
      const double rv = zp.rv;
      const double rv_b = zp.rv_b;
      const double dz = p.z() - tz.center.z();
      if (std::abs(dz) >= rv_b) continue;
      const double dx = p.x() - tz.center.x();
      if (std::abs(dx) >= reach_b) continue;
      const double dy = p.y() - tz.center.y();
      if (std::abs(dy) >= reach_b) continue;
      const double q = std::sqrt(
          (dx * dx + dy * dy) / (tz.reach * tz.reach) +
          (dz * dz) / (rv * rv));
      if (q >= 1.0 + kBarrierRampFrac) continue;
      Eigen::Vector3d grad_q = Eigen::Vector3d::Zero();
      if (q > 1e-9) {
        grad_q = Eigen::Vector3d(
            dx / (tz.reach * tz.reach),
            dy / (tz.reach * tz.reach), dz / (rv * rv)) / q;
      }

      double visibility = 1.0;
      Eigen::Vector3d grad_visibility = Eigen::Vector3d::Zero();
      if (risk_visibility_) {
        visibility = std::clamp(
            risk_visibility_(zi, p, &grad_visibility), 0.0, 1.0);
      }

      // Shared terrain-masked moat. Product rule makes the LOS shadow edge
      // usable by L-BFGS while preserving the old radial gradient.
      if (q < 1.0) {
        const double u = 1.0 - q;
        const double base_m = tz.peak * u * u;
        const double m = std::min(base_m * visibility, kMoatCap);
        Sm *= (1.0 - m);
        Eigen::Vector3d grad_base_m = Eigen::Vector3d::Zero();
        if (m < kMoatCap && q > 1e-9) {
          grad_base_m = (tz.peak * 2.0 * u * -1.0) * grad_q;
        }
        if (m < kMoatCap && (1.0 - m) > 1e-9) {
          Gm += (visibility * grad_base_m + base_m * grad_visibility) /
                (1.0 - m);
        }
      }

      // Smoothed barrier on non-exempt zones (front-end: +K inside ANY such
      // zone -> OR of indicators; smooth OR = 1 - prod(1 - s_i)).
      const bool exempt =
          zi < zone_barrier_exempt_.size() && zone_barrier_exempt_[zi];
      if (wei_risk_barrier_ > 0.0 && !exempt) {
        const double t = std::min(
            ((1.0 + kBarrierRampFrac) - q) / kBarrierRampFrac,
            1.0);  // 0 at outer support, 1 at the ellipsoid rim
        // Smoothstep ramp: C1 at BOTH ends. (t^2 had a derivative kink at the
        // saturation circle d = reach — exactly the kind of stiff feature
        // L-BFGS line searches die on.) Saturated interior stays flat by
        // design — chord shortening via ||v|| still applies.
        const double base_s = t * t * (3.0 - 2.0 * t);
        Eigen::Vector3d grad_base_s = Eigen::Vector3d::Zero();
        if (t < 1.0 && q > 1e-9) {
          grad_base_s =
              (6.0 * t * (1.0 - t) *
               (-1.0 / kBarrierRampFrac)) * grad_q;
        }
        const double s = base_s * visibility;
        Sb *= (1.0 - s);
        if ((1.0 - s) > 1e-9) {
          Gb += (visibility * grad_base_s + base_s * grad_visibility) /
                (1.0 - s);
        }
      }
    }
    const double r = 1.0 - Sm;
    const double b = 1.0 - Sb;
    if (r <= 0.0 && b <= 0.0) return false;

    const double vnorm = v.norm();
    const double density = wei_risk_ * r + wei_risk_barrier_ * b;  // per length
    costp = density * vnorm;
    gradp = vnorm * (wei_risk_ * (Sm * Gm) + wei_risk_barrier_ * (Sb * Gb));
    if (vnorm > 1e-9) gradv = density * (v / vnorm);
    return true;
  }

  // Even-progression term: variance of squared HORIZONTAL spacing between
  // consecutive constraint points. Its job is to keep waypoint PROGRESSION
  // along the route uniform (degenerate short pieces make the MINCO time
  // allocation loiter — looping knots). Progression is a horizontal notion;
  // the 3D version had a loophole: over open water z is penalty-free (between
  // the alt floor and cap), so the cheapest way to "equalize" a short head
  // piece was to BULGE IT VERTICALLY — reproducible 68 m humps at the start
  // of over-water missions (seed 60978556; initial guess flat, all spatial
  // gradients zero, hump vanished with wei_sqrvar=0). Measuring xy spacing
  // closes the loophole while keeping the anti-loitering force intact; z
  // geometry stays owned by terrain/band/smoothness. Gradient z-component is
  // zero by construction (cost does not depend on z).
  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    // Member scratch (called once per cost evaluation): no per-call heap churn.
    Eigen::MatrixXd &dps = sqrvar_dps_;
    dps.resize(3, N);
    dps = ps.rightCols(N) - ps.leftCols(N);
    dps.row(2).setZero();  // horizontal spacing only (see block comment)
    Eigen::VectorXd &dsqrs = sqrvar_dsqrs_;
    dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(3, N + 1);
    gdp.setZero();

    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
  }

  void PolyTrajOptimizer::setParam(const rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    node_->declare_parameter("optimization/constrain_points_perPiece", 3);
    node_->get_parameter("optimization/constrain_points_perPiece", cps_num_prePiece_);
    // K = cps_num_prePiece_ is the per-piece integration divisor in
    // addPVAGradCost2CT (step = T1/K, alpha = j/K, cost += .../K) and the
    // constraint-point count in getInitConstrainPoints (pts sized N*K+1, then
    // written at i_dp = 0..N-1). K < 1 gives +inf/NaN gradients AND an
    // out-of-bounds pts write — clamp so a mistuned param cannot corrupt the
    // solve or crash.
    if (cps_num_prePiece_ < 1) {
        if (log_manager_)
            log_manager_->warnf(
                "[PARAM] constrain_points_perPiece=%d < 1; clamping to 1",
                cps_num_prePiece_);
        cps_num_prePiece_ = 1;
    }

    // [RISK-PAR] 0 = all cores, 1 = serial inline (pre-parallel code path),
    // n > 1 = capped. Results are bit-identical for every setting.
    node_->declare_parameter("optimization/risk_parallel_threads", 0);
    node_->get_parameter("optimization/risk_parallel_threads",
                         risk_parallel_threads_);
    if (risk_parallel_threads_ < 0) risk_parallel_threads_ = 0;

    node_->declare_parameter("enable_obstacles", true);
    node_->get_parameter("enable_obstacles", enable_obstacles_);
    
    // Get enable_debug_logs parameter (declared in replan_fsm)
    node_->get_parameter("enable_debug_logs", enable_debug_logs_);

    // Declare + get (bug fix: this param was previously read without being
    // declared, so it always fell back to default-constructed false).
    if (!node_->has_parameter("enable_lbfgs_detail_logs")) {
        node_->declare_parameter("enable_lbfgs_detail_logs", false);
    }
    node_->get_parameter("enable_lbfgs_detail_logs", enable_lbfgs_detail_logs_);
    
    // Use conditional logging - only RCLCPP when debug logs disabled, only LogManager when enabled
    if (!enable_debug_logs_) {
        RCLCPP_INFO(node_->get_logger(), "Obstacle avoidance: %s", enable_obstacles_ ? "enabled" : "disabled");
        RCLCPP_INFO(node_->get_logger(), "Debug logging: disabled (using RCLCPP only)");
    }
    node_->declare_parameter("optimization/weight_obstacle", 1000.0);
    node_->get_parameter("optimization/weight_obstacle", wei_obs_);
    // Crash-plane barrier: must dominate the strongest soft gradient
    // (wei_dynamics_ * penalty_speed = 50000 in the live config) — see the
    // ground half-space in sdfGradCostP.
    node_->declare_parameter("optimization/weight_ground_barrier", 250000.0);
    node_->get_parameter("optimization/weight_ground_barrier", wei_ground_barrier_);
    node_->declare_parameter("optimization/weight_swarm", 0.0);
    node_->get_parameter("optimization/weight_swarm", wei_swarm_);
    node_->declare_parameter("optimization/weight_feasibility", 1.0);
    node_->get_parameter("optimization/weight_feasibility", wei_feas_);
    node_->declare_parameter("optimization/weight_sqrvariance", 1.0);
    node_->get_parameter("optimization/weight_sqrvariance", wei_sqrvar_);
    node_->declare_parameter("optimization/weight_time", 0.0);
    node_->get_parameter("optimization/weight_time", wei_time_);
    node_->declare_parameter("optimization/weight_formation", 0.0);
    node_->get_parameter("optimization/weight_formation", wei_formation_);
    wei_formation_base_ = wei_formation_;  // Store base weight for adaptive adjustment

    node_->declare_parameter("optimization/weight_smoothness", 1.0);
    node_->get_parameter("optimization/weight_smoothness", wei_smooth_);
    node_->declare_parameter("optimization/weight_Risk", 0.0);
    node_->get_parameter("optimization/weight_Risk", wei_risk_);
    node_->declare_parameter("optimization/weight_Risk_barrier", 0.0);
    node_->get_parameter("optimization/weight_Risk_barrier", wei_risk_barrier_);

    node_->declare_parameter("optimization/dynamics_enable", true);
    node_->get_parameter("optimization/dynamics_enable", dynamics_enable_);
    node_->declare_parameter("optimization/weight_dynamics", 0.0);
    node_->get_parameter("optimization/weight_dynamics", wei_dynamics_);
    node_->declare_parameter("optimization/dynamics_unit_xy_m", 100.0);
    node_->get_parameter("optimization/dynamics_unit_xy_m", dyn_unit_xy_m_);
    node_->declare_parameter("optimization/dynamics_unit_z_m", 100.0);
    node_->get_parameter("optimization/dynamics_unit_z_m", dyn_unit_z_m_);
    auto dynamics_param = [this](const char *name, double default_value,
                                 double &value) {
      node_->declare_parameter(name, default_value);
      node_->get_parameter(name, value);
    };
    dynamics_param("optimization/dynamics_mass_kg", 1300.0,
                   dynamics_params_.mass_kg);
    dynamics_param("optimization/dynamics_wing_area_m2", 1.0,
                   dynamics_params_.wing_area_m2);
    dynamics_param("optimization/dynamics_g", 9.80665,
                   dynamics_params_.gravity_mps2);
    dynamics_param("optimization/dynamics_rho0_kgpm3", 1.225,
                   dynamics_params_.sea_level_density_kgpm3);
    dynamics_param("optimization/dynamics_density_scale_height_m", 8500.0,
                   dynamics_params_.density_scale_height_m);
    dynamics_param("optimization/dynamics_altitude_reference_m", 0.0,
                   dynamics_params_.altitude_reference_m);
    dynamics_param("optimization/dynamics_cd0", 0.035,
                   dynamics_params_.zero_lift_drag_coefficient);
    dynamics_param("optimization/dynamics_induced_drag_factor", 0.080,
                   dynamics_params_.induced_drag_factor);
    dynamics_param("optimization/dynamics_cl_min", -0.40,
                   dynamics_params_.lift_coefficient_min);
    dynamics_param("optimization/dynamics_cl_max", 1.40,
                   dynamics_params_.lift_coefficient_max);
    dynamics_param("optimization/dynamics_load_factor_max", 2.50,
                   dynamics_params_.load_factor_max);
    dynamics_param("optimization/dynamics_thrust_min_n", 0.0,
                   dynamics_params_.thrust_min_n);
    dynamics_param("optimization/dynamics_thrust_max_n", 3200.0,
                   dynamics_params_.thrust_max_n);
    dynamics_param("optimization/dynamics_speed_min_mps", 120.0,
                   dynamics_params_.speed_min_mps);
    dynamics_param("optimization/dynamics_speed_max_mps", 230.0,
                   dynamics_params_.speed_max_mps);
    dynamics_param("optimization/dynamics_activation_speed_mps", 40.0,
                   dynamics_params_.model_activation_speed_mps);
    dynamics_param("optimization/dynamics_dynamic_pressure_max_pa", 45000.0,
                   dynamics_params_.dynamic_pressure_max_pa);
    double bank_max_deg = 65.0;
    double flight_path_max_deg = 30.0;
    dynamics_param("optimization/dynamics_bank_max_deg", 65.0,
                   bank_max_deg);
    dynamics_param("optimization/dynamics_flight_path_max_deg", 30.0,
                   flight_path_max_deg);
    dynamics_params_.bank_angle_max_rad = bank_max_deg * M_PI / 180.0;
    dynamics_params_.flight_path_angle_max_rad =
        flight_path_max_deg * M_PI / 180.0;
    dynamics_param("optimization/dynamics_margin", 0.02,
                   dynamics_params_.constraint_margin);
    dynamics_param("optimization/dynamics_penalty_speed", 5.0,
                   dynamics_params_.penalty_speed);
    dynamics_param("optimization/dynamics_penalty_lift", 2.0,
                   dynamics_params_.penalty_lift);
    dynamics_param("optimization/dynamics_penalty_load", 2.0,
                   dynamics_params_.penalty_load);
    dynamics_param("optimization/dynamics_penalty_thrust", 2.0,
                   dynamics_params_.penalty_thrust);
    dynamics_param("optimization/dynamics_penalty_dynamic_pressure", 1.0,
                   dynamics_params_.penalty_dynamic_pressure);
    dynamics_param("optimization/dynamics_penalty_bank", 1.0,
                   dynamics_params_.penalty_bank);
    dynamics_param("optimization/dynamics_penalty_flight_path", 1.0,
                   dynamics_params_.penalty_flight_path);
    // SELF-CONSISTENCY of the declared envelope (valid != consistent):
    // level flight at speed v needs CL = 2 m g / (rho v^2 S); the AERO stall
    // speed implied by CL_max must not exceed the declared speed_min, or the
    // envelope contains no feasible level-flight state at its own minimum
    // speed and the audit will report unavoidable lift-coefficient
    // violations right at cruise entry (observed: CL 1.86 at V=120 when the
    // implied stall was 122 m/s). Same idea for a sustained turn at
    // n = load_factor_max: if the required thrust at cruise exceeds
    // thrust_max the n-limit is INSTANTANEOUS-only — worth knowing when
    // tuning, not an error.
    {
      const auto &dp = dynamics_params_;
      const double rho0 = dp.sea_level_density_kgpm3;
      const double wl = dp.mass_kg * dp.gravity_mps2 / dp.wing_area_m2;
      const double v_stall =
          std::sqrt(2.0 * wl / (rho0 * dp.lift_coefficient_max));
      if (v_stall > dp.speed_min_mps) {
        LOG_WARN("[DYNAMICS] declared speed_min %.0f m/s is BELOW the aero "
                 "stall implied by CL_max (%.1f m/s): level flight at "
                 "speed_min needs CL %.2f > CL_max %.2f — raise speed_min "
                 "or CL_max, else cruise-entry lift violations are "
                 "unavoidable",
                 dp.speed_min_mps, v_stall,
                 2.0 * wl / (rho0 * dp.speed_min_mps * dp.speed_min_mps *
                             dp.lift_coefficient_max) *
                     dp.lift_coefficient_max,
                 dp.lift_coefficient_max);
      }
      const double v_c = 0.5 * (dp.speed_min_mps + dp.speed_max_mps);
      const double q_c = 0.5 * rho0 * v_c * v_c;
      const double cl_n = dp.mass_kg * dp.load_factor_max *
          dp.gravity_mps2 / (q_c * dp.wing_area_m2);
      const double cd_n = dp.zero_lift_drag_coefficient +
          dp.induced_drag_factor * cl_n * cl_n;
      const double t_n = q_c * dp.wing_area_m2 * cd_n;
      if (t_n > dp.thrust_max_n) {
        LOG_INFO("[DYNAMICS] note: sustained n=%.1f turn at %.0f m/s needs "
                 "%.0f N > thrust_max %.0f N — the load-factor limit is "
                 "reachable only transiently (energy-bleeding maneuvers)",
                 dp.load_factor_max, v_c, t_n, dp.thrust_max_n);
      }
    }
    if (!mmp_vehicle_dynamics::parametersAreValid(dynamics_params_)) {
      LOG_ERROR("invalid fixed-wing dynamics parameters; disabling dynamics term");
      dynamics_enable_ = false;
    }

    node_->declare_parameter("optimization/swarm_clearance", 0.5);
    node_->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node_->declare_parameter("optimization/max_vel", 1.0);
    node_->get_parameter("optimization/max_vel", max_vel_);
    node_->declare_parameter("optimization/max_acc", 1.0);
    node_->get_parameter("optimization/max_acc", max_acc_);
    // Stall floor (frame units/s); 0 disables. See feasibilityGradCostV.
    node_->declare_parameter("optimization/min_vel", 0.0);
    node_->get_parameter("optimization/min_vel", min_vel_);

    // Initial-velocity lead-in horizon in seconds (0 = disabled/legacy behavior).
    node_->declare_parameter("optimization/lead_in_time", 1.0);
    node_->get_parameter("optimization/lead_in_time", lead_in_time_);

    // Post-convergence per-term vertical-force attribution sweep (heavy log,
    // one-shot on the final trajectory). Turn on to diagnose "trajectory
    // climbs over open water" humps: it names the lifting term or proves the
    // hump is intrinsic min-jerk overshoot. Default OFF.
    node_->declare_parameter("optimization/diag_vertical", false);
    node_->get_parameter("optimization/diag_vertical", diag_vertical_);

    // Arc-varying cap: cone slope + headroom. alt_cap_headroom is DECLARED by
    // path_manager on this same node (shared value: scalar cap and the
    // envelope must use one headroom), so only read it here; declaring twice
    // throws ParameterAlreadyDeclared.
    node_->declare_parameter("optimization/alt_cap_slope", 0.10);
    node_->get_parameter("optimization/alt_cap_slope", alt_cap_slope_);
    if (node_->has_parameter("optimization/alt_cap_headroom")) {
      node_->get_parameter("optimization/alt_cap_headroom", alt_cap_headroom_opt_);
    }
    // [ZONE-RELAX] Stage 3: zone-proximity cap relaxation (0 = off/legacy).
    node_->declare_parameter("optimization/alt_cap_zone_relax", 0.0);
    node_->get_parameter("optimization/alt_cap_zone_relax", alt_cap_zone_relax_);
    // [SHADOW-CAP] H3: exposure-owned cap, hidden-margin below the LOS shadow
    // ceiling in z-units (<=0 = off/legacy).
    node_->declare_parameter("optimization/alt_cap_shadow_margin", 0.0);
    node_->get_parameter("optimization/alt_cap_shadow_margin", alt_cap_shadow_margin_);
    // [RIDE] H1: per-piece time-weight relief over rough terrain (0 = off).
    node_->declare_parameter("optimization/time_rough_relief", 0.0);
    node_->get_parameter("optimization/time_rough_relief", time_rough_relief_);
    time_rough_relief_ = std::clamp(time_rough_relief_, 0.0, 0.9);
    // [RIDE] H1: direct speed price over rough ground (0 = off).
    node_->declare_parameter("optimization/weight_ride", 0.0);
    node_->get_parameter("optimization/weight_ride", wei_ride_);
    // [H5] bounded-z warp (0/false = off, byte-identical).
    node_->declare_parameter("optimization/h5_bounded_z", false);
    node_->get_parameter("optimization/h5_bounded_z", h5_bounded_z_);
    node_->declare_parameter("optimization/h5_min_width", 0.10);
    node_->get_parameter("optimization/h5_min_width", h5_min_width_);
    node_->declare_parameter("optimization/h5_seed_margin", 0.05);
    node_->get_parameter("optimization/h5_seed_margin", h5_seed_margin_);
    node_->declare_parameter("optimization/h5_floor_slack", 0.0);
    node_->get_parameter("optimization/h5_floor_slack", h5_floor_slack_);
    node_->declare_parameter("optimization/h5_fd_check", false);
    node_->get_parameter("optimization/h5_fd_check", h5_fd_check_);
    // [H4] z-corridor decision-layer diagnostic (logging only; see
    // logH4ZCorridor). Off = byte-identical.
    node_->declare_parameter("optimization/h4_corridor_diag", false);
    node_->get_parameter("optimization/h4_corridor_diag", h4_corridor_diag_);
    node_->declare_parameter("optimization/h4_shadow_margin", 0.10);
    node_->get_parameter("optimization/h4_shadow_margin", h4_shadow_margin_);
    node_->declare_parameter("optimization/h4_climb_slope", 0.60);
    node_->get_parameter("optimization/h4_climb_slope", h4_climb_slope_);
    // [H4-COMMIT] seed-authority z commit (default off = byte-identical).
    node_->declare_parameter("optimization/h4_commit", false);
    node_->get_parameter("optimization/h4_commit", h4_commit_);
    node_->declare_parameter("optimization/h4_commit_swath", 3.0);
    node_->get_parameter("optimization/h4_commit_swath", h4_commit_swath_);

    // Log initialization based on enable_debug_logs setting
    if (enable_debug_logs_) {
        if (log_manager_) {
            log_manager_->infof("PolyTrajOptimizer parameters initialized");
            log_manager_->infof("Obstacle avoidance: %s", enable_obstacles_ ? "enabled" : "disabled");
            log_manager_->infof("Debug logging: enabled (using LogManager)");
            log_manager_->infof("L-BFGS detail logging: %s", enable_lbfgs_detail_logs_ ? "enabled" : "disabled");
        }
    }

    swarm_graph_.reset(new SwarmGraph);

    // Set initial formation based on optimizer_params.yaml
    int initial_formation_type = 2; // Default to REGULAR_SQUARE
    node_->declare_parameter("optimization/formation_type", initial_formation_type);
    node_->get_parameter("optimization/formation_type", initial_formation_type);

    LOG_INFO("Setting initial formation type: %d", initial_formation_type);
    setDesiredFormation(initial_formation_type);
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr)
  {
    swarm_trajs_ = swarm_trajs_ptr;
  }

  void PolyTrajOptimizer::setDroneId(const int drone_id)
  {
    drone_id_ = drone_id;
  }

  void PolyTrajOptimizer::setFormation(const std::vector<Eigen::Vector3d>& formation_positions, int formation_size)
  {
    // Safety check: ensure node_ is properly initialized before using logger
    if (!node_) {
      LOG_ERROR("PolyTrajOptimizer node_ is null in setFormation!");
      return;
    }

    if (swarm_graph_ && !formation_positions.empty()) {
      formation_size_ = formation_size;

      // Ensure formation positions match expected formation size
      std::vector<Eigen::Vector3d> adjusted_formation = formation_positions;
      if (static_cast<int>(adjusted_formation.size()) != formation_size_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Formation positions size (%zu) doesn't match formation_size (%d), adjusting...",
                    adjusted_formation.size(), formation_size_);
        adjusted_formation.resize(formation_size_, Eigen::Vector3d::Zero());
      }

      // Check if this is NONE mode (all positions are zero)
      bool is_none_mode = true;
      for (const auto& pos : adjusted_formation) {
        if (pos.norm() > 1e-6) {  // Non-zero position found
          is_none_mode = false;
          break;
        }
      }

      if (is_none_mode) {
        // NONE mode: disable formation cost
        use_formation_ = false;
        wei_formation_ = 0.0;
        LOG_INFO("NONE mode found - formation cost DISABLED (weight=0)");
      } else {
        // Normal formation mode
        swarm_graph_->setDesiredForm(adjusted_formation);
        use_formation_ = true;
        wei_formation_ = wei_formation_base_;  // Restore base weight

        LOG_INFO("Formation set with %d positions for optimizer (drone %d)",
                 static_cast<int>(adjusted_formation.size()), drone_id_);

        // Print current formation details
        LOG_INFO("=== CURRENT FORMATION CONFIGURATION ===");
        LOG_INFO("Formation Size: %d", formation_size_);
        LOG_INFO("Drone ID: %d", drone_id_);
        LOG_INFO("Formation Positions:");
        for (size_t i = 0; i < adjusted_formation.size(); ++i) {
          LOG_INFO("  Drone %zu: [%.3f, %.3f, %.3f]",
                   i, adjusted_formation[i].x(), adjusted_formation[i].y(), adjusted_formation[i].z());
        }
        LOG_INFO("=====================================");
      }
    } else {
      use_formation_ = false;
      formation_size_ = 0;
      LOG_WARN("Failed to set formation - swarm_graph not initialized or empty positions");
    }
  }

  void PolyTrajOptimizer::setDesiredFormation(int type)
  {
    std::vector<Eigen::Vector3d> swarm_des;
    switch (type)
    {
      case FORMATION_TYPE::NONE_FORMATION:
      {
        use_formation_ = false;
        formation_size_ = 0;
        break;
      }

      case FORMATION_TYPE::REGULAR_HEXAGON:
      {
        // set the desired formation
        Eigen::Vector3d v0(0, 0, 0);
        Eigen::Vector3d v1(1.7321, -1, 0);
        Eigen::Vector3d v2(0, -2, 0);
        Eigen::Vector3d v3(-1.7321, -1, 0);
        Eigen::Vector3d v4(-1.7321, 1, 0);
        Eigen::Vector3d v5(0, 2, 0);
        Eigen::Vector3d v6(1.7321, 1, 0);

        swarm_des.push_back(v0);
        swarm_des.push_back(v1);
        swarm_des.push_back(v2);
        swarm_des.push_back(v3);
        swarm_des.push_back(v4);
        swarm_des.push_back(v5);
        swarm_des.push_back(v6);

        formation_size_ = swarm_des.size();
        // construct the desired swarm graph
        if (swarm_graph_) {
          swarm_graph_->setDesiredForm(swarm_des);
        }
        break;
      }

      case FORMATION_TYPE::REGULAR_SQUARE:
      {
        // Square formation
        Eigen::Vector3d v0(0, 0, 0);
        Eigen::Vector3d v1(1, 0, 0);
        Eigen::Vector3d v2(1, 1, 0);
        Eigen::Vector3d v3(0, 1, 0);

        swarm_des.push_back(v0);
        swarm_des.push_back(v1);
        swarm_des.push_back(v2);
        swarm_des.push_back(v3);

        formation_size_ = swarm_des.size();
        if (swarm_graph_) {
          swarm_graph_->setDesiredForm(swarm_des);
        }
        break;
      }

      default:
        use_formation_ = false;
        formation_size_ = 0;
        break;
    }
  }

  // Compute total jerk: ∫ ||d³P/dt³||² dt
  double PolyTrajOptimizer::computeTotalJerk(const poly_traj::Trajectory &traj)
  {
    double total_jerk = 0.0;
    double dt = 0.01;  // Sample every 10ms
    double T = traj.getTotalDuration();
    int num_samples = static_cast<int>(T / dt);

    for (int i = 0; i < num_samples; ++i)
    {
      double t = i * dt;
      Eigen::Vector3d jerk = traj.getJer(t);  // Get jerk at time t
      total_jerk += jerk.squaredNorm() * dt;  // Integrate ||jerk||² * dt
    }

    return total_jerk;
  }

  // Compute maximum jerk: max_{t∈[0,T]} ||d³P/dt³(t)||
  double PolyTrajOptimizer::computeMaxJerk(const poly_traj::Trajectory &traj)
  {
    double max_jerk = 0.0;
    double dt = 0.01;  // Sample every 10ms
    double T = traj.getTotalDuration();
    int num_samples = static_cast<int>(T / dt);

    for (int i = 0; i < num_samples; ++i)
    {
      double t = i * dt;
      Eigen::Vector3d jerk = traj.getJer(t);
      double jerk_norm = jerk.norm();

      if (jerk_norm > max_jerk)
      {
        max_jerk = jerk_norm;
      }
    }

    return max_jerk;
  }

} // namespace ego_planner
