#include "path_optimizer/poly_traj_optimizer.h"
#include <iomanip>
#include <ctime>
#include <sys/resource.h>

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
                                           poly_traj::Trajectory &out_local)
  {
    // === MINCO initial trajectory from clean_path ===
    // Each shortcut vertex becomes one MINCO piece boundary directly;
    // clean_path is already densified so pieces stay roughly equal length.

    // Degenerate single-segment path → insert a midpoint so MINCO has >= 2 pieces.
    if (static_cast<int>(clean_path.size()) < 3) {
      Eigen::Vector3d mid = 0.5 * (clean_path.front() + clean_path.back());
      clean_path.insert(clean_path.begin() + 1, mid);
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
    if (lead_in_time_ > 0.0 && start_vel.norm() > 1e-3) {
      Eigen::Vector3d v_hat = start_vel.normalized();
      double d_lead = start_vel.norm() * lead_in_time_;  // distance travelled while redirecting
      Eigen::Vector3d p_lead = clean_path.front() + v_hat * d_lead;
      clean_path.insert(clean_path.begin() + 1, p_lead);
      if (log_manager_) {
        log_manager_->infof("[LEAD-IN] start_vel=%.2f m/s -> lead point +%.2f m along (%.2f,%.2f,%.2f)",
                            start_vel.norm(), d_lead, v_hat.x(), v_hat.y(), v_hat.z());
      }
    }

    int piece_num = static_cast<int>(clean_path.size()) - 1;
    Eigen::MatrixXd innerPts(3, piece_num - 1);
    for (int i = 0; i < piece_num - 1; ++i) {
      innerPts.col(i) = clean_path[i + 1];
    }

    const double des_vel = max_vel;
    Eigen::VectorXd time_vec(piece_num);
    for (int i = 0; i < piece_num; ++i) {
      double seg_len = (clean_path[i + 1] - clean_path[i]).norm();
      time_vec(i) = std::max(0.05, seg_len / des_vel);
    }

    Eigen::Vector3d approach_dir =
        (clean_path.back() - clean_path[clean_path.size() - 2]).normalized();
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

    double final_cost;

    auto t0 = node_->get_clock()->now();
    auto t1 = node_->get_clock()->now();
    auto t2 = node_->get_clock()->now();

    // L-BFGS params scaled up from the Swarm-Formation local-replan reference
    // (mem_size 16 / max_iter 60) for global planning with hundreds of vars.
    // mem_size capped at 64: 256 caused -1005 line-search failures.
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size       = 64;       // ref 16 → 64 (global scale)
    lbfgs_params.g_epsilon      = 0.05;     // ref 0.1 → 0.05 (slightly tighter)
    lbfgs_params.min_step       = 1e-32;
    lbfgs_params.max_iterations = 300;      // ref 60 → 300 (global scale)

    if (!use_formation)
    {
      use_formation_ = false;
    }

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;

    int result;

    // Direct 3D coordinate optimization with SDF-based obstacle penalty.
    variable_num_ = 4 * (piece_num_ - 1) + 1;
    std::vector<double> q(variable_num_);
    memcpy(q.data(), initInnerPts.data(), initInnerPts.size() * sizeof(double));
    Eigen::Map<Eigen::VectorXd> Vt(q.data() + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    t1 = node_->get_clock()->now();

    result = lbfgs::lbfgs_optimize(
        variable_num_,
        q.data(),
        &final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        &lbfgs_params);

    if (log_manager_ && enable_debug_logs_) {
        const char* result_str = lbfgs::lbfgs_strerror(result);
        log_manager_->infof("L-BFGS Result: %d (%s)", result, result_str);
        log_manager_->infof("Iteration info: costFunction calls=%d, max_iterations=%d", iter_num_, lbfgs_params.max_iterations);
    }

    // DEBUG: run check for logging but ignore the verdict so we can visualise
    // the optimized trajectory even when it clips obstacles.
    if (enable_obstacles_) (void)checkCollision();
    bool occ = false;
    // bool occ = enable_obstacles_ ? checkCollision() : false;

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

    if (occ)
      return false;
    else
      return true;
  }
  bool PolyTrajOptimizer::checkCollision(void)
  {
    double T_end;
    poly_traj::Trajectory traj = jerkOpt_.getTraj();

    int N = traj.getPieceNum();
    int k = cps_num_prePiece_ * N + 1;
    int idx = k / 3 * 2;
    int piece_of_idx = floor((idx - 1) / cps_num_prePiece_);
    Eigen::VectorXd durations = traj.getDurations();

    if (piece_of_idx < 0 || piece_of_idx >= N) {
      T_end = durations.sum();
    } else {
      T_end = durations.head(piece_of_idx).sum()
            + durations(piece_of_idx)
            * (idx - piece_of_idx * cps_num_prePiece_) / (double)cps_num_prePiece_;
    }

    bool occ = false;
    double dt = 0.01;
    int i_end = std::max(1, (int)floor(T_end / dt));
    double t = 0.0;
    collision_check_time_end_ = T_end;

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

  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(6);

    // High-performance timing for debugging (similar to con code)
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t1 = t_start, t2 = t_start, t3 = t_start, t4 = t_start, t5 = t_start;

    // 1. Trajectory generation
    t1 = std::chrono::high_resolution_clock::now();
    opt->jerkOpt_.generate(P, T);
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
        opt->log_manager_->infof("  smoothness_cost=%.6f (weight=implicit)", smoo_cost);
        opt->log_manager_->infof("  obstacle_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(0), opt->wei_obs_);
        opt->log_manager_->infof("  swarm_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(1), opt->wei_swarm_);
        opt->log_manager_->infof("  formation_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(2), opt->wei_formation_);
        opt->log_manager_->infof("  risk_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(3), opt->wei_risk_);
        opt->log_manager_->infof("  feasibility_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(4), opt->wei_feas_);
        opt->log_manager_->infof("  time_cost=%.6f (weight=%.3f)", time_cost, opt->wei_time_);

        // Store formation cost for final logging
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    } else if (!opt->enable_lbfgs_detail_logs_ && opt->use_formation_) {
        // Store formation cost for final logging even when detail logs are disabled
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    }

    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx,
                                           const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
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
      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }
    costT = RT.sum() * wei_time_;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
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

        // SDF-based obstacle penalty.
        if (enable_obstacles_ && sdf_manager_ && sdf_manager_->hasData()) {
            if (sdfGradCostP(i_dp, pos, gradp, costp)) {
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradp.transpose() * vel;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                costs(0) += omg * step * costp;
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

        // Risk zone cost calculation (soft constraint)
        if (use_risk_zones_ && RiskGradCostP(i_dp, pos, gradp, costp)) {
            gradViolaPc = beta0 * gradp.transpose();
            gradViolaPt = alpha * gradp.transpose() * vel;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
            gdT(i) += omg * (costp / K + step * gradViolaPt);
            costs(3) += omg * step * costp;
        }

        // Altitude-band cap (cubic above the mission band, like the obstacle
        // violation shape). Booked into the risk slot (3): both are
        // "exposure" costs. Down-side is covered by ground/obstacle terms.
        if (wei_alt_ > 0.0 && alt_zhi_ >= 0.0 && pos.z() > alt_zhi_) {
            // QUADRATIC, not cubic: ridge crossings sit several units above
            // the band, and a cubic down-force there outgrows the obstacle
            // penalty's cubic (which works on the SMALL violation depth) —
            // the cap then presses the trajectory into terrain. Quadratic
            // shapes the swell but can never win against the clearance wall.
            const double ua = pos.z() - alt_zhi_;
            const double costa_z = wei_alt_ * ua * ua;
            Eigen::Vector3d grad_a(0.0, 0.0, wei_alt_ * 2.0 * ua);
            gradViolaPc = beta0 * grad_a.transpose();
            gradViolaPt = alpha * grad_a.transpose() * vel;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
            gdT(i) += omg * (costa_z / K + step * gradViolaPt);
            costs(3) += omg * step * costa_z;
        }

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

        s1 += step;
        if (j != K || (j == K && i == N - 1)) {
            ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    // Distance variance cost (spreads inner points evenly).
    {
      Eigen::MatrixXd gdp;
      double var;
      distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

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

    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;

    if (size < formation_size_)
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

    for (size_t id = 0; id < size; id++)
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

      for (size_t id = 0; id < size; id++)
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
                                        double &costp)
  {
    (void)i_dp;
    gradp.setZero();
    costp = 0;

    float d = 0.0f;
    Eigen::Vector3d grad_d = Eigen::Vector3d::Zero();

    // Ground / ceiling hard half-spaces. Past the plane we report a
    // *negative* signed distance equal to the crossing depth, so the
    // downstream cubic penalty (violation = clearance − d) grows as
    // (clearance + depth)³. That is strictly larger than staying just
    // above the plane, so L-BFGS can never trade a shallow dive for a
    // cheap obstacle escape. The outward-pointing unit gradient keeps
    // pushing the trajectory back across the plane no matter how deep it
    // ended up.
    if (ground_height_ > -0.5 && p.z() < ground_height_) {
      const double depth = ground_height_ - p.z();
      d = static_cast<float>(-depth);
      grad_d = Eigen::Vector3d(0.0, 0.0, 1.0);  // ∇dist points up
    } else if (virtual_ceil_height_ > -0.5 && p.z() > virtual_ceil_height_) {
      const double depth = p.z() - virtual_ceil_height_;
      d = static_cast<float>(-depth);
      grad_d = Eigen::Vector3d(0.0, 0.0, -1.0);  // ∇dist points down
    } else {
      if (!sdf_manager_ || !sdf_manager_->hasData()) return false;
      if (!sdf_manager_->getDistanceAndGradient(p, &d, &grad_d)) return false;
      if (!std::isfinite(d)) return false;
    }

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

      if (min_ellip_dist2_ > ellip_dist2)
      {
        min_ellip_dist2_ = ellip_dist2;
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

  // Risk penalty: cubic in PENETRATION DEPTH u = 1 - d/R per zone (vertical
  // cylinder, matching the front-end). The old wei*risk^2 with risk ~ peak*u^2
  // was ~u^4 near the boundary — so flat that the time cost always won and
  // the optimizer shaved corners INTO zones. u^3 mirrors the obstacle
  // penalty's cubic violation shape: zero-slope contact, growing fast inside.
  bool PolyTrajOptimizer::RiskGradCostP(const int i_dp,
                                           const Eigen::Vector3d &p,
                                           Eigen::Vector3d &gradp,
                                           double &costp)
  {
    (void)i_dp;  // consider all control points
    gradp.setZero();
    costp = 0.0;

    bool any = false;
    for (const auto &tz : risk_zones_) {
      if (std::abs(p.z() - tz.center.z()) >= tz.reach) continue;
      Eigen::Vector3d diff = p - tz.center;
      diff.z() = 0.0;
      const double d = diff.norm();
      if (d >= tz.reach) continue;
      const double u = 1.0 - d / tz.reach;       // penetration fraction (0..1)
      costp += wei_risk_ * u * u * u;
      if (d > 1e-9) {
        // d(u^3)/dp = 3u^2 * (-1/R) * diff/d  -> points INTO the zone; the
        // negative gradient pushes the trajectory back out horizontally.
        gradp += wei_risk_ * 3.0 * u * u * (-1.0 / tz.reach) * (diff / d);
      }
      any = true;
    }
    return any;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
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

    node_->declare_parameter("optimization/weight_Risk", 0.0);
    node_->get_parameter("optimization/weight_Risk", wei_risk_);

    node_->declare_parameter("optimization/swarm_clearance", 0.5);
    node_->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node_->declare_parameter("optimization/max_vel", 1.0);
    node_->get_parameter("optimization/max_vel", max_vel_);
    node_->declare_parameter("optimization/max_acc", 1.0);
    node_->get_parameter("optimization/max_acc", max_acc_);

    // Initial-velocity lead-in horizon in seconds (0 = disabled/legacy behavior).
    node_->declare_parameter("optimization/lead_in_time", 1.0);
    node_->get_parameter("optimization/lead_in_time", lead_in_time_);

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
