#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <thread>
#include <chrono>
#include <fstream>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <swarm_graph/swarm_graph.hpp>
#include "../../common/log_manager.hpp"

// Forward declaration for LogManager
using LogManager = swarm_formation::LogManager;
#include "lbfgs.hpp"
#include "plan_container.hpp"
#include "poly_traj_utils.hpp"
#include "path_planner/sdf/distance_field.h"

#define LOG_INFO(msg, ...) do { \
  if (!enable_debug_logs_) { \
      RCLCPP_INFO(node_->get_logger(), "[POLY_TRAJ_OPT][drone %d] " msg, drone_id_, ##__VA_ARGS__); \
  } else if (log_manager_) { \
      log_manager_->infof("[POLY_TRAJ_OPT][drone %d] " msg, drone_id_, ##__VA_ARGS__); \
  } \
} while(0)

#define LOG_WARN(msg, ...) do { \
  if (!enable_debug_logs_) { \
      RCLCPP_WARN(node_->get_logger(), "[POLY_TRAJ_OPT][drone %d] " msg, drone_id_, ##__VA_ARGS__); \
  } else if (log_manager_) { \
      log_manager_->warnf("[POLY_TRAJ_OPT][drone %d] " msg, drone_id_, ##__VA_ARGS__); \
  } \
} while(0)

#define LOG_ERROR(msg, ...) do { \
  if (!enable_debug_logs_) { \
      RCLCPP_ERROR(node_->get_logger(), "[POLY_TRAJ_OPT][drone %d] " msg, drone_id_, ##__VA_ARGS__); \
  } else if (log_manager_) { \
      log_manager_->errorf("[POLY_TRAJ_OPT][drone %d] " msg, drone_id_, ##__VA_ARGS__); \
  } \
} while(0)

namespace ego_planner
{
  // risk zone (shared definition with path_manager / front-end RiskZoneLite).
  struct RiskZone {
    Eigen::Vector3d center;
    double reach;   // meters; risk is exactly zero outside the vertical cylinder
    double peak;    // raw yaml max_risk_level; values >= 1 saturate the moat
                    // at the shared cap (1-1e-3) over most of the cylinder
  };

  enum FORMATION_TYPE
  {
    NONE_FORMATION = 0,
    REGULAR_HEXAGON = 1,
    REGULAR_SQUARE = 2,
    TEST_FORMATION = 3
  };

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;

    void resize_cp(const int size_set)
    {
      cp_size = size_set;
      points.resize(3, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class PolyTrajOptimizer
  {
  private:
    double dbg_cost_formation_{0.0};
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{nullptr};
    ConstrainPoints cps_;
    SwarmGraph::Ptr swarm_graph_;
    swarm_formation::LogManager::Ptr log_manager_;

    int drone_id_;
    int cps_num_prePiece_;
    int variable_num_;
    int piece_num_;
    int iter_num_;
    double min_ellip_dist2_;

    std::string result_fn_;
    std::fstream result_file_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;


    double wei_obs_;
    double wei_swarm_;
    double wei_feas_;
    double wei_sqrvar_;
    double wei_time_;
    // Explicit smoothness (jerk-energy) weight. The jerk integral scales as
    // ~1/T^5, so at mission scale (km routes, 50-100 s pieces) it collapses
    // 6-7 orders of magnitude below the time cost and stops ironing out
    // z-ringing through noisy front-end pins. Keep the weighted smoothness
    // cost well below time_cost/5 or it inflates durations (jerk decreases
    // with T). 1.0 = legacy implicit weight.
    double wei_smooth_{1.0};
    double wei_formation_;
    double wei_formation_base_;  // Base formation weight (from config)
    double wei_risk_;          // Risk zone cost weight for trajectory optimization
    // Smoothed trajectory-level equivalent of the front-end's finite barrier
    // K (manager/risk_barrier): per-length cost on NON-exempt zones so MINCO
    // keeps the same standoff at the cylinder rim that FM2 planned with.
    // 0 disables (moat-only sharing).
    double wei_risk_barrier_{0.0};

    double swarm_clearance_;
    double max_vel_, max_acc_;
    // Initial-velocity lead-in horizon (seconds): plant a short path point along
    // start_vel so the head piece coasts out in the direction the drone is already
    // moving, instead of arcing wildly toward a far first waypoint. 0.0 = disabled.
    double lead_in_time_;

    int formation_size_ = 4;  // Default to 4 drones
    bool use_formation_ = true;
    bool is_other_assigning_ = false;
    uint64_t seq_ = 0;
    double debug_similarity_ = 0.0;

    double t_now_;
    bool enable_obstacles_;
    bool enable_debug_logs_;
    bool enable_lbfgs_detail_logs_;
    // Post-convergence per-term VERTICAL-force attribution sweep. Answers
    // "which cost term (if any) lifts the trajectory HERE" for every hump,
    // and flags humps with NO spatial vertical force as intrinsic
    // smoothness/variance (min-jerk) overshoot. Opt-in (heavy log), off in
    // production. See logVerticalAttribution().
    bool diag_vertical_{false};

    rclcpp::Node::SharedPtr node_;

    // SDF-based obstacle avoidance (replaces SFC corridor penalty).
    const path_planner::sdf::IDistanceField *sdf_manager_{nullptr};
    double obstacle_clearance_{0.5};  // safety margin used by SDF penalty

    // 2.5D terrain heightmap (frame units; -inf over water/invalid). Terrain is a
    // height FUNCTION, so querying it directly gives an EXACT clearance
    // (z - h(x,y)) instead of the 3D SDF's 10 m z-quantised approximation — the
    // SDF under-sees terrain, so trajectories that read "clear" to it penetrate
    // the finer DEM the clearance panel shows. When set, this term is the
    // authoritative terrain-collision check.
    std::function<float(double, double)> terrain_height_;
    // Heightmap value + ANALYTIC gradient of the same surface (world coords).
    // The terrain term and the alt-cap gate MUST take cost and gradient from
    // one consistent surface — a smoothed slope estimate paired with the
    // bilinear value made them disagree near DEM-cell edges and the line
    // search died (-1008) on cliff cells. Signature: (x, y, &h, &dhdx, &dhdy)
    // -> false over pure water / outside the DEM.
    std::function<bool(double, double, float *, float *, float *)> terrain_hgrad_;

    // Hard half-space constraints applied outside the SDF so the clearance
    // band does not contaminate them. Sentinel: ≤ -0.5 disables the plane.
    double ground_height_{-1.0};
    double virtual_ceil_height_{-1.0};

    // Risk zone data for trajectory optimization.
    std::vector<RiskZone> risk_zones_;
    bool use_risk_zones_{false};
    // Per-zone: 1 = contains the plan start/goal, barrier OFF (moat only).
    // Same must-enter exemption rule as the front-end's prepareBarrier.
    std::vector<char> zone_barrier_exempt_;
    // Altitude band: quadratic penalty on z above alt_zhi_ (geodesic max +
    // headroom — keeps the sparse-piece quintic from ballooning above the
    // front-end's committed profile) and below alt_zlo_ (mission min −
    // slack — stops min-jerk sags bouncing off the water/terrain clearance
    // floor; the lower half of the FM2 alt band). 0 weight disables both;
    // a floor <= 0 simply never binds.
    double wei_alt_{0.0};
    double alt_zhi_{-1.0};
    double alt_zlo_{-1e9};
    // ARC-VARYING cap: per-piece ceiling built in optimizeFromPath from the
    // front-end's committed z profile (cap reference) + a slope-cone envelope
    // + alt_cap_headroom. Restores the z trust-region that the SFC corridor's
    // bounded inflation used to provide for free: over open water z is
    // otherwise ungoverned between the scalar floor and the GLOBAL-max cap
    // (a ~100 m dead band at mission scale where T^5-starved smoothness
    // cannot clear optimization debris — observed 60-80 m start humps).
    // Each entry is CONSTANT w.r.t. the decision variables, so gradients keep
    // their form (cost/gradient stay a consistent pair). Size piece_num when
    // active; empty -> scalar alt_zhi_ fallback. Rebuilt/cleared per plan.
    Eigen::VectorXd alt_zhi_pieces_;
    // Cone slope [z-units per xy-unit] licensing climb/descent anticipation:
    // high cap values bleed sideways at this grade, so the back-end may lead
    // a committed ramp without penalty. Sized above observed FE ramp grades
    // (~0.08 on the tall map). Yaml: optimization/alt_cap_slope.
    double alt_cap_slope_{0.10};
    // Headroom added on top of the envelope (same value path_manager uses
    // for the scalar cap). Yaml: optimization/alt_cap_headroom (shared).
    double alt_cap_headroom_opt_{0.4};

    // Cruise dynamics (lateral/normal-acceleration limit). Evaluated in
    // PHYSICAL metres: v_m = S*v with S = diag(unit_xy, unit_xy, unit_z);
    // since the 2026-07 frame fix the frame is isotropic (1 unit = 100 m on
    // every axis), so both units default to 100. The limit is n_lat * g as the
    // normal-acceleration ceiling. (The old "reserved" speed_mps / n_lon params
    // were never used by any term and were removed.)
    // Minimum-speed (stall) floor, frame units/s. 0 = OFF (default: platform
    // stall spec undecided; also must stay off for rest-start missions).
    // Companion of the lateral-g term below: without a speed floor the
    // optimizer escapes every curvature limit by braking to ~zero.
    double min_vel_{0.0};

    bool dynamics_enable_{true};
    double wei_dynamics_{0.0};
    double dyn_unit_xy_m_{100.0};
    double dyn_unit_z_m_{100.0};
    double dyn_n_lat_{30.0};
    double dyn_g_{9.81};
    double dyn_min_speed_mps_{1.0};

  public:
    PolyTrajOptimizer() {}
    // ~PolyTrajOptimizer() { }
    ~PolyTrajOptimizer() = default;

    void setParam(const rclcpp::Node::SharedPtr &node);
    void setLogManager(swarm_formation::LogManager::Ptr log_manager);
    void setSDFManager(const path_planner::sdf::IDistanceField *sdf) { sdf_manager_ = sdf; }
    void setObstacleClearance(double c) { obstacle_clearance_ = c; }
    void setGroundHeight(double h)      { ground_height_ = h; }
    void setVirtualCeilHeight(double h) { virtual_ceil_height_ = h; }
    void setTerrainHeightmap(std::function<float(double, double)> f) { terrain_height_ = std::move(f); }
    void setTerrainHeightGrad(std::function<bool(double, double, float *, float *, float *)> f) { terrain_hgrad_ = std::move(f); }
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setFormation(const std::vector<Eigen::Vector3d>& formation_positions, int formation_size);
    void setMaxVel(double vel) { max_vel_ = vel; }
    void setRiskZones(const std::vector<RiskZone> &zones) {
        risk_zones_ = zones;
        use_risk_zones_ = !zones.empty();
        // Stale exemptions must not outlive the zone list they were computed
        // for; prepareRiskBarrier() recomputes them per plan.
        zone_barrier_exempt_.clear();
    }

    // Mark zones containing the plan start/goal as barrier-exempt (they must
    // be entered, so they stay soft) — mirrors dyn_a_star.h prepareBarrier.
    // Call once per plan, after setRiskZones, before optimizing.
    void prepareRiskBarrier(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

    // Extend the exemption to zones the FRONT-END ROUTE already crosses: the
    // front-end's finite barrier permits crossing when every alternative is
    // worse (e.g. randomly-oriented ships walling the good corridor), and a
    // back-end barrier on that zone would fight the committed crossing —
    // the observed failure was a 1M-scale risk/obstacle tug-of-war ending in
    // a -1005 line-search death with an unfinished iterate published. Point
    // sampling of the densified path is intentional: deep crossings exempt,
    // shallow corner-clips stay barred (the ramp SHOULD push those out).
    void markFrontEndCrossings(const std::vector<Eigen::Vector3d> &path);

    void setAltitudeBand(double z_lo, double z_hi, double weight) {
        alt_zlo_ = z_lo;
        alt_zhi_ = z_hi;
        wei_alt_ = std::max(0.0, weight);
    }

    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; };
    inline double getSwarmClearance(void) { return swarm_clearance_; }

    bool OptimizeTrajectory_lbfgs(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                  const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                  Eigen::MatrixXd &optimal_points, const bool use_formation);

    // High-level entry: build a MINCO initial trajectory from the front-end
    // path, then optimize it. This is the single replaceable seam — a custom
    // optimizer backend reimplements this and never exposes MINCO internals.
    // clean_path may be modified (a midpoint is inserted for degenerate input).
    //
    // cap_ref_z: the front-end's COMMITTED z per clean_path vertex (pre-
    // z-denoise profile, elementwise-maxed with the safety-clamped one; built
    // by path_manager alongside clean_path). Drives the arc-varying altitude
    // cap. Must be the same size as clean_path on entry — a mismatch logs a
    // warning and falls back to the scalar cap (never silently mis-indexed).
    // Empty = scalar cap only.
    bool optimizeFromPath(std::vector<Eigen::Vector3d> &clean_path,
                          const Eigen::Vector3d &start_pos,
                          const Eigen::Vector3d &start_vel,
                          const Eigen::Vector3d &start_acc,
                          const std::vector<Eigen::Vector3d> &waypoints,
                          double max_vel,
                          poly_traj::Trajectory &out_global,
                          poly_traj::Trajectory &out_local,
                          const std::vector<double> &cap_ref_z = {});

    void setDesiredFormation(int type);

  private:
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool sdfGradCostP(const int i_dp,
                      const Eigen::Vector3d &p,
                      Eigen::Vector3d &gradp,
                      double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool swarmGraphGradCostP(const int i_dp,
                             const double t,
                             const Eigen::Vector3d &p,
                             const Eigen::Vector3d &v,
                             Eigen::Vector3d &gradp,
                             double &gradt,
                             double &grad_prev_t,
                             double &costp);

    bool RiskGradCostP(const int i_dp,
                         const Eigen::Vector3d &p,
                         const Eigen::Vector3d &v,
                         Eigen::Vector3d &gradp,
                         Eigen::Vector3d &gradv,
                         double &costp);

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    bool cruiseDynamicsGradCostVA(const Eigen::Vector3d &v,
                                  const Eigen::Vector3d &a,
                                  Eigen::Vector3d &gradv,
                                  Eigen::Vector3d &grada,
                                  double &cost);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    bool checkCollision(void);

    // Per-term VERTICAL-force attribution over the final trajectory.
    // For each sample logs fz (= -d(cost)/dz; fz>0 pushes altitude UP) of
    // every z-affecting cost term, terrain height/clearance, the active
    // alt band, and the trajectory's own vz/az. Ends with an automatic
    // hump scan that classifies each z-peak as term-driven or intrinsic
    // (smoothness/variance min-jerk overshoot). Gated on diag_vertical_.
    void logVerticalAttribution(void);

    double computeTotalJerk(const poly_traj::Trajectory &traj);
    double computeMaxJerk(const poly_traj::Trajectory &traj);

  public:
    typedef std::unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif
