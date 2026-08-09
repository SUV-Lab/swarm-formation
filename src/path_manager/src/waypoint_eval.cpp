// [WPE] Waypoint extraction + reproduction measurement — implementation.
// Contract and caveats: see waypoint_eval.h.

#include "path_manager/waypoint_eval.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_manager {
namespace waypoint_eval {

namespace md = mmp_vehicle_dynamics;
namespace tp = transition_phase;

const char *failName(FailReason r)
{
  switch (r) {
    case FailReason::kNone: return "none";
    case FailReason::kTimeout: return "timeout";
    case FailReason::kNonFinite: return "nonfinite";
    case FailReason::kUnrepresentable: return "unrepresentable";
    case FailReason::kDiverged: return "diverged";
    case FailReason::kNoWaypoints: return "no-waypoints";
  }
  return "unknown";
}

// ===================== source path =====================

SourcePath buildSourcePath(const poly_traj::Trajectory &traj,
                           const FrameScale &fs, int samples)
{
  SourcePath sp;
  if (traj.getPieceNum() <= 0) return sp;
  const double T = traj.getTotalDuration();
  if (!(T > 1e-6)) return sp;

  if (samples <= 0) {
    // ~25 m chord spacing from a cheap length estimate, clamped so neither
    // a short hop nor a very long flight lands on a useless resolution.
    const traj_sampling::ArcTable probe =
        traj_sampling::buildArcTable(traj, 512);
    const double est_len = fs.toSi(Eigen::Vector3d(1, 0, 0)).x() > 0.0
                               ? probe.total * fs.unit_xy_m
                               : probe.total;
    samples = static_cast<int>(std::round(est_len / 25.0));
    samples = std::min(16384, std::max(1024, samples));
  }

  sp.t_s.reserve(static_cast<size_t>(samples) + 1);
  sp.s_m.reserve(static_cast<size_t>(samples) + 1);
  sp.pos_m.reserve(static_cast<size_t>(samples) + 1);
  sp.vel_mps.reserve(static_cast<size_t>(samples) + 1);
  sp.acc_mps2.reserve(static_cast<size_t>(samples) + 1);
  sp.kappa.reserve(static_cast<size_t>(samples) + 1);

  double s = 0.0;
  Eigen::Vector3d prev = fs.toSi(traj.getPos(0.0));
  for (int k = 0; k <= samples; ++k) {
    const double tt = std::min(T, k * T / samples);
    const Eigen::Vector3d p = fs.toSi(traj.getPos(tt));
    const Eigen::Vector3d v = fs.toSi(traj.getVel(tt));
    const Eigen::Vector3d a = fs.toSi(traj.getAcc(tt));
    s += (p - prev).norm();
    prev = p;
    sp.t_s.push_back(tt);
    sp.s_m.push_back(s);
    sp.pos_m.push_back(p);
    sp.vel_mps.push_back(v);
    sp.acc_mps2.push_back(a);
    sp.kappa.push_back(traj_sampling::curvature3(v, a));
  }
  sp.total_len_m = s;
  sp.total_time_s = T;
  return sp;
}

namespace {

// Index of the first sample with s_m >= target (clamped).
size_t arcIndex(const SourcePath &src, double s_target)
{
  const auto it =
      std::lower_bound(src.s_m.begin(), src.s_m.end(), s_target);
  size_t i = static_cast<size_t>(std::distance(src.s_m.begin(), it));
  if (i >= src.s_m.size()) i = src.s_m.size() - 1;
  return i;
}

Waypoint sampleAtArc(const SourcePath &src, double s_target)
{
  Waypoint w;
  const size_t i = arcIndex(src, s_target);
  if (i == 0) {
    w.pos_m = src.pos_m.front();
    w.speed_mps = src.vel_mps.front().norm();
    w.src_arc_m = src.s_m.front();
    w.src_time_s = src.t_s.front();
    return w;
  }
  const double s0 = src.s_m[i - 1], s1 = src.s_m[i];
  const double u = (s1 > s0) ? (s_target - s0) / (s1 - s0) : 0.0;
  w.pos_m = src.pos_m[i - 1] + u * (src.pos_m[i] - src.pos_m[i - 1]);
  const Eigen::Vector3d v =
      src.vel_mps[i - 1] + u * (src.vel_mps[i] - src.vel_mps[i - 1]);
  w.speed_mps = v.norm();
  w.src_arc_m = s_target;
  w.src_time_s = src.t_s[i - 1] + u * (src.t_s[i] - src.t_s[i - 1]);
  return w;
}

Waypoint endpointWaypoint(const SourcePath &src)
{
  Waypoint w;
  w.pos_m = src.pos_m.back();
  w.speed_mps = src.vel_mps.back().norm();
  w.src_arc_m = src.s_m.back();
  w.src_time_s = src.t_s.back();
  return w;
}

}  // namespace

Waypoint waypointAtTime(const poly_traj::Trajectory &traj,
                        const FrameScale &fs, const SourcePath &src,
                        double t_s)
{
  Waypoint w;
  if (src.empty()) return w;
  const double t = std::min(std::max(t_s, 0.0), src.total_time_s);
  // Exact from the polynomial — a dense-sample lookup would land up to a
  // sample spacing away from the junction it is meant to pin.
  w.pos_m = fs.toSi(traj.getPos(t));
  w.speed_mps = fs.toSi(traj.getVel(t)).norm();
  w.src_time_s = t;
  const auto it = std::lower_bound(src.t_s.begin(), src.t_s.end(), t);
  size_t i = static_cast<size_t>(std::distance(src.t_s.begin(), it));
  if (i == 0) {
    w.src_arc_m = src.s_m.front();
  } else if (i >= src.t_s.size()) {
    w.src_arc_m = src.s_m.back();
  } else {
    const double t0 = src.t_s[i - 1], t1 = src.t_s[i];
    const double u = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
    w.src_arc_m = src.s_m[i - 1] + u * (src.s_m[i] - src.s_m[i - 1]);
  }
  return w;
}

namespace {

// Merge mandatory anchors into a strategy's output: kept verbatim (they
// carry exact polynomial state), dropped when the follower could not
// resolve them against an existing waypoint.
void mergeAnchors(std::vector<Waypoint> *out, const SourcePath &src,
                  const std::vector<Waypoint> &anchors, double min_sep_m)
{
  if (!out || anchors.empty()) return;
  for (const auto &a : anchors) {
    if (!(a.src_arc_m > 0.0) || a.src_arc_m >= src.total_len_m) continue;
    bool blocked = false;
    for (const auto &w : *out)
      if (std::abs(w.src_arc_m - a.src_arc_m) < min_sep_m) blocked = true;
    if (blocked) continue;
    out->push_back(a);
  }
  std::sort(out->begin(), out->end(),
            [](const Waypoint &x, const Waypoint &y) {
              return x.src_arc_m < y.src_arc_m;
            });
}

}  // namespace

// ===================== extraction =====================

std::vector<Waypoint> extractUniformArc(const SourcePath &src, int n,
                                        const std::vector<Waypoint> &anchors,
                                        double min_sep_m)
{
  std::vector<Waypoint> out;
  if (src.empty() || n < 1) return out;
  out.reserve(static_cast<size_t>(n) + anchors.size());
  for (int i = 1; i <= n; ++i) {
    if (i == n) {
      out.push_back(endpointWaypoint(src));   // endpoint EXACT
    } else {
      out.push_back(sampleAtArc(src, i * src.total_len_m / n));
    }
  }
  mergeAnchors(&out, src, anchors, min_sep_m);
  return out;
}

std::vector<Waypoint> extractCurvatureAdaptive(
    const SourcePath &src, int n, double lambda, double eps_straight,
    const std::vector<Waypoint> &anchors, double min_sep_m)
{
  std::vector<Waypoint> out;
  if (src.empty() || n < 1) return out;
  if (!(lambda > 0.0))
    return extractUniformArc(src, n, anchors, min_sep_m);

  // Cumulative weight W(s) = integral of (eps + lambda * kappa) ds.
  std::vector<double> W(src.s_m.size(), 0.0);
  for (size_t k = 1; k < src.s_m.size(); ++k) {
    const double ds = src.s_m[k] - src.s_m[k - 1];
    const double w = eps_straight +
                     lambda * 0.5 * (src.kappa[k] + src.kappa[k - 1]);
    W[k] = W[k - 1] + w * ds;
  }
  const double W_total = W.back();
  if (!(W_total > 0.0)) return extractUniformArc(src, n, anchors, min_sep_m);

  out.reserve(static_cast<size_t>(n));
  for (int i = 1; i <= n; ++i) {
    if (i == n) {
      out.push_back(endpointWaypoint(src));
      break;
    }
    const double target = i * W_total / n;
    const auto it = std::lower_bound(W.begin(), W.end(), target);
    size_t k = static_cast<size_t>(std::distance(W.begin(), it));
    if (k == 0) k = 1;
    if (k >= W.size()) k = W.size() - 1;
    const double w0 = W[k - 1], w1 = W[k];
    const double u = (w1 > w0) ? (target - w0) / (w1 - w0) : 0.0;
    const double s_at = src.s_m[k - 1] + u * (src.s_m[k] - src.s_m[k - 1]);
    out.push_back(sampleAtArc(src, s_at));
  }
  mergeAnchors(&out, src, anchors, min_sep_m);
  return out;
}

// ===================== follower =====================

double derivedBankLevel(const FollowerParams &p)
{
  if (p.bank_level_rad > 0.0) return p.bank_level_rad;
  // 70% of the model's bank limit: real authority, margin left for the
  // law's own proportional band. Derived, never a literal.
  return 0.7 * p.dyn.bank_angle_max_rad;
}

double minTurnRadius(const FollowerParams &p, double v_mps)
{
  const double b = derivedBankLevel(p);
  return v_mps * v_mps /
         (p.dyn.gravity_mps2 * std::max(1e-6, std::tan(b)));
}

double derivedAcceptRadius(const FollowerParams &p)
{
  if (p.accept_radius_m > 0.0) return p.accept_radius_m;
  // A small fraction of the model's own level-turn radius: the capture
  // geometry scales with the vehicle, never a literal. Deliberately small
  // — the along-leg half-plane is the primary advance and a large radius
  // would swallow neighbouring waypoints whole (measured: at 0.15 x the
  // turn radius, a 64-waypoint list captured several per test and the
  // follower effectively flew to a far waypoint).
  const double v_nom = 0.5 * (p.dyn.speed_min_mps + p.dyn.speed_max_mps);
  return std::max(50.0, 0.05 * minTurnRadius(p, v_nom));
}

RolloutResult flyWaypoints3Dof(const std::vector<Waypoint> &wps,
                               const FollowerStart &start,
                               const FollowerParams &prm)
{
  RolloutResult out;
  out.follower_id = "pointmass_pursuit_v1";
  if (wps.empty()) {
    out.fail = FailReason::kNoWaypoints;
    return out;
  }
  const md::Parameters &dyn = prm.dyn;
  const double R_acc = derivedAcceptRadius(prm);
  const double bank_level = derivedBankLevel(prm);
  const double diverge = prm.diverge_factor * R_acc;
  const double v_cruise =
      prm.cruise_speed_mps > 0.0
          ? prm.cruise_speed_mps
          : 0.5 * (dyn.speed_min_mps + dyn.speed_max_mps);

  // Seed the point-mass state from the SI start (dynamics_sim recipe).
  md::PointMassState s;
  s.position_m = start.pos_m;
  const double v0 = start.vel_mps.norm();
  s.speed_mps = v0;
  s.flight_path_angle_rad =
      v0 > 1e-9 ? std::asin(std::min(1.0, std::max(-1.0,
                                                   start.vel_mps.z() / v0)))
                : 0.0;
  s.heading_rad = std::atan2(start.vel_mps.y(), start.vel_mps.x());

  double t_ref = 0.0;
  for (size_t i = 0; i + 1 < wps.size(); ++i)
    t_ref += (wps[i + 1].pos_m - wps[i].pos_m).norm();
  t_ref = (t_ref + (wps.front().pos_m - start.pos_m).norm()) /
          std::max(1.0, v_cruise);
  const double t_max = std::max(30.0, prm.timeout_factor * t_ref);
  const int max_steps = static_cast<int>(std::ceil(t_max / prm.dt_s));

  size_t active = 0;
  Eigen::Vector3d prev_wp = start.pos_m;
  double gamma_cmd = s.flight_path_angle_rad;
  // First-order command lag state (0 lag: commands apply immediately).
  double cl_state = 0.0, thrust_state = 0.0, bank_state = 0.0;
  bool lag_seeded = false;

  for (int k = 0; k <= max_steps; ++k) {
    const double t = k * prm.dt_s;
    if (!s.position_m.allFinite() || !std::isfinite(s.speed_mps) ||
        !std::isfinite(s.flight_path_angle_rad) ||
        !std::isfinite(s.heading_rad)) {
      out.fail = FailReason::kNonFinite;
      return out;
    }
    out.samples.push_back(RolloutSample{
        t, s.position_m, md::pointMassVelocity(s), Eigen::Vector3d::Zero()});

    // --- advance test: along-leg half plane OR acceptance radius ---
    // Advance through EVERY satisfied waypoint in this step, then steer —
    // deferring to the next step would freeze the state for a tick per
    // capture and quietly stretch the flight.
    bool done = false;
    for (;;) {
      Eigen::Vector3d leg_i = wps[active].pos_m - prev_wp;
      if (leg_i.norm() < 1e-6) leg_i = md::pointMassVelocity(s);
      const Eigen::Vector3d lh =
          leg_i.norm() > 1e-9 ? leg_i.normalized() : Eigen::Vector3d::UnitX();
      const Eigen::Vector3d d = wps[active].pos_m - s.position_m;
      const double dist_i = d.norm();
      const double lat_i = (d - d.dot(lh) * lh).norm();
      const bool passed = d.dot(lh) <= 0.0 && lat_i <= 4.0 * R_acc;
      if (!(dist_i <= R_acc || passed)) break;
      out.arrivals.push_back(
          WaypointArrival{static_cast<int>(active), t, dist_i});
      prev_wp = wps[active].pos_m;
      ++active;
      if (active == wps.size()) {
        out.completed = true;
        out.total_steps = k + 1;
        done = true;
        break;
      }
    }
    if (done) return out;

    Eigen::Vector3d leg = wps[active].pos_m - prev_wp;
    if (leg.norm() < 1e-6) leg = md::pointMassVelocity(s);
    const Eigen::Vector3d leg_hat =
        leg.norm() > 1e-9 ? leg.normalized() : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d to_wp = wps[active].pos_m - s.position_m;
    const double dist = to_wp.norm();
    const double lateral = (to_wp - to_wp.dot(leg_hat) * leg_hat).norm();
    if (dist > diverge && lateral > diverge) {
      out.fail = FailReason::kDiverged;
      out.total_steps = k + 1;
      return out;
    }

    // --- aim: carrot lead_time seconds ahead on the leg line ---
    const double along = to_wp.dot(leg_hat);
    const Eigen::Vector3d aim =
        wps[active].pos_m -
        leg_hat * std::max(0.0, along - std::max(1.0, s.speed_mps) *
                                            prm.lead_time_s);
    const Eigen::Vector3d to_aim = aim - s.position_m;
    const double cone =
        dyn.flight_path_angle_max_rad * (1.0 - 1e-3);
    const double los = std::atan2(
        to_aim.z(), std::max(1e-6, to_aim.head<2>().norm()));
    const double g_los = std::min(std::max(los, -cone), cone);
    const double step = prm.gamma_ramp_rad_per_s * prm.dt_s;
    const double dg = g_los - gamma_cmd;
    gamma_cmd += std::min(std::max(dg, -step), step);

    // --- leg speed: pinch the law's own servo band onto the command ---
    double v_cmd = wps[active].speed_mps > 0.0 ? wps[active].speed_mps
                                               : v_cruise;
    v_cmd = std::min(std::max(v_cmd, dyn.speed_min_mps),
                     dyn.speed_max_mps);
    tp::TransitionLimits lim;
    lim.dyn = dyn;
    // synthesizeCommands brackets the servo with v_lo = end_speed_min *
    // 1.02 and v_hi = end_speed_max * 0.98, so invert those factors to
    // make the band straddle v_cmd exactly.
    lim.end_speed_min_mps = v_cmd / 1.02;
    lim.end_speed_max_mps = v_cmd / 0.98;

    tp::Commands cmd =
        tp::synthesizeCommands(dyn, s, aim, gamma_cmd, bank_level, lim);
    gamma_cmd = cmd.gamma_cmd_applied;   // anti-windup

    if (prm.command_lag_s > 0.0) {
      // Sensitivity ablation only: a first-order lag between the law's
      // demand and what the airframe realizes. Off by default because the
      // benchmark follower is otherwise instantaneous — the point of the
      // knob is to test whether STRATEGY RANKINGS survive a less ideal
      // follower, which is the claim that has to hold for this whole
      // measurement to be useful.
      const double a = prm.dt_s / (prm.command_lag_s + prm.dt_s);
      if (!lag_seeded) {
        cl_state = cmd.cl;
        thrust_state = cmd.thrust_n;
        bank_state = cmd.bank_rad;
        lag_seeded = true;
      }
      cl_state += a * (cmd.cl - cl_state);
      thrust_state += a * (cmd.thrust_n - thrust_state);
      bank_state += a * (cmd.bank_rad - bank_state);
      cmd.cl = cl_state;
      cmd.thrust_n = thrust_state;
      cmd.bank_rad = bank_state;
    }

    // Saturation is COUNTED, not fatal: a follower riding a limit is
    // normal tracking. Only an unrepresentable state aborts.
    const auto closed =
        md::pointMassForces(dyn, s, cmd.cl, cmd.thrust_n, cmd.bank_rad);
    out.samples.back().acc_mps2 =
        tp::pointMassAcceleration(dyn, s, closed.inputs);
    if (closed.saturated() || !cmd.demand_interior) ++out.saturated_steps;

    tp::StepFlags fl;
    const md::PointMassState next = tp::rk4Step(
        dyn, s, prm.dt_s, cmd.cl, cmd.thrust_n, cmd.bank_rad, &fl);
    if (!fl.representable) {
      out.fail = FailReason::kUnrepresentable;
      out.total_steps = k + 1;
      return out;
    }
    s = next;
    out.total_steps = k + 1;
  }
  out.fail = FailReason::kTimeout;
  return out;
}

RolloutResult flyReferenceTrack(const SourcePath &src,
                                const FollowerStart &start,
                                const FollowerParams &prm)
{
  RolloutResult out;
  out.follower_id = "pointmass_reftrack_v1";
  if (src.empty()) {
    out.fail = FailReason::kNoWaypoints;
    return out;
  }
  const md::Parameters &dyn = prm.dyn;
  const double bank_level = derivedBankLevel(prm);

  md::PointMassState s;
  s.position_m = start.pos_m;
  const double v0 = start.vel_mps.norm();
  s.speed_mps = v0;
  s.flight_path_angle_rad =
      v0 > 1e-9
          ? std::asin(std::min(1.0, std::max(-1.0, start.vel_mps.z() / v0)))
          : 0.0;
  s.heading_rad = std::atan2(start.vel_mps.y(), start.vel_mps.x());

  const double t_max =
      std::max(30.0, prm.timeout_factor * src.total_time_s);
  const int max_steps = static_cast<int>(std::ceil(t_max / prm.dt_s));
  double gamma_cmd = s.flight_path_angle_rad;
  size_t cursor = 0;

  for (int k = 0; k <= max_steps; ++k) {
    const double t = k * prm.dt_s;
    if (!s.position_m.allFinite() || !std::isfinite(s.speed_mps)) {
      out.fail = FailReason::kNonFinite;
      return out;
    }
    out.samples.push_back(RolloutSample{
        t, s.position_m, md::pointMassVelocity(s), Eigen::Vector3d::Zero()});

    // Closest point on the source AHEAD of the carried cursor, then a
    // carrot lead_time seconds further along the path.
    double best = std::numeric_limits<double>::infinity();
    size_t near = cursor;
    const double win = std::max(2000.0, 0.05 * src.total_len_m);
    for (size_t i = cursor; i < src.pos_m.size(); ++i) {
      if (src.s_m[i] > src.s_m[cursor] + win) break;
      const double d = (src.pos_m[i] - s.position_m).norm();
      if (d < best) {
        best = d;
        near = i;
      }
    }
    cursor = near;
    const double s_aim =
        src.s_m[near] + std::max(1.0, s.speed_mps) * prm.lead_time_s;
    if (s_aim >= src.total_len_m &&
        (src.pos_m.back() - s.position_m).norm() <=
            derivedAcceptRadius(prm)) {
      out.completed = true;
      out.total_steps = k + 1;
      return out;
    }
    const size_t ai = arcIndex(src, std::min(s_aim, src.total_len_m));
    const Eigen::Vector3d aim = src.pos_m[ai];
    const double v_cmd =
        std::min(std::max(src.vel_mps[ai].norm(), dyn.speed_min_mps),
                 dyn.speed_max_mps);

    const Eigen::Vector3d to_aim = aim - s.position_m;
    const double cone = dyn.flight_path_angle_max_rad * (1.0 - 1e-3);
    const double los =
        std::atan2(to_aim.z(), std::max(1e-6, to_aim.head<2>().norm()));
    const double g_los = std::min(std::max(los, -cone), cone);
    const double step = prm.gamma_ramp_rad_per_s * prm.dt_s;
    const double dg = g_los - gamma_cmd;
    gamma_cmd += std::min(std::max(dg, -step), step);

    tp::TransitionLimits lim;
    lim.dyn = dyn;
    lim.end_speed_min_mps = v_cmd / 1.02;
    lim.end_speed_max_mps = v_cmd / 0.98;
    tp::Commands cmd =
        tp::synthesizeCommands(dyn, s, aim, gamma_cmd, bank_level, lim);
    gamma_cmd = cmd.gamma_cmd_applied;

    const auto closed =
        md::pointMassForces(dyn, s, cmd.cl, cmd.thrust_n, cmd.bank_rad);
    out.samples.back().acc_mps2 =
        tp::pointMassAcceleration(dyn, s, closed.inputs);
    if (closed.saturated() || !cmd.demand_interior) ++out.saturated_steps;

    tp::StepFlags fl;
    const md::PointMassState next = tp::rk4Step(
        dyn, s, prm.dt_s, cmd.cl, cmd.thrust_n, cmd.bank_rad, &fl);
    if (!fl.representable) {
      out.fail = FailReason::kUnrepresentable;
      out.total_steps = k + 1;
      return out;
    }
    s = next;
    out.total_steps = k + 1;
  }
  out.fail = FailReason::kTimeout;
  return out;
}

// ===================== metrics =====================

namespace {

// Nearest distance from a point to the source polyline, searched forward
// from a carried index within a window — a global nearest search would let
// a flight that doubled back match the wrong lobe.
double nearestOnSource(const SourcePath &src, const Eigen::Vector3d &p,
                       size_t *cursor, double window_m)
{
  double best = std::numeric_limits<double>::infinity();
  size_t best_i = *cursor;
  const double s_start = src.s_m[*cursor];
  for (size_t i = *cursor; i + 1 < src.pos_m.size(); ++i) {
    if (src.s_m[i] > s_start + window_m) break;
    const Eigen::Vector3d a = src.pos_m[i], b = src.pos_m[i + 1];
    const Eigen::Vector3d ab = b - a;
    const double L2 = ab.squaredNorm();
    double u = L2 > 1e-18 ? (p - a).dot(ab) / L2 : 0.0;
    u = std::min(1.0, std::max(0.0, u));
    const double d = (p - (a + u * ab)).norm();
    if (d < best) {
      best = d;
      best_i = i;
    }
  }
  *cursor = best_i;
  return best;
}

}  // namespace

WindowStats windowStats(const SourcePath &src, const RolloutResult &flown,
                        double t_lo_s, double t_hi_s, double t_centre_s)
{
  WindowStats w;
  if (src.empty() || flown.samples.size() < 2 || !flown.completed) return w;
  // Flown arc, so source and flown are compared at the same progress
  // fraction — the same matching the whole-flight metric uses.
  std::vector<double> f_s(flown.samples.size(), 0.0);
  for (size_t i = 1; i < flown.samples.size(); ++i)
    f_s[i] = f_s[i - 1] +
             (flown.samples[i].pos_m - flown.samples[i - 1].pos_m).norm();
  const double flown_len = f_s.back();
  if (!(flown_len > 0.0) || !(src.total_len_m > 0.0)) return w;

  const auto flownAtFrac = [&](double f) {
    const auto it = std::lower_bound(f_s.begin(), f_s.end(), f * flown_len);
    size_t i = static_cast<size_t>(std::distance(f_s.begin(), it));
    if (i >= flown.samples.size()) i = flown.samples.size() - 1;
    return i;
  };
  const auto fracAtTime = [&](double t) {
    const auto it = std::lower_bound(src.t_s.begin(), src.t_s.end(), t);
    size_t i = static_cast<size_t>(std::distance(src.t_s.begin(), it));
    if (i >= src.s_m.size()) i = src.s_m.size() - 1;
    return std::make_pair(i, src.s_m[i] / src.total_len_m);
  };

  double sum_sq = 0.0;
  int n = 0;
  for (size_t i = 0; i < src.t_s.size(); ++i) {
    if (src.t_s[i] < t_lo_s || src.t_s[i] > t_hi_s) continue;
    const size_t fi = flownAtFrac(src.s_m[i] / src.total_len_m);
    const double d = (flown.samples[fi].pos_m - src.pos_m[i]).norm();
    w.max_xtrack_m = std::max(w.max_xtrack_m, d);
    sum_sq += d * d;
    ++n;
  }
  if (n == 0) return w;
  w.rms_xtrack_m = std::sqrt(sum_sq / n);
  const auto c = fracAtTime(t_centre_s);
  const size_t fc = flownAtFrac(c.second);
  w.pos_err_at_t_m = (flown.samples[fc].pos_m - src.pos_m[c.first]).norm();
  w.speed_err_at_t_mps = std::abs(flown.samples[fc].vel_mps.norm() -
                                  src.vel_mps[c.first].norm());
  w.measured = true;
  return w;
}

ReproductionMetrics evaluateReproduction(
    const SourcePath &src, const RolloutResult &flown,
    const md::Parameters &dyn, const SafetyHooks &hooks,
    const EvalParams &ep)
{
  ReproductionMetrics m;
  m.fail = flown.fail;
  if (src.empty() || flown.samples.size() < 2) return m;
  // A flight that did not complete gets NO quality claim: the printer will
  // emit n/a(reason). Partial geometry is not comparable to a full run.
  if (!flown.completed) return m;
  m.measured = true;
  m.gate_complete = true;

  // --- deviation: nearest-point (primary) ---
  size_t cursor = 0;
  const double window = std::max(2000.0, 0.05 * src.total_len_m);
  double sum_sq = 0.0;
  for (const auto &fs : flown.samples) {
    const double d = nearestOnSource(src, fs.pos_m, &cursor, window);
    m.max_xtrack_m = std::max(m.max_xtrack_m, d);
    sum_sq += d * d;
  }
  m.rms_xtrack_m = std::sqrt(sum_sq / flown.samples.size());

  // --- flown arc + length ratio (the anti-shortcut gate) ---
  std::vector<double> f_s(flown.samples.size(), 0.0);
  for (size_t i = 1; i < flown.samples.size(); ++i)
    f_s[i] = f_s[i - 1] +
             (flown.samples[i].pos_m - flown.samples[i - 1].pos_m).norm();
  const double flown_len = f_s.back();
  m.len_ratio = src.total_len_m > 1e-9 ? flown_len / src.total_len_m : 0.0;
  m.gate_len = m.len_ratio >= ep.len_ratio_lo && m.len_ratio <= ep.len_ratio_hi;

  // --- deviation: arc-fraction matched (shape distortion) + time skew ---
  for (int j = 0; j <= 100; ++j) {
    const double f = j / 100.0;
    const size_t si = arcIndex(src, f * src.total_len_m);
    const auto fit =
        std::lower_bound(f_s.begin(), f_s.end(), f * flown_len);
    size_t fi = static_cast<size_t>(std::distance(f_s.begin(), fit));
    if (fi >= flown.samples.size()) fi = flown.samples.size() - 1;
    m.max_arcmatch_m = std::max(
        m.max_arcmatch_m,
        (flown.samples[fi].pos_m - src.pos_m[si]).norm());
    m.max_time_skew_s = std::max(
        m.max_time_skew_s,
        std::abs(flown.samples[fi].t_s - src.t_s[si]));
  }
  m.duration_ratio = src.total_time_s > 1e-9
                         ? flown.samples.back().t_s / src.total_time_s
                         : 0.0;

  // --- terminal ---
  const Eigen::Vector3d &pe = flown.samples.back().pos_m;
  m.terminal_pos_err_m = (pe - src.pos_m.back()).norm();
  m.terminal_speed_err_mps = std::abs(flown.samples.back().vel_mps.norm() -
                                      src.vel_mps.back().norm());
  m.gate_terminal = m.terminal_pos_err_m <= ep.terminal_pos_gate_m;
  // <=0 disables the deviation gate (the printer reports it as skipped);
  // with a tolerance set, PASS means the path was actually reproduced.
  m.gate_xtrack =
      ep.max_xtrack_gate_m <= 0.0 || m.max_xtrack_m <= ep.max_xtrack_gate_m;

  for (const auto &a : flown.arrivals)
    m.max_wp_miss_m = std::max(m.max_wp_miss_m, a.miss_m);
  m.sat_frac = flown.total_steps > 0
                   ? static_cast<double>(flown.saturated_steps) /
                         flown.total_steps
                   : 0.0;

  // --- flown-path envelope (statistic: characterizes the demand) ---
  for (const auto &fs : flown.samples) {
    const auto ev = md::evaluateInverseDynamics(dyn, fs.pos_m, fs.vel_mps,
                                                fs.acc_mps2);
    if (!ev.valid) continue;
    if (!md::isWithinEnvelope(dyn, ev)) ++m.env_violation_steps;
    m.env_peak_util = std::max(
        m.env_peak_util,
        std::max(ev.load_factor / std::max(1e-9, dyn.load_factor_max),
                 ev.thrust_required_n / std::max(1e-9, dyn.thrust_max_n)));
  }

  // --- flown-path safety: sentinels keep "not measured" out of "passed" ---
  if (hooks.terrain_z) {
    m.agl_measured = true;
    m.gate_agl_skipped = false;
    m.min_agl_m = std::numeric_limits<double>::infinity();
    for (const auto &fs : flown.samples) {
      double elev = 0.0;
      if (!hooks.terrain_z(fs.pos_m.x(), fs.pos_m.y(), &elev)) {
        if (ep.terrain_lookup_required) {
          // A failed lookup is NOT sea level. Assuming an elevation here
          // would let a flight over unmapped ground pass an AGL gate it
          // was never checked against (review find: fail-open).
          m.measured = false;
          return m;
        }
        elev = 0.0;
      }
      m.min_agl_m = std::min(m.min_agl_m, fs.pos_m.z() - elev);
    }
    m.gate_agl_pass = m.min_agl_m >= hooks.min_agl_m;
  }
  if (hooks.zone_probe) {
    m.zone_measured = true;
    m.gate_zone_skipped = false;
    for (const auto &fs : flown.samples) {
      const auto z = hooks.zone_probe(fs.pos_m);
      if (z == tp::ZoneProbe::CONTACT_HARD) ++m.zone_hard_contacts;
      if (z == tp::ZoneProbe::STALE_OR_INVALID) {
        // Never readable as clear: the whole evaluation is void.
        m.measured = false;
        return m;
      }
    }
    m.gate_zone_pass = m.zone_hard_contacts == 0;
  }
  return m;
}

// ===================== refinement =====================

RefineResult refineByError(const SourcePath &src, const FlyFn &fly,
                           const FollowerStart &start,
                           const RefineParams &prm, double accept_radius_m)
{
  RefineResult res;
  if (src.empty() || !fly) return res;
  const double min_sep = prm.min_spacing_m > 0.0 ? prm.min_spacing_m
                                                 : 2.0 * accept_radius_m;

  std::vector<Waypoint> route{endpointWaypoint(src)};
  double best_err = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < prm.max_waypoints; ++iter) {
    const RolloutResult r = fly(route, start);
    RefineRow row;
    row.count = static_cast<int>(route.size());
    row.completed = r.completed;

    double insert_s = -1.0;
    if (r.completed && r.samples.size() >= 2) {
      // Worst deviation point, and the source arc it maps to.
      size_t cursor = 0;
      const double window = std::max(2000.0, 0.05 * src.total_len_m);
      double worst = -1.0;
      size_t worst_src = 0;
      for (const auto &fs : r.samples) {
        size_t c = cursor;
        const double d = nearestOnSource(src, fs.pos_m, &c, window);
        cursor = c;
        if (d > worst) {
          worst = d;
          worst_src = c;
        }
      }
      row.max_xtrack_m = worst;
      row.measured = true;
      insert_s = src.s_m[worst_src];
      res.any_completed = true;
      if (worst < best_err) {
        best_err = worst;
        res.best = route;   // best-so-far: an insertion never degrades it
      }
      if (worst <= prm.xtrack_tol_m) {
        res.converged = true;
        res.trace.push_back(row);
        break;
      }
    } else {
      // A failed rollout has no trustworthy error field — fall back to
      // splitting the longest leg instead of inventing a number.
      double longest = -1.0;
      double prev_s = 0.0;
      for (const auto &w : route) {
        const double span = w.src_arc_m - prev_s;
        if (span > longest) {
          longest = span;
          insert_s = 0.5 * (prev_s + w.src_arc_m);
        }
        prev_s = w.src_arc_m;
      }
    }
    res.trace.push_back(row);
    if (static_cast<int>(route.size()) >= prm.max_waypoints) break;
    if (!(insert_s > 0.0)) break;

    // Min separation: an insertion the follower cannot resolve only
    // corrupts the comparison.
    bool blocked = false;
    for (const auto &w : route)
      if (std::abs(w.src_arc_m - insert_s) < min_sep) blocked = true;
    if (blocked) {
      double longest = -1.0, alt = -1.0, prev_s = 0.0;
      for (const auto &w : route) {
        const double span = w.src_arc_m - prev_s;
        if (span > longest) {
          longest = span;
          alt = 0.5 * (prev_s + w.src_arc_m);
        }
        prev_s = w.src_arc_m;
      }
      if (!(alt > 0.0) || longest < 2.0 * min_sep) break;
      insert_s = alt;
    }
    route.push_back(sampleAtArc(src, insert_s));
    std::sort(route.begin(), route.end(),
              [](const Waypoint &a, const Waypoint &b) {
                return a.src_arc_m < b.src_arc_m;
              });
  }
  if (res.best.empty() && !res.any_completed) res.best = route;
  return res;
}

}  // namespace waypoint_eval
}  // namespace path_manager
