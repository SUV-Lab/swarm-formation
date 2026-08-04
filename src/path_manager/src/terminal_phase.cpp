#include "path_manager/terminal_phase.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace path_manager {

namespace {

// One quintic Hermite piece matching exact P/V/A at both ends. Coefficient
// convention (poly_traj::Piece): p(t) = sum_i col(i) * t^(5-i), col(5) the
// constant. Verified against getPos/getVel/getAcc closed forms.
poly_traj::CoefficientMat hermite(const Eigen::Vector3d &p0,
                                  const Eigen::Vector3d &v0,
                                  const Eigen::Vector3d &a0,
                                  const Eigen::Vector3d &p1,
                                  const Eigen::Vector3d &v1,
                                  const Eigen::Vector3d &a1, double T)
{
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const Eigen::Vector3d A = p1 - p0 - v0 * T - 0.5 * a0 * T2;
  const Eigen::Vector3d B = v1 - v0 - a0 * T;
  const Eigen::Vector3d C = a1 - a0;
  poly_traj::CoefficientMat M;
  M.col(5) = p0;
  M.col(4) = v0;
  M.col(3) = 0.5 * a0;
  M.col(2) = 10.0 * A / T3 - 4.0 * B / T2 + 0.5 * C / T;
  M.col(1) = -15.0 * A / T4 + 7.0 * B / T3 - C / T2;
  M.col(0) = 6.0 * A / T5 - 3.0 * B / T4 + 0.5 * C / T3;
  return M;
}

// Quintic smoothstep and its first two derivatives on x in [0, 1]:
// C2 with zero rate and curvature at both ends.
double smooth5(double x) { return ((6.0 * x - 15.0) * x + 10.0) * x * x * x; }
double smooth5_d(double x) { return ((30.0 * x - 60.0) * x + 30.0) * x * x; }
double smooth5_dd(double x) { return ((120.0 * x - 180.0) * x + 60.0) * x; }

}  // namespace

poly_traj::Trajectory TerminalPhase::helixDescent(
    const Eigen::Vector3d &entry_pos, const Eigen::Vector3d &entry_vel,
    const TerminalHelixParams &prm,
    const std::function<bool(double, double, double *)> &terrain_z,
    double *min_agl_out)
{
  poly_traj::Trajectory traj;
  const double v_h = entry_vel.head<2>().norm();
  if (v_h < 1e-6 || prm.radius < 1e-6 || prm.turns <= 0.0) return traj;

  const double sgn = prm.right ? -1.0 : 1.0;
  const double kmax = 1.0 / prm.radius;
  // Trapezoidal |curvature| profile over arc length: ramp-in L_r, hold,
  // ramp-out L_r. Total heading change = kmax * (L_hold + L_r) = 2*pi*turns.
  double L_r = std::max(1e-3, prm.ramp_frac * prm.radius);
  double L_hold = 2.0 * M_PI * prm.turns / kmax - L_r;
  if (L_hold < 0.0) {  // tiny turns: degenerate to a triangle profile
    // Triangle heading budget = ∫|kappa| ds = kmax * L_r (each ramp
    // contributes kmax * L_r / 2), so L_r takes the WHOLE 2*pi*turns —
    // the earlier pi*turns silently delivered half the requested turn.
    L_r = 2.0 * M_PI * prm.turns / kmax;
    L_hold = 0.0;
  }
  const double S = 2.0 * L_r + L_hold;
  const double T_total = S / v_h;

  const auto kappa_at = [&](double s) -> double {
    if (s < L_r) return kmax * (s / L_r);
    if (s < L_r + L_hold) return kmax;
    if (s < S) return kmax * ((S - s) / L_r);
    return 0.0;
  };

  // Pass 1 — integrate the horizontal path finely (RK2 midpoint on the
  // heading; accuracy shapes the circle, continuity never depends on it:
  // the Hermite pieces interpolate whatever states are sampled).
  const double dt = 0.05;
  const int n_steps = static_cast<int>(std::ceil(T_total / dt));
  std::vector<double> xs(n_steps + 1), ys(n_steps + 1), ths(n_steps + 1);
  double x = entry_pos.x(), y = entry_pos.y();
  double th = std::atan2(entry_vel.y(), entry_vel.x());
  for (int k = 0; k <= n_steps; ++k) {
    xs[static_cast<size_t>(k)] = x;
    ys[static_cast<size_t>(k)] = y;
    ths[static_cast<size_t>(k)] = th;
    if (k == n_steps) break;
    const double h = std::min(dt, T_total - k * dt);
    const double s0 = (k * dt) * v_h;
    const double th_mid = th + sgn * kappa_at(s0 + 0.5 * h * v_h) *
                                   (0.5 * h * v_h);
    x += v_h * std::cos(th_mid) * h;
    y += v_h * std::sin(th_mid) * h;
    th += sgn * kappa_at(s0 + 0.5 * h * v_h) * (h * v_h);
  }

  // Vertical profile: entry z down to terrain + final_agl AT THE EXIT
  // POINT, quintic smoothstep over the whole duration.
  double ground = 0.0;
  terrain_z(xs.back(), ys.back(), &ground);  // false -> sea level 0
  const double z0 = entry_pos.z();
  const double z1 = ground + prm.final_agl;

  // Terrain sweep at the FINE sample resolution (a piece-boundary-only
  // probe misses ridges narrower than one piece): the lowest AGL along the
  // whole helix is reported so the caller can warn about a prescribed
  // geometry cutting terrain (nothing audits this phase yet — stage 4).
  double min_agl = std::numeric_limits<double>::infinity();
  for (int k = 0; k <= n_steps; ++k) {
    const double u = std::min(1.0, (k * dt) / T_total);
    const double z = z0 + (z1 - z0) * smooth5(u);
    double gk = 0.0;
    terrain_z(xs[static_cast<size_t>(k)], ys[static_cast<size_t>(k)], &gk);
    min_agl = std::min(min_agl, z - gk);
  }
  // Index-based: sample k IS the state at min(T_total, k*dt) — the last
  // fine step is short, so a nearest-in-TIME lookup could snap the exit
  // back one sample while z is anchored to the true end.
  const auto sample_time = [&](int k) { return std::min(T_total, k * dt); };
  const auto state_at = [&](int k, Eigen::Vector3d *p, Eigen::Vector3d *v,
                            Eigen::Vector3d *a) {
    const double t = sample_time(k);
    const double s = std::min(S, t * v_h);
    const double kap = kappa_at(s);
    const double c = std::cos(ths[static_cast<size_t>(k)]);
    const double sn = std::sin(ths[static_cast<size_t>(k)]);
    const double u = t / T_total;
    (*p) << xs[static_cast<size_t>(k)], ys[static_cast<size_t>(k)],
        z0 + (z1 - z0) * smooth5(u);
    (*v) << v_h * c, v_h * sn, (z1 - z0) * smooth5_d(u) / T_total;
    (*a) << -v_h * v_h * kap * sgn * sn, v_h * v_h * kap * sgn * c,
        (z1 - z0) * smooth5_dd(u) / (T_total * T_total);
  };

  const int n_pieces =
      std::max(1, static_cast<int>(std::ceil(T_total / prm.piece_dt)));
  Eigen::Vector3d p0, v0, a0, p1, v1, a1;
  int k0 = 0;
  state_at(0, &p0, &v0, &a0);
  // Entry exactness: sample 0 carries the caller's own position/heading and
  // kappa(0) = 0, smooth5_d(0) = 0 — P/V/A equal the handoff state.
  //
  // Piece boundaries ON SAMPLE INDICES: a piece's duration must be exactly
  // the time its sampled arc takes. Labelling a snapped sample with an
  // unsnapped request time made every piece fly its arc in the wrong
  // duration — a ±1.6% speed ripple and a spurious tangential acceleration
  // ~37% of the centripetal one, on a "constant-speed" phase whose analytic
  // model has exactly zero (review find, independently reproduced; the
  // index-driven form measures 0 ripple with joints still at 1e-16). The
  // final boundary is sample n_steps, which sits at T_total exactly — the
  // last fine step is short — so exit exactness is preserved too.
  for (int j = 1; j <= n_pieces; ++j) {
    const int k1 = (j == n_pieces)
                       ? n_steps
                       : static_cast<int>(std::llround(
                             static_cast<double>(j) * n_steps / n_pieces));
    if (k1 <= k0) continue;  // pathological piece_dt <= dt only
    const double T = sample_time(k1) - sample_time(k0);
    state_at(k1, &p1, &v1, &a1);
    traj.emplace_back(T, hermite(p0, v0, a0, p1, v1, a1, T));
    p0 = p1; v0 = v1; a0 = a1; k0 = k1;
  }
  if (min_agl_out) *min_agl_out = min_agl;
  return traj;
}

}  // namespace path_manager
