#ifndef PATH_MANAGER_TRAJ_SAMPLING_H
#define PATH_MANAGER_TRAJ_SAMPLING_H

// Dense sampling of a planned polynomial trajectory: the (t, cumulative
// arc) table and the arc->time lookup. Lifted out of the chain planner's
// file-local copy so the waypoint machinery and the chain report share ONE
// definition (the quintic_hermite.h precedent) — an arc convention that
// forks would make two reports incomparable.
//
// Arc is CHORDAL (sum of sample-to-sample distances), so it underestimates
// true arc on curved pieces; the underestimate shrinks with sample count
// and the harness pins its convergence.

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {
namespace traj_sampling {

struct ArcTable {
  std::vector<double> t, s;
  double total{0.0};
};

// Uniform-in-TIME samples with the cumulative chordal arc alongside.
// samples < 1 or a degenerate trajectory yields an empty table (total 0).
inline ArcTable buildArcTable(const poly_traj::Trajectory &traj, int samples)
{
  ArcTable a;
  if (traj.getPieceNum() <= 0 || samples < 1) return a;
  const double T = traj.getTotalDuration();
  if (!(T > 1e-6)) return a;
  a.t.reserve(static_cast<size_t>(samples) + 1);
  a.s.reserve(static_cast<size_t>(samples) + 1);
  Eigen::Vector3d prev = traj.getPos(0.0);
  double s = 0.0;
  for (int k = 0; k <= samples; ++k) {
    const double tt = std::min(T, k * T / samples);
    const Eigen::Vector3d p = traj.getPos(tt);
    s += (p - prev).norm();
    prev = p;
    a.t.push_back(tt);
    a.s.push_back(s);
  }
  a.total = s;
  return a;
}

// Trajectory time at a fraction of total arc (linear interpolation between
// bracketing samples; clamped at both ends).
inline double timeAtArcFrac(const ArcTable &a, double frac)
{
  if (a.t.empty()) return 0.0;
  const double target = frac * a.total;
  const auto it = std::lower_bound(a.s.begin(), a.s.end(), target);
  const size_t i = static_cast<size_t>(std::distance(a.s.begin(), it));
  if (i == 0) return a.t.front();
  if (i >= a.s.size()) return a.t.back();
  const double s0 = a.s[i - 1], s1 = a.s[i];
  const double w = (s1 > s0) ? (target - s0) / (s1 - s0) : 0.0;
  return a.t[i - 1] + w * (a.t[i] - a.t[i - 1]);
}

// Trajectory time at an ABSOLUTE arc length (same interpolation).
inline double timeAtArc(const ArcTable &a, double s_arc)
{
  if (a.total <= 0.0) return a.t.empty() ? 0.0 : a.t.front();
  return timeAtArcFrac(a, s_arc / a.total);
}

// Geometric curvature |v x a| / |v|^3 of a sampled trajectory point.
// Guarded at low speed: below v_floor the direction is not meaningful, so
// the curvature reads 0 rather than exploding (a straight-line reading is
// the fail-safe here — it never CONCENTRATES waypoints on noise).
inline double curvature3(const Eigen::Vector3d &v, const Eigen::Vector3d &a,
                         double v_floor = 1.0)
{
  const double sp = v.norm();
  if (!(sp > v_floor)) return 0.0;
  return v.cross(a).norm() / (sp * sp * sp);
}

}  // namespace traj_sampling
}  // namespace path_manager

#endif  // PATH_MANAGER_TRAJ_SAMPLING_H
