#include "path_manager/route_postprocessing.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "path_planner/sdf/distance_field.h"

namespace path_manager {

TaggedRoute roundRouteCorners(TaggedRoute route, double radius,
                              const CornerAcceptance &sample_ok) {
  if (!(radius > 1e-6) || route.points.size() < 3) return route;
  if (route.edge_leg.size() + 1 != route.points.size())
    throw std::invalid_argument("active corner fillets require one tag per route edge");

  const auto &full_route = route.points;
  const auto &route_leg = route.edge_leg;
  TaggedRoute rounded;
  rounded.points.reserve(full_route.size() * 4);
  rounded.edge_leg.reserve(full_route.size() * 4);
  rounded.points.push_back(full_route.front());
  for (size_t n = 1; n + 1 < full_route.size(); ++n) {
    const Eigen::Vector3d &A = full_route[n - 1];
    const Eigen::Vector3d &B = full_route[n];
    const Eigen::Vector3d &C = full_route[n + 1];
    const size_t leg_in = route_leg[n - 1];
    const size_t leg_out = route_leg[n];
    const double d1 = (A - B).norm(), d2 = (C - B).norm();
    if (d1 < 1e-6 || d2 < 1e-6) {
      rounded.points.push_back(B);
      rounded.edge_leg.push_back(leg_in);
      continue;
    }
    const Eigen::Vector3d u = (A - B) / d1, w = (C - B) / d2;
    const double cosphi = std::clamp(u.dot(w), -1.0, 1.0);
    const double phi = std::acos(cosphi);
    if (phi > M_PI - 0.05) {
      rounded.points.push_back(B);
      rounded.edge_leg.push_back(leg_in);
      continue;
    }

    bool placed = false;
    for (double R = radius; R > 1.0; R *= 0.5) {
      double t = R / std::tan(0.5 * phi);
      const double t_cap = 0.45 * std::min(d1, d2);
      double R_eff = R;
      if (t > t_cap) { t = t_cap; R_eff = t * std::tan(0.5 * phi); }
      const Eigen::Vector3d P1 = B + u * t, P2 = B + w * t;
      int N = std::max(3, static_cast<int>(std::ceil((P1 - P2).norm() / 3.0)));
      // A real apex vertex separates the incoming and outgoing leg policies.
      const bool junction = (leg_in != leg_out);
      if (junction && (N % 2) != 0) ++N;
      std::vector<Eigen::Vector3d> arc;
      bool ok = true;
      for (int k = 1; k < N; ++k) {
        const double a = static_cast<double>(k) / N, b = 1.0 - a;
        const Eigen::Vector3d p = b * b * P1 + 2 * a * b * B + a * a * P2;
        if (sample_ok && !sample_ok(p, B)) { ok = false; break; }
        arc.push_back(p);
      }
      if (!ok) continue;
      rounded.points.push_back(P1);
      rounded.edge_leg.push_back(leg_in);
      for (int k = 1; k < N; ++k) {
        rounded.points.push_back(arc[k - 1]);
        rounded.edge_leg.push_back(
            (junction && k > N / 2) ? leg_out : leg_in);
      }
      rounded.points.push_back(P2);
      rounded.edge_leg.push_back(leg_out);
      placed = true;
      (void)R_eff;
      break;
    }
    if (!placed) {
      rounded.points.push_back(B);
      rounded.edge_leg.push_back(leg_in);
    }
  }
  rounded.points.push_back(full_route.back());
  rounded.edge_leg.push_back(route_leg.back());
  return rounded;
}

RouteProfileResult prepareRouteProfile(
    TaggedRoute route, const Eigen::Vector3d &start_pos,
    const std::vector<Eigen::Vector3d> &waypoints,
    const RouteProfileConfig &config, const ElevationQuery &elevation,
    const path_planner::sdf::IDistanceField *distance) {
  RouteProfileResult result;
  result.filtered_route = std::move(route.points);
  auto &full_route = result.filtered_route;
  const auto &route_leg = route.edge_leg;
  std::vector<double> raw_z;
  raw_z.reserve(full_route.size());
  for (const auto &p : full_route) {
    raw_z.push_back(p.z());
    result.raw_max_z = std::max(result.raw_max_z, p.z());
  }

  if (full_route.size() >= 3) {
    const size_t n = full_route.size();
    std::vector<double> zs(n);
    for (size_t i = 0; i < n; ++i) zs[i] = full_route[i].z();
    auto &stats = result.denoise;
    stats.applied = true;
    stats.min_before = zs[0];
    stats.max_before = zs[0];
    for (double z : zs) {
      stats.min_before = std::min(stats.min_before, z);
      stats.max_before = std::max(stats.max_before, z);
    }
    double mission_min_z = start_pos.z();
    for (const auto &wp : waypoints)
      mission_min_z = std::min(mission_min_z, wp.z());
    const double water_pin_floor = std::max(0.02, mission_min_z);
    const int W = 2;
    for (size_t i = 1; i + 1 < n; ++i) {
      double acc = 0.0;
      int cnt = 0;
      for (int k = -W; k <= W; ++k) {
        const long j = static_cast<long>(i) + k;
        if (j < 0 || j >= static_cast<long>(n)) continue;
        acc += zs[j];
        ++cnt;
      }
      double z_new = acc / cnt;
      if (elevation) {
        const float elev = elevation(full_route[i].x(), full_route[i].y());
        if (elev > -1e9f) {
          z_new = std::max(z_new, static_cast<double>(elev) + config.obstacle_clearance);
        } else {
          z_new = std::max(z_new, water_pin_floor);
        }
      }
      const Eigen::Vector3d cand(full_route[i].x(), full_route[i].y(), z_new);
      if (distance && distance->hasData()) {
        const float d = distance->getDistance(cand);
        if (!(std::isfinite(d) && d >= config.obstacle_clearance)) continue;
      }
      if (std::abs(z_new - zs[i]) > 1e-9) ++stats.adjusted;
      full_route[i].z() = z_new;
    }
    stats.min_after = full_route[0].z();
    stats.max_after = stats.min_after;
    for (const auto &p : full_route) {
      stats.min_after = std::min(stats.min_after, p.z());
      stats.max_after = std::max(stats.max_after, p.z());
    }
  }
  // Clamps may lift a pin, but smoothing must never lower its committed cap.
  for (size_t i = 0; i < raw_z.size() && i < full_route.size(); ++i)
    raw_z[i] = std::max(raw_z[i], full_route[i].z());

  result.max_segment_length = std::max(1.0, config.length_per_piece * 4.0);
  result.input_tags_match = (route_leg.size() + 1 == full_route.size());
  if (full_route.empty()) return result;
  result.path.reserve(full_route.size() * 4);
  result.path.push_back(full_route.front());
  result.cap_ref.reserve(full_route.size() * 4);
  result.cap_ref.push_back(raw_z.empty() ? full_route.front().z() : raw_z.front());
  result.edge_leg.reserve(full_route.size() * 4);
  for (size_t i = 0; i + 1 < full_route.size(); ++i) {
    const Eigen::Vector3d &a = full_route[i];
    const Eigen::Vector3d &b = full_route[i + 1];
    const double seg_len = (b - a).norm();
    if (seg_len < 1e-6) continue;
    const int n_sub = std::max(1, static_cast<int>(std::ceil(seg_len / result.max_segment_length)));
    const double ra = (i < raw_z.size()) ? raw_z[i] : a.z();
    const double rb = (i + 1 < raw_z.size()) ? raw_z[i + 1] : b.z();
    for (int kk = 1; kk <= n_sub; ++kk) {
      const double t = static_cast<double>(kk) / n_sub;
      result.path.push_back(a + (b - a) * t);
      result.cap_ref.push_back(ra + (rb - ra) * t);
      if (result.input_tags_match) result.edge_leg.push_back(route_leg[i]);
    }
  }
  if (!result.input_tags_match || result.edge_leg.size() + 1 != result.path.size())
    result.edge_leg.clear();
  return result;
}

}  // namespace path_manager
