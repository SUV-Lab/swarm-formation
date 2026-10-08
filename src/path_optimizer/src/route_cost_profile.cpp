#include "path_optimizer/route_cost_profile.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner {

PieceAltitudeCapResult buildPieceAltitudeCaps(
    const std::vector<Eigen::Vector3d> &route,
    const std::vector<double> &cap_ref,
    const std::vector<RouteCostZone> &zones,
    const PieceAltitudeCapConfig &config,
    const TerrainHeightQuery &height,
    const TerrainHeightGradientQuery &height_gradient,
    const ShadowCeilingQuery &shadow)
{
  PieceAltitudeCapResult result;
  if (cap_ref.empty()) return result;
  if (route.size() < 2 || cap_ref.size() != route.size()) {
    result.status = PieceAltitudeCapStatus::REFERENCE_MISMATCH;
    return result;
  }
  const int piece_num = static_cast<int>(route.size()) - 1;
  // Two-pass slope-cone envelope of the paired reference. XY arc distance
  // lets high reference values license anticipation without changing the
  // constant per-piece ceiling during optimization.
  std::vector<double> env = cap_ref;
  for (int i = 1; i <= piece_num; ++i) {   // forward pass
    const double d = (route[i] - route[i - 1]).head<2>().norm();
    env[i] = std::max(env[i], env[i - 1] - config.slope * d);
  }
  for (int i = piece_num - 1; i >= 0; --i) { // backward pass
    const double d = (route[i + 1] - route[i]).head<2>().norm();
    env[i] = std::max(env[i], env[i + 1] - config.slope * d);
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
  // once from route (decision-variable independent, gradients keep
  // their exact form); over open water it adds nothing, so the hump
  // suppression is untouched, and the lift is LOCAL (a tall islet no
  // longer raises the whole route's ceiling — only its own +-kSwathR).
  const double kSwathR = 25.0;      // ~ one piece length of deviation slack
  // Sampling pitch = one DEM cell (2.3 u matched the 250 m
  // full_map grid;
  // corridor crops are 30-40 m, and a coarser-than-cell stride can step
  // clean over a one-cell ridge — the exact failure this floor guards).
  // Runs once per plan on a decision-variable-independent quantity, so
  // the finer pitch costs ~0.1-0.2 s worst case, not per-iteration time.
  const double kStepMax = 2.3;
  const double step = (config.terrain_cell_u > 0.0)
                          ? std::min(kStepMax, config.terrain_cell_u)
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
        if (height_gradient) {
          float h, gx, gy;
          if (height_gradient(x, y, &h, &gx, &gy) && h > hmax) hmax = h;
        } else if (height) {
          const float h = height(x, y);
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
  // Computed once from route — decision-variable independent, cap
  // gradients keep their exact form; zone-free ridge terrain-following
  // missions are
  // untouched by construction. Default 0 = off (legacy cap).
  auto zone_infl = [&](const Eigen::Vector3d &a,
                       const Eigen::Vector3d &b) -> double {
    if (!config.use_risk_zones || config.zone_relax <= 0.0) return 0.0;
    double infl = 0.0;
    const Eigen::Vector2d p0 = a.head<2>(), p1 = b.head<2>();
    const Eigen::Vector2d ab = p1 - p0;
    const double L2 = ab.squaredNorm();
    for (const auto &tz : zones) {
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

  // [OCCLUSION-CAP] (H3) visibility-aware cap: sample the same swath the
  // terrain floor uses; at each point take the min over zones IN XY REACH
  // of the LOS shadow ceiling (PathManager's per-zone radial-horizon
  // table). Below (min - margin) the piece is terrain-occluded from every
  // zone that
  // could reach it, so the band's downward pressure owns nothing there
  // and the cap may rise ("only low where visible"). A sample with an
  // in-reach zone but no finite ceiling (mask off, bare over-water ray,
  // at-center query) kills the min — a piece with ANY unshadowed in-reach
  // sample never relaxes, and pieces no zone can reach keep the legacy
  // band (its hump-suppression role is untouched). The swath, not just
  // the chord, keeps the license consistent with the lateral drift the
  // cap floor already budgets for: the trajectory may deviate kSwathR
  // sideways, so the relax must hold in the lit seams it could drift
  // into. Computed once from route — decision-variable independent,
  // cap gradients keep their exact form.
  auto shadow_ceil = [&](const Eigen::Vector3d &a,
                         const Eigen::Vector3d &b) -> double {
    const double kNegInf = -std::numeric_limits<double>::infinity();
    if (!config.use_risk_zones || config.shadow_margin <= 0.0 ||
        !shadow)
      return kNegInf;
    const Eigen::Vector2d p0 = a.head<2>(), p1 = b.head<2>();
    const Eigen::Vector2d ab = p1 - p0;
    const double L = ab.norm(), L2 = ab.squaredNorm();
    std::vector<size_t> cand;
    for (size_t zi = 0; zi < zones.size(); ++zi) {
      const auto &tz = zones[zi];
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
          const auto &tz = zones[zi];
          if ((p.head<2>() - tz.center.head<2>()).norm() > tz.reach)
            continue;
          reached = true;
          ceil_min = std::min(ceil_min, shadow(zi, p));
          if (!(ceil_min > kNegInf)) return kNegInf;  // unshadowed sample
        }
      }
    }
    return reached ? ceil_min : kNegInf;
  };

  result.ceilings.resize(piece_num);
  double cap_min = 1e30, cap_max = -1e30;
  int relaxed_pieces = 0;
  int shadow_pieces = 0;
  double shadow_lift_max = 0.0;
  for (int i = 0; i < piece_num; ++i) {
    double ref = std::max(env[i], env[i + 1]);
    const double terr = swath_terr(route[i], route[i + 1]);
    if (terr > -1e29) {
      ref = std::max(ref, terr + config.obstacle_clearance);
    }
    const double zr =
        config.zone_relax * zone_infl(route[i], route[i + 1]);
    if (zr > 1e-9) ++relaxed_pieces;
    result.ceilings(i) = ref + config.headroom + zr;
    const double sc = shadow_ceil(route[i], route[i + 1]);
    if (std::isfinite(sc)) {
      const double cand_cap = sc - config.shadow_margin;
      if (cand_cap > result.ceilings(i)) {
        shadow_lift_max =
            std::max(shadow_lift_max, cand_cap - result.ceilings(i));
        result.ceilings(i) = cand_cap;
        ++shadow_pieces;
      }
    }
    cap_min = std::min(cap_min, result.ceilings(i));
    cap_max = std::max(cap_max, result.ceilings(i));
  }
  result.status = PieceAltitudeCapStatus::ACTIVE;
  result.cap_min = cap_min;
  result.cap_max = cap_max;
  result.relaxed_pieces = relaxed_pieces;
  result.shadow_pieces = shadow_pieces;
  result.shadow_lift_max = shadow_lift_max;
  return result;
}

PieceRoughnessResult buildPieceRoughness(
    const std::vector<Eigen::Vector3d> &route,
    const PieceRoughnessConfig &config,
    const TerrainHeightQuery &height,
    const TerrainHeightGradientQuery &height_gradient)
{
  PieceRoughnessResult result;
  if (route.size() < 2 ||
      !((config.time_relief > 0.0 || config.ride_weight > 0.0) &&
        (height_gradient || height))) return result;
  const int piece_num = static_cast<int>(route.size()) - 1;
  constexpr double kSlope0 = 0.2, kSlopeFull = 0.8;
  const double rstep = (config.terrain_cell_u > 0.0)
                           ? std::min(2.3, config.terrain_cell_u)
                           : 2.3;
  result.roughness.resize(piece_num);
  if (config.time_relief > 0.0) result.time_factors.resize(piece_num);
  int relieved = 0;
  double factor_min = 1.0;
  for (int i = 0; i < piece_num; ++i) {
    const Eigen::Vector3d &a = route[i], &b = route[i + 1];
    const double L = (b - a).head<2>().norm();
    double excess_sum = 0.0;
    int n = 0;
    for (double s = 0.0; s <= L + 1e-9; s += rstep) {
      const double t = (L > 1e-9) ? s / L : 0.0;
      const double x = a.x() + (b.x() - a.x()) * t;
      const double y = a.y() + (b.y() - a.y()) * t;
      float h = 0.f, gx = 0.f, gy = 0.f;
      if (height_gradient && height_gradient(x, y, &h, &gx, &gy)) {
        const double slope = std::hypot(static_cast<double>(gx),
                                        static_cast<double>(gy));
        // AGL fade (same 1.0 -> 2.5 u band as the FE [ROUGH] field,
        // evaluated on the COMMITTED profile z — frozen per solve): a
        // high transit over a ridge is not terrain-following and gets
        // no relief; only the terrain-following regime does.
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
    result.roughness(i) = rough;
    if (rough > 1e-9) ++relieved;
    if (config.time_relief > 0.0) {
      const double f = 1.0 - config.time_relief * rough;
      result.time_factors(i) = f;
      factor_min = std::min(factor_min, f);
    }
  }
  result.active = true;
  result.rough_pieces = relieved;
  result.factor_min = factor_min;
  return result;
}

}  // namespace ego_planner
