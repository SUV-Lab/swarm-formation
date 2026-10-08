// Analytic route profiles and observable terrain queries, without ROS/MINCO.
#include "path_optimizer/route_cost_profile.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace ego_planner;
using Route = std::vector<Eigen::Vector3d>;
int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

bool near(double a, double b) { return std::abs(a - b) < 1e-10; }

bool values(const Eigen::VectorXd &actual, const std::vector<double> &expected)
{
  if (actual.size() != static_cast<Eigen::Index>(expected.size())) return false;
  for (size_t i = 0; i < expected.size(); ++i)
    if (!near(actual[i], expected[i])) return false;
  return true;
}

bool point(const Eigen::Vector2d &p, double x, double y)
{
  return near(p.x(), x) && near(p.y(), y);
}

bool sameRoute(const Route &a, const Route &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!((a[i] - b[i]).norm() == 0.0)) return false;
  return true;
}

void coneAndFallback()
{
  const Route route{{0, 0, 0}, {10, 0, 100}, {20, 0, 0}, {30, 0, 100}};
  const auto original = route;
  const std::vector<double> reference{1, 5, 1, 1};
  const auto forward = buildPieceAltitudeCaps(route, reference, {}, {});
  check(forward.status == PieceAltitudeCapStatus::ACTIVE &&
            values(forward.ceilings, {5.4, 5.4, 4.4}) &&
            near(forward.cap_min, 4.4) && near(forward.cap_max, 5.4),
        "a peak licenses the following pieces through the XY slope cone");
  const auto backward = buildPieceAltitudeCaps(route, {1, 1, 5, 1}, {}, {});
  check(values(backward.ceilings, {4.4, 5.4, 5.4}),
        "a later peak licenses climb anticipation before reaching that peak");
  auto flat = route;
  for (auto &p : flat) p.z() = 0.0;
  check(values(buildPieceAltitudeCaps(flat, reference, {}, {}).ceilings,
               {5.4, 5.4, 4.4}) && sameRoute(route, original) &&
            reference == std::vector<double>({1, 5, 1, 1}),
        "cone distance ignores vertical excursion and inputs remain unchanged");
  PieceAltitudeCapConfig steep;
  steep.slope = 1.0;
  steep.headroom = 2.0;
  check(values(buildPieceAltitudeCaps(route, reference, {}, steep).ceilings,
               {7.0, 7.0, 3.0}),
        "explicit slope and headroom change only their expected cone limits");

  int queries = 0;
  TerrainHeightQuery height = [&](double, double) { ++queries; return 8.0f; };
  TerrainHeightGradientQuery gradient = [&](double, double, float *, float *, float *) {
    ++queries; return false;
  };
  ShadowCeilingQuery shadow = [&](size_t, const Eigen::Vector3d &) {
    ++queries; return 8.0;
  };
  PieceAltitudeCapConfig enabled;
  enabled.use_risk_zones = true;
  enabled.zone_relax = 2.0;
  enabled.shadow_margin = 1.0;
  const std::vector<RouteCostZone> zones{{Eigen::Vector3d::Zero(), 100.0}};
  const auto absent = buildPieceAltitudeCaps(route, {}, zones, enabled,
                                            height, gradient, shadow);
  const auto mismatch = buildPieceAltitudeCaps(route, {1}, zones, enabled,
                                              height, gradient, shadow);
  const auto singleton = buildPieceAltitudeCaps({{0, 0, 0}}, {1}, zones, enabled,
                                               height, gradient, shadow);
  const auto empty = buildPieceAltitudeCaps({}, {}, zones, enabled,
                                           height, gradient, shadow);
  check(absent.status == PieceAltitudeCapStatus::SCALAR_FALLBACK &&
            empty.status == PieceAltitudeCapStatus::SCALAR_FALLBACK &&
            mismatch.status == PieceAltitudeCapStatus::REFERENCE_MISMATCH &&
            singleton.status == PieceAltitudeCapStatus::REFERENCE_MISMATCH &&
            absent.ceilings.size() == 0 && mismatch.ceilings.size() == 0 &&
            singleton.ceilings.size() == 0 && queries == 0,
        "absent and malformed references return explicit empty outcomes without queries");
}

struct HeightRecorder {
  std::vector<Eigen::Vector2d> queries;
  float operator()(double x, double y)
  {
    queries.emplace_back(x, y);
    return near(x, 1.0) && near(y, 0.0) ? 9.0f : 0.0f;
  }
};

struct GradientRecorder {
  std::vector<Eigen::Vector2d> queries;
  bool operator()(double x, double y, float *h, float *gx, float *gy)
  {
    queries.emplace_back(x, y);
    *h = 0.0f; *gx = 0.5f; *gy = 0.0f;
    return true;
  }
};

struct ShadowRecorder {
  std::vector<std::pair<size_t, Eigen::Vector3d>> queries;
  double operator()(size_t index, const Eigen::Vector3d &p)
  {
    queries.emplace_back(index, p);
    return index == 2 ? 12.0 : 8.0;
  }
};

void swathCases()
{
  const Route route{{0, 0, 4}, {2, 0, 4}};
  PieceAltitudeCapConfig fine;
  fine.terrain_cell_u = 1.0;
  TerrainHeightQuery height = HeightRecorder{};
  const auto ridge = buildPieceAltitudeCaps(route, {0, 0}, {}, fine, height);
  const auto &q = height.target<HeightRecorder>()->queries;
  check(values(ridge.ceilings, {9.9}) && q.size() == 153 &&
            point(q[0], 0, -25) && point(q[1], 0, -24) &&
            point(q[50], 0, 25) && point(q[51], 1, -25) &&
            point(q[76], 1, 0) && point(q[152], 2, 25),
        "cell-pitch swath samples along then across the chord and finds a one-cell ridge");
  check(!q.empty(), "terrain functor state changes in the borrowed original callback");

  TerrainHeightQuery coarse_height = HeightRecorder{};
  const auto coarse = buildPieceAltitudeCaps(route, {0, 0}, {}, {}, coarse_height);
  const auto &c = coarse_height.target<HeightRecorder>()->queries;
  check(values(coarse.ceilings, {0.9}) && c.size() == 22 &&
            point(c.front(), 0, -25) && point(c[1], 0, -22.7) &&
            point(c.back(), 0, 23.3),
        "default 2.3 pitch preserves floating-step positions without forcing the chord endpoint");
  PieceAltitudeCapConfig larger;
  larger.terrain_cell_u = 99.0;
  TerrainHeightQuery capped_height = HeightRecorder{};
  buildPieceAltitudeCaps(route, {0, 0}, {}, larger, capped_height);
  const auto &clamped = capped_height.target<HeightRecorder>()->queries;
  bool same_queries = clamped.size() == c.size();
  for (size_t i = 0; same_queries && i < c.size(); ++i)
    same_queries = (clamped[i] - c[i]).norm() == 0.0;
  check(same_queries, "a coarse terrain cell retains the 2.3 maximum sampling pitch");

  int height_calls = 0, gradient_calls = 0;
  TerrainHeightQuery fallback = [&](double, double) { ++height_calls; return 99.0f; };
  TerrainHeightGradientQuery unavailable = [&](double, double, float *, float *, float *) {
    ++gradient_calls; return false;
  };
  const auto failed = buildPieceAltitudeCaps(route, {0, 0}, {}, fine,
                                            fallback, unavailable);
  check(values(failed.ceilings, {0.4}) && gradient_calls == 153 && height_calls == 0,
        "a present failed gradient provider prevents height fallback for every swath sample");
  TerrainHeightQuery infinite_height = [](double, double) {
    return std::numeric_limits<float>::infinity();
  };
  TerrainHeightGradientQuery infinite_gradient = [](double, double, float *h,
                                                    float *gx, float *gy) {
    *h = std::numeric_limits<float>::infinity(); *gx = *gy = 0.0f; return true;
  };
  const auto h_inf = buildPieceAltitudeCaps(route, {0, 0}, {}, fine, infinite_height);
  const auto g_inf = buildPieceAltitudeCaps(route, {0, 0}, {}, fine, {}, infinite_gradient);
  check(values(h_inf.ceilings, {0.4}) && g_inf.ceilings.size() == 1 &&
            std::isinf(g_inf.ceilings[0]) && g_inf.ceilings[0] > 0.0,
        "height fallback rejects infinity while a successful gradient retains legacy height handling");

  std::vector<Eigen::Vector2d> vertical_queries;
  TerrainHeightQuery vertical_height = [&](double x, double y) {
    vertical_queries.emplace_back(x, y); return 0.0f;
  };
  buildPieceAltitudeCaps({{3, 7, 1}, {3, 7, 20}}, {1, 1}, {}, fine, vertical_height);
  check(vertical_queries.size() == 51 && point(vertical_queries.front(), 3, -18) &&
            point(vertical_queries.back(), 3, 32),
        "a zero-XY chord uses the fixed positive-X chord direction once");
}

void zoneCases()
{
  const Route route{{0, 0, 4}, {10, 0, 4}};
  PieceAltitudeCapConfig relaxed;
  relaxed.use_risk_zones = true;
  relaxed.zone_relax = 2.0;
  const auto inside = buildPieceAltitudeCaps(route, {1, 1},
      {{{5, 0, 100}, 2}, {{5, 0, -100}, 2}}, relaxed);
  const auto halfway = buildPieceAltitudeCaps(route, {1, 1}, {{{5, 2.5, 0}, 2}}, relaxed);
  const auto outside = buildPieceAltitudeCaps(route, {1, 1}, {{{5, 3, 0}, 2}}, relaxed);
  const auto beyond_end = buildPieceAltitudeCaps(route, {1, 1}, {{{-3, 0, 0}, 2}}, relaxed);
  check(values(inside.ceilings, {3.4}) && inside.relaxed_pieces == 1 &&
            values(halfway.ceilings, {2.4}) && halfway.relaxed_pieces == 1 &&
            values(outside.ceilings, {1.4}) && outside.relaxed_pieces == 0 &&
            values(beyond_end.ceilings, {1.4}),
        "zone relief uses maximum XY proximity, rim fade and finite chord endpoints");

  const Route point_route{{0, 0, 4}, {0, 0, 5}};
  const std::vector<RouteCostZone> zones{
      {{0, 0, 0}, 0}, {{1000, 0, 0}, 2}, {{0, 0, 3}, 100},
      {{0, 0, 0}, -1}, {{0, 0, 9}, 100}};
  PieceAltitudeCapConfig shadow_config;
  shadow_config.use_risk_zones = true;
  shadow_config.shadow_margin = 1.0;
  shadow_config.terrain_cell_u = 1.0;
  ShadowCeilingQuery shadow = ShadowRecorder{};
  auto &calls = shadow.target<ShadowRecorder>()->queries;
  const auto lit = buildPieceAltitudeCaps(point_route, {1, 1}, zones, shadow_config,
                                         {}, {}, shadow);
  bool identities = calls.size() == 102;
  for (size_t i = 0; identities && i < calls.size(); ++i)
    identities = calls[i].first == (i % 2 == 0 ? 2u : 4u) && calls[i].second.z() == 0.0;
  check(values(lit.ceilings, {7.0}) && lit.shadow_pieces == 1 &&
            near(lit.shadow_lift_max, 5.6) && identities &&
            calls.front().second == Eigen::Vector3d(0, -25, 0) &&
            calls.back().second == Eigen::Vector3d(0, 25, 0),
        "shadow queries retain original ordered zone indices and minimum ceiling across the swath");

  calls.clear();
  ShadowCeilingQuery unshadowed = [&](size_t index, const Eigen::Vector3d &p) {
    calls.emplace_back(index, p);
    return index == 4 && p.y() == -24.0
        ? -std::numeric_limits<double>::infinity() : 12.0;
  };
  const auto killed = buildPieceAltitudeCaps(point_route, {1, 1}, zones,
                                             shadow_config, {}, {}, unshadowed);
  check(values(killed.ceilings, {1.4}) && killed.shadow_pieces == 0 &&
            killed.shadow_lift_max == 0.0 && calls.size() == 4,
        "one unshadowed in-reach sample cancels the entire piece lift and stops sampling");
  calls.clear();
  ShadowCeilingQuery first_unshadowed = [&](size_t index, const Eigen::Vector3d &p) {
    calls.emplace_back(index, p); return -std::numeric_limits<double>::infinity();
  };
  buildPieceAltitudeCaps(point_route, {1, 1}, zones, shadow_config,
                         {}, {}, first_unshadowed);
  check(calls.size() == 1 && calls.front().first == 2,
        "negative infinity short-circuits before later zones at the same sample");
  calls.clear();
  const auto unreached = buildPieceAltitudeCaps(point_route, {1, 1},
      {{{0, 0.5, 0}, 0.1}}, shadow_config, {}, {}, shadow);
  shadow_config.use_risk_zones = false;
  const auto disabled = buildPieceAltitudeCaps(point_route, {1, 1}, zones,
                                              shadow_config, {}, {}, shadow);
  check(values(unreached.ceilings, {1.4}) && values(disabled.ceilings, {1.4}) &&
            calls.empty() && disabled.shadow_pieces == 0 &&
            values(lit.ceilings, {7.0}),
        "unreached and disabled zones cannot reuse an earlier shadow lift");
}

void roughnessCases()
{
  const Route rising{{0, 0, 1}, {2, 0, 2.5}};
  const PieceRoughnessConfig config{0.4, 0.0, 1.0};
  TerrainHeightGradientQuery gradient = GradientRecorder{};
  auto &queries = gradient.target<GradientRecorder>()->queries;
  const auto faded = buildPieceRoughness(rising, config, {}, gradient);
  check(faded.active && values(faded.roughness, {0.25}) &&
            values(faded.time_factors, {0.9}) && faded.rough_pieces == 1 &&
            near(faded.factor_min, 0.9) && queries.size() == 3 &&
            point(queries[0], 0, 0) && point(queries[1], 1, 0) && point(queries[2], 2, 0),
        "committed AGL at near, half-fade and far samples averages to one-quarter roughness");
  const auto near_ground = buildPieceRoughness({{0, 0, 1}, {2, 0, 1}}, config, {}, gradient);
  const auto high = buildPieceRoughness({{0, 0, 2.5}, {2, 0, 2.5}}, config, {}, gradient);
  check(values(near_ground.roughness, {0.5}) && values(near_ground.time_factors, {0.8}) &&
            values(high.roughness, {0.0}) && values(high.time_factors, {1.0}) &&
            high.rough_pieces == 0 && high.factor_min == 1.0,
        "terrain-following relief fades completely at the 2.5 AGL boundary");

  int height_calls = 0, gradient_calls = 0;
  TerrainHeightQuery height = [&](double, double) { ++height_calls; return 0.0f; };
  TerrainHeightGradientQuery partial = [&](double x, double, float *h,
                                           float *gx, float *gy) {
    ++gradient_calls; *h = 0.0f; *gx = 0.5f; *gy = 0.0f; return x != 1.0;
  };
  const Route low{{0, 0, 1}, {2, 0, 1}};
  const auto failed = buildPieceRoughness(low, config, height, partial);
  check(values(failed.roughness, {1.0 / 3.0}) &&
            values(failed.time_factors, {1.0 - 0.4 / 3.0}) &&
            gradient_calls == 3 && height_calls == 0,
        "a failed gradient counts in the denominator and never falls back to height");
  const auto height_only = buildPieceRoughness(low, config, height);
  check(height_only.active && values(height_only.roughness, {0.0}) &&
            values(height_only.time_factors, {1.0}) && height_only.rough_pieces == 0 &&
            height_only.factor_min == 1.0 && height_calls == 0,
        "height-only terrain enables zero roughness without calling the height provider");
  const auto ride_only = buildPieceRoughness(low, {0.0, 3.0, 1.0}, {}, gradient);
  check(ride_only.active && values(ride_only.roughness, {0.5}) &&
            ride_only.time_factors.size() == 0 && ride_only.rough_pieces == 1 &&
            ride_only.factor_min == 1.0,
        "ride cost can request roughness without creating time relief factors");
  TerrainHeightGradientQuery steep = [](double, double, float *h, float *gx, float *gy) {
    *h = 0.0f; *gx = 1.0f; *gy = 0.0f; return true;
  };
  TerrainHeightGradientQuery mild = [](double, double, float *h, float *gx, float *gy) {
    *h = 0.0f; *gx = 0.125f; *gy = 0.0f; return true;
  };
  check(values(buildPieceRoughness(low, config, {}, steep).roughness, {1.0}) &&
            values(buildPieceRoughness(low, config, {}, mild).roughness, {0.0}),
        "slope deadband and full-roughness saturation keep the expected bounds");

  queries.clear();
  const auto coarse = buildPieceRoughness(rising, {0.4, 0.0, 0.0}, {}, gradient);
  check(values(coarse.roughness, {0.5}) && queries.size() == 1 && point(queries.front(), 0, 0),
        "roughness uses the same maximum pitch and does not append an unsampled endpoint");
  queries.clear();
  const auto off = buildPieceRoughness(low, {}, height, gradient);
  const auto absent = buildPieceRoughness(low, config);
  const auto empty = buildPieceRoughness({}, config, height, gradient);
  const auto single = buildPieceRoughness({{0, 0, 1}}, config, height, gradient);
  check(!off.active && !absent.active && !empty.active && !single.active &&
            off.roughness.size() == 0 && off.time_factors.size() == 0 &&
            absent.roughness.size() == 0 && empty.roughness.size() == 0 &&
            single.roughness.size() == 0 && off.rough_pieces == 0 &&
            off.factor_min == 1.0 && queries.empty() && height_calls == 0 &&
            values(near_ground.roughness, {0.5}),
        "disabled, missing and short inputs return fresh inactive tables without queries");
}

void ownership()
{
  PieceAltitudeCapResult caps;
  PieceRoughnessResult rough;
  std::weak_ptr<float> height_lifetime, gradient_lifetime, shadow_lifetime;
  {
    Route route{{0, 0, 1}, {2, 0, 1}};
    std::vector<double> reference{1, 1};
    auto height_value = std::make_shared<float>(9.0f);
    auto gradient_value = std::make_shared<float>(0.5f);
    auto shadow_value = std::make_shared<float>(20.0f);
    height_lifetime = height_value;
    gradient_lifetime = gradient_value;
    shadow_lifetime = shadow_value;
    TerrainHeightQuery height = [height_value](double, double) { return *height_value; };
    TerrainHeightGradientQuery gradient = [gradient_value](double, double, float *h,
                                                           float *gx, float *gy) {
      *h = 0.0f; *gx = *gradient_value; *gy = 0.0f; return true;
    };
    ShadowCeilingQuery shadow = [shadow_value](size_t, const Eigen::Vector3d &) {
      return static_cast<double>(*shadow_value);
    };
    PieceAltitudeCapConfig config;
    config.use_risk_zones = true;
    config.shadow_margin = 1.0;
    caps = buildPieceAltitudeCaps(route, reference, {{{0, 0, 0}, 100}}, config,
                                  height, {}, shadow);
    rough = buildPieceRoughness(route, {0.4, 0.0, 1.0}, {}, gradient);
    check(sameRoute(route, {{0, 0, 1}, {2, 0, 1}}) && reference == std::vector<double>({1, 1}),
          "terrain-aware profile construction leaves geometry and references unchanged");
    route.clear(); reference.clear(); *height_value = 1000.0f;
    *gradient_value = 0.0f; *shadow_value = 1000.0f;
    check(values(caps.ceilings, {19.0}) && values(rough.roughness, {0.5}) &&
              values(rough.time_factors, {0.8}),
          "mutating caller storage and live callback backing data cannot rewrite completed tables");
  }
  check(height_lifetime.expired() && gradient_lifetime.expired() &&
            shadow_lifetime.expired() && values(caps.ceilings, {19.0}) &&
            values(rough.roughness, {0.5}) &&
            values(rough.time_factors, {0.8}) && near(caps.cap_min, 19.0) &&
            near(rough.factor_min, 0.8),
        "owned results outlive all inputs without retaining terrain callbacks");
}
}  // namespace

int main()
{
  coneAndFallback();
  swathCases();
  zoneCases();
  roughnessCases();
  ownership();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
