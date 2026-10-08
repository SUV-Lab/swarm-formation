// Analytic geometry fixtures for the reusable mission preparation functions.
// This target compiles with Eigen and the standard library only.
#include "path_manager/mission_geometry.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

bool samePoint(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  return (a - b).norm() < 1e-9;
}

bool samePath(const std::vector<Eigen::Vector3d> &a,
              const std::vector<Eigen::Vector3d> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!samePoint(a[i], b[i])) return false;
  return true;
}

bool sameQueries(const std::vector<Eigen::Vector2d> &a,
                 const std::vector<Eigen::Vector2d> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if ((a[i] - b[i]).norm() > 1e-9) return false;
  return true;
}

using path_manager::computeMissionBounds;
using path_manager::ElevationQuery;
using path_manager::MissionBoundsConfig;
using path_manager::MissionBoundsResult;
using path_manager::ResolvedMissionWaypoints;
using path_manager::resolveMissionWaypoints;
using path_manager::RiskZone;

void waypointCases()
{
  const std::vector<Eigen::Vector3d> authored = {
      {0.0, 0.0, 2.0}, {1.0, 0.0, 0.1}, {2.0, 0.0, 3.0},
      {3.0, 0.0, 4.0}, {4.0, 0.0, -0.2}, {5.0, 0.0, 1.0}};
  const auto no_terrain = resolveMissionWaypoints(authored, false, 0.5);
  check(samePath(no_terrain.waypoints, authored) &&
            no_terrain.altitude_changes.empty(),
        "no terrain preserves authored heights, including values below the AGL floor");

  const std::vector<float> heights = {
      10.0f, -1e9f, std::numeric_limits<float>::quiet_NaN(), -2.0f,
      -std::numeric_limits<float>::infinity(), 0.0f};
  std::vector<Eigen::Vector2d> queries;
  const ElevationQuery terrain = [&](double x, double y) {
    queries.emplace_back(x, y);
    return heights[static_cast<size_t>(x)];
  };
  const auto resolved = resolveMissionWaypoints(authored, false, 0.5, terrain);
  const std::vector<Eigen::Vector3d> expected = {
      {0.0, 0.0, 12.0}, {1.0, 0.0, 0.5}, {2.0, 0.0, 3.0},
      {3.0, 0.0, 2.0}, {4.0, 0.0, 0.5}, {5.0, 0.0, 1.0}};
  check(samePath(resolved.waypoints, expected),
        "AGL conversion preserves negative terrain and uses zero for sentinel/NaN/negative infinity");
  check(sameQueries(queries, {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0},
                              {3.0, 0.0}, {4.0, 0.0}, {5.0, 0.0}}),
        "waypoint terrain queries retain authored order and coordinates");
  const auto &changes = resolved.altitude_changes;
  const bool four = changes.size() == 4;
  check(four && changes[0].index == 0 && changes[1].index == 1 &&
            changes[2].index == 3 && changes[3].index == 4,
        "altitude diagnostics include only changed waypoints in original index order");
  if (four) {
    check(near(changes[0].original_z, 2.0) && near(changes[0].terrain_base, 10.0) &&
              near(changes[0].agl, 2.0) && near(changes[0].absolute_z, 12.0) &&
              near(changes[1].terrain_base, 0.0) && near(changes[1].agl, 0.5) &&
              near(changes[2].terrain_base, -2.0) &&
              near(changes[3].original_z, -0.2),
          "altitude diagnostics own the original, terrain, AGL and absolute values");
  }
  check(samePath(authored, {{0.0, 0.0, 2.0}, {1.0, 0.0, 0.1},
                            {2.0, 0.0, 3.0}, {3.0, 0.0, 4.0},
                            {4.0, 0.0, -0.2}, {5.0, 0.0, 1.0}}),
        "resolving waypoints leaves the caller's command values unchanged");

  queries.clear();
  const auto junction = resolveMissionWaypoints(
      {{0.0, 0.0, 2.0}, {1.0, 0.0, 0.1}, {5.0, 0.0, -0.25}}, true, 0.5,
      terrain);
  check(samePath(junction.waypoints,
                 {{0.0, 0.0, 12.0}, {1.0, 0.0, 0.5}, {5.0, 0.0, -0.25}}) &&
            queries.size() == 2 && junction.altitude_changes.size() == 2,
        "only the final junction goal bypasses terrain queries and AGL floor");

  const auto tiny = resolveMissionWaypoints(
      {{0.0, 0.0, 1.0}}, false, 0.5,
      [](double, double) { return 1e-10f; });
  check(tiny.waypoints.front().z() > 1.0 && tiny.altitude_changes.empty(),
        "sub-threshold altitude changes are applied without a diagnostic row");
  queries.clear();
  const auto empty = resolveMissionWaypoints({}, false, 0.5, terrain);
  check(empty.waypoints.empty() && empty.altitude_changes.empty() && queries.empty(),
        "empty waypoint input performs no terrain lookup");
}

void boundsCases()
{
  const Eigen::Vector3d start(0.0, 0.0, 3.0);
  const MissionBoundsConfig config;
  const auto basic = computeMissionBounds(
      start, {{4.0, 0.0, 100.0}, {4.0, 3.0, -2.0}}, {}, config);
  check(samePoint(basic.lower, {-10.0, -10.0, -7.0}) &&
            samePoint(basic.upper, {14.0, 13.0, 23.0}) &&
            basic.max_terrain_z == 0.0 && basic.bound_margin_xy == 10.0,
        "mission points expand XY/lower Z while high waypoint Z leaves the existing start-only ceiling");
  check(basic.zone_in.empty() && basic.included_zone_count == 0 &&
            basic.max_included_reach == 0.0 && basic.max_zone_top == 0.0,
        "terrain-free, zone-free bounds have empty diagnostics");

  const std::vector<Eigen::Vector3d> waypoints = {{10.0, 0.0, 3.0}};
  // Each footprint touches the next. Reverse order requires repeated passes
  // before the initial mission box can reach the farthest included zone.
  const RiskZone a{{25.0, 0.0, 2.0}, 5.0, 0.5};
  const RiskZone b{{44.0, 0.0, 4.0}, 5.0, 0.5};
  const RiskZone c{{63.0, 0.0, 100.0}, 5.0, 0.5};
  const RiskZone distant{{10000.0, 10000.0, 5000.0}, 1000.0, 0.5};
  const auto reverse = computeMissionBounds(start, waypoints, {c, b, a, distant}, config);
  const auto forward = computeMissionBounds(start, waypoints, {a, b, c, distant}, config);
  check(reverse.zone_in == std::vector<char>({1, 1, 1, 0}) &&
            reverse.included_zone_count == 3 && reverse.max_included_reach == 5.0 &&
            samePoint(reverse.lower, {-10.0, -10.0, -2.0}) &&
            samePoint(reverse.upper, {73.0, 10.0, 23.0}),
        "transitive zone inclusion reaches the reverse-ordered chain and excludes distant large zones");
  check(samePoint(reverse.lower, forward.lower) &&
            samePoint(reverse.upper, forward.upper) &&
            reverse.zone_in == forward.zone_in &&
            near(reverse.max_zone_top, 101.75) && reverse.max_zone_top > reverse.upper.z(),
        "zone order does not change the fixpoint and zone tops remain diagnostic only");
  const auto custom = computeMissionBounds(start, waypoints, {a}, {7.0, 2.0});
  check(custom.upper.z() == 10.0 && custom.max_zone_top == 12.0 &&
            custom.included_zone_count == 1,
        "explicit headroom and vertical ratio independently set ceiling and zone-top diagnostic");

  const auto tangent = computeMissionBounds(
      start, waypoints, {{{30.0, 0.0, 0.0}, 5.0, 0.5}}, config);
  const auto just_outside = computeMissionBounds(
      start, waypoints, {{{30.0001, 0.0, 0.0}, 5.0, 0.5}}, config);
  check(tangent.included_zone_count == 1 && just_outside.included_zone_count == 0,
        "a zone touching the mission footprint is included at the exact boundary");
  const auto corner_in = computeMissionBounds(
      start, waypoints, {{{26.0, 16.0, 0.0}, 5.0, 0.5}}, config);
  const auto corner_out = computeMissionBounds(
      start, waypoints, {{{28.0, 18.0, 0.0}, 5.0, 0.5}}, config);
  check(corner_in.included_zone_count == 1 && corner_out.included_zone_count == 0,
        "zone contact uses radial distance at a mission-box corner");

  const auto late_margin = computeMissionBounds(
      start, waypoints,
      {{{20.0, 0.0, 1.0}, 20.0, 0.5}, {{-30.0, 0.0, 999.0}, 1.0, 0.5}}, config);
  check(late_margin.zone_in == std::vector<char>({1, 0}) &&
            late_margin.bound_margin_xy == 25.0 &&
            samePoint(late_margin.lower, {-25.0, -25.0, -2.0}) &&
            samePoint(late_margin.upper, {45.0, 25.0, 23.0}) &&
            near(late_margin.max_zone_top, 8.0),
        "expanded waypoint berth does not trigger another zone inclusion pass");

  const Eigen::Vector3d shift(120.0, -70.0, 0.0);
  auto shifted_zones = std::vector<RiskZone>{c, b, a, distant};
  for (auto &zone : shifted_zones) zone.center += shift;
  const auto translated = computeMissionBounds(
      start + shift, {waypoints.front() + shift}, shifted_zones, config);
  check(samePoint(translated.lower, reverse.lower + shift) &&
            samePoint(translated.upper, reverse.upper + shift) &&
            translated.zone_in == reverse.zone_in &&
            translated.max_zone_top == reverse.max_zone_top &&
            translated.bound_margin_xy == reverse.bound_margin_xy,
        "XY translation preserves zone membership and translates only the XY bounds");
}

void terrainQueriesAndLifetime()
{
  const Eigen::Vector3d start(0.0, 0.0, 3.0);
  std::vector<Eigen::Vector2d> queries;
  const ElevationQuery ridge = [&](double x, double y) {
    queries.emplace_back(x, y);
    return x == 2.0 && y == 10.0 ? 37.0f : 0.0f;
  };
  const auto sampled = computeMissionBounds(start, {{4.0, 0.0, 3.0}}, {}, {}, ridge);
  const std::vector<Eigen::Vector2d> expected = {
      {-10.0, 0.0}, {0.0, -10.0}, {0.0, 0.0}, {0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0},
      {-8.0, 0.0}, {2.0, -10.0}, {2.0, 0.0}, {2.0, 0.0}, {12.0, 0.0}, {2.0, 10.0},
      {-6.0, 0.0}, {4.0, -10.0}, {4.0, 0.0}, {4.0, 0.0}, {14.0, 0.0}, {4.0, 10.0}};
  check(sameQueries(queries, expected),
        "terrain sampling retains exact lateral/axis order and duplicate center queries");
  check(sampled.max_terrain_z == 37.0 && sampled.upper.z() == 57.0,
        "a lateral ridge sample sets the ceiling using configured headroom");

  queries.clear();
  computeMissionBounds(start, {{4.0, 0.0, 3.0}, {4.0, 4.0, 3.0}}, {}, {}, ridge);
  check(queries.size() == 36 &&
            (queries[18] - Eigen::Vector2d(-6.0, 0.0)).norm() == 0.0 &&
            (queries[19] - Eigen::Vector2d(4.0, -10.0)).norm() == 0.0 &&
            (queries.back() - Eigen::Vector2d(4.0, 14.0)).norm() == 0.0,
        "multi-leg terrain sampling repeats the shared endpoint before advancing");
  queries.clear();
  computeMissionBounds(start, {{0.0, 0.0, 500.0}}, {}, {}, ridge);
  check(queries.size() == 18,
        "zero XY-length leg still performs the existing three sample stations");
  queries.clear();
  const auto empty = computeMissionBounds(start, {}, {}, {}, ridge);
  check(queries.empty() && samePoint(empty.lower, {-10.0, -10.0, -2.0}) &&
            samePoint(empty.upper, {10.0, 10.0, 23.0}),
        "bounds with only a start perform no terrain queries");

  size_t invalid_count = 0;
  const std::vector<float> invalid = {
      std::numeric_limits<float>::quiet_NaN(),
      -std::numeric_limits<float>::infinity(), -1e10f, -2e9f};
  const auto off_dem = computeMissionBounds(
      {0.0, 0.0, -4.0}, {{4.0, 0.0, -4.0}}, {}, {},
      [&](double, double) { return invalid[invalid_count++ % invalid.size()]; });
  check(invalid_count == 18 && off_dem.max_terrain_z == 0.0 &&
            off_dem.upper.z() == 20.0,
        "invalid and negative terrain samples retain the existing zero terrain maximum");

  ResolvedMissionWaypoints retained_waypoints;
  MissionBoundsResult retained_bounds;
  std::weak_ptr<float> provider;
  int calls = 0;
  {
    auto height = std::make_shared<float>(9.0f);
    provider = height;
    const ElevationQuery terrain = [height, &calls](double, double) {
      ++calls;
      return *height;
    };
    retained_waypoints = resolveMissionWaypoints({{4.0, 0.0, 1.0}}, false, 0.5, terrain);
    retained_bounds = computeMissionBounds(start, retained_waypoints.waypoints,
                                           {}, {}, terrain);
    *height = 50.0f;
    check(calls == 19 && retained_waypoints.waypoints.front().z() == 10.0 &&
              retained_bounds.max_terrain_z == 9.0 && retained_bounds.upper.z() == 29.0,
          "computed outputs keep their values after the terrain provider changes");
  }
  check(provider.expired() && calls == 19 &&
            retained_waypoints.waypoints.front().z() == 10.0 &&
            retained_waypoints.altitude_changes.front().terrain_base == 9.0 &&
            retained_bounds.upper.z() == 29.0,
        "results retain no terrain callback and survive provider destruction");
}

}  // namespace

int main()
{
  waypointCases();
  boundsCases();
  terrainQueriesAndLifetime();
  return failures == 0 ? 0 : 1;
}
