// Analytic geometry and callback-contract checks, independent of ROS/search.
#include "path_manager/route_postprocessing.h"
#include "path_planner/sdf/distance_field.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
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

bool samePoints(const std::vector<Eigen::Vector3d> &a,
                const std::vector<Eigen::Vector3d> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!((a[i] - b[i]).norm() <= 1e-9)) return false;
  return true;
}

bool sameScalars(const std::vector<double> &a, const std::vector<double> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!near(a[i], b[i])) return false;
  return true;
}

using path_manager::CornerAcceptance;
using path_manager::ElevationQuery;
using path_manager::prepareRouteProfile;
using path_manager::roundRouteCorners;
using path_manager::RouteProfileConfig;
using path_manager::RouteProfileResult;
using path_manager::TaggedRoute;

TaggedRoute corner()
{
  return {{{0.0, 0.0, 4.0}, {20.0, 0.0, 4.0}, {20.0, 20.0, 4.0}}, {3, 9}};
}

TaggedRoute crest()
{
  return {{{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {2.0, 0.0, 9.0},
           {3.0, 0.0, 1.0}, {4.0, 0.0, 1.0}}, {3, 3, 9, 9}};
}

class RecordingDistance : public path_planner::sdf::IDistanceField {
 public:
  bool ready{true};
  float value{1.0f};
  std::function<float(const Eigen::Vector3d &)> sample;
  std::vector<std::string> *events{nullptr};
  mutable std::vector<Eigen::Vector3d> queries;
  mutable size_t readiness_queries{0};

  bool hasData() const override {
    ++readiness_queries;
    if (events) events->push_back("hasData");
    return ready;
  }
  float getDistance(const Eigen::Vector3d &point) const override {
    queries.push_back(point);
    if (events) events->push_back("distance");
    return sample ? sample(point) : value;
  }
  bool getDistanceAndGradient(const Eigen::Vector3d &, float *,
                              Eigen::Vector3d *) const override {
    throw std::logic_error("postprocessing must use only distance samples");
  }
};

void filletCases()
{
  const auto input = corner();
  std::vector<Eigen::Vector3d> samples, original_corners;
  const CornerAcceptance record = [&](const Eigen::Vector3d &point,
                                      const Eigen::Vector3d &original) {
    samples.push_back(point);
    original_corners.push_back(original);
    return true;
  };
  const auto rounded = roundRouteCorners(input, 4.0, record);
  const std::vector<Eigen::Vector3d> expected = {
      {0.0, 0.0, 4.0}, {16.0, 0.0, 4.0}, {17.75, 0.25, 4.0},
      {19.0, 1.0, 4.0}, {19.75, 2.25, 4.0}, {20.0, 4.0, 4.0},
      {20.0, 20.0, 4.0}};
  check(samePoints(rounded.points, expected) &&
            rounded.edge_leg == std::vector<size_t>({3, 3, 3, 9, 9, 9}),
        "junction fillet inserts an exact apex with incoming tags through it and outgoing tags after it");
  check(samePoints(samples, {{17.75, 0.25, 4.0}, {19.0, 1.0, 4.0},
                             {19.75, 2.25, 4.0}}) &&
            samePoints(original_corners, {{20.0, 0.0, 4.0}, {20.0, 0.0, 4.0},
                                         {20.0, 0.0, 4.0}}),
        "acceptance sees internal arc samples in order, with original corner and no endpoint probes");
  check(samePoints(input.points, corner().points) && input.edge_leg == corner().edge_leg,
        "rounding leaves the caller's route and provenance unchanged");

  samples.clear();
  const auto retry = roundRouteCorners(input, 4.0,
      [&](const Eigen::Vector3d &point, const Eigen::Vector3d &) {
        samples.push_back(point);
        return samples.size() != 1;
      });
  check(samePoints(samples, {{17.75, 0.25, 4.0}, {18.875, 0.125, 4.0},
                             {19.5, 0.5, 4.0}, {19.875, 1.125, 4.0}}) &&
            samePoints(retry.points,
                       {{0.0, 0.0, 4.0}, {18.0, 0.0, 4.0}, {18.875, 0.125, 4.0},
                        {19.5, 0.5, 4.0}, {19.875, 1.125, 4.0},
                        {20.0, 2.0, 4.0}, {20.0, 20.0, 4.0}}),
        "a rejected sample stops its candidate immediately and radius-halving retries the next arc");
  size_t rejected_calls = 0;
  const auto rejected = roundRouteCorners(input, 4.0,
      [&](const Eigen::Vector3d &, const Eigen::Vector3d &) {
        ++rejected_calls;
        return false;
      });
  check(rejected_calls == 2 && samePoints(rejected.points, input.points) &&
            rejected.edge_leg == input.edge_leg,
        "exhausted radius candidates retain the kink and original policy tags");

  auto one_leg = input;
  one_leg.edge_leg = {7, 7};
  const auto ordinary = roundRouteCorners(one_leg, 4.0, {});
  check(ordinary.points.size() == 6 &&
            ordinary.edge_leg == std::vector<size_t>({7, 7, 7, 7, 7}),
        "an ordinary corner retains the odd sample count without inventing a leg junction");
  const auto capped = roundRouteCorners(input, 100.0, {});
  check(capped.points.size() == 9 &&
            (capped.points[1] - Eigen::Vector3d(11.0, 0.0, 4.0)).norm() < 1e-9 &&
            (capped.points[4] - Eigen::Vector3d(17.75, 2.25, 4.0)).norm() < 1e-9,
        "oversized radius keeps the existing 45-percent segment trim cap");

  samples.clear();
  const TaggedRoute straight{
      {{0.0, 0.0, 4.0}, {20.0, 0.0, 4.0}, {40.0, 0.0, 4.0}}, {3, 9}};
  const TaggedRoute duplicate{
      {{0.0, 0.0, 4.0}, {0.0, 0.0, 4.0}, {20.0, 0.0, 4.0}}, {3, 9}};
  check(samePoints(roundRouteCorners(straight, 4.0, record).points, straight.points) &&
            samePoints(roundRouteCorners(duplicate, 4.0, record).points, duplicate.points) &&
            samePoints(roundRouteCorners(input, 1.0, record).points, input.points) &&
            samples.empty(),
        "straight, zero-length and radius-at-retry-floor corners make no acceptance calls");

  TaggedRoute malformed = input;
  malformed.edge_leg.clear();
  const auto inactive = roundRouteCorners(malformed, 0.0, record);
  const auto nan_radius = roundRouteCorners(
      malformed, std::numeric_limits<double>::quiet_NaN(), record);
  check(samePoints(inactive.points, input.points) && inactive.edge_leg.empty() &&
            samePoints(nan_radius.points, input.points) && samples.empty(),
        "inactive fillets preserve malformed or absent tags without narrowing prior geometry acceptance");
  bool invalid_tags_rejected = false;
  try {
    roundRouteCorners(malformed, 4.0, record);
  } catch (const std::invalid_argument &) {
    invalid_tags_rejected = true;
  }
  check(invalid_tags_rejected && samples.empty(),
        "active fillet refuses malformed provenance before unchecked indexing or callback use");
  check(roundRouteCorners({{{0.0, 0.0, 4.0}}, {1, 2}}, 4.0, record).points.size() == 1 &&
            samples.empty(),
        "fewer than three points bypass fillet validation and sampling");
}

void profileCases()
{
  const auto input = crest();
  const RouteProfileConfig wide{10.0, 1.0};
  const auto smooth = prepareRouteProfile(input, input.points.front(),
                                         {input.points.back()}, wide);
  const std::vector<Eigen::Vector3d> expected = {
      {0.0, 0.0, 1.0}, {1.0, 0.0, 3.0}, {2.0, 0.0, 2.6},
      {3.0, 0.0, 3.0}, {4.0, 0.0, 1.0}};
  check(samePoints(smooth.filtered_route, expected) && samePoints(smooth.path, expected) &&
            sameScalars(smooth.cap_ref, {1.0, 3.0, 9.0, 3.0, 1.0}) &&
            smooth.raw_max_z == 9.0 && smooth.edge_leg == input.edge_leg,
        "denoise uses the immutable original window and preserves the raw crest in cap reference");
  check(smooth.denoise.applied && smooth.denoise.adjusted == 3 &&
            smooth.denoise.min_before == 1.0 && smooth.denoise.max_before == 9.0 &&
            smooth.denoise.min_after == 1.0 && smooth.denoise.max_after == 3.0 &&
            smooth.input_tags_match && smooth.max_segment_length == 40.0,
        "denoise statistics and segment limit describe the returned profile");
  check(samePoints(input.points, crest().points) && input.edge_leg == crest().edge_leg,
        "profile preparation does not modify caller geometry or provenance");

  const auto raised = prepareRouteProfile(input, input.points.front(),
      {input.points.back()}, wide, [](double, double) { return 10.0f; });
  check(sameScalars(raised.cap_ref, {1.0, 11.0, 11.0, 11.0, 1.0}) &&
            raised.raw_max_z == 9.0 && raised.denoise.max_after == 11.0 &&
            raised.filtered_route.front().z() == 1.0 &&
            raised.filtered_route.back().z() == 1.0,
        "terrain clamps raise committed caps but retain pre-denoise raw maximum and fixed endpoints");

  const TaggedRoute low{
      {{0.0, 0.0, 0.1}, {1.0, 0.0, -0.4}, {2.0, 0.0, -0.4},
       {3.0, 0.0, -0.4}, {4.0, 0.0, 0.1}}, {1, 1, 1, 1}};
  const auto no_terrain = prepareRouteProfile(low, low.points.front(),
                                             {low.points.back()}, {10.0, 0.38});
  const auto water = prepareRouteProfile(low, low.points.front(),
      {low.points.back()}, {10.0, 0.38},
      [](double, double) { return std::numeric_limits<float>::quiet_NaN(); });
  const auto land = prepareRouteProfile(low, low.points.front(),
      {low.points.back()}, {10.0, 0.38}, [](double, double) { return 0.0f; });
  check(near(no_terrain.filtered_route[2].z(), -0.2) &&
            near(water.filtered_route[2].z(), 0.1) &&
            near(land.filtered_route[2].z(), 0.38),
        "absent terrain skips clamping; invalid water uses mission floor; valid zero terrain uses clearance");
  auto multiple = low;
  multiple.points.front().z() = 0.5;
  multiple.points.back().z() = 0.3;
  const auto mission_floor = prepareRouteProfile(multiple, multiple.points.front(),
      {{2.0, 0.0, 0.1}, multiple.points.back()}, {10.0, 0.38},
      [](double, double) { return -1e9f; });
  check(near(mission_floor.filtered_route[2].z(), 0.1),
        "water floor includes every absolute mission waypoint, not only route endpoints");
  auto underwater = low;
  underwater.points.front().z() = -0.2;
  underwater.points.back().z() = -0.1;
  const auto guarded = prepareRouteProfile(underwater, underwater.points.front(),
      {underwater.points.back()}, wide,
      [](double, double) { return -std::numeric_limits<float>::infinity(); });
  check(near(guarded.filtered_route[2].z(), 0.02) &&
            guarded.filtered_route.front().z() == -0.2 &&
            guarded.filtered_route.back().z() == -0.1,
        "water floor retains the absolute guard while leaving even low endpoints fixed");

  auto malformed = input;
  malformed.edge_leg = {42};
  const auto dropped = prepareRouteProfile(malformed, input.points.front(),
                                           {input.points.back()}, wide);
  check(!dropped.input_tags_match && dropped.edge_leg.empty() &&
            samePoints(dropped.path, smooth.path) && dropped.cap_ref == smooth.cap_ref,
        "malformed profile tags drop attribution while preserving accepted geometry and caps");
}

void distanceAndSubdivisionCases()
{
  const auto input = crest();
  const RouteProfileConfig wide{10.0, 1.0};
  std::vector<std::string> events;
  std::vector<Eigen::Vector3d> terrain_queries;
  RecordingDistance distance;
  distance.events = &events;
  distance.sample = [](const Eigen::Vector3d &point) {
    if (point.x() == 1.0) return 1.0f;  // equality must pass
    if (point.x() == 2.0) return std::numeric_limits<float>::quiet_NaN();
    return std::numeric_limits<float>::infinity();
  };
  const auto filtered = prepareRouteProfile(input, input.points.front(),
      {input.points.back()}, wide,
      [&](double x, double y) {
        terrain_queries.emplace_back(x, y, 0.0);
        events.push_back("terrain");
        return -1e9f;
      }, &distance);
  check(events == std::vector<std::string>({"terrain", "hasData", "distance",
                                           "terrain", "hasData", "distance",
                                           "terrain", "hasData", "distance"}) &&
            samePoints(terrain_queries,
                       {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}) &&
            samePoints(distance.queries,
                       {{1.0, 0.0, 3.0}, {2.0, 0.0, 2.6}, {3.0, 0.0, 3.0}}),
        "each interior candidate queries terrain, readiness and distance in original order");
  check(samePoints(filtered.filtered_route,
                   {{0.0, 0.0, 1.0}, {1.0, 0.0, 3.0}, {2.0, 0.0, 9.0},
                    {3.0, 0.0, 1.0}, {4.0, 0.0, 1.0}}) &&
            filtered.denoise.adjusted == 1,
        "clearance equality is accepted while NaN and positive infinity retain raw height");
  RecordingDistance blocked;
  blocked.value = 0.5f;
  const auto refused = prepareRouteProfile(input, input.points.front(),
                                           {input.points.back()}, wide, {}, &blocked);
  check(samePoints(refused.filtered_route, input.points) &&
            refused.denoise.adjusted == 0 && blocked.queries.size() == 3,
        "failed clearance queries preserve every original interior height");
  RecordingDistance absent;
  absent.ready = false;
  const auto unavailable = prepareRouteProfile(input, input.points.front(),
      {input.points.back()}, wide, {}, &absent);
  check(absent.readiness_queries == 3 && absent.queries.empty() &&
            near(unavailable.filtered_route[2].z(), 2.6),
        "an unavailable distance field is checked per interior but never sampled");

  const auto vertical = prepareRouteProfile(
      {{{0.0, 0.0, 0.0}, {0.0, 0.0, 8.0}}, {6}}, {0.0, 0.0, 0.0},
      {{0.0, 0.0, 8.0}}, {0.75, 1.0});
  check(!vertical.denoise.applied && vertical.max_segment_length == 3.0 &&
            samePoints(vertical.path,
                       {{0.0, 0.0, 0.0}, {0.0, 0.0, 8.0 / 3.0},
                        {0.0, 0.0, 16.0 / 3.0}, {0.0, 0.0, 8.0}}) &&
            sameScalars(vertical.cap_ref, {0.0, 8.0 / 3.0, 16.0 / 3.0, 8.0}) &&
            vertical.edge_leg == std::vector<size_t>({6, 6, 6}),
        "subdivision measures full 3D length and interpolates caps with inherited edge tags");
  const auto peak = prepareRouteProfile(
      {{{0.0, 0.0, 0.0}, {4.0, 0.0, 8.0}, {8.0, 0.0, 0.0}}, {3, 9}},
      {0.0, 0.0, 0.0}, {{8.0, 0.0, 0.0}}, {1.0, 1.0});
  check(samePoints(peak.path,
                   {{0.0, 0.0, 0.0}, {2.0, 0.0, 4.0 / 3.0},
                    {4.0, 0.0, 8.0 / 3.0}, {6.0, 0.0, 4.0 / 3.0}, {8.0, 0.0, 0.0}}) &&
            sameScalars(peak.cap_ref, {0.0, 4.0, 8.0, 4.0, 0.0}) &&
            peak.edge_leg == std::vector<size_t>({3, 3, 9, 9}),
        "subdivided caps follow committed raw heights rather than smoothed path heights");
  const auto duplicates = prepareRouteProfile(
      {{{0.0, 0.0, 4.0}, {0.0, 0.0, 4.0}, {8.0, 0.0, 4.0}}, {11, 22}},
      {0.0, 0.0, 4.0}, {{8.0, 0.0, 4.0}}, {0.5, 1.0});
  check(duplicates.path.size() == 5 && duplicates.cap_ref.size() == 5 &&
            duplicates.edge_leg == std::vector<size_t>({22, 22, 22, 22}) &&
            duplicates.path.front().x() == 0.0 && duplicates.path.back().x() == 8.0,
        "duplicate edges are skipped in geometry, caps and tags without shifting remaining attribution");
  const auto degenerate = prepareRouteProfile(
      {{{0.0, 0.0, 4.0}, {0.0, 0.0, 4.0}, {0.0, 0.0, 4.0}}, {11, 22}},
      {0.0, 0.0, 4.0}, {}, wide);
  check(degenerate.path.size() == 1 && degenerate.cap_ref == std::vector<double>({4.0}) &&
            degenerate.edge_leg.empty(),
        "fully duplicate route leaves the one-point product for the caller's acceptance check");
  const auto minimum = prepareRouteProfile(
      {{{0.0, 0.0, 1.0}, {2.5, 0.0, 1.0}}, {8}}, {0.0, 0.0, 1.0},
      {{2.5, 0.0, 1.0}}, {-1.0, 1.0});
  check(minimum.max_segment_length == 1.0 && minimum.path.size() == 4,
        "nonpositive piece length retains the existing one-unit subdivision minimum");
}

void lifetimeAndEmptyCases()
{
  TaggedRoute retained_corners;
  RouteProfileResult retained_profile;
  std::weak_ptr<int> callback_owner;
  std::weak_ptr<float> terrain_owner;
  {
    auto corner_state = std::make_shared<int>(0);
    callback_owner = corner_state;
    const CornerAcceptance accept = [corner_state](const Eigen::Vector3d &,
                                                  const Eigen::Vector3d &) {
      ++*corner_state;
      return true;
    };
    auto source = corner();
    retained_corners = roundRouteCorners(source, 4.0, accept);
    source.points.clear();
    source.edge_leg.clear();

    auto height = std::make_shared<float>(10.0f);
    terrain_owner = height;
    const ElevationQuery terrain = [height](double, double) { return *height; };
    RecordingDistance distance;
    distance.value = 100.0f;
    auto route = crest();
    retained_profile = prepareRouteProfile(route, route.points.front(),
        {route.points.back()}, {10.0, 1.0}, terrain, &distance);
    *height = 100.0f;
    route.points.clear();
    route.edge_leg.clear();
    check(*corner_state == 3 && retained_corners.points.size() == 7 &&
              retained_profile.filtered_route[2].z() == 11.0,
          "outputs own their geometry independently of source mutation and later terrain changes");
  }
  check(callback_owner.expired() && terrain_owner.expired() &&
            retained_corners.edge_leg == std::vector<size_t>({3, 3, 3, 9, 9, 9}) &&
            retained_profile.raw_max_z == 9.0 &&
            sameScalars(retained_profile.cap_ref, {1.0, 11.0, 11.0, 11.0, 1.0}),
        "results survive input/provider destruction without retaining borrowed callbacks or distance field");
  size_t calls = 0;
  RecordingDistance distance;
  const auto empty = prepareRouteProfile({}, Eigen::Vector3d::Zero(), {}, {},
      [&](double, double) { ++calls; return 0.0f; }, &distance);
  check(empty.filtered_route.empty() && empty.path.empty() && empty.cap_ref.empty() &&
            empty.edge_leg.empty() && empty.raw_max_z == -1e9 &&
            !empty.denoise.applied && calls == 0 && distance.readiness_queries == 0,
        "empty route produces empty owned values without querying providers");
}

}  // namespace

int main()
{
  filletCases();
  profileCases();
  distanceAndSubdivisionCases();
  lifetimeAndEmptyCases();
  return failures == 0 ? 0 : 1;
}
