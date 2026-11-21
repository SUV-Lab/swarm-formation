#include "path_manager/probability_heatmap.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace rm = risk_model;
namespace viz = path_manager::visualization;
namespace {
int checks = 0, failures = 0;
void check(bool value, const std::string& description) {
  ++checks;
  if (!value) { ++failures; std::cerr << "[FAIL] " << description << '\n'; }
}
void near(double actual, double expected, double tolerance, const std::string& description) {
  check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, description);
}
template<class Function> void rejects(Function function, const std::string& description) {
  bool rejected = false;
  try { function(); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, description);
}

class TestTerrain : public rm::TerrainSnapshot {
 public:
  double ground{-50};
  bool hole_positive_x{false}, fail_positive_x{false}, assume_visibility{false};
  std::uint64_t revision() const override { return 9; }
  std::optional<double> elevation(double x, double) const override {
    if (fail_positive_x && x > 0) throw std::runtime_error("synthetic provider failure");
    if (hole_positive_x && x > 0) return std::nullopt;
    return ground;
  }
  rm::VisibilitySample sample(const rm::Source&, rm::Vec3) const override {
    rm::VisibilitySample result;
    result.state = assume_visibility ? rm::VisibilityState::MissingDem
                                    : rm::VisibilityState::NoOcclusion;
    result.ground_known = true;
    result.ground_m = ground;
    return result;
  }
};

rm::EnvironmentSnapshot environment() {
  rm::EnvironmentSnapshot result;
  result.revision = 12;
  result.terrain_revision = 9;
  result.scale = {2, 5};
  result.settings.rcs = {.3172, .1784, 1.003, 1, .5, 2};
  result.settings.smooth = {2, 2, .02, .03, 2, .5, true};
  result.terrain = std::make_shared<TestTerrain>();
  rm::Source first;
  first.id = "teaching-source-a";
  first.center_m = {0, 0, -50};
  first.sensor = {1.2, rm::calibratedC2(100, 1)};
  first.damage = {100, 80, 0, 0, .8, 0, 2};
  result.sources = {first};
  return result;
}

int rows(const grid_map_msgs::msg::GridMap& grid) {
  return static_cast<int>(grid.data.front().layout.dim.at(1).size);
}
int columns(const grid_map_msgs::msg::GridMap& grid) {
  return static_cast<int>(grid.data.front().layout.dim.at(0).size);
}
std::size_t index(const grid_map_msgs::msg::GridMap& grid, int row, int column) {
  return static_cast<std::size_t>(column) * rows(grid) + row;
}
rm::Vec3 physicalCell(const grid_map_msgs::msg::GridMap& grid,
                      const rm::EnvironmentSnapshot& env, int row, int column) {
  const double x = grid.info.pose.position.x + .5 * grid.info.length_x -
                   (row + .5) * grid.info.resolution;
  const double y = grid.info.pose.position.y + .5 * grid.info.length_y -
                   (column + .5) * grid.info.resolution;
  return {x * env.scale.xy_m, y * env.scale.xy_m,
          grid.data.at(3).data.at(index(grid, row, column))};
}
std::uint32_t rgb(float packed) {
  std::uint32_t result;
  std::memcpy(&result, &packed, sizeof(result));
  return result;
}
bool allNaN(const grid_map_msgs::msg::GridMap& grid, std::size_t cell) {
  return std::all_of(grid.data.begin(), grid.data.end(),
                     [cell](const auto& layer) { return std::isnan(layer.data.at(cell)); });
}
bool renderMissing(const grid_map_msgs::msg::GridMap& grid, std::size_t cell) {
  return std::isnan(grid.data.at(0).data.at(cell)) &&
         std::isnan(grid.data.at(1).data.at(cell));
}
bool horizontalSupport(const rm::EnvironmentSnapshot& env, rm::Vec3 point) {
  return std::any_of(env.sources.begin(), env.sources.end(), [&](const auto& source) {
    return std::hypot(point.x - source.center_m.x, point.y - source.center_m.y) <=
           source.damage.r_max_m;
  });
}

void verifyCoreAndLayout() {
  auto env = environment();
  auto second = env.sources.front();
  second.id = "teaching-source-b";
  second.center_m.x = 20;
  env.sources.push_back(second);
  viz::ProbabilityHeatmapSettings settings;
  settings.height_m = 10;
  settings.max_dim = 7;
  settings.drape_offset_frame = 1;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 41;
  const auto result = viz::makeProbabilityHeatmap(&env, settings, stamp);
  const auto& grid = result.grid;
  check(grid.header.frame_id == "map" && grid.header.stamp.sec == 41,
        "caller frame and timestamp retained");
  check(grid.layers == std::vector<std::string>({"elevation", "color", "probability",
        "evaluation_height_m", "validity"}), "raw probability, query altitude, validity layers available");
  check(rows(grid) <= 7 && columns(grid) <= 7, "both dimensions respect display budget");
  near(grid.info.length_x * env.scale.xy_m, 220, 1e-10,
       "extent covers full physical damage radius and second source");
  bool all_match = true, has_overlap = false, raw_known = true, render_matches_support = true;
  int hidden_cells = 0, rendered_cells = 0;
  for (int row = 0; row < rows(grid); ++row)
    for (int column = 0; column < columns(grid); ++column) {
      const auto i = index(grid, row, column);
      const auto point = physicalCell(grid, env, row, column);
      const auto direct = rm::evaluateCombined(env.sources, point, {}, env.settings,
          rm::FormulaMode::Exact, rm::RcsMode::Reference, env.terrain.get());
      all_match &= std::abs(grid.data[2].data[i] - direct.risk.value) < 3e-8;
      raw_known &= grid.data[4].data[i] == 1;
      has_overlap |= direct.sources[0].risk.value > 0 && direct.sources[1].risk.value > 0 &&
          direct.risk.value > std::max(direct.sources[0].risk.value, direct.sources[1].risk.value);
      raw_known &= grid.data[3].data[i] == -40;
      if (horizontalSupport(env, point)) {
        ++rendered_cells;
        render_matches_support &= grid.data[0].data[i] == -9 && std::isfinite(grid.data[1].data[i]);
      } else {
        ++hidden_cells;
        render_matches_support &= renderMissing(grid, i);
      }
    }
  check(all_match, "every mirrored column-major cell equals shared Exact/Reference core");
  check(has_overlap, "overlapping cells contain OR rather than max or separate source colors");
  check(raw_known, "negative DEM and anisotropic XY/Z retain every raw sample altitude and validity");
  check(render_matches_support && hidden_cells > 0 && rendered_cells > 0,
        "only horizontal damage footprints render with correct anisotropic Z; rectangular corners are transparent");
  const auto high_x = physicalCell(grid, env, 0, 0);
  const auto lower_x = physicalCell(grid, env, 1, 0);
  const auto lower_y = physicalCell(grid, env, 0, 1);
  check(high_x.x > lower_x.x && high_x.y > lower_y.y,
        "row X and column Y follow existing mirrored map convention");
  check(result.description.find("Exact / Reference") != std::string::npos &&
        result.description.find("local T, not cumulative") != std::string::npos &&
        result.description.find("environment revision=12") != std::string::npos &&
        result.description.find("sigma_ref=1 m^2") != std::string::npos &&
        result.description.find("10 m AGL") != std::string::npos,
        "description identifies exact display meaning and environment");
  const auto reference = grid.data[2].data;
  settings.drape_offset_frame = 20;
  const auto lifted = viz::makeProbabilityHeatmap(&env, settings, stamp);
  check(lifted.grid.data[2].data == reference && lifted.grid.data[3].data == grid.data[3].data,
        "render lift never changes probability or evaluation altitude");
}

void verifyMissingAndAssumed() {
  auto env = environment();
  auto terrain = std::make_shared<TestTerrain>();
  terrain->hole_positive_x = true;
  env.terrain = terrain;
  viz::ProbabilityHeatmapSettings settings;
  settings.height_m = 10;
  settings.max_dim = 4;
  auto output = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  bool holes_omitted = true, visible_retained = true;
  for (int row = 0; row < rows(output); ++row)
    for (int column = 0; column < columns(output); ++column) {
      const auto i = index(output, row, column);
      if (row < rows(output) / 2) holes_omitted &= allNaN(output, i);
      else visible_retained &= std::isfinite(output.data[2].data[i]);
    }
  check(holes_omitted && visible_retained, "unknown AGL elevation is omitted, known half retains raw data");

  settings.height_mode = "msl";
  settings.height_m = -40;
  output = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  const auto missing_ground_inside = index(output, 0, 1);  // x=75 m, y=25 m; inside R=100 m.
  check(std::isfinite(output.data[2].data[missing_ground_inside]) &&
        output.data[3].data[missing_ground_inside] == -40 &&
        output.data[4].data[missing_ground_inside] == 0 &&
        rgb(output.data[1].data[missing_ground_inside]) == 0x919191,
        "MSL with unknown terrain retains raw value but neutral grey validity zero");
  check(std::isfinite(output.data[2].data[0]) && output.data[3].data[0] == -40 &&
        output.data[4].data[0] == 0 && renderMissing(output, 0),
        "MSL assumed cell outside footprint retains raw data without painting grey rectangle");
  terrain->hole_positive_x = false;
  terrain->assume_visibility = true;
  output = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  const auto middle = index(output, 1, 1);
  check(output.data[2].data[middle] > 0 && output.data[4].data[middle] == 0 &&
        rgb(output.data[1].data[middle]) == 0x919191,
        "core visibility assumption is not painted as a known probability");

  terrain->assume_visibility = false;
  settings.height_m = -51;
  const auto below = viz::makeProbabilityHeatmap(&env, settings, {});
  check(std::all_of(below.grid.data[2].data.begin(), below.grid.data[2].data.end(),
                   [](float value) { return std::isnan(value); }) &&
        below.description.find("below terrain=16") != std::string::npos,
        "below-ground MSL slice is unavailable rather than zero-risk");
  env.terrain.reset();
  settings.height_mode = "agl";
  settings.height_m = 10;
  const auto no_dem = viz::makeProbabilityHeatmap(&env, settings, {});
  check(std::all_of(no_dem.grid.data[2].data.begin(), no_dem.grid.data[2].data.end(),
                   [](float value) { return std::isnan(value); }) &&
        no_dem.description.find("N/A: no drawable samples") != std::string::npos,
        "AGL without any DEM clears every sample and describes N/A");
  env.terrain = terrain;
  terrain->fail_positive_x = true;
  const auto failed = viz::makeProbabilityHeatmap(&env, settings, {});
  check(failed.description.find("query errors=") != std::string::npos &&
        allNaN(failed.grid, 0), "provider failure produces omitted samples and diagnostic count");
}

void verifyPaletteAndSupport() {
  auto env = environment();
  env.scale = {1, 1};
  env.sources.front().damage.p0 = .2;
  viz::ProbabilityHeatmapSettings settings;
  settings.height_mode = "msl";
  settings.height_m = -40;
  settings.max_dim = 4;
  const auto first = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  auto second = env.sources.front();
  second.id = "distant-higher-probability";
  second.center_m.x = 1000;
  second.damage.p0 = 1;
  env.sources.push_back(second);
  settings.max_dim = 24;
  const auto with_high = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  const auto old_i = index(first, 2, 1);       // x=-25, y=25
  const auto new_i = index(with_high, 22, 1);  // same physical cell
  near(first.data[2].data[old_i], with_high.data[2].data[new_i], 1e-7,
       "distant nonoverlapping source leaves existing probability unchanged");
  check(rgb(first.data[1].data[old_i]) == rgb(with_high.data[1].data[new_i]),
        "adding a higher maximum and changing grid size never renormalizes existing color");
  check(*std::max_element(with_high.data[2].data.begin(), with_high.data[2].data.end()) >
        *std::max_element(first.data[2].data.begin(), first.data[2].data.end()),
        "normalization fixture really introduces a larger displayed maximum");

  env = environment();
  env.scale = {1, 1};
  env.sources.front().center_m.z = 0;
  env.sources.front().damage.p0 = 1;
  env.sources.front().damage.rho_min = .5;
  settings.height_m = 0;
  settings.max_dim = 3;
  const auto certain = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  const auto center = index(certain, 1, 1);
  check(certain.data[2].data[center] == 1 && rgb(certain.data[1].data[center]) == 0xf21818,
        "probability one remains exact and uses fixed 100-percent red");
  env.sources.front().damage.p0 = .5;
  const auto midpoint = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  check(midpoint.data[2].data[center] == .5 && rgb(midpoint.data[1].data[center]) == 0xf2f218,
        "probability midpoint uses fixed 50-percent yellow");
  env.sources.front().damage.p0 = .25;
  const auto quarter = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  check(quarter.data[2].data[center] == .25 && rgb(quarter.data[1].data[center]) == 0x9ddf41,
        "25-percent probability uses the fixed halfway green-to-yellow color despite being this map's maximum");
  env.sources.front().damage.p0 = .75;
  const auto three_quarters = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  check(three_quarters.data[2].data[center] == .75 &&
        rgb(three_quarters.data[1].data[center]) == 0xf28518,
        "75-percent probability uses the fixed halfway yellow-to-red color without per-map normalization");
  env.sources.front().damage.p0 = 0;
  const auto zero = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  check(zero.data[2].data[center] == 0 && rgb(zero.data[1].data[center]) == 0x48cc6a &&
        zero.data[4].data[center] == 1 && std::isfinite(zero.data[0].data[center]),
        "known zero stays visible green, distinct from unavailable and assumed cells");

  env.sources.front().damage.p0 = .8;
  settings.height_m = 100;  // Above this source's 80 m vertical damage extent.
  settings.max_dim = 4;
  const auto above_damage = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  const auto inside_disk = index(above_damage, 1, 1);
  check(above_damage.data[2].data[inside_disk] == 0 &&
        above_damage.data[3].data[inside_disk] == 100 &&
        above_damage.data[4].data[inside_disk] == 1 &&
        std::isfinite(above_damage.data[0].data[inside_disk]) &&
        rgb(above_damage.data[1].data[inside_disk]) == 0x48cc6a,
        "horizontal footprint remains visible green when evaluation altitude is outside 3D damage support");
  check(above_damage.data[2].data[0] == 0 && above_damage.data[3].data[0] == 100 &&
        above_damage.data[4].data[0] == 1 && renderMissing(above_damage, 0),
        "outside horizontal footprint only render layers are NaN even at zero-risk altitude");

  auto distant = env.sources.front();
  distant.id = "thin-axis-grid";
  distant.center_m.x = 100000;
  env.sources.push_back(distant);
  settings.max_dim = 8;
  const auto thin = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  check(rows(thin) == 8 && columns(thin) == 2 && thin.data.front().data.size() == 16,
        "thin physical extent retains two cells on the short axis for mesh updates");
}

void verifySeparatedUnequalFootprints() {
  auto env = environment();  // Deliberately nonunit and anisotropic: XY=2 m/u, Z=5 m/u.
  auto second = env.sources.front();
  second.id = "separated-smaller-source";
  second.center_m.x = 400;
  second.damage.r_max_m = 60;
  env.sources.push_back(second);
  viz::ProbabilityHeatmapSettings settings;
  settings.height_m = 10;
  settings.max_dim = 28;
  const auto grid = viz::makeProbabilityHeatmap(&env, settings, {}).grid;
  near(grid.info.length_x * env.scale.xy_m, 560, 1e-9,
       "unequal source radii set physical display bounds in SI coordinates");
  near(grid.info.resolution * env.scale.xy_m, 20, 1e-9,
       "nonunit frame scale does not double footprint radius");
  check(rows(grid) == 28 && columns(grid) == 10, "separated fixture has expected sample layout");

  const auto gap = index(grid, 10, 4);             // (250, 10) m: between disks.
  const auto corner = index(grid, 0, 0);           // (450, 90) m: bounding corner.
  const auto outside_small = index(grid, 0, 1);    // (450, 70) m: >60 m but <100 m from source B.
  const auto inside_small = index(grid, 0, 4);     // (450, 10) m: inside B.
  const auto inside_large = index(grid, 18, 4);    // (90, 10) m: inside A's larger radius.
  for (const auto cell : {gap, corner, outside_small}) {
    check(renderMissing(grid, cell) && grid.data[2].data[cell] == 0 &&
          grid.data[3].data[cell] == -40 && grid.data[4].data[cell] == 1,
          "gap/corner/outside smaller radius keeps valid zero sample but no render mesh or color");
  }
  for (const auto cell : {inside_small, inside_large}) {
    check(std::isfinite(grid.data[0].data[cell]) && std::isfinite(grid.data[1].data[cell]) &&
          grid.data[2].data[cell] > 0,
          "each separated source renders using its own physical damage radius");
  }
  bool all_raw_match = true;
  for (int row=0; row<rows(grid); ++row)
    for (int column=0; column<columns(grid); ++column) {
      const auto i=index(grid,row,column);
      const auto direct=rm::evaluateCombined(env.sources,physicalCell(grid,env,row,column),{},
          env.settings,rm::FormulaMode::Exact,rm::RcsMode::Reference,env.terrain.get());
      all_raw_match &= std::abs(grid.data[2].data[i]-direct.risk.value)<3e-8;
    }
  check(all_raw_match, "footprint clipping leaves shared core probability intact for entire rectangular grid");
}

void verifyClearAndValidation() {
  viz::ProbabilityHeatmapSettings settings;
  auto missing = viz::makeProbabilityHeatmap(nullptr, settings, {});
  check(rows(missing.grid) == 2 && columns(missing.grid) == 2 &&
        missing.grid.info.length_x == 2 && missing.grid.info.length_y == 2 &&
        allNaN(missing.grid, 0) && allNaN(missing.grid, 1) &&
        allNaN(missing.grid, 2) && allNaN(missing.grid, 3) &&
        missing.description.find("environment unavailable") != std::string::npos,
        "missing snapshot publishes an explicit NaN clearing stub");
  auto env = environment();
  env.sources.clear();
  auto empty = viz::makeProbabilityHeatmap(&env, settings, {});
  check(rows(empty.grid) == 2 && columns(empty.grid) == 2 &&
        allNaN(empty.grid, 0) && allNaN(empty.grid, 1) &&
        allNaN(empty.grid, 2) && allNaN(empty.grid, 3) &&
        empty.description.find("Empty accepted environment: model T=0") !=
        std::string::npos, "accepted empty environment distinguished from missing environment");
  env = environment();
  for (int invalid_dim : {0, -1, 1, 513}) {
    settings.max_dim = invalid_dim;
    rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); },
            "out-of-budget dimension rejected before allocation");
  }
  settings.max_dim = 8;
  settings.height_mode = "unrecognized";
  rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); }, "unknown height mode rejected");
  settings.height_mode = "agl";
  settings.height_m = -1;
  rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); }, "negative AGL height rejected");
  settings.height_m = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); }, "nonfinite altitude rejected");
  settings.height_m = 10;
  settings.drape_offset_frame = -1;
  rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); }, "negative drape offset rejected");
  settings.drape_offset_frame = 1;
  env.scale.z_m = 0;
  rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); }, "invalid snapshot scale rejected");
  env = environment();
  env.sources.push_back(env.sources.front());
  rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); }, "duplicate snapshot IDs rejected");
  env = environment();
  env.sources.front().damage.r_max_m = std::numeric_limits<double>::infinity();
  rejects([&] { viz::makeProbabilityHeatmap(&env, settings, {}); }, "invalid physical source radius rejected");
}
}  // namespace

int main() {
  try {
    verifyCoreAndLayout();
    verifyMissingAndAssumed();
    verifyPaletteAndSupport();
    verifySeparatedUnequalFootprints();
    verifyClearAndValidation();
  } catch (const std::exception& error) {
    check(false, std::string("unexpected exception: ") + error.what());
  }
  std::cout << "Probability heatmap checks: " << checks - failures << '/' << checks << '\n';
  return failures == 0 ? 0 : 1;
}
