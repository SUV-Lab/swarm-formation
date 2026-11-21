#include "path_manager/probability_heatmap.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <new>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace path_manager::visualization {
namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

std_msgs::msg::Float32MultiArray layer(int rows, int columns) {
  std_msgs::msg::Float32MultiArray result;
  result.layout.dim.resize(2);
  result.layout.dim[0].label = "column_index";
  result.layout.dim[0].size = static_cast<std::uint32_t>(columns);
  result.layout.dim[0].stride = static_cast<std::uint32_t>(columns * rows);
  result.layout.dim[1].label = "row_index";
  result.layout.dim[1].size = static_cast<std::uint32_t>(rows);
  result.layout.dim[1].stride = static_cast<std::uint32_t>(rows);
  result.data.assign(static_cast<std::size_t>(rows) * columns,
                    std::numeric_limits<float>::quiet_NaN());
  return result;
}

void initializeGrid(grid_map_msgs::msg::GridMap& grid, int rows, int columns) {
  grid.layers = {"elevation", "color", "probability", "evaluation_height_m", "validity"};
  grid.basic_layers = {"elevation", "probability"};
  grid.data.clear();
  for (std::size_t i = 0; i < grid.layers.size(); ++i) grid.data.push_back(layer(rows, columns));
  grid.outer_start_index = 0;
  grid.inner_start_index = 0;
}

float packedColor(const std::array<int, 3>& rgb) {
  const std::uint32_t bits = (static_cast<std::uint32_t>(rgb[0]) << 16) |
                             (static_cast<std::uint32_t>(rgb[1]) << 8) |
                              static_cast<std::uint32_t>(rgb[2]);
  float result;
  static_assert(sizeof(result) == sizeof(bits), "packed GridMap color needs 32-bit float");
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

float probabilityColor(double probability) {
  // Reuse the visibility heatmap's green/yellow/red hues, while retaining
  // fixed probability anchors rather than its visibility-height categories.
  constexpr std::array<int, 3> low{{72, 204, 106}};
  constexpr std::array<int, 3> middle{{242, 242, 24}};
  constexpr std::array<int, 3> high{{242, 24, 24}};
  const auto& first = probability <= .5 ? low : middle;
  const auto& second = probability <= .5 ? middle : high;
  const double weight = probability <= .5 ? probability * 2 : (probability - .5) * 2;
  std::array<int, 3> rgb;
  for (std::size_t i = 0; i < rgb.size(); ++i)
    rgb[i] = static_cast<int>(std::lround(first[i] + weight * (second[i] - first[i])));
  return packedColor(rgb);
}

std::optional<double> elevation(const risk_model::EnvironmentSnapshot& environment,
                                double x_m, double y_m) {
  if (!environment.terrain) return std::nullopt;
  const auto value = environment.terrain->elevation(x_m, y_m);
  return value && std::isfinite(*value) ? value : std::nullopt;
}

bool representableFloat(double value) {
  return std::isfinite(value) &&
         std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

std::string configurationDescription(const risk_model::EnvironmentSnapshot* environment,
                                     const ProbabilityHeatmapSettings& settings) {
  std::ostringstream out;
  out << std::setprecision(9)
      << "Reference probability map | Exact / Reference | local T, not cumulative\n"
      << "Sample height: " << settings.height_m << " m "
      << (settings.height_mode == "agl" ? "AGL" : "MSL")
      << "; rendered as a terrain drape (render offset "
      << settings.drape_offset_frame << " frame units)\n";
  if (environment) {
    out << "sigma_ref=" << environment->settings.rcs.sigma_ref_m2 << " m^2"
        << "; environment revision=" << environment->revision
        << "; terrain revision=" << environment->terrain_revision << '\n';
  }
  out << "Fixed linear colors: 0% green, 50% yellow, 100% red; grey=assumed visibility; "
      << "transparent=outside source footprints or unavailable. "
      << "Colors are drawn within the sources' horizontal damage radii. "
      << "Zero is a model value, not a safety guarantee.";
  return out.str();
}

}  // namespace

ProbabilityHeatmap makeProbabilityHeatmap(
    const risk_model::EnvironmentSnapshot* environment,
    const ProbabilityHeatmapSettings& settings,
    const builtin_interfaces::msg::Time& stamp) {
  require(settings.height_mode == "agl" || settings.height_mode == "msl",
          "probability heatmap height mode must be agl or msl");
  require(std::isfinite(settings.height_m) &&
              (settings.height_mode != "agl" || settings.height_m >= 0),
          "probability heatmap height must be finite; AGL height must be nonnegative");
  require(settings.max_dim >= 2 && settings.max_dim <= 512,
          "probability heatmap max_dim must be in [2,512]");
  require(std::isfinite(settings.drape_offset_frame) && settings.drape_offset_frame >= 0,
          "probability heatmap drape offset must be finite and nonnegative");

  ProbabilityHeatmap result;
  auto& grid = result.grid;
  grid.header.frame_id = "map";
  grid.header.stamp = stamp;
  grid.info.pose.orientation.w = 1;
  result.description = configurationDescription(environment, settings);
  if (!environment || environment->sources.empty()) {
    grid.info.resolution = 1;
    // A one-cell GridMap cannot prepare a mesh. Keep the clearing payload
    // large enough for the renderer to replace its previous geometry, while
    // all cells remain explicitly unavailable rather than fabricated zero.
    grid.info.length_x = 2;
    grid.info.length_y = 2;
    initializeGrid(grid, 2, 2);
    result.description += environment
        ? "\nEmpty accepted environment: model T=0; no source extent to draw. Previous map cleared."
        : "\nN/A: probability environment unavailable. Previous map cleared.";
    return result;
  }

  risk_model::validate(environment->scale);
  risk_model::validate(environment->settings);
  const auto scale = environment->scale;
  double x0 = std::numeric_limits<double>::infinity();
  double y0 = x0, x1 = -x0, y1 = -x0;
  std::unordered_set<std::string> source_ids;
  for (const auto& source : environment->sources) {
    risk_model::validate(source);
    require(source_ids.insert(source.id).second, "duplicate source in probability heatmap snapshot");
    const double cx = source.center_m.x / scale.xy_m;
    const double cy = source.center_m.y / scale.xy_m;
    const double radius = source.damage.r_max_m / scale.xy_m;
    require(std::isfinite(cx) && std::isfinite(cy) && std::isfinite(radius) && radius > 0,
            "probability heatmap source extent is not representable");
    x0 = std::min(x0, cx - radius); x1 = std::max(x1, cx + radius);
    y0 = std::min(y0, cy - radius); y1 = std::max(y1, cy + radius);
  }
  const double width = x1 - x0, height = y1 - y0;
  require(std::isfinite(width) && width > 0 && std::isfinite(height) && height > 0,
          "probability heatmap extent must be finite and nonempty");
  // One upward rounding prevents ceil(width/resolution) producing max_dim+1
  // from floating-point division when the nominal ratio equals max_dim.
  const double resolution = std::nextafter(std::max(width, height) / settings.max_dim,
                                           std::numeric_limits<double>::infinity());
  require(std::isfinite(resolution) && resolution > 0 &&
              x0 + resolution > x0 && y0 + resolution > y0,
          "probability heatmap resolution is not representable at this origin");
  const int rows = std::clamp(static_cast<int>(std::ceil(width / resolution)), 2, settings.max_dim);
  const int columns = std::clamp(static_cast<int>(std::ceil(height / resolution)), 2, settings.max_dim);
  const double length_x = rows * resolution, length_y = columns * resolution;
  require(std::isfinite(length_x) && std::isfinite(length_y),
          "probability heatmap grid length is not representable");
  grid.info.resolution = resolution;
  grid.info.length_x = length_x;
  grid.info.length_y = length_y;
  grid.info.pose.position.x = x0 + .5 * length_x;
  grid.info.pose.position.y = y0 + .5 * length_y;
  initializeGrid(grid, rows, columns);

  std::size_t known = 0, assumed = 0, missing_ground = 0, below_ground = 0, failed = 0;
  std::size_t drawn = 0, outside_footprint = 0;
  for (int row = 0; row < rows; ++row) {
    // GridMap ROS layout matches TerrainData: mirrored rows=X, columns=Y,
    // column-major payload. Query height never comes from the render offset.
    const double x = (x0 + length_x - (row + .5) * resolution) * scale.xy_m;
    for (int column = 0; column < columns; ++column) {
      const double y = (y0 + length_y - (column + .5) * resolution) * scale.xy_m;
      try {
        require(std::isfinite(x) && std::isfinite(y), "probability heatmap query exceeds SI range");
        const auto ground = elevation(*environment, x, y);
        if (settings.height_mode == "agl" && !ground) { ++missing_ground; continue; }
        const double query_z = settings.height_mode == "agl" ? *ground + settings.height_m
                                                             : settings.height_m;
        if (ground && query_z < *ground) { ++below_ground; continue; }
        require(representableFloat(query_z), "probability heatmap query height exceeds message range");
        const auto sample = risk_model::evaluateCombined(
            environment->sources, {x, y, query_z}, {}, environment->settings,
            risk_model::FormulaMode::Exact, risk_model::RcsMode::Reference,
            environment->terrain.get());
        const double probability = sample.risk.value;
        require(std::isfinite(probability) && probability >= 0 && probability <= 1,
                "probability heatmap core returned an invalid probability");

        const std::size_t index = static_cast<std::size_t>(column) * rows + row;
        // Keep the numeric probability slice intact, including valid zeroes
        // between sources. Only the rendered surface is clipped to the union
        // of their horizontal damage footprints. In-footprint zeroes remain
        // green (e.g. terrain shadow or a slice outside the vertical envelope).
        const bool uses_assumption = sample.flags.visibility_assumed || !ground;
        const bool inside_footprint = std::any_of(
            environment->sources.begin(), environment->sources.end(),
            [x, y](const risk_model::Source& source) {
              return std::hypot(x - source.center_m.x, y - source.center_m.y) <=
                     source.damage.r_max_m;
            });
        if (inside_footprint) {
          // Draw above terrain corners where available. With MSL and no DEM,
          // use the query altitude and disclose the assumption in grey.
          double render_ground = ground.value_or(query_z);
          if (ground) {
            const double half_cell_m = .5 * resolution * scale.xy_m;
            for (double dx : {-half_cell_m, half_cell_m})
              for (double dy : {-half_cell_m, half_cell_m}) {
                const auto corner = elevation(*environment, x + dx, y + dy);
                if (corner) render_ground = std::max(render_ground, *corner);
              }
          }
          const double render_z = render_ground / scale.z_m + settings.drape_offset_frame;
          require(representableFloat(render_z), "probability heatmap render height exceeds message range");
          grid.data[0].data[index] = static_cast<float>(render_z);
          grid.data[1].data[index] = uses_assumption ? packedColor({{145, 145, 145}})
                                                   : probabilityColor(probability);
          ++drawn;
        } else {
          ++outside_footprint;  // Keep render height/color NaN outside the disks.
        }
        grid.data[2].data[index] = static_cast<float>(probability);
        grid.data[3].data[index] = static_cast<float>(query_z);
        grid.data[4].data[index] = uses_assumption ? 0.0f : 1.0f;
        if (uses_assumption) ++assumed; else ++known;
      } catch (const std::bad_alloc&) {
        throw;
      } catch (const std::exception&) {
        // An unavailable cell remains NaN; a provider failure is never
        // silently displayed as model zero. Report the count below.
        ++failed;
      }
    }
  }
  std::ostringstream details;
  details << std::setprecision(9)
          << "\nGrid " << rows << 'x' << columns
          << "; resolution=" << resolution * scale.xy_m << " m"
          << "; known=" << known << "; assumed=" << assumed
          << "; drawn=" << drawn << "; outside source footprints=" << outside_footprint
          << "; unavailable AGL=" << missing_ground << "; below terrain=" << below_ground
          << "; query errors=" << failed
          << ". Cell-center samples; features smaller than the grid resolution may be missed.";
  if (drawn == 0) details << "\nN/A: no drawable samples at this height and display resolution.";
  else if (assumed || missing_ground || failed)
    details << "\nPartial/assumed display: inspect validity; do not interpret unavailable cells as zero probability.";
  result.description += details.str();
  return result;
}

}  // namespace path_manager::visualization
