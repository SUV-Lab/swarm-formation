#pragma once

#include <string>
#include <builtin_interfaces/msg/time.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <risk_model/model.hpp>

namespace path_manager::visualization {

// Display settings only. They do not alter the environment, planner, or model.
// height_m is the sampled AGL/MSL altitude in SI metres, independently of the
// terrain-draped render height. max_dim is in [2,512] and bounds both grid axes.
// Both axes retain at least two cells for GridMap mesh renderer compatibility.
struct ProbabilityHeatmapSettings {
  std::string height_mode{"agl"};
  double height_m{100.0};
  int max_dim{192};
  double drape_offset_frame{1.0};
};

struct ProbabilityHeatmap {
  grid_map_msgs::msg::GridMap grid;
  std::string description;
};

// Observes one immutable snapshot. The result owns all message data.
// Uses Exact/Reference and the shared evaluateCombined implementation; no policy
// taper, barrier, trajectory heading, or cumulative exposure is applied.
// Layers: elevation (render frame Z), color (packed RGB), probability [0,1],
// evaluation_height_m (SI MSL query Z), validity (1: no visibility assumption,
// 0: assumed visibility). Unknown cells are NaN in all layers. Assumed cells
// retain their numeric probability but use grey instead of the probability ramp.
// Rendering is limited to the union of horizontal damage disks (R_max), not
// policy radii or a positive-probability threshold. Outside those footprints,
// elevation/color remain NaN while numeric probability/query/validity are kept.
// Known zeroes inside a footprint stay visible green, including terrain shadows.
// Palette: linear (72,204,106) at 0, (242,242,24) at .5, (242,24,24) at 1;
// assumed visibility is (145,145,145). There is no per-grid normalization.
// Null/empty environments return a 2x2 all-NaN clearing stub with a description.
// Invalid display settings/snapshot geometry throw std::invalid_argument.
ProbabilityHeatmap makeProbabilityHeatmap(
    const risk_model::EnvironmentSnapshot* environment,
    const ProbabilityHeatmapSettings& settings,
    const builtin_interfaces::msg::Time& stamp);

}  // namespace path_manager::visualization
