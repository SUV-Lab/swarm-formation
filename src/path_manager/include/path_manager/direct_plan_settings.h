#pragma once

#include <rclcpp/parameter.hpp>
#include <stdexcept>
#include <vector>

namespace path_manager {

// Serialized ROS adapter input for one complete direct solve. These sparse
// values overlay the mission settings without writing to the node. Duplicate
// names use the last value, matching the legacy segment-profile reader.
// Unlike the narrower parallel worker profile, this includes settings consumed
// by route preparation, the vehicle model and altitude/risk queries.
struct DirectPlanSettings {
    std::vector<rclcpp::Parameter> overrides;
};

// Only configuration failures are wrapped. Search/solve failures and exceptions
// retain their normal semantics, so fallback does not hide computation errors.
class DirectPlanConfigurationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

}  // namespace path_manager
