# =============================================================================
# COMMUNICATION TYPE: EXTERNAL (Can be serialized for inter-drone communication)
# =============================================================================
# Purpose: a trajectory command (target + waypoints) for one agent
# Flow: RViz MissionConfig panel → /trajectory_command → ReplanFSM
# Note: carries one agent's target and waypoints
# =============================================================================

std_msgs/Header header

# Target drone identification
int32 drone_id

# Mission sequencing
int32 sequence
string mission_id

# Start position for this agent (set by the RViz MissionConfig panel)
geometry_msgs/Point start_position

# Individual target for this specific drone
geometry_msgs/Point target_position
geometry_msgs/Vector3 target_velocity

# Individual waypoints for this specific drone
geometry_msgs/Point[] waypoints

# Formation metadata (for optimizer)
string formation_type
float64 formation_scale
geometry_msgs/Vector3 formation_offset

# Full formation pattern (relative positions for all drones)
# Used by optimizer for formation cost calculation
geometry_msgs/Point[] formation_pattern

# Trajectory resolution: length per polynomial piece (m)
# Lower = smoother terrain following, Higher = simpler/coarser trajectory
# Default: 2.0, Typical range: 2.0 ~ 20.0
float64 length_per_piece

# Minimum horizontal clearance from obstacles (m)
# Default: 0.5, Typical range: 0.5 ~ 5.0
float64 obstacle_clearance

# Minimum flight altitude above terrain (m)
# Applied to start/goal height AND route-wide terrain clearance
# Default: 5.0, Typical range: 1.0 ~ 50.0
float64 min_flight_altitude

# Initial speed at trajectory start (m/s)
# Direction is auto-calculated from the first route leg.
# 0 = start from rest. Publishers may choose a non-zero operational default.
float64 initial_speed

# Optional map-frame initial-state vectors. These are physical SI values,
# converted to planner-frame units by ReplanFSM. An enabled velocity overrides
# initial_speed; acceleration can be enabled independently.
bool use_initial_velocity
geometry_msgs/Vector3 initial_velocity
bool use_initial_acceleration
geometry_msgs/Vector3 initial_acceleration
