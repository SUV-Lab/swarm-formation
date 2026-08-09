#!/bin/bash

# ROS2 bag recording script for /opt_trajectory topic
# Usage: ./record_trajectory.sh [output_dir] [duration_seconds]

# Default values
DEFAULT_OUTPUT_DIR="$HOME/rosbag_data"
DEFAULT_DURATION=0  # 0 means record until Ctrl+C

# Parse arguments
OUTPUT_DIR="${1:-$DEFAULT_OUTPUT_DIR}"
DURATION="${2:-$DEFAULT_DURATION}"

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Generate timestamp for filename
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="trajectory_${TIMESTAMP}"

echo "========================================="
echo "ROS2 Bag Recording - Trajectory Topics"
echo "========================================="
echo "Output directory: $OUTPUT_DIR"
echo "Bag name: $BAG_NAME"
echo "Topic: /optimized_trajectory"
if [ "$DURATION" -eq 0 ]; then
    echo "Duration: Until Ctrl+C"
else
    echo "Duration: ${DURATION} seconds"
fi
echo "========================================="
echo ""

# Build the ros2 bag record command
CMD="ros2 bag record -o ${OUTPUT_DIR}/${BAG_NAME}"

# Add topic
CMD="${CMD} /optimized_trajectory"

# Add duration if specified
if [ "$DURATION" -gt 0 ]; then
    CMD="${CMD} --duration ${DURATION}"
fi

# Add compression (mcap format, default in ROS2)
CMD="${CMD} -s mcap"

echo "Starting recording..."
echo "Command: $CMD"
echo ""

# Execute the command
eval "$CMD"

echo ""
echo "========================================="
echo "Recording stopped"
echo "Bag saved to: ${OUTPUT_DIR}/${BAG_NAME}"
echo "========================================="
echo ""
echo "To replay: ros2 bag play ${OUTPUT_DIR}/${BAG_NAME}"
echo "To get info: ros2 bag info ${OUTPUT_DIR}/${BAG_NAME}"
