#!/bin/bash
set -e

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
    echo "Sourced ROS2 Humble workspace"
fi

# Set up environment variables
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_DISTRO=humble

# Print environment info
echo "======================================"
echo "ROS2 Humble Container Ready"
echo "======================================"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "======================================"
echo "Available commands:"
echo "  build          - Build workspace"
echo "  source_ws      - Source workspace"
echo "  cb             - Build and source workspace"
echo "  clean          - Clean build artifacts"
echo ""
echo "SLAM commands:"
echo "  slam-async     - Online async SLAM"
echo "  slam-sync      - Online sync SLAM"
echo "  slam-offline   - Offline SLAM"
echo "  slam-localization - Localization mode"
echo "  slam-lifelong  - Lifelong SLAM"
echo "======================================"

# Keep container running and allow interactive use
exec "$@"