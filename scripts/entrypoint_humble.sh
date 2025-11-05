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
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
export ROS_DISTRO=humble

# Graphics/Gazebo environment - Auto-detect GPU or use software rendering
if [ -d /dev/dri ] && [ -n "$(ls -A /dev/dri 2>/dev/null)" ]; then
    echo "GPU detected - using hardware acceleration"
    export GZ_RENDERING_ENGINE=ogre2
else
    echo "No GPU detected - using software rendering"
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    export GZ_RENDERING_ENGINE=ogre
fi
export MESA_GL_VERSION_OVERRIDE=3.3
export XDG_RUNTIME_DIR=/tmp/runtime-dir
rm -rf /tmp/runtime-dir
mkdir -p /tmp/runtime-dir
chmod 0700 /tmp/runtime-dir


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
echo ""
echo "Gazebo commands:"
echo "  gazebo         - Launch Gazebo Classic simulator"
echo "  ros2 launch gazebo_ros ... - Run Gazebo ROS2 demos"
echo "======================================"

# Container will be kept running by docker-compose command
