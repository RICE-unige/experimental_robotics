#!/bin/bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
    echo "Sourced ROS2 Jazzy workspace"
fi

# Set up environment variables
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
export ROS_DISTRO=jazzy

# Graphics/Gazebo environment - Auto-detect GPU or use software rendering
export GZ_RENDER_ENGINE=ogre2
export OGRE_RHI=OpenGL
export QT_X11_NO_MITSHM=1

if [ -d /dev/dri ] && [ -n "$(ls -A /dev/dri 2>/dev/null)" ]; then
    echo "GPU detected - using hardware acceleration (OpenGL)"
    # HW path - unset software rendering variables
    unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE
else
    echo "No GPU detected - using software rendering"
    # SW fallback (slow but works)
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
fi

export XDG_RUNTIME_DIR=/tmp/runtime-dir
rm -rf /tmp/runtime-dir
mkdir -p /tmp/runtime-dir
chmod 0700 /tmp/runtime-dir

export DISPLAY=localhost:0.0

# Print environment info
echo "======================================"
echo "ROS2 Jazzy Container Ready"
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
echo "  gz sim         - Launch Gazebo Harmonic simulator"
echo "  ros2 launch ros_gz_sim_demos ... - Run Gazebo demos"
echo "======================================"

# Container will be kept running by docker-compose command
