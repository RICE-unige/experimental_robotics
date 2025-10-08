#!/bin/bash
set -e

# Source ROS2 Humble (base for dev environment)
source /opt/ros/humble/setup.bash

# Source workspaces if they exist
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
    echo "Sourced ROS2 Humble workspace"
fi

if [ -f /root/ros2_jazzy_ws/install/setup.bash ]; then
    source /root/ros2_jazzy_ws/install/setup.bash
    echo "Sourced ROS2 Jazzy workspace"
fi

# Set up environment variables
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
export ROS_DISTRO=humble

# Start code-server in background if requested
if [ "${START_CODE_SERVER:-false}" = "true" ]; then
    echo "Starting code-server..."
    code-server --bind-addr 0.0.0.0:8080 --auth none /root/ros2_ws &
fi

# Print environment info
echo "=========================================="
echo "ROS2 Development Environment Ready"
echo "=========================================="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "=========================================="
echo "Available development commands:"
echo "  build              - Build workspace"
echo "  build-debug        - Build with debug symbols"
echo "  build-release      - Build optimized release"
echo "  source_ws          - Source workspace"
echo "  cb                 - Build and source workspace"
echo "  test               - Run tests"
echo "  test-results       - Show test results"
echo "  clean              - Clean build artifacts"
echo "  format             - Format C++ code"
echo ""
echo "SLAM commands:"
echo "  slam-async         - Online async SLAM"
echo "  slam-sync          - Online sync SLAM"
echo "  slam-offline       - Offline SLAM"
echo "  slam-localization  - Localization mode"
echo "  slam-lifelong      - Lifelong SLAM"
echo "=========================================="
echo "Development tools available:"
echo "  - GDB debugger"
echo "  - Valgrind memory checker"
echo "  - Clang-format/tidy"
echo "  - Jupyter Lab (run: jupyter lab --allow-root --ip=0.0.0.0)"
if [ "${START_CODE_SERVER:-false}" = "true" ]; then
    echo "  - Code Server running on port 8080"
fi
echo "=========================================="

# Keep container running and allow interactive use
exec "$@"