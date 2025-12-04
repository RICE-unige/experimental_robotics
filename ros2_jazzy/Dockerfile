# ROS2 Jazzy with Navigation2, Gazebo, and development tools
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Global Gazebo rendering environment
ENV QT_X11_NO_MITSHM=1 \
    GZ_RENDER_ENGINE=ogre2 \
    OGRE_RHI=OpenGL

# Update and install essential packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      # Build tools
      build-essential \
      cmake \
      git \
      wget \
      curl \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      # Development tools
      nano \
      vim \
      tmux \
      htop \
      tree \
      # Simulation - Gazebo Harmonic
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-image \
      ros-jazzy-ros-gz-sim-demos \
      # ROS2 packages
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-msgs \
      ros-jazzy-nav2-util \
      ros-jazzy-nav2-common \
      ros-jazzy-nav2-core \
      ros-jazzy-nav2-costmap-2d \
      ros-jazzy-nav2-dwb-controller \
      ros-jazzy-nav2-lifecycle-manager \
      ros-jazzy-nav2-map-server \
      ros-jazzy-nav2-planner \
      ros-jazzy-nav2-rviz-plugins \
      ros-jazzy-nav2-simple-commander \
      ros-jazzy-nav2-smoother \
      ros-jazzy-nav2-theta-star-planner \
      ros-jazzy-nav2-velocity-smoother \
      ros-jazzy-nav2-waypoint-follower \
      # SLAM
      ros-jazzy-slam-toolbox \
      ros-jazzy-cartographer \
      ros-jazzy-cartographer-ros \
      # Robot models and control
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-xacro \
      ros-jazzy-urdf \
      ros-jazzy-tf2 \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-tools \
      ros-jazzy-tf2-geometry-msgs \
      # Control
      ros-jazzy-controller-manager \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-control-msgs \
      ros-jazzy-realtime-tools \
      # Sensors
      ros-jazzy-sensor-msgs \
      ros-jazzy-image-transport \
      ros-jazzy-cv-bridge \
      ros-jazzy-vision-msgs \
      ros-jazzy-laser-filters \
      ros-jazzy-laser-geometry \
      # Visualization
      ros-jazzy-rviz2 \
      ros-jazzy-rviz-common \
      ros-jazzy-rviz-default-plugins \
      ros-jazzy-rviz-rendering \
      ros-jazzy-rqt \
      ros-jazzy-rqt-common-plugins \
      ros-jazzy-plotjuggler-ros \
      # Communication
      ros-jazzy-rmw-fastrtps-cpp \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-example-interfaces \
      ros-jazzy-demo-nodes-cpp \
      ros-jazzy-demo-nodes-py \
      # Utilities
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-twist-mux \
      ros-jazzy-joy \
      ros-jazzy-joy-teleop \
      # X11 and graphics
      x11-apps \
      mesa-utils \
      libgl1-mesa-dri \
      libglu1-mesa \
      freeglut3-dev \
      libgbm1 \
      libxrandr2 \
      libxss1 \
      libxcursor1 \
      libxcomposite1 \
      libasound2t64 \
      libxi6 \
      libxtst6 \
    && rm -rf /var/lib/apt/lists/*

# Install Vulkan userspace and additional GUI libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-vulkan-drivers \
    libvulkan1 \
    vulkan-tools \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages for robotics
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    scikit-learn \
    transforms3d \
    pyyaml

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create runtime directory for graphics
RUN mkdir -p /tmp/runtime-dir && chmod 1777 /tmp/runtime-dir

# Create workspace
ENV ROS2_WS=/root/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# Setup environment and Gazebo
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-43}" >> /root/.bashrc && \
    echo "alias build='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias source_ws='source install/setup.bash'" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install && source install/setup.bash'" >> /root/.bashrc && \
    echo "alias slam-async='ros2 launch slam_toolbox online_async_launch.py'" >> /root/.bashrc && \
    echo "alias slam-sync='ros2 launch slam_toolbox online_sync_launch.py'" >> /root/.bashrc && \
    echo "alias slam-offline='ros2 launch slam_toolbox offline_launch.py'" >> /root/.bashrc && \
    echo "alias slam-localization='ros2 launch slam_toolbox localization_launch.py'" >> /root/.bashrc && \
    echo "alias slam-lifelong='ros2 launch slam_toolbox lifelong_launch.py'" >> /root/.bashrc && \
    echo "# --- Gazebo rendering selection ---" >> /root/.bashrc && \
    echo "export GZ_RENDER_ENGINE=ogre2" >> /root/.bashrc && \
    echo "export OGRE_RHI=OpenGL" >> /root/.bashrc && \
    echo "export QT_X11_NO_MITSHM=1" >> /root/.bashrc && \
    echo "if [ -d /dev/dri ] && [ -n \"\$(ls -A /dev/dri 2>/dev/null)\" ]; then" >> /root/.bashrc && \
    echo "  # HW path" >> /root/.bashrc && \
    echo "  unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE" >> /root/.bashrc && \
    echo "else" >> /root/.bashrc && \
    echo "  # SW fallback (slow but works)" >> /root/.bashrc && \
    echo "  export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc && \
    echo "  export GALLIUM_DRIVER=llvmpipe" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc && \
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-dir" >> /root/.bashrc && \
    echo "mkdir -p /tmp/runtime-dir && chmod 0700 /tmp/runtime-dir 2>/dev/null || true" >> /root/.bashrc

# Set up colored prompt for better UX
RUN echo 'export PS1="\[\e[1;33m\][ROS2-Jazzy]\[\e[0m\] \[\e[1;34m\]\w\[\e[0m\] $ "' >> /root/.bashrc

WORKDIR /root