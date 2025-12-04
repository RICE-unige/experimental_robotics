# ROS2 Humble with Navigation2, Gazebo, and development tools
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

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
      # ROS2 packages
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-msgs \
      ros-humble-nav2-util \
      ros-humble-nav2-common \
      ros-humble-nav2-core \
      ros-humble-nav2-costmap-2d \
      ros-humble-nav2-dwb-controller \
      ros-humble-nav2-lifecycle-manager \
      ros-humble-nav2-map-server \
      ros-humble-nav2-planner \
      ros-humble-nav2-rviz-plugins \
      ros-humble-nav2-simple-commander \
      ros-humble-nav2-smoother \
      ros-humble-nav2-theta-star-planner \
      ros-humble-nav2-velocity-smoother \
      ros-humble-nav2-waypoint-follower \
      # SLAM
      ros-humble-slam-toolbox \
      ros-humble-cartographer \
      ros-humble-cartographer-ros \
      # Simulation
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-plugins \
      ros-humble-gazebo-msgs \
      # Robot models and control
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
      ros-humble-xacro \
      ros-humble-urdf \
      ros-humble-tf2 \
      ros-humble-tf2-ros \
      ros-humble-tf2-tools \
      ros-humble-tf2-geometry-msgs \
      # Control
      ros-humble-controller-manager \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-control-msgs \
      ros-humble-realtime-tools \
      # Sensors
      ros-humble-sensor-msgs \
      ros-humble-image-transport \
      ros-humble-cv-bridge \
      ros-humble-vision-msgs \
      ros-humble-laser-filters \
      ros-humble-laser-geometry \
      # Visualization
      ros-humble-rviz2 \
      ros-humble-rviz-common \
      ros-humble-rviz-default-plugins \
      ros-humble-rviz-rendering \
      ros-humble-rqt \
      ros-humble-rqt-common-plugins \
      ros-humble-plotjuggler-ros \
      # Communication
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-example-interfaces \
      ros-humble-demo-nodes-cpp \
      ros-humble-demo-nodes-py \
      # Utilities
      ros-humble-teleop-twist-keyboard \
      ros-humble-twist-mux \
      ros-humble-joy \
      ros-humble-joy-teleop \
      # X11 and graphics
      x11-apps \
      mesa-utils \
      libgl1-mesa-dri \
      libgl1-mesa-glx \
      libglu1-mesa \
      freeglut3-dev \
      libegl1-mesa \
      libgbm1 \
      libxrandr2 \
      libxss1 \
      libxcursor1 \
      libxcomposite1 \
      libasound2 \
      libxi6 \
      libxtst6 \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages for robotics
RUN pip3 install --no-cache-dir \
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
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
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
    echo "# Graphics setup - Auto-detect GPU or use software rendering" >> /root/.bashrc && \
    echo "if [ -d /dev/dri ] && [ -n \"\$(ls -A /dev/dri 2>/dev/null)\" ]; then" >> /root/.bashrc && \
    echo "  export GZ_RENDERING_ENGINE=ogre2" >> /root/.bashrc && \
    echo "else" >> /root/.bashrc && \
    echo "  export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc && \
    echo "  export GALLIUM_DRIVER=llvmpipe" >> /root/.bashrc && \
    echo "  export GZ_RENDERING_ENGINE=ogre" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc && \
    echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> /root/.bashrc && \
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-dir" >> /root/.bashrc && \
    echo "mkdir -p /tmp/runtime-dir && chmod 0700 /tmp/runtime-dir 2>/dev/null || true" >> /root/.bashrc

# Set up colored prompt for better UX
RUN echo 'export PS1="\[\e[1;32m\][ROS2-Humble]\[\e[0m\] \[\e[1;34m\]\w\[\e[0m\] $ "' >> /root/.bashrc

WORKDIR /root