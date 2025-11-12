FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-yaml \
        ros-humble-control-msgs \
        ros-humble-rviz2 \
        ros-humble-rqt \
        ros-humble-rqt-common-plugins \
        ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Provide a colored prompt inside the bridge container for clarity
RUN echo 'export PS1="\[\e[1;35m\][ROS2-Bridge]\[\e[0m\] \[\e[1;34m\]\w\[\e[0m\] $ "' >> /root/.bashrc
