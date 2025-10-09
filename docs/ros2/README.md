---
description: ROS 2 essentials for the course
---

# ROS 2 Basics

This page collects quick references and patterns you’ll use throughout the course.

## Environment and shells

Inside a course container, the ROS 2 environment is already set up. To verify:

```bash
printenv | grep -E "^ROS_|AMENT|COLCON"
ros2 --help
```

## Workspace basics

```bash
# Create a workspace
data_dir=~/ros2_ws
mkdir -p $data_dir/src
cd $data_dir

# Build
colcon build --symlink-install

# Source the overlay (add to ~/.bashrc if desired)
source install/setup.bash

# Run tests
colcon test && colcon test-result --verbose
```

## Nodes, topics, and services

```bash
# List/run
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener

# Introspection
ros2 node list
ros2 topic list
ros2 topic echo /chatter
ros2 service list
```

## Launch files and parameters

> [!TIP]
> Keep separate terminals for building (colcon), launching, and introspection to avoid sourcing issues.
