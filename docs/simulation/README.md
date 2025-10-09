---
description: Simulation, SLAM, and Navigation overview
---

# Simulation

> [!NOTE]
> This page outlines the expected tools and workflows.

## Tools

- Gazebo (simulator)
- RViz (visualization)
- SLAM Toolbox (mapping)
- Nav2 (navigation stack)

## Typical workflow

1. Launch a world and robot model in Gazebo
2. Visualize topics and TF in RViz
3. Run SLAM for mapping (or load a map)
4. Launch Nav2 for localization + path planning + control

## Example commands

```bash
# Example patterns — actual packages/launch files may differ
ros2 launch <robot_bringup_pkg> sim.launch.py
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## Next steps

- Return to [ROS 2 Basics](../ros2/README.md)
- Proceed to the [Labs](../labs/README.md)
