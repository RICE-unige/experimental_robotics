# Simulation

Guide to running robot simulations with Gazebo, Webots, and O3DE.

## Quick Start

### Gazebo (Recommended)

```bash
# Start ROSbot XL in Gazebo
./run.sh sim rosbotxl gazebo

# With SLAM mapping
./run.sh sim rosbotxl gazebo --slam

# Full autonomy stack (SLAM + Navigation)
./run.sh sim rosbotxl gazebo --slam --nav
```

### Webots

```bash
# Start ROSbot 2R in Webots
./run.sh sim rosbot2r webots

# With SLAM
./run.sh sim rosbotxl webots --slam
```

### O3DE (Experimental)

```bash
# Start ROSbot XL in O3DE
./run.sh sim rosbotxl o3de
```

## Supported Configurations

### Robots

| Robot | Gazebo | Webots | O3DE |
|-------|:------:|:------:|:----:|
| **ROSbot 2R** | ✅ | ✅ | ❌ |
| **ROSbot XL** | ✅ | ✅ | ✅ |
| **Panther** | ✅ | ❌ | ❌ |

## Using SLAM Mapping

The SLAM Toolbox enables real-time mapping:

```bash
./run.sh sim rosbotxl gazebo --slam

# In RViz2, use Teleop to drive robot
# Map will be continuously updated
```

## Autonomous Navigation (Nav2)

For full autonomous navigation with goal-setting:

```bash
./run.sh sim rosbotxl gazebo --slam --nav

# Set navigation goals in RViz2
```

!!! tip "Manual Control"
    While simulation is running, manually control the robot with:
    ```bash
    docker compose exec sim-gazebo-rosbotxl bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

## Troubleshooting

### GUI Not Appearing

```bash
# Enable X11 forwarding
xhost +local:docker

# Stop and restart
./run.sh stop sim
./run.sh sim rosbotxl gazebo
```

### Simulation Crashes

```bash
# Check logs
docker compose logs -f sim-gazebo-rosbotxl

# Rebuild images
./run.sh sim rosbotxl gazebo --build
```

## Next Steps

- 🎮 [Try a Lab Exercise](labs.md)
- 📚 [ROS 2 Basics](ros2-basics.md)
- 📝 [Assignment Guide](assignments.md)
