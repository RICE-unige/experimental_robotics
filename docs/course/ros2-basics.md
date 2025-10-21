# ROS 2 Basics

Introduction to ROS 2 concepts and workflow for the Experimental Robotics course.

!!! info "Work in Progress"
    This section is being populated with detailed ROS 2 tutorials and best practices.

## Key Topics

- ROS 2 nodes and topics
- Services and actions
- Parameter servers
- Launch files
- Debugging with `ros2` CLI tools

## Quick Reference

### Essential Commands

```bash
# Node management
ros2 run <package> <node>
ros2 node list
ros2 node info <node_name>

# Topics
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic pub <topic> <msg_type> <data>

# Services
ros2 service list
ros2 service call <service> <srv_type> <args>

# Logging
ros2 run rclpy logging_demo
ros2 run roscpp_tutorials talker
```

### Common Packages

| Package | Purpose |
|---------|---------|
| `rclpy` | Python ROS 2 client library |
| `rclcpp` | C++ ROS 2 client library |
| `geometry_msgs` | Common message types (Twist, Pose, etc.) |
| `sensor_msgs` | Sensor data types (Image, LaserScan, etc.) |
| `std_msgs` | Standard message types (Int32, String, etc.) |

## Next Steps

- 📚 [Simulation Guide](simulation.md)
- 🛠️ [Lab Exercises](labs.md)
