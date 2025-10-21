---
template: home.html
---

# Experimental Robotics

Welcome to the Experimental Robotics course documentation! This space accompanies the Docker-based development environment in the repository and provides the setup steps, labs, assignments, ROS 2 notes, and project guidelines.

!!! warning "Work in Progress"
    This documentation is actively being developed. Students will be informed when all assignments and projects are ready.

## 📚 What you'll find here

- Step-by-step environment setup (ROS 2 Jazzy and Humble)
- Labs and assignments with tips and references
- ROS 2 basics and quick commands for development
- Simulation, SLAM, and Navigation notes
- Final project requirements and evaluation rubric

## 🚀 Quick Start

Run these commands from your host machine to start the development environment:

=== "ROS2 Jazzy (Recommended)"

    ```bash
    # Clone the repository
    git clone https://github.com/RICE-unige/experimental_robotics.git
    cd experimental_robotics
    
    # Enable GUI applications
    xhost +local:docker
    
    # Start Jazzy development container
    ./run.sh dev jazzy
    
    # Access the container
    docker compose exec dev_jazzy bash
    ```

=== "ROS2 Humble"

    ```bash
    # Start Humble development container
    ./run.sh dev humble
    
    # Access the container
    docker compose exec dev_humble bash
    ```

=== "Robot Simulation"

    ```bash
    # Start ROSbot XL in Gazebo with full autonomy stack
    ./run.sh sim rosbotxl gazebo --slam --nav
    
    # Or just the simulator (no SLAM/Nav2)
    ./run.sh sim rosbotxl gazebo
    ```

## 💡 Tips

!!! tip "GPU Acceleration"
    If you have an NVIDIA GPU and drivers installed, the setup automatically detects and enables GPU acceleration. Check [GPU Setup](reference/gpu-setup.md) for details.

!!! tip "Container Connection"
    Use `./connect.sh` as a shortcut instead of typing full docker commands:
    ```bash
    ./connect.sh dev          # Connect to dev container
    ./connect.sh dev jazzy    # Connect to specific distro
    ./connect.sh sim          # Connect to simulation
    ```

## 🎯 Available Environments

| Environment | Description | Use Case |
|-------------|-------------|----------|
| **Jazzy** | ROS 2 Jazzy (default) | Latest features, recommended for new assignments |
| **Humble** | ROS 2 Humble LTS | Stable, for compatibility testing |
| **Gazebo** | Gazebo classic simulator | Physics simulation, SLAM, navigation |
| **Webots** | Webots professional simulator | Photorealistic simulation, learning |
| **O3DE** | Open 3D Engine | Advanced graphics, experimental |

### Supported Robots

- **ROSbot 2R/2 PRO** - Differential/Mecanum drive, basic sensors
- **ROSbot XL** - Mecanum drive, camera, premium sensors
- **Panther** - Tracked platform, rugged terrain

## 📞 Support

- **Teaching Assistant:** Omotoye Adekoya — [omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it)
- **Course Professor:** Prof. Carmine Recchiuto — [carmine.recchiuto@unige.it](mailto:carmine.recchiuto@unige.it)
- **Technical Issues:** Open a [GitHub issue](https://github.com/RICE-unige/experimental_robotics/issues)

## 📖 Next Steps

- 👉 Start with [Setup](getting-started/setup.md)
- 📚 Review [ROS 2 Basics](course/ros2-basics.md)
- 🎮 Explore [Simulation](course/simulation.md)
- 📝 Check [Labs](course/labs.md) and [Assignments](course/assignments.md)
- 🎓 See the [Final Project](course/final-project.md) brief
- ❓ Browse [FAQ](reference/faq.md) and [Resources](reference/resources.md)
