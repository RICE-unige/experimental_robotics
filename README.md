# Experimental Robotics Course

[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![GitBook](https://img.shields.io/static/v1?message=Documented%20on%20GitBook&logo=gitbook&logoColor=ffffff&label=%20&labelColor=5c5c5c&color=3F89A1)](https://ricelab.gitbook.io/experimental-robotics-docs)
[![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)

> [!WARNING]  
> This repository is actively in development and is **not yet ready for use**. Students will be informed by the professor when the repository, projects, and assignments are ready for the course.

<p align="center">
  <img src="imgs/ricelab_logo.jpg" alt="RICELab" height="30" style="margin-right: 16px;">
  <img src="imgs/university_of_genoa_logo.png" alt="University of Genoa" height="30">
</p>

Docker-based development environment for the Experimental Robotics course at University of Genoa, featuring ROS2 Humble and ROS2 Jazzy with slam, navigation, simulation, and development tools.

## Prerequisites

- Docker Engine (>= 20.10) and Docker Compose (>= 2.0)
- X11 forwarding for GUI applications
- **Optional:** NVIDIA GPU + Container Toolkit for GPU-accelerated simulations (automatically falls back to CPU rendering if unavailable)
  - To explicitly disable GPU: `export NVIDIA_VISIBLE_DEVICES=void`

## Quick Start

```bash
# Clone the repository
git clone https://github.com/RICE-unige/experimental_robotics.git
cd experimental_robotics

# Start development environment
./run.sh dev jazzy                 # ROS2 Jazzy (default)
./run.sh dev humble                # ROS2 Humble

# Start robot simulation
./run.sh sim rosbotxl gazebo       # ROSbot XL in Gazebo
./run.sh sim rosbot2r gazebo --slam  # With SLAM mapping

# Access containers
docker compose exec dev_jazzy bash
docker compose exec sim-gazebo-rosbotxl bash

# Stop containers
./run.sh stop all                  # Stop everything
```

## 📚 Documentation

For detailed setup guides, troubleshooting tips, and curated resources, visit our **GitBook page**:

📝 **[Experimental Robotics Course Documentation](https://ricelab.gitbook.io/experimental-robotics-docs)**

## Available Commands

### Development Containers
```bash
./run.sh dev <distro>             # Start dev container (jazzy or humble)
./run.sh dev jazzy --build        # Rebuild and start
```

### Robot Simulations
```bash
./run.sh sim <robot> <simulator> [options]

# Robots: rosbot2r, rosbotxl, panther
# Simulators: gazebo, webots, o3de
# Options: --slam, --nav, --dev <distro>, --build

# Examples:
./run.sh sim rosbotxl gazebo                    # Basic simulation
./run.sh sim rosbot2r gazebo --slam             # With SLAM
./run.sh sim rosbotxl gazebo --slam --nav       # Full autonomy
./run.sh sim rosbot2r gazebo --dev humble       # With dev container
```

> [!NOTE]
> **O3DE now uses a VirtualGL + VNC pipeline.** After launching `./run.sh sim rosbotxl o3de --build`, open a VNC client on the host (e.g. `vncviewer localhost:5901`) to interact with the Editor. Override `O3DE_VNC_PORT`, `O3DE_VNC_RESOLUTION`, or `VNC_PASSWORD` in your environment if you need different settings.

### Connecting to Containers
```bash
./connect.sh dev                  # Connect to dev container (auto-detect)
./connect.sh dev jazzy            # Connect to Jazzy dev
./connect.sh sim                  # Connect to simulation (auto-detect)
./connect.sh sim rosbotxl         # Connect to ROSbot XL sim
./connect.sh list                 # List all running containers
```

### Management
```bash
./run.sh stop [dev|sim|all]       # Stop containers
./run.sh clean                    # Remove all containers
./run.sh status                   # Show running containers
./run.sh help                     # Show full help
```

## 🆘 Support

For questions and support:

**Teaching Assistant:**
- **Omotoye Adekoya** - [omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it)

**Course Professor:**
- **Prof. Carmine Recchiuto** - [carmine.recchiuto@unige.it](mailto:carmine.recchiuto@unige.it)

You can also create an issue in this repository for technical problems.

---

© 2025 **University of Genoa - RICELab**
