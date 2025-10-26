# Experimental Robotics Course

[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![GitBook](https://img.shields.io/static/v1?message=Documented%20on%20GitBook&logo=gitbook&logoColor=ffffff&label=%20&labelColor=5c5c5c&color=3F89A1)
[![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)

> [!WARNING]  
> This repository is actively in development and is **not yet ready for use**. Students will be informed by the professor when the repository, projects, and assignments are ready for the course.

<p align="center">
  <img src="imgs/ricelab_logo.jpg" alt="RICELab" height="30" style="margin-right: 16px;">
  <img src="imgs/university_of_genoa_logo.png" alt="University of Genoa" height="30">
</p>

Docker-based development environment for the Experimental Robotics course at University of Genoa, featuring ROS2 Humble and ROS2 Jazzy with SLAM, navigation, simulation, and development tools.

## 📋 Table of Contents

- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Documentation](#-documentation)
- [Setup Guide](#-setup-guide)
- [Container Overview](#-container-overview)
- [Workspace Structure](#-workspace-structure)
- [Windows + WSL Users](#-windows--wsl-users)
- [Simulator Compatibility](#-simulator-compatibility)
- [Building ROS2 Packages](#-building-ros2-packages)
- [Available Commands](#available-commands)
- [Environment Variables](#-environment-variables)
- [Tips & Best Practices](#-tips--best-practices)
- [Troubleshooting](#-troubleshooting)
- [Support](#-support)

## Prerequisites

- Docker Engine (>= 20.10) and Docker Compose (>= 2.0)
- X11 forwarding for GUI applications
- **Optional:** NVIDIA GPU + Container Toolkit for GPU-accelerated simulations (automatically falls back to CPU rendering if unavailable)
  - To explicitly disable GPU: `export NVIDIA_VISIBLE_DEVICES=void`

> [!TIP]
> No discrete GPU? Set `DOCKER_GPUS=0` before running `./run.sh` to force CPU rendering.


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
./run.sh sim rosbot2r gazebo       # ROSbot 2R in Gazebo

# Access containers
docker compose exec dev_jazzy bash
docker compose exec sim-gazebo-rosbotxl bash

# Stop containers
./run.sh stop all                  # Stop everything
```

## 📚 Documentation

> [!NOTE]  
> **This README is your primary documentation source.** All setup instructions, usage guides, and troubleshooting information are contained here. Additional documentation will be announced when available.

## 🚀 Setup Guide

### First-Time Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/RICE-unige/experimental_robotics.git
   cd experimental_robotics
   ```

2. **Enable X11 forwarding** (for GUI applications):
   ```bash
   xhost +local:docker
   ```
   > [!NOTE]
   > This is automatically done by `run.sh`, but you may need to run it manually after reboot.

3. **Start your development environment:**
   ```bash
   # For ROS2 Jazzy (recommended)
   ./run.sh dev jazzy
   
   # For ROS2 Humble
   ./run.sh dev humble
   ```

4. **Access the container:**
   ```bash
   ./connect.sh dev
   # or
   docker compose exec dev_jazzy bash
   ```

5. **Verify the setup:**
   ```bash
   # Inside the container
   ros2 topic list
   ros2 run demo_nodes_cpp talker
   ```

### Starting a Simulation

```bash
# Launch simulation (in a new terminal on host)
./run.sh sim rosbotxl gazebo

# Connect to simulation container (optional)
./connect.sh sim
```

Your dev container and simulation can run simultaneously and communicate via ROS2 topics.

## 📦 Container Overview

This environment uses **two types of containers** that work together:

### Development Containers (`dev_jazzy` / `dev_humble`)
- **Purpose:** Your workspace for writing, building, and testing ROS2 code
- **What's inside:** Full ROS2 desktop installation, Navigation2, SLAM Toolbox, build tools, Python packages
- **Your code:** Place your packages in `ros2_jazzy_ws/src/` or `ros2_humble_ws/src/`
- **Use this for:** Writing code, building packages, running your nodes, development tasks

> [!IMPORTANT]
> **All ROS2 commands must be run inside the dev container**, not on your host machine. The workspace folders (`ros2_jazzy_ws`, `ros2_humble_ws`) are mounted volumes - edit files on your host, but build and run inside the container.

### Simulation Containers (`sim-*`)
- **Purpose:** Run pre-configured robot simulations (ROSbot, Panther, etc.)
- **What's inside:** Gazebo/Webots/O3DE simulator with robot models, sensors, and world environments
- **Use this for:** Testing your algorithms, visualizing robot behavior, collecting sensor data
- **Note:** Generally read-only - avoid modifying unless you need custom robot configurations

**How they connect:** Both containers use `network_mode: host`, allowing seamless ROS2 communication. Topics published in your dev container are visible in the simulation, and vice versa.

## 📁 Workspace Structure

```
experimental_robotics/
├── ros2_jazzy_ws/          # ROS2 Jazzy workspace (mounted to dev container)
│   └── src/                # 👉 Put your packages here
├── ros2_humble_ws/         # ROS2 Humble workspace (mounted to dev container)
│   └── src/                # 👉 Put your packages here
├── config/                 # Robot configuration files (mounted read-only)
├── scripts/                # Container entrypoint scripts
├── run.sh                  # Main launcher script
├── connect.sh              # Container connection helper
├── docker-compose.yml      # Container orchestration
└── .env                    # Environment configuration
```

> [!TIP]
> **Your code in `ros2_*_ws/src/` persists on your host machine.** Containers are ephemeral, but your work is safe. Edit files with your favorite IDE on the host, then build inside the container.

## 🪟 Windows + WSL Users

### Networking: reach real robots from WSL 2

> [!IMPORTANT]
> Mirror Windows networking so your WSL distro lands on the same LAN subnet—essential when you connect to real robots over Wi-Fi.

1. Create or edit `C:\Users\<you>\.wslconfig` and add:

   ```ini
   [wsl2]
   networkingMode=mirrored
   dnsTunneling=true
   autoProxy=true
   firewall=true
   ```

2. Apply the change from PowerShell:

   ```powershell
   wsl --shutdown
   ```

3. Reopen WSL; the distro now gets an IP on the Windows LAN. Approve any Windows Firewall prompts the first time.

> [!TIP]
> Run `wslinfo --networking-mode` to confirm it says `mirrored`.

> [!CAUTION]
> Mirrored networking needs Windows 11 22H2+ and an up-to-date WSL. Older builds silently ignore these settings.

### GPU acceleration with Docker Desktop

> [!IMPORTANT]
> Enable GPU passthrough so Gazebo/RViz can use hardware acceleration inside WSL.

1. Install the latest NVIDIA (or vendor) GPU driver on Windows—reboot afterwards.
2. Inside WSL, install the CUDA userspace bits (e.g. `sudo apt-get install -y nvidia-cuda-toolkit`) and confirm `nvidia-smi` works.
3. In Docker Desktop: Settings → Resources → WSL Integration, enable your distro and toggle on GPU support.
4. Test Docker access:

   ```bash
   docker run --rm --gpus all nvidia/cuda:12.3.0-runtime-ubuntu22.04 nvidia-smi
   ```

> [!WARNING]
> If the test fails, ensure your distro is WSL 2, Docker Desktop runs in WSL mode, and enterprise policies aren’t blocking GPU access.

> [!TIP]
> On machines without a usable GPU, set `DOCKER_GPUS=0` (or `export NVIDIA_VISIBLE_DEVICES=void`) before calling `./run.sh` to force CPU rendering.

## 🎮 Simulator Compatibility

<table>
  <tr>
    <th>Simulator</th>
    <th>Logo</th>
    <th>Compatible Robots</th>
    <th>Status</th>
  </tr>
  <tr>
    <td><strong>Gazebo</strong></td>
    <td align="center"><a href="https://gazebosim.org/docs/latest/getstarted/"><img src="imgs/gazebo_horz_pos.svg" alt="Gazebo" height="40"/></a></td>
    <td>rosbot2r, rosbotxl, rosbotxl-manip, panther</td>
    <td>✅ <strong>Recommended</strong> - Most stable, covered in class</td>
  </tr>
  <tr>
    <td><strong>Webots</strong></td>
    <td align="center"><a href="https://cyberbotics.com/"><img src="imgs/webots_logo.png" alt="Webots" height="40"/></a></td>
    <td>rosbot2r, rosbotxl</td>
    <td>🧪 Experimental - For exploration and practice</td>
  </tr>
  <tr>
    <td><strong>O3DE</strong></td>
    <td align="center"><a href="https://o3de.org/"><img src="imgs/O3DE_Color_Logo.svg.png" alt="O3DE" height="40"/></a></td>
    <td>rosbotxl</td>
    <td>🧪 Advanced - Requires VNC, for testing only</td>
  </tr>
</table>

> [!IMPORTANT]
> **Use Gazebo for all coursework and assignments.** It's the most stable, well-documented, and was covered in your lectures. Webots and O3DE are provided for exploration and to familiarize yourself with different simulators used in robotics research and industry.

### About Other Simulators

Simulators like **NVIDIA Isaac Sim** are also important in robotics but require significant GPU resources (typically RTX 3000+ series). Since most students don't have access to such hardware, it's not included here. If you're interested in learning more:

- [NVIDIA Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
- [Isaac Sim ROS2 Integration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

## 🔨 Building ROS2 Packages

> [!IMPORTANT]
> All commands in this section must be executed **inside the dev container**. Connect using `./connect.sh dev` or `docker compose exec dev_jazzy bash`.

### Creating a New Package

```bash
# Inside your dev container
cd ~/ros2_ws/src

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_package

# Or a Python package
ros2 pkg create --build-type ament_python my_py_package
```

### Building Your Workspace

```bash
# Build all packages
cd ~/ros2_ws
colcon build --symlink-install

# Build a specific package
colcon build --packages-select my_package

# Source the workspace
source install/setup.bash
```

> [!TIP]
> **Helper aliases available in the container:**
> ```bash
> build           # Alias for: colcon build --symlink-install
> source_ws       # Alias for: source install/setup.bash
> cb              # Alias for: build && source_ws (build and source in one command)
> ```

### Running Your Nodes

```bash
# After building and sourcing
ros2 run my_package my_node

# Or using launch files
ros2 launch my_package my_launch.py
```

## Available Commands

### Development Containers
```bash
./run.sh dev <distro>             # Start dev container (jazzy or humble)
./run.sh dev jazzy --build        # Rebuild and start
```

### Robot Simulations
```bash
./run.sh sim <robot> <simulator> [options]

# Robots: rosbot2r, rosbotxl, rosbotxl-manip, panther
# Simulators: gazebo, webots, o3de
# Options: --dev <distro>, --build

# Examples:
./run.sh sim rosbotxl gazebo                    # Basic simulation
./run.sh sim rosbotxl-manip gazebo              # XL with manipulator
./run.sh sim rosbot2r gazebo --dev humble       # With dev container
./run.sh sim rosbotxl gazebo --dev jazzy        # Simulation + dev
```

> [!NOTE]
> **O3DE now uses a VirtualGL + VNC pipeline.** After launching `./run.sh sim rosbotxl o3de --build`, open a VNC client on the host (e.g. `vncviewer localhost:5901`) to interact with the Editor. Override `O3DE_VNC_PORT`, `O3DE_VNC_RESOLUTION`, or `VNC_PASSWORD` in your environment if you need different settings.

### Connecting to Containers
```bash
./connect.sh dev                  # Connect to dev container (auto-detect)
./connect.sh dev jazzy            # Connect to Jazzy dev
./connect.sh dev humble           # Connect to Humble dev
./connect.sh sim                  # Connect to simulation (auto-detect)
./connect.sh sim rosbotxl         # Connect to ROSbot XL sim
./connect.sh list                 # List all running containers
```

> [!TIP]
> You can run both dev containers simultaneously (Jazzy and Humble). If both are running and you use `./connect.sh dev`, you'll be prompted to choose which one to connect to.

### Management
```bash
./run.sh stop [dev|sim|all]       # Stop containers
./run.sh clean                    # Remove all containers
./run.sh status                   # Show running containers
./run.sh help                     # Show full help
```

## ⚙️ Environment Variables

Customize the environment by editing the `.env` file in the project root:

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DOMAIN_ID` | `43` | ROS2 domain for inter-container communication. Change if running multiple ROS2 systems on the same network. |
| `DISPLAY` | `:0` | X11 display for GUI applications (auto-detected). |
| `MECANUM` | `False` | Set to `True` for mecanum wheels, `False` for differential drive. |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU visibility. Set to `void` to disable GPU. |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS2 middleware. Alternative: `rmw_cyclonedds_cpp`. |
| `O3DE_VNC_PORT` | `5901` | VNC port for O3DE simulator access. |
| `O3DE_VNC_RESOLUTION` | `1920x1080` | VNC display resolution for O3DE. |

> [!WARNING]
> After modifying `.env`, you must restart containers for changes to take effect:
> ```bash
> ./run.sh stop all
> ./run.sh dev jazzy  # or your desired command
> ```

## 💡 Tips & Best Practices

### Development Workflow

> [!TIP]
> **Recommended workflow for maximum efficiency:**
> 1. **Keep containers running:** Use `./run.sh dev jazzy` once, then connect/disconnect with `./connect.sh dev` as needed.
> 2. **Separate terminals:** Run simulation in one terminal, dev container in another for parallel work.
> 3. **Use aliases:** Inside containers, use `cb` (colcon build + source) for quick rebuilds.
> 4. **Version control:** Your `ros2_*_ws/src/` is on the host - use git normally for your packages.

### ROS2 Communication

```bash
# Monitor topics from any container or host (if ROS2 installed)
ros2 topic list
ros2 topic echo /scan

# Check node connectivity
ros2 node list
ros2 node info /my_node
```

### Container Management

```bash
# View logs without entering container
docker compose logs -f dev_jazzy

# Check resource usage
docker stats

# Stop only simulations, keep dev running
./run.sh stop sim
```

> [!TIP]
> **Performance optimization tips:**
> - **GPU acceleration:** Ensure NVIDIA drivers and Docker GPU support are properly installed for best simulation performance.
> - **Shared memory:** Containers use 2GB shared memory by default. Increase in `docker-compose.yml` if needed for large sensor data.
> - **Build speed:** Use `colcon build --parallel-workers 4` to limit CPU usage during builds.

## 🔧 Troubleshooting

### GUI Applications Don't Open

```bash
# Re-enable X11 forwarding
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY  # Should show :0 or :1
```

> [!WARNING]
> **"Cannot connect to X server" Error**
>
> Ensure X11 forwarding is enabled and DISPLAY is set correctly in `.env`.

### Containers Won't Start

```bash
# Check Docker daemon
sudo systemctl status docker

# View detailed logs
docker compose logs

# Clean and rebuild
./run.sh clean
./run.sh dev jazzy --build
```

> [!CAUTION]
> **ROS2 Nodes Can't Communicate**
>
> - Verify `ROS_DOMAIN_ID` is the same in all containers (check `.env`)
> - Ensure containers use `network_mode: host` (default in docker-compose.yml)
> - Check firewall isn't blocking multicast traffic

### Build Errors

```bash
# Clean build artifacts
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

> [!NOTE]
> **Simulation Runs Slowly**
>
> - **CPU rendering mode:** If you don't have NVIDIA GPU, simulation uses software rendering (slower).
> - **Check GPU access:** Run `nvidia-smi` inside container to verify GPU is detected.
> - **Reduce sensor load:** Modify simulation launch files to disable unnecessary sensors.

### Container Disk Space Issues

```bash
# Clean unused Docker resources
docker system prune -a

# Remove only stopped containers and unused images
docker system prune
```

**Still having issues?** Check the [Issues](https://github.com/RICE-unige/experimental_robotics/issues) page or contact support below.

## 🆘 Support

For questions and support:

**Teaching Assistant:**
- **Omotoye Adekoya** - [omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it)

**Course Professor:**
- **Prof. Carmine Recchiuto** - [carmine.recchiuto@unige.it](mailto:carmine.recchiuto@unige.it)

You can also create an issue in this repository for technical problems.

---

© 2025 **University of Genoa - RICELab**
