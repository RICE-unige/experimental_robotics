# Setup

Setting up the Experimental Robotics development environment.

## Prerequisites

### System Requirements

- **OS:** Linux (Ubuntu 20.04+) or macOS with Docker support
- **Docker Engine:** >= 20.10
- **Docker Compose:** >= 2.0
- **Disk Space:** 20GB (minimum, 40GB recommended for multiple images)
- **RAM:** 8GB (16GB recommended for simulation)

### X11 Forwarding (Linux only)

For GUI applications (Gazebo, RViz2, etc.), enable X11 forwarding:

```bash
# Check X11 is running
echo $DISPLAY

# Allow Docker containers to connect to X11
xhost +local:docker
```

### Optional: GPU Support

For GPU-accelerated simulations:

1. **Install NVIDIA drivers**
   ```bash
   nvidia-smi  # Verify drivers are installed
   ```

2. **Install NVIDIA Container Toolkit**
   ```bash
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
     sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
   sudo systemctl restart docker
   ```

3. **Verify GPU access**
   ```bash
   docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
   ```

## Quick Installation

### 1. Clone the Repository

```bash
git clone https://github.com/RICE-unige/experimental_robotics.git
cd experimental_robotics
```

### 2. Verify Docker Installation

```bash
docker --version
docker compose version
```

### 3. Start Development Container

=== "Jazzy (Recommended)"

    ```bash
    ./run.sh dev jazzy
    ```

=== "Humble"

    ```bash
    ./run.sh dev humble
    ```

### 4. Access the Container

```bash
# Auto-detect and connect
./connect.sh dev

# Or manually
docker compose exec dev_jazzy bash
```

## Verifying Installation

Once inside the container, verify ROS 2 is working:

```bash
# Check ROS 2 version
ros2 --version

# List installed packages
ros2 pkg list | head

# Test talker/listener
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2 (new terminal in same container):
ros2 run demo_nodes_cpp listener
```

## Common Issues

### X11 Forwarding Not Working

```bash
# Try enabling verbose X11 logging
export QT_DEBUG_PLUGINS=1
./connect.sh dev

# Check DISPLAY variable
echo $DISPLAY
```

### Permission Denied

```bash
# Ensure Docker daemon is running
sudo systemctl start docker

# Add user to docker group (Linux)
sudo usermod -aG docker $USER
newgrp docker
```

### Out of Disk Space

```bash
# Clean up old images
docker image prune -a

# Remove unused containers
docker container prune

# Clear build cache
docker builder prune
```

## Next Steps

- 📚 [ROS 2 Basics](../course/ros2-basics.md)
- 🎮 [Start a Simulation](../course/simulation.md)
- 📝 [Review Labs](../course/labs.md)
