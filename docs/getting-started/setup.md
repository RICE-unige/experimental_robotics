# Getting Started

Set up the Experimental Robotics development environment on your workstation and confirm everything is ready before you dive into development or simulation.

## Prerequisites

### System Requirements

- Linux (Ubuntu 20.04+) or macOS with Docker support
- Docker Engine ≥ 20.10 and Docker Compose ≥ 2.0
- 20 GB free disk space (40 GB recommended for multiple simulators)
- 8 GB RAM minimum (16 GB recommended for Gazebo/Webots)

> [!note]
> Windows users can follow these steps inside WSL2 with Docker Desktop, but native Linux offers the smoothest experience—especially for GUI applications.

### Enable X11 (Linux GUI Support)

```bash
echo $DISPLAY            # Confirm X11 is running
xhost +local:docker      # Permit containers to use your display
```

### Optional: NVIDIA GPU Acceleration

1. Confirm drivers are installed:
   ```bash
   nvidia-smi
   ```
2. Install the NVIDIA Container Toolkit:
   ```bash
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
     sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
   sudo systemctl restart docker
   ```
3. Verify Docker can see the GPU:
   ```bash
   docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
   ```

> [!tip]
> Export `NVIDIA_VISIBLE_DEVICES=void` before running the helper scripts if you want to test the CPU-only fallback.

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

### 3. Launch a Development Container

{% tabs %}
{% tab title="ROS 2 Jazzy (recommended)" %}
```bash
./run.sh dev jazzy
```
{% endtab %}

{% tab title="ROS 2 Humble" %}
```bash
./run.sh dev humble
```
{% endtab %}
{% endtabs %}

### 4. Access the Container Shell

```bash
./connect.sh dev          # Auto-detect active dev container
docker compose exec dev_jazzy bash  # Manual alternative
```

## Verify the Environment

Run these checks from inside the container:

```bash
ros2 --version
ros2 pkg list | head

# Demo publisher/subscriber pair
ros2 run demo_nodes_cpp talker
# In another terminal attached to the same container
ros2 run demo_nodes_cpp listener
```

## Troubleshooting

### GUI Not Displaying

```bash
export QT_DEBUG_PLUGINS=1
echo $DISPLAY
./connect.sh dev
```

Ensure `xhost +local:docker` ran on the host and retry the container.

### Permission Denied Errors

```bash
sudo systemctl start docker
sudo usermod -aG docker $USER
newgrp docker
```

### Running Out of Disk Space

```bash
docker image prune -a
docker container prune
docker builder prune
```

## Where to Go Next

- Read the [GPU Setup](../reference/gpu-setup.md) guide if you plan to use hardware acceleration.
- Check the [FAQ](../reference/faq.md) whenever you hit a workflow snag.
- Explore the [Resources](../reference/resources.md) list for deeper ROS 2 learning material.
