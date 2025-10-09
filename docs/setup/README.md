---
description: Install prerequisites and run the course environment
---

# Setup

> [!NOTE]
> These steps assume Ubuntu Linux with Docker installed. macOS and Windows (WSL2) are possible but may require extra GUI configuration.

## Prerequisites

- Docker Engine ≥ 20.10 and Docker Compose ≥ 2.0
- X11 forwarding enabled for GUI apps (Linux)
- Optional: NVIDIA GPU with NVIDIA Container Toolkit for acceleration

Verify Docker is working:

```bash
docker --version
docker compose version
```

Optional: verify GPU runtime is available:

```bash
# Should print from inside the container later if you have a GPU
nvidia-smi || echo "No NVIDIA GPU detected on host"
```

## Clone and start

```bash
git clone https://github.com/RICE-unige/experimental_robotics.git
cd experimental_robotics

# Allow local containers to connect to the X server (Linux GUI apps)
xhost +local:docker

# Start one of the profiles
./start.sh                    # ROS2 Jazzy (default)
./start.sh --profile humble   # ROS2 Humble
./start.sh --profile all      # Run both (same DDS domain)
```

Open a shell in the running container:

```bash
docker compose exec ros2_jazzy bash
# or
docker compose exec ros2_humble bash
```

> [!TIP]
> If GUI apps don’t render, make sure you ran `xhost +local:docker` on the host. Wayland users may need to enable X11 compatibility.

## Next steps

- Continue to [ROS 2 Basics](../ros2/README.md)
- Try the [Simulation](../simulation/README.md)
- Check the [Labs](../labs/README.md) and [Assignments](../assignments/README.md)
