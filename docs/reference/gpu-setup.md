# GPU Setup

Guide to setting up NVIDIA GPU acceleration for simulations.

## Overview

All simulation containers (Gazebo, Webots, O3DE) are configured to use NVIDIA GPU acceleration when available, which significantly improves rendering performance.

## Requirements

### For GPU Acceleration (Optional)

- **NVIDIA GPU** - RTX series recommended, GTX 1060+ minimum
- **NVIDIA Drivers** - 450+ (check with `nvidia-smi`)
- **NVIDIA Container Toolkit** - For Docker GPU support
- **8GB+ VRAM** - Recommended for Gazebo

### System Prerequisites

Check if you have an NVIDIA GPU:

```bash
lspci | grep -i nvidia
# or
nvidia-smi
```

## Installation Steps

### 1. Install NVIDIA Drivers

On Ubuntu:

```bash
# Install drivers
sudo apt update
sudo apt install -y nvidia-driver-550

# Verify installation
nvidia-smi
```

### 2. Install NVIDIA Container Toolkit

```bash
# Add NVIDIA repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install container toolkit
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Restart Docker daemon
sudo systemctl restart docker
```

### 3. Verify GPU Support

Test Docker GPU access:

```bash
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

You should see your GPU listed in the output.

## CPU Fallback

If you don't have an NVIDIA GPU or the toolkit isn't installed:

- ✅ **Simulations will still work** - They automatically fall back to CPU rendering
- ⚠️ **Performance will be slower** - Using Mesa software rendering instead of GPU
- 📊 **CPU usage increases** - GUI rendering can use 500%+ of CPU
- ⏱️ **Startup takes longer** - Asset loading is CPU-bound

## Performance Comparison

Measured on RTX 4090 with ROSbot XL in Gazebo:

| Metric | CPU Only (Mesa) | GPU Accelerated (NVIDIA) |
|--------|-----------------|--------------------------|
| GUI CPU Usage | ~1500% | ~150% |
| GPU Memory Used | 0 MB | 2100 MB |
| GPU Utilization | 0% | 38-45% |
| Frame Rate | 5-15 FPS | 60+ FPS |
| Startup Time | 30-45s | 15-20s |

## How It Works

### GPU Detection

The container automatically detects GPU availability and configures rendering:

```bash
if [ -d /dev/dri ] && [ -n "$(ls -A /dev/dri 2>/dev/null)" ]; then
    # GPU detected → use hardware acceleration
    export GZ_RENDERING_ENGINE=ogre2
else
    # No GPU → use software rendering
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    export GZ_RENDERING_ENGINE=ogre
fi
```

### Device Passthrough

Docker containers get access to GPU via:

- `NVIDIA_VISIBLE_DEVICES` - Which GPUs to use
- `NVIDIA_DRIVER_CAPABILITIES` - Graphics, compute, utility access
- `/dev/dri:/dev/dri` - Direct Rendering Infrastructure device

## Troubleshooting

### GPU Not Being Used

Check if container has GPU access:

```bash
docker exec sim-gazebo-rosbotxl bash -c "env | grep NVIDIA"
```

Should show:
```
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
```

### Container Fails to Start with GPU Errors

1. Verify NVIDIA Container Toolkit is installed:
   ```bash
   which nvidia-container-runtime
   ```

2. Restart Docker:
   ```bash
   sudo systemctl restart docker
   ```

3. If still failing, disable GPU:
   ```bash
   export NVIDIA_VISIBLE_DEVICES=void
   ./run.sh sim rosbotxl gazebo
   ```

### Simulation Performance Issues

Check what's using resources:

```bash
# Monitor container resource usage
docker stats

# Check GPU usage
nvidia-smi dmon

# See if simulation is GPU-accelerated
docker logs sim-gazebo-rosbotxl 2>&1 | grep -i "rendering\|gpu\|ogre"
```

### X11/Graphics Issues

```bash
# Check if GPU libraries are mounted
docker exec sim-gazebo-rosbotxl ldconfig -p | grep nvidia

# Verify rendering is working
docker exec sim-gazebo-rosbotxl glxinfo | grep "direct rendering"
```

## Configuration Details

### Environment Variables

In `docker-compose.yml`:

```yaml
environment:
  NVIDIA_VISIBLE_DEVICES: ${NVIDIA_VISIBLE_DEVICES:-all}
  NVIDIA_DRIVER_CAPABILITIES: ${NVIDIA_DRIVER_CAPABILITIES:-graphics,utility,compute}
  GZ_RENDERING_ENGINE: ogre2  # For GPU, ogre for CPU
```

### Device Mapping

```yaml
devices:
  - /dev/dri:/dev/dri  # GPU device access
```

## Performance Tips

### For Best Performance

1. **Use Gazebo** - Better optimized than Webots/O3DE
2. **Close other GPU-using apps** - Video players, Chrome tabs with WebGL
3. **Keep driver updated** - Run `sudo apt install -y --only-upgrade nvidia-driver-*`
4. **Monitor temperature** - Use `nvidia-smi dmon` to check GPU temp

### For Lower-End GPUs

1. **Reduce Gazebo graphics quality** - Check Gazebo settings
2. **Close unnecessary GUI windows** - RViz2, Gazebo clients
3. **Allocate more CPU cores to containers** - Helps with loading
4. **Consider CPU-only if <2GB VRAM** - Too much overhead

## Advanced Configuration

### Using Multiple GPUs

To use GPU 1 instead of GPU 0:

```bash
export NVIDIA_VISIBLE_DEVICES=1
./run.sh sim rosbotxl gazebo
```

### Disable GPU Temporarily

```bash
export NVIDIA_VISIBLE_DEVICES=void
./run.sh sim rosbotxl gazebo
# Simulations will use CPU rendering
```

### Monitor GPU

In real-time:

```bash
# Update every 1 second
nvidia-smi dmon -s puc
```

## Verification Checklist

- [ ] `nvidia-smi` works on host
- [ ] `docker run --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi` works
- [ ] Simulation starts with GPU container
- [ ] Gazebo GUI appears and runs smoothly (60+ FPS)
- [ ] `docker stats` shows reasonable CPU usage (~150% for GUI)

## Support

If GPU setup still isn't working:

- 📧 Email TA: omotoye.adekoya@edu.unige.it
- 💬 Open GitHub Issue with:
  - Output of `nvidia-smi`
  - Output of `docker run --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi`
  - Full logs: `docker compose logs sim-gazebo-rosbotxl`
