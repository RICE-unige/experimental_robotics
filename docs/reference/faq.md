# FAQ

Frequently asked questions about the Experimental Robotics course and development environment.

## Setup & Environment

### Q: Do I need an NVIDIA GPU?

**A:** No, it's optional. The environment automatically detects your GPU and enables acceleration if available. Without a GPU, simulations will use software rendering (Mesa), which is slower but still functional.

### Q: How much disk space do I need?

**A:** Minimum 20GB, but 40GB recommended if you want multiple simulators (Gazebo, Webots, O3DE) and development environments (Jazzy, Humble).

### Q: My X11 forwarding isn't working. What do I do?

**A:** Try:

```bash
# Enable X11 for docker
xhost +local:docker

# Check your DISPLAY variable
echo $DISPLAY

# If empty or :0 doesn't work, try :1
export DISPLAY=:1
```

### Q: Can I use this on Windows/macOS?

**A:** Docker runs on all three platforms, but X11 forwarding (needed for GUIs) requires extra setup on Windows/macOS. Linux is recommended for the smoothest experience.

## Docker & Containers

### Q: How do I enter a running container?

**A:** Use the quick shortcut:

```bash
./connect.sh dev          # Auto-detect dev container
./connect.sh dev jazzy    # Specific container
./connect.sh sim          # Connect to simulation
```

Or manually:

```bash
docker compose exec dev_jazzy bash
```

### Q: How do I check what containers are running?

**A:** 

```bash
./connect.sh list        # Quick view
docker compose ps        # Full details
```

### Q: I need to stop all containers. How?

**A:**

```bash
./run.sh stop all        # Stop everything
docker compose down      # Remove and stop
```

### Q: Why is my simulation so slow?

**A:** Common causes:

1. **No GPU** - Running on CPU rendering. Enable NVIDIA Container Toolkit if you have a GPU.
2. **Not enough RAM** - Close other applications or increase Docker's allocated memory.
3. **Old hardware** - Some old CPUs can't keep up with simulation.

Try:

```bash
# See what's using resources
docker stats
```

## ROS 2 & Development

### Q: How do I run a ROS 2 node?

**A:**

```bash
# Connect to container first
./connect.sh dev

# Then run
ros2 run package_name node_name
```

### Q: How do I see what's happening on a topic?

**A:**

```bash
ros2 topic list
ros2 topic echo /topic_name
```

### Q: How do I debug a crashing node?

**A:**

```bash
# Run with logging
ros2 launch package launch.py --log-level debug

# Or manually run with output
ros2 run package node --ros-args --log-level debug
```

### Q: Can I create my own package?

**A:** Yes! Inside the container:

```bash
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python my_package
```

## Simulation

### Q: Why won't Gazebo start?

**A:**

```bash
# Check if X11 is set up
echo $DISPLAY

# Try restarting with verbose output
./run.sh sim rosbotxl gazebo --build
docker compose logs -f sim-gazebo-rosbotxl
```

### Q: What's the difference between Gazebo, Webots, and O3DE?

**A:**

- **Gazebo** - Physics-accurate, widely used in ROS community
- **Webots** - Professional simulator, photorealistic rendering
- **O3DE** - Advanced graphics, experimental support (display issues)

Use Gazebo for most tasks.

### Q: Can I drive the robot manually?

**A:** Yes, if simulation is running:

```bash
docker compose exec sim-gazebo-rosbotxl bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Then use arrow keys to drive.

### Q: How do I set a navigation goal?

**A:** If running with Nav2 (`--nav` flag):

1. In RViz2, click "2D Goal Pose" button
2. Click on map where you want the robot to go
3. Drag arrow to point robot in desired direction

## Assignments & Projects

### Q: Do I really need to write everything from scratch?

**A:** Yes for assignments, but you can:

- Use ROS 2 packages and examples
- Follow tutorials and adapt code
- Use proper attribution and citations

See [CODE_OF_CONDUCT.md](../../CODE_OF_CONDUCT.md) for guidelines.

### Q: Can I use ChatGPT/Copilot?

**A:** Yes, but:

1. **You must disclose it** - Note that you used AI tools
2. **You must understand it** - Be able to explain every line
3. **You must test it** - Verify it works in your environment
4. **Don't submit without review** - AI-generated code needs review before submission

### Q: What if I'm stuck on an assignment?

**A:** 

1. Check course materials and examples
2. Review [Resources](resources.md) section
3. Check GitHub Issues for similar problems
4. Email TA or ask in office hours (don't share code, just ask concepts)

## Contributing & GitHub

### Q: How do I submit a bug report?

**A:** 

1. Check if issue already exists
2. Use GitHub's "New Issue" button
3. Include: OS, Docker version, ROS distro, error message, steps to reproduce
4. Attach logs if relevant

### Q: Can I contribute fixes?

**A:** Yes! We welcome contributions:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request
5. Describe what you fixed and how to test it

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for details.

## Troubleshooting

### Q: I see "permission denied" errors. What now?

**A:** Ensure Docker daemon is running:

```bash
sudo systemctl start docker
sudo systemctl status docker
```

Or add your user to docker group:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

### Q: Container exited with code 137 or 139?

**A:** Usually memory/resource issues:

```bash
# Check resource usage
docker stats

# Increase Docker memory limit in Docker Desktop Settings
# Or reduce other running applications
```

### Q: Build is failing with "Could not resolve host"

**A:** Network issue. Try:

```bash
docker system prune --all
docker compose build --no-cache
```

## Still Have Questions?

- 📧 Email TA: omotoye.adekoya@edu.unige.it
- 🎓 Professor: carmine.recchiuto@unige.it
- 💬 GitHub Issues: https://github.com/RICE-unige/experimental_robotics/issues
- 📚 Check [Resources](resources.md) for tutorials and docs
