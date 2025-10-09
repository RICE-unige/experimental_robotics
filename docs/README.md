---
description: Course hub for the Experimental Robotics course (RICELab, University of Genoa)
layout:
  width: wide
  title:
    visible: true
  description:
    visible: true
  tableOfContents:
    visible: true
  outline:
    visible: true
  pagination:
    visible: true
  metadata:
    visible: true
---

# Experimental Robotics

> [!WARNING]
> This documentation is a work in progress. Students will be informed when the assignments and projects are ready.

Welcome to the Experimental Robotics course documentation. This space accompanies the Docker-based development environment in the repository and provides the setup steps, labs, assignments, ROS 2 notes, and project guidelines.

## What you’ll find here

- Step-by-step environment setup (ROS 2 Jazzy and Humble)
- Labs and assignments with tips and references
- ROS 2 basics and quick commands for development
- Simulation, SLAM, and Navigation notes
- Final project requirements and evaluation rubric

## Quick start

Run these from your host machine to start a course container and access ROS 2:

```bash
# Clone the repository
git clone https://github.com/RICE-unige/experimental_robotics.git
cd experimental_robotics

# Enable GUI applications on Linux
xhost +local:docker

# Start environment (choose one)
./start.sh                    # ROS2 Jazzy (default)
./start.sh --profile humble   # ROS2 Humble
./start.sh --profile all      # Both distributions (same domain)

# Access a running container shell
docker compose exec ros2_jazzy bash
# or
docker compose exec ros2_humble bash
```

> [!TIP]
> If you have an NVIDIA GPU and drivers installed, the setup supports GPU acceleration via the NVIDIA Container Toolkit.

## Available environments

- jazzy — ROS 2 Jazzy (default, recommended for assignments)
- humble — ROS 2 Humble LTS (stable, for compatibility testing)
- all — Run Jazzy and Humble together on the same domain

## Support

- Teaching Assistant: Omotoye Adekoya — omotoye.adekoya@edu.unige.it
- Course Professor: Prof. Carmine Recchiuto — carmine.recchiuto@unige.it
- For technical issues, open a GitHub issue in the repository.

## Next steps

- Complete the [Setup](setup/README.md)
- Review [ROS 2 Basics](ros2/README.md)
- Explore [Simulation](simulation/README.md)
- Check [Labs](labs/README.md) and [Assignments](assignments/README.md)
- See the [Final Project](final-project/README.md) brief
- Browse [Resources](resources.md) and the [FAQ](faq.md)
