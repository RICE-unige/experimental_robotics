# Experimental Robotics Course

[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![GitBook](https://img.shields.io/static/v1?message=Documented%20on%20GitBook&logo=gitbook&logoColor=ffffff&label=%20&labelColor=5c5c5c&color=3F89A1)](https://rice-unige.gitbook.io/experimental-robotics/)
[![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)


<p align="center">
  <img src="imgs/ricelab_logo.jpg" alt="RICELab" height="30" style="margin-right: 16px;">
  <img src="imgs/university_of_genoa_logo.png" alt="University of Genoa" height="30">
</p>

> [!WARNING]  
> This repository is actively in development and is **not yet ready for use**. Students will be informed by the professor when the repository, projects, and assignments are ready for the course.


Docker-based development environment for the Experimental Robotics course at University of Genoa, featuring ROS2 Humble and ROS2 Jazzy with slam, navigation, simulation, and development tools.

## Prerequisites

- Docker Engine (>= 20.10) and Docker Compose (>= 2.0)
- X11 forwarding for GUI applications
- NVIDIA Docker runtime (recommended for GPU acceleration)

## Quick Start

```bash
# Clone the repository
git clone https://github.com/RICE-unige/experimental_robotics.git
cd experimental_robotics

# Enable GUI applications
xhost +local:docker

# Start environment (choose one):
./start.sh                    # ROS2 Jazzy (default)
./start.sh --profile humble   # ROS2 Humble  
./start.sh --profile all      # Both distributions (same domain)

# Access container
docker compose exec ros2_jazzy bash
docker compose exec ros2_humble bash
```

## 📚 Documentation

For detailed documentation, tutorials, and assignments, visit our **GitBook page**:

📝 **[Experimental Robotics Course Documentation](https://rice-unige.gitbook.io/experimental-robotics/)**

## Available Environments

- **`jazzy`**: ROS2 Jazzy (default, latest features, recommended for assignments)
- **`humble`**: ROS2 Humble LTS (stable, for compatibility testing)
- **`all`**: Both distributions on same domain for cross-compatibility communication

## 🆘 Support

For questions and support:

**Teaching Assistant:**
- **Omotoye Adekoya** - [omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it)

**Course Professor:**
- **Prof. Carmine Recchiuto** - [carmine.recchiuto@unige.it](mailto:carmine.recchiuto@unige.it)

You can also create an issue in this repository for technical problems.

---

© 2025 **University of Genoa - RICELab**
