# Experimental Robotics Docker Environment

Docker-based development environment for the Experimental Robotics course at University of Genoa, featuring ROS2 Humble and ROS2 Jazzy with navigation, simulation, and development tools.

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

**University of Genoa - RICELab** 
