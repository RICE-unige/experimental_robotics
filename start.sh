#!/bin/bash

# Experimental Robotics Environment Startup Script

set -e

echo "=========================================="
echo "Experimental Robotics Docker Environment"
echo "=========================================="

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    echo "ERROR: Docker is not running. Please start Docker first."
    exit 1
fi

# Enable X11 forwarding for GUI applications
echo "Enabling X11 forwarding for GUI applications..."
xhost +local:docker >/dev/null 2>&1 || echo "Warning: Could not enable X11 forwarding"

# Parse command line arguments
PROFILE="jazzy"  # Default profile
BUILD_FLAG=""
CLEAN_FLAG=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --build)
            BUILD_FLAG="--build"
            shift
            ;;
        --clean)
            CLEAN_FLAG="1"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --profile PROFILE   Choose environment: jazzy (default), humble, all"
            echo "  --build             Force rebuild of containers"
            echo "  --clean             Stop containers, remove volumes and local images for this project"
            echo "  --help, -h          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                    # Start ROS2 Jazzy"
            echo "  $0 --profile humble  # Start ROS2 Humble"
            echo "  $0 --profile all      # Start both Humble and Jazzy"
            echo ""
            echo "📚 For detailed documentation and tutorials:"
            echo "   https://rice-unige.gitbook.io/experimental-robotics/"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Validate profile
case $PROFILE in
    humble|jazzy|all)
        ;;
    *)
        echo "ERROR: Invalid profile '$PROFILE'"
        echo "Valid profiles: humble, jazzy, all"
        exit 1
        ;;
esac

# Clean mode
if [ -n "$CLEAN_FLAG" ]; then
    echo "Cleaning environment (containers, volumes, local images for this compose project)..."
    docker compose down -v --rmi local --remove-orphans || true
    exit 0
fi

# Start the environment
echo "Starting $PROFILE environment..."
echo "Profile: $PROFILE"

if [ "$PROFILE" = "all" ]; then
    echo "Starting both ROS2 Humble and Jazzy containers..."
    echo "Warning: running both containers uses significant CPU/RAM/GPU; prefer a single profile on low-memory systems."
    docker compose --profile all up -d $BUILD_FLAG
    echo ""
    echo "Both environments started successfully!"
    echo ""
    echo "Access containers with:"
    echo "  docker compose exec ros2_humble bash    # For ROS2 Humble"
    echo "  docker compose exec ros2_jazzy bash     # For ROS2 Jazzy"
else
    echo "Starting ROS2 $PROFILE environment..."
    docker compose --profile $PROFILE up -d $BUILD_FLAG
    echo ""
    echo "ROS2 $PROFILE environment started successfully!"
    echo ""
    echo "Access container with:"
    echo "  docker compose exec ros2_$PROFILE bash"
fi

echo ""
echo "=========================================="
echo "Environment Status:"
docker compose ps
echo "=========================================="
echo ""
echo "To stop the environment:"
echo "  docker compose down"
echo ""
echo "To view logs:"
echo "  docker compose logs -f"
echo ""
echo "Happy robotics development! 🤖"