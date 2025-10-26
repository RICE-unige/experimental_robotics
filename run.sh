#!/bin/bash

# Experimental Robotics - Unified Launcher
# Handles both development containers and robot simulations

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Platform detection
IS_WSL=false
if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    IS_WSL=true
elif [ -n "${WSL_DISTRO_NAME:-}" ]; then
    IS_WSL=true
fi

# Helpers
print_header() {
    echo -e "${BLUE}=========================================="
    echo "Experimental Robotics Environment"
    echo -e "==========================================${NC}"
}

print_error() {
    echo -e "${RED}ERROR: $1${NC}" >&2
}

print_success() {
    echo -e "${GREEN}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}$1${NC}"
}

print_info() {
    echo -e "${BLUE}$1${NC}"
}

check_docker() {
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker is not running. Please start Docker first."
        exit 1
    fi
}

enable_x11() {
    if $IS_WSL; then
        print_info "WSL environment detected - relying on WSLg for X11 access (skipping xhost)"
    else
        print_info "Enabling X11 forwarding for GUI applications..."
        xhost +local:docker >/dev/null 2>&1 || print_warning "Warning: Could not enable X11 forwarding"
    fi
}

show_help() {
    print_header
    echo ""
    echo "DESCRIPTION"
    echo "  Unified launcher for ROS2 development containers and robot simulations."
    echo "  Manages Docker containers for the Experimental Robotics course."
    echo ""
    echo "USAGE"
    echo "  $0 <command> [arguments] [options]"
    echo ""
    echo "COMMANDS"
    echo ""
    echo "  dev <distro> [--build]"
    echo "      Start a ROS2 development container for coding and building packages."
    echo ""
    echo "      Arguments:"
    echo "        <distro>     ROS2 distribution: humble or jazzy (default: jazzy)"
    echo ""
    echo "      Options:"
    echo "        --build      Rebuild the container before starting"
    echo ""
    echo "      Examples:"
    echo "        $0 dev jazzy                    # Start Jazzy dev container"
    echo "        $0 dev humble                   # Start Humble dev container"
    echo "        $0 dev jazzy --build            # Rebuild and start Jazzy"
    echo ""
    echo "      After starting:"
    echo "        docker compose exec dev_jazzy bash    # Enter the container"
    echo ""
    echo "  sim <robot> <simulator> [options]"
    echo "      Start a robot simulation with optional SLAM/Nav2 autonomy stack."
    echo ""
    echo "      Arguments:"
    echo "        <robot>      Robot platform: rosbot2r, rosbotxl, rosbotxl-manip, panther"
    echo "        <simulator>  Simulator engine: gazebo, webots, o3de"
    echo ""
    echo "      Options:"
    echo "        --dev <distro>    Also start dev container (humble or jazzy)"
    echo "        --build           Rebuild containers before starting"
    echo ""
    echo "      Examples:"
    echo "        $0 sim rosbotxl gazebo                    # Basic simulation with built-in RViz2"
    echo "        $0 sim rosbotxl-manip gazebo             # XL with manipulator (Gazebo + MoveIt2)"
    echo "        $0 sim rosbot2r gazebo --dev humble       # Simulation + dev container"
    echo "        $0 sim rosbotxl gazebo --dev jazzy        # Sim + dev container"
    echo ""
    echo "      Robot/Simulator compatibility:"
    echo "        rosbot2r:  gazebo, webots"
    echo "        rosbotxl:        gazebo, webots, o3de"
    echo "        rosbotxl-manip:  gazebo only"
    echo "        panther:   gazebo only"
    echo ""
    echo "      After starting:"
    echo "        # RViz2 opens automatically in the simulation container"
    echo "        # Manual control:"
    echo "        docker exec -it sim-gazebo-rosbotxl bash"
    echo "        ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo ""
    echo "  stop [target]"
    echo "      Stop running containers."
    echo ""
    echo "      Arguments:"
    echo "        [target]     What to stop: dev, sim, or all (default: all)"
    echo ""
    echo "      Examples:"
    echo "        $0 stop                         # Stop all containers"
    echo "        $0 stop sim                     # Stop only simulation containers"
    echo "        $0 stop dev                     # Stop only dev containers"
    echo ""
    echo "  status"
    echo "      Show all running containers and their status."
    echo ""
    echo "      Example:"
    echo "        $0 status"
    echo ""
    echo "  clean"
    echo "      Remove all containers, volumes, and locally built images."
    echo "      ⚠️  WARNING: This deletes all data in containers!"
    echo ""
    echo "      Example:"
    echo "        $0 clean                        # Prompts for confirmation"
    echo ""
    echo "  help"
    echo "      Show this help message."
    echo ""
    echo "COMMON WORKFLOWS"
    echo ""
    echo "  1. Start development environment:"
    echo "       $0 dev jazzy"
    echo "       docker compose exec dev_jazzy bash"
    echo ""
    echo "  2. Test a basic simulation:"
    echo "       $0 sim rosbotxl gazebo"
    echo "       # RViz2 opens automatically"
    echo ""
    echo "  3. Full development + simulation setup:"
    echo "       $0 sim rosbotxl gazebo --slam --dev humble"
    echo "       docker compose exec dev_humble bash       # Code in here"
    echo "       docker exec -it sim-gazebo-rosbotxl bash  # Control robot here"
    echo ""
    echo "  4. Stop everything when done:"
    echo "       $0 stop all"
    echo ""
    echo "TIPS"
    echo "  • Run 'xhost +local:docker' before starting simulations (done automatically)"
    echo "  • Use '$0 status' to see what's currently running"
    echo "  • Dev containers persist - stop simulations with '$0 stop sim'"
    echo "  • View logs with: docker compose logs -f"
    echo "  • All containers use host networking for easy ROS2 communication"
    echo ""
    echo "For more help: Check the README.md or create an issue on GitHub"
    echo ""
}

# Command: dev
cmd_dev() {
    local DISTRO="$1"
    shift || true
    local BUILD_FLAG=""
    
    # Parse options
    while [[ $# -gt 0 ]]; do
        case $1 in
            --build)
                BUILD_FLAG="--build"
                shift
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done
    
    # Validate distro
    if [ -z "$DISTRO" ]; then
        DISTRO="jazzy"  # Default
    fi
    
    case "$DISTRO" in
        humble|jazzy)
            PROFILE="dev-$DISTRO"
            ;;
        *)
            print_error "Invalid distro: $DISTRO"
            echo "Valid distros: humble, jazzy"
            exit 1
            ;;
    esac
    
    print_header
    echo ""
    print_info "Starting ROS2 $DISTRO development container..."
    if $IS_WSL; then
        print_warning "Running under WSL - Docker Desktop (WSL backend) is required. GPU access relies on WSLg; /dev/dri mappings will be ignored."
    fi
    
    check_docker
    enable_x11
    
    docker compose --profile "$PROFILE" up -d $BUILD_FLAG
    
    echo ""
    print_success "✓ Dev container started!"
    echo ""
    print_info "Access container:"
    echo "  docker compose exec dev_$DISTRO bash"
    echo ""
}

# Command: sim
cmd_sim() {
    local ROBOT="$1"
    local SIMULATOR="${2:-gazebo}"
    shift 2 || true
    
    local DEV_DISTRO=""
    local BUILD_FLAG=""
    
    # Parse options
    while [[ $# -gt 0 ]]; do
        case $1 in
            --dev)
                DEV_DISTRO="$2"
                shift 2
                ;;
            --build)
                BUILD_FLAG="--build"
                shift
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done
    
    # Validate inputs
    if [ -z "$ROBOT" ]; then
        print_error "No robot specified!"
        echo "Available robots: rosbot2r, rosbotxl, panther"
        exit 1
    fi
    
    
    # Validate robot/simulator combo
    local BASE_PROFILE="${SIMULATOR}-${ROBOT}"
    
    case "$BASE_PROFILE" in
        gazebo-rosbot2r|gazebo-rosbotxl|gazebo-rosbotxl-manip|gazebo-panther)
            ;;
        webots-rosbot2r|webots-rosbotxl)
            ;;
        o3de-rosbotxl)
            ;;
        webots-panther)
            print_error "Webots not available for Panther"
            echo "Use: gazebo"
            exit 1
            ;;
        o3de-rosbot2r|o3de-panther)
            print_error "O3DE only available for ROSbot XL"
            exit 1
            ;;
        *)
            print_error "Invalid robot/simulator: $ROBOT + $SIMULATOR"
            exit 1
            ;;
    esac
    

    # Build profile list (base only)
    local PROFILE_FLAGS=(--profile "$BASE_PROFILE")
    
    # Add dev profile if requested
    if [ -n "$DEV_DISTRO" ]; then
        case "$DEV_DISTRO" in
            humble|jazzy)
                PROFILE_FLAGS+=(--profile "dev-$DEV_DISTRO")
                ;;
            *)
                print_error "Invalid dev distro: $DEV_DISTRO"
                echo "Valid distros: humble, jazzy"
                exit 1
                ;;
        esac
    fi
    
    # Launch
    print_header
    echo ""
    print_info "Configuration:"
    echo "  Robot:     $ROBOT"
    echo "  Simulator: $SIMULATOR"
    [ -n "$DEV_DISTRO" ] && echo "  Dev:       $DEV_DISTRO" || echo "  Dev:       No"
    echo ""
    if $IS_WSL; then
        print_warning "WSL detected: Gazebo/RViz will use WSLg graphics. For best results keep only one heavy GUI per container and verify GPU support in Docker Desktop."
    fi
    
    check_docker
    enable_x11
    
    print_info "Launching simulation..."
    docker compose "${PROFILE_FLAGS[@]}" up -d ${BUILD_FLAG}
    
    echo ""
    print_success "✓ Simulation launched!"
    echo ""
    print_info "Running containers:"
    docker compose ps --filter "status=running" | grep -v "NAME"
    echo ""
    
    if [ "$SIMULATOR" = "o3de" ]; then
        local VNC_PORT="${O3DE_VNC_PORT:-5901}"
        print_info "O3DE Editor is available via VNC on localhost:${VNC_PORT}"
        echo "  Use any VNC client (e.g., 'vncviewer localhost:${VNC_PORT}')"
        echo "  Default resolution: ${O3DE_VNC_RESOLUTION:-1920x1080}"
        echo ""
    fi
    
    print_info "Useful commands:"
    echo "  View logs:    docker compose logs -f"
    echo "  Stop:         $0 stop sim"
    echo "  Enter sim:    docker compose exec sim-${SIMULATOR}-${ROBOT} bash"
    if [ -n "$DEV_DISTRO" ]; then
        echo "  Enter dev:    docker compose exec dev_${DEV_DISTRO} bash"
    fi
    echo ""
}

# Command: stop
cmd_stop() {
    local TARGET="${1:-all}"
    
    print_header
    print_info "Stopping $TARGET containers..."
    echo ""
    
    case "$TARGET" in
        dev)
            docker compose --profile dev-humble --profile dev-jazzy down --remove-orphans 2>/dev/null
            ;;
        sim)
            docker compose --profile gazebo-rosbot2r --profile gazebo-rosbotxl --profile gazebo-rosbotxl-manip --profile gazebo-panther \
                           --profile webots-rosbot2r --profile webots-rosbotxl \
                           --profile o3de-rosbotxl \
                           down --remove-orphans 2>/dev/null
            ;;
        all)
            # Stop all profiles explicitly
            docker compose --profile dev-humble --profile dev-jazzy \
                           --profile gazebo-rosbot2r --profile gazebo-rosbotxl --profile gazebo-rosbotxl-manip --profile gazebo-panther \
                           --profile webots-rosbot2r --profile webots-rosbotxl \
                           --profile o3de-rosbotxl \
                           down --remove-orphans 2>/dev/null
            ;;
        *)
            print_error "Invalid target: $TARGET"
            echo "Valid targets: dev, sim, all"
            exit 1
            ;;
    esac
    
    print_success "✓ Stopped $TARGET containers"
}

# Command: clean
cmd_clean() {
    print_header
    print_warning "This will remove all containers, networks, and volumes!"
    echo -n "Are you sure? (y/N) "
    read -r response
    
    if [[ "$response" =~ ^[Yy]$ ]]; then
        print_info "Cleaning up..."
        docker compose down -v --rmi local --remove-orphans 2>/dev/null || true
        print_success "✓ Cleanup complete"
    else
        print_info "Cancelled"
    fi
}

# Command: status
cmd_status() {
    print_header
    echo ""
    print_info "Running containers:"
    echo ""
    docker compose ps
    echo ""
}

# Main
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

COMMAND="$1"
shift

case "$COMMAND" in
    dev)
        cmd_dev "$@"
        ;;
    sim)
        cmd_sim "$@"
        ;;
    stop)
        cmd_stop "$@"
        ;;
    clean)
        cmd_clean
        ;;
    status)
        cmd_status
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        echo "Use '$0 help' for usage information"
        exit 1
        ;;
esac
