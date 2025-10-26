#!/bin/bash

# Experimental Robotics - Container Connection Script
# Easily connect to running dev or simulation containers

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}=========================================="
    echo "Connect to Container"
    echo -e "==========================================${NC}"
}

print_error() {
    echo -e "${RED}ERROR: $1${NC}" >&2
}

print_success() {
    echo -e "${GREEN}$1${NC}"
}

print_info() {
    echo -e "${BLUE}$1${NC}"
}

show_help() {
    print_header
    echo ""
    echo "DESCRIPTION"
    echo "  Connect to a running development or simulation container."
    echo ""
    echo "USAGE"
    echo "  $0 <target> [shell]"
    echo ""
    echo "TARGETS"
    echo "  dev [distro]        Connect to dev container (jazzy or humble)"
    echo "  sim [robot]         Connect to simulation container"
    echo "  list                Show all running containers"
    echo ""
    echo "OPTIONS"
    echo "  [shell]             Shell to use: bash (default), zsh, sh"
    echo ""
    echo "EXAMPLES"
    echo "  $0 dev              # Connect to dev container (auto-detect distro)"
    echo "  $0 dev jazzy        # Connect to Jazzy dev container"
    echo "  $0 dev humble       # Connect to Humble dev container"
    echo ""
    echo "  $0 sim              # Connect to sim container (auto-detect)"
    echo "  $0 sim rosbotxl     # Connect to ROSbot XL simulation"
    echo "  $0 sim rosbot2r           # Connect to ROSbot 2R simulation"
    echo "  $0 sim rosbotxl-manip     # Connect to ROSbot XL manip simulation"
    echo ""
    echo "  $0 list             # List all running containers"
    echo ""
    echo "NOTES"
    echo "  • Containers must be running first (use ./run.sh)"
    echo "  • If only one container is running, it's auto-selected"
    echo "  • If both dev containers are running, you'll be prompted to choose"
    echo "  • Exit the container with 'exit' or Ctrl+D"
    echo ""
}

# Get running containers
get_running_containers() {
    docker compose ps --services --filter "status=running" 2>/dev/null
}

# Find dev container
find_dev_container() {
    local DISTRO="$1"
    local CONTAINERS
    CONTAINERS=$(get_running_containers)
    
    if [ -n "$DISTRO" ]; then
        # Specific distro requested
        if echo "$CONTAINERS" | grep -q "^dev_$DISTRO$"; then
            echo "dev_$DISTRO"
            return 0
        else
            return 1
        fi
    else
        # Auto-detect
        local JAZZY_RUNNING=false
        local HUMBLE_RUNNING=false
        
        echo "$CONTAINERS" | grep -q "^dev_jazzy$" && JAZZY_RUNNING=true
        echo "$CONTAINERS" | grep -q "^dev_humble$" && HUMBLE_RUNNING=true
        
        # If both are running, let user choose
        if $JAZZY_RUNNING && $HUMBLE_RUNNING; then
            echo "MULTIPLE" # Special marker
            return 0
        elif $JAZZY_RUNNING; then
            echo "dev_jazzy"
            return 0
        elif $HUMBLE_RUNNING; then
            echo "dev_humble"
            return 0
        else
            return 1
        fi
    fi
}

# Find sim container
find_sim_container() {
    local ROBOT="$1"
    local CONTAINERS
    CONTAINERS=$(docker compose ps --format "{{.Service}}" --filter "status=running" 2>/dev/null)
    
    if [ -n "$ROBOT" ]; then
        # Specific robot requested
        local FOUND
        FOUND=$(echo "$CONTAINERS" | grep "^sim-.*-$ROBOT$" | head -1)
        if [ -n "$FOUND" ]; then
            echo "$FOUND"
            return 0
        else
            return 1
        fi
    else
        # Auto-detect: any sim container
        local FOUND
        FOUND=$(echo "$CONTAINERS" | grep "^sim-" | head -1)
        if [ -n "$FOUND" ]; then
            echo "$FOUND"
            return 0
        else
            return 1
        fi
    fi
}

# List containers
cmd_list() {
    print_header
    echo ""
    print_info "Running containers:"
    echo ""
    docker compose ps
    echo ""
    print_info "Connect with:"
    echo "  $0 dev       # Connect to dev container"
    echo "  $0 sim       # Connect to simulation container"
    echo ""
}

# Connect to dev
cmd_dev() {
    local DISTRO="$1"
    local SHELL="${2:-bash}"
    
    print_header
    echo ""
    
    if ! CONTAINER=$(find_dev_container "$DISTRO"); then
        if [ -n "$DISTRO" ]; then
            print_error "Dev container for '$DISTRO' is not running"
            echo ""
            echo "Start it with: ./run.sh dev $DISTRO"
        else
            print_error "No dev containers are running"
            echo ""
            echo "Start one with: ./run.sh dev jazzy"
            echo "            or: ./run.sh dev humble"
        fi
        echo ""
        exit 1
    fi
    
    # Handle case where both containers are running
    if [ "$CONTAINER" = "MULTIPLE" ]; then
        print_info "Both dev containers are running. Which one?"
        echo ""
        echo "  1) Jazzy (ROS2 Jazzy)"
        echo "  2) Humble (ROS2 Humble)"
        echo ""
        while true; do
            read -r -p "Enter choice (1 or 2): " choice
            case "${choice,,}" in
                1|jazzy)
                    CONTAINER="dev_jazzy"
                    break
                    ;;
                2|humble)
                    CONTAINER="dev_humble"
                    break
                    ;;
                *)
                    print_warning "Please enter 1 for Jazzy or 2 for Humble."
                    ;;
            esac
        done
        echo ""
    fi
    
    print_info "Connecting to $CONTAINER..."
    echo ""
    print_success "✓ Connected! (exit with 'exit' or Ctrl+D)"
    echo ""
    
    docker compose exec "$CONTAINER" "$SHELL"
}

# Connect to sim
cmd_sim() {
    local ROBOT="$1"
    local SHELL="${2:-bash}"
    
    print_header
    echo ""
    
    if ! CONTAINER=$(find_sim_container "$ROBOT"); then
        if [ -n "$ROBOT" ]; then
            print_error "Simulation container for '$ROBOT' is not running"
        else
            print_error "No simulation containers are running"
        fi
        echo ""
        echo "Start a simulation with: ./run.sh sim rosbotxl gazebo"
        echo ""
        exit 1
    fi
    
    print_info "Connecting to $CONTAINER..."
    echo ""
    print_success "✓ Connected! (exit with 'exit' or Ctrl+D)"
    echo ""
    
    docker compose exec "$CONTAINER" "$SHELL"
}

# Main
if [ $# -eq 0 ]; then
    # No arguments: show list and let user choose
    cmd_list
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
    list|ls)
        cmd_list
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
