#!/bin/bash

# MkDocs helper script for building and serving documentation

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Help function
show_help() {
    cat << EOF
${BLUE}MkDocs Documentation Helper${NC}

${GREEN}Usage:${NC}
    ./docs.sh [COMMAND] [OPTIONS]

${GREEN}Commands:${NC}
    serve       Build and serve documentation locally (default)
    build       Build documentation without serving
    clean       Remove built documentation
    install     Install MkDocs dependencies
    help        Show this help message

${GREEN}Examples:${NC}
    ./docs.sh               # Build and serve at http://localhost:8000
    ./docs.sh serve         # Same as above
    ./docs.sh build         # Build only, don't serve
    ./docs.sh clean         # Clean build artifacts
    ./docs.sh install       # Install dependencies

${GREEN}Options:${NC}
    For serve: mkdocs serve options are passed through
    Example: ./docs.sh serve --dev-addr=0.0.0.0:8080

${GREEN}Note:${NC}
    - Requires Python 3.7+
    - Install dependencies with: ./docs.sh install
    - Access docs at http://localhost:8000 (default)
EOF
}

# Check if Python is available
check_python() {
    if ! command -v python3 &> /dev/null; then
        echo -e "${RED}Error: Python 3 is not installed${NC}"
        echo "Please install Python 3.7 or later"
        exit 1
    fi
}

# Install dependencies
install_deps() {
    echo -e "${BLUE}Installing MkDocs dependencies...${NC}"
    check_python
    
    if [ ! -f "requirements.txt" ]; then
        echo -e "${RED}Error: requirements.txt not found${NC}"
        exit 1
    fi
    
    python3 -m pip install -r requirements.txt
    echo -e "${GREEN}✓ Dependencies installed successfully${NC}"
}

# Build documentation
build_docs() {
    echo -e "${BLUE}Building documentation...${NC}"
    check_python
    
    # Check if mkdocs is installed
    if ! python3 -m mkdocs --version &> /dev/null; then
        echo -e "${RED}Error: mkdocs is not installed${NC}"
        echo "Run: ./docs.sh install"
        exit 1
    fi
    
    python3 -m mkdocs build
    echo -e "${GREEN}✓ Documentation built successfully${NC}"
    echo -e "  Output: ${SCRIPT_DIR}/site/"
}

# Serve documentation
serve_docs() {
    echo -e "${BLUE}Serving documentation...${NC}"
    check_python
    
    # Check if mkdocs is installed
    if ! python3 -m mkdocs --version &> /dev/null; then
        echo -e "${RED}Error: mkdocs is not installed${NC}"
        echo "Run: ./docs.sh install"
        exit 1
    fi
    
    echo -e "${GREEN}✓ Starting server...${NC}"
    echo -e "  Access at: ${BLUE}http://localhost:8000${NC}"
    echo -e "  Press Ctrl+C to stop"
    echo ""
    
    python3 -m mkdocs serve "$@"
}

# Clean build artifacts
clean_docs() {
    echo -e "${BLUE}Cleaning build artifacts...${NC}"
    
    if [ -d "site" ]; then
        rm -rf site
        echo -e "${GREEN}✓ Cleaned: site/${NC}"
    else
        echo -e "  Nothing to clean"
    fi
}

# Main
main() {
    local command="${1:-serve}"
    
    case "$command" in
        serve)
            shift || true
            serve_docs "$@"
            ;;
        build)
            build_docs
            ;;
        clean)
            clean_docs
            ;;
        install)
            install_deps
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            echo -e "${RED}Unknown command: $command${NC}"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

main "$@"
