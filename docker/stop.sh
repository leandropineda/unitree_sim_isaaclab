#!/bin/bash
# =============================================================================
# G1 Navigation - Stop All Containers
# =============================================================================
#
# Usage:
#   ./stop.sh              # Stop all containers (simulation + robot)
#   ./stop.sh --sim        # Stop simulation containers only
#   ./stop.sh --robot      # Stop robot containers only
#   ./stop.sh --clean      # Stop and remove all images
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║              G1 Navigation - Stop Containers              ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Parse arguments
STOP_SIM=""
STOP_ROBOT=""
CLEAN=""

# Default: stop both if no argument specified
if [ $# -eq 0 ]; then
    STOP_SIM="yes"
    STOP_ROBOT="yes"
fi

for arg in "$@"; do
    case $arg in
        --sim|--simulation)
            STOP_SIM="yes"
            ;;
        --robot|--jetson)
            STOP_ROBOT="yes"
            ;;
        --all)
            STOP_SIM="yes"
            STOP_ROBOT="yes"
            ;;
        --clean)
            STOP_SIM="yes"
            STOP_ROBOT="yes"
            CLEAN="yes"
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --sim        Stop simulation containers only"
            echo "  --robot      Stop robot (Jetson) containers only"
            echo "  --all        Stop all containers (default if no args)"
            echo "  --clean      Stop containers and remove all images"
            echo "  --help       Show this help"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $arg${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Stop simulation containers
if [ "$STOP_SIM" == "yes" ]; then
    echo -e "${YELLOW}Stopping simulation containers...${NC}"
    if [ -f "docker-compose.yml" ]; then
        docker compose down 2>/dev/null || true
        echo -e "${GREEN}Simulation containers stopped.${NC}"
    else
        echo "No docker-compose.yml found (simulation not configured here)"
    fi
fi

# Stop robot containers
if [ "$STOP_ROBOT" == "yes" ]; then
    echo -e "${YELLOW}Stopping robot containers...${NC}"
    if [ -f "docker-compose.jetson.yml" ]; then
        docker compose -f docker-compose.jetson.yml down 2>/dev/null || true
        echo -e "${GREEN}Robot containers stopped.${NC}"
    else
        echo "No docker-compose.jetson.yml found (robot not configured here)"
    fi
fi

# Clean images if requested
if [ "$CLEAN" == "yes" ]; then
    echo ""
    echo -e "${YELLOW}Removing Docker images...${NC}"
    
    # Remove simulation image
    if docker image inspect unitree-sim:latest &>/dev/null; then
        echo "Removing unitree-sim:latest..."
        docker rmi unitree-sim:latest 2>/dev/null || echo "  (in use, skipped)"
    fi
    
    # Remove navigation images
    if docker image inspect g1-navigation:x86 &>/dev/null; then
        echo "Removing g1-navigation:x86..."
        docker rmi g1-navigation:x86 2>/dev/null || echo "  (in use, skipped)"
    fi
    
    if docker image inspect g1-navigation:jetson &>/dev/null; then
        echo "Removing g1-navigation:jetson..."
        docker rmi g1-navigation:jetson 2>/dev/null || echo "  (in use, skipped)"
    fi
    
    echo -e "${GREEN}Cleanup complete.${NC}"
fi

echo ""
echo -e "${GREEN}Done!${NC}"
echo ""
echo "To restart:"
echo "  ./run_simulation.sh    # For simulation"
echo "  ./run_robot.sh         # For real robot"
echo ""
