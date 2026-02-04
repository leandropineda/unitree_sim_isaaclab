#!/bin/bash
# =============================================================================
# G1 Navigation - Simulation Launcher
# =============================================================================
#
# This script starts the complete navigation stack in containers:
# 1. Isaac Lab simulation with G1 robot
# 2. DDS-ROS2 bridge
# 3. Optional: Nav2 and RViz
#
# Usage:
#   ./run_simulation.sh              # Basic: simulation + bridge
#   ./run_simulation.sh --nav2       # Include Nav2 (SLAM mode, launches RViz)
#   ./run_simulation.sh --rviz       # Include standalone RViz visualization
#   ./run_simulation.sh --all        # Everything
#   ./run_simulation.sh --rebuild    # Force rebuild all images
#   ./run_simulation.sh --rebuild-sim   # Force rebuild simulation image only
#   ./run_simulation.sh --rebuild-nav   # Force rebuild navigation image only
#   ./run_simulation.sh --down       # Stop all containers
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║         G1 Navigation - Simulation Environment            ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Parse arguments
NAV2=""
VIZ=""
DOWN=""
REBUILD_SIM=""
REBUILD_NAV=""

for arg in "$@"; do
    case $arg in
        --nav2)
            NAV2="--profile nav2"
            ;;
        --rviz|--viz)
            VIZ="--profile viz"
            ;;
        --all)
            NAV2="--profile nav2"
            VIZ="--profile viz"
            ;;
        --down)
            DOWN="yes"
            ;;
        --rebuild)
            REBUILD_SIM="yes"
            REBUILD_NAV="yes"
            ;;
        --rebuild-sim)
            REBUILD_SIM="yes"
            ;;
        --rebuild-nav)
            REBUILD_NAV="yes"
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --nav2         Start Nav2 navigation stack (SLAM mode, RViz)"
            echo "  --rviz         Start standalone RViz visualization"
            echo "  --all          Start everything"
            echo "  --rebuild      Force rebuild all Docker images"
            echo "  --rebuild-sim  Force rebuild simulation image only"
            echo "  --rebuild-nav  Force rebuild navigation image only"
            echo "  --down         Stop all containers"
            echo "  --help         Show this help"
            exit 0
            ;;
    esac
done

# Allow X11 forwarding
echo -e "${YELLOW}Enabling X11 forwarding...${NC}"
xhost +local:docker 2>/dev/null || true

# Stop containers if requested
if [ "$DOWN" == "yes" ]; then
    echo -e "${YELLOW}Stopping all containers...${NC}"
    docker compose down
    echo -e "${GREEN}All containers stopped.${NC}"
    exit 0
fi

# Function to check if image needs rebuild (Dockerfile newer than image)
needs_rebuild() {
    local image_name="$1"
    local dockerfile="$2"
    
    # If image doesn't exist, needs build
    if ! docker image inspect "$image_name" &>/dev/null; then
        return 0
    fi
    
    # Get image creation time
    local image_time=$(docker image inspect "$image_name" --format '{{.Created}}' 2>/dev/null)
    local image_epoch=$(date -d "$image_time" +%s 2>/dev/null || echo 0)
    
    # Get Dockerfile modification time
    local dockerfile_epoch=$(stat -c %Y "$dockerfile" 2>/dev/null || echo 0)
    
    # If Dockerfile is newer than image, needs rebuild
    if [ "$dockerfile_epoch" -gt "$image_epoch" ]; then
        return 0
    fi
    
    return 1
}

# Build simulation image if needed
SIM_DOCKERFILE="../Dockerfile"
if [ "$REBUILD_SIM" == "yes" ]; then
    echo -e "${YELLOW}Force rebuilding simulation image...${NC}"
    echo "This may take a while (15-30 minutes)..."
    cd ..
    docker build --no-cache -t unitree-sim:latest -f Dockerfile .
    cd docker
elif ! docker image inspect unitree-sim:latest &>/dev/null; then
    echo -e "${YELLOW}Simulation image not found. Building...${NC}"
    echo "This may take a while (15-30 minutes)..."
    cd ..
    docker build -t unitree-sim:latest -f Dockerfile .
    cd docker
elif needs_rebuild "unitree-sim:latest" "$SIM_DOCKERFILE"; then
    echo -e "${BLUE}Dockerfile changed. Rebuilding simulation image...${NC}"
    echo "This may take a while (15-30 minutes)..."
    cd ..
    docker build -t unitree-sim:latest -f Dockerfile .
    cd docker
else
    echo -e "${GREEN}Simulation image up to date.${NC}"
fi

# Build navigation image if needed
NAV_DOCKERFILE="Dockerfile.navigation.x86"
if [ "$REBUILD_NAV" == "yes" ]; then
    echo -e "${YELLOW}Force rebuilding navigation image...${NC}"
    docker build --no-cache -f "$NAV_DOCKERFILE" -t g1-navigation:x86 ..
elif ! docker image inspect g1-navigation:x86 &>/dev/null; then
    echo -e "${YELLOW}Navigation image not found. Building...${NC}"
    docker build -f "$NAV_DOCKERFILE" -t g1-navigation:x86 ..
elif needs_rebuild "g1-navigation:x86" "$NAV_DOCKERFILE"; then
    echo -e "${BLUE}Dockerfile changed. Rebuilding navigation image...${NC}"
    docker build -f "$NAV_DOCKERFILE" -t g1-navigation:x86 ..
else
    echo -e "${GREEN}Navigation image up to date.${NC}"
fi

# Create data directories
mkdir -p data maps

echo ""
echo -e "${GREEN}Starting containers...${NC}"
echo "  - Simulation (Isaac Lab + G1)"
echo "  - Navigation (DDS-ROS2 bridge)"
[ -n "$NAV2" ] && echo "  - Nav2 (SLAM mode, RViz)"
[ -n "$VIZ" ] && echo "  - RViz (visualization)"
echo ""

# Start docker compose
docker compose $NAV2 $VIZ up -d

echo ""
echo -e "${GREEN}Containers started!${NC}"
echo ""
echo "View logs:"
echo "  docker compose logs -f simulation    # Simulation logs"
echo "  docker compose logs -f navigation    # Bridge logs"
[ -n "$NAV2" ] && echo "  docker compose logs -f nav2          # Nav2 logs"
echo ""
echo "Interactive shells:"
echo "  docker exec -it g1-simulation bash   # Enter simulation"
echo "  docker exec -it g1-navigation bash   # Enter navigation"
echo ""
echo "Teleop control:"
echo "  docker exec -it g1-simulation bash -c 'cd /home/code/unitree_sim_isaaclab && python send_commands_keyboard.py'"
echo ""
echo "Stop all:"
echo "  ./run_simulation.sh --down"
echo ""
