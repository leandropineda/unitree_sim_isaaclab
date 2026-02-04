#!/bin/bash
# =============================================================================
# G1 Navigation - Real Robot Launcher (Jetson Orin)
# =============================================================================
#
# This script starts the navigation stack on the real G1 robot.
# Run this ON the Jetson Orin inside the G1.
#
# Usage:
#   ./run_robot.sh                   # Basic: bridge + camera
#   ./run_robot.sh --slam            # Add SLAM Toolbox
#   ./run_robot.sh --isaac           # Add Isaac ROS (GPU SLAM)
#   ./run_robot.sh --nav2            # Add Nav2 with saved map (launches RViz)
#   ./run_robot.sh --all             # Everything
#   ./run_robot.sh --rebuild         # Force rebuild image
#   ./run_robot.sh --down            # Stop all containers
#
# Prerequisites:
# - JetPack 6.0+ installed
# - NVIDIA Container Runtime configured
# - Docker image built: g1-navigation:jetson
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║           G1 Navigation - Real Robot (Jetson)             ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Parse arguments
SLAM=""
ISAAC=""
NAV2=""
VIZ=""
DOWN=""
REBUILD=""

for arg in "$@"; do
    case $arg in
        --slam)
            SLAM="--profile slam"
            ;;
        --isaac)
            ISAAC="--profile isaac"
            ;;
        --nav2)
            NAV2="--profile nav2"
            ;;
        --viz|--rviz)
            VIZ="--profile viz"
            ;;
        --all)
            ISAAC="--profile isaac"
            NAV2="--profile nav2"
            ;;
        --down)
            DOWN="yes"
            ;;
        --rebuild)
            REBUILD="yes"
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --slam     Start SLAM Toolbox (CPU-based)"
            echo "  --isaac    Start Isaac ROS VSLAM + nvblox (GPU)"
            echo "  --nav2     Start Nav2 (requires saved map, launches RViz)"
            echo "  --viz      Start standalone RViz (optional)"
            echo "  --all      Start Isaac ROS + Nav2"
            echo "  --rebuild  Force rebuild Docker image"
            echo "  --down     Stop all containers"
            echo "  --help     Show this help"
            echo ""
            echo "Note: Either --slam OR --isaac should be used, not both"
            exit 0
            ;;
    esac
done

# Stop if requested
if [ "$DOWN" == "yes" ]; then
    echo -e "${YELLOW}Stopping all containers...${NC}"
    docker compose -f docker-compose.jetson.yml down
    echo -e "${GREEN}All containers stopped.${NC}"
    exit 0
fi

# Check we're on Jetson
if ! command -v tegrastats &>/dev/null; then
    echo -e "${RED}WARNING: This doesn't appear to be a Jetson device.${NC}"
    echo "This script is intended to run on the Jetson Orin inside the G1 robot."
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
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

# Build navigation image if needed
NAV_DOCKERFILE="Dockerfile.navigation.jetson"
if [ "$REBUILD" == "yes" ]; then
    echo -e "${YELLOW}Force rebuilding navigation image...${NC}"
    echo "This may take 20-40 minutes on Jetson..."
    docker build --no-cache -f "$NAV_DOCKERFILE" -t g1-navigation:jetson ..
elif ! docker image inspect g1-navigation:jetson &>/dev/null; then
    echo -e "${YELLOW}Navigation image not found. Building...${NC}"
    echo "This may take 20-40 minutes on Jetson..."
    docker build -f "$NAV_DOCKERFILE" -t g1-navigation:jetson ..
elif needs_rebuild "g1-navigation:jetson" "$NAV_DOCKERFILE"; then
    echo -e "${BLUE}Dockerfile changed. Rebuilding navigation image...${NC}"
    echo "This may take 20-40 minutes on Jetson..."
    docker build -f "$NAV_DOCKERFILE" -t g1-navigation:jetson ..
else
    echo -e "${GREEN}Navigation image up to date.${NC}"
fi

# Create directories
mkdir -p maps

# Enable max performance
echo -e "${YELLOW}Enabling Jetson max performance...${NC}"
sudo jetson_clocks 2>/dev/null || echo "Note: Run 'sudo jetson_clocks' for best performance"

# Allow X11 if using visualization
if [ -n "$VIZ" ] || [ -n "$NAV2" ]; then
    xhost +local:docker 2>/dev/null || true
fi

echo ""
echo -e "${GREEN}Starting containers...${NC}"
echo "  - Bridge (DDS-ROS2)"
echo "  - RealSense camera"
[ -n "$SLAM" ] && echo "  - SLAM Toolbox (CPU)"
[ -n "$ISAAC" ] && echo "  - Isaac ROS VSLAM (GPU)"
[ -n "$ISAAC" ] && echo "  - Isaac ROS nvblox (GPU)"
[ -n "$NAV2" ] && echo "  - Nav2 navigation (RViz)"
[ -n "$VIZ" ] && echo "  - RViz visualization"
echo ""

# Start containers
docker compose -f docker-compose.jetson.yml $SLAM $ISAAC $NAV2 $VIZ up -d

echo ""
echo -e "${GREEN}Containers started!${NC}"
echo ""
echo "View logs:"
echo "  docker compose -f docker-compose.jetson.yml logs -f bridge"
echo "  docker compose -f docker-compose.jetson.yml logs -f realsense"
[ -n "$ISAAC" ] && echo "  docker compose -f docker-compose.jetson.yml logs -f vslam"
[ -n "$NAV2" ] && echo "  docker compose -f docker-compose.jetson.yml logs -f nav2"
echo ""
echo "Check robot state:"
echo "  docker exec -it g1-bridge bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'"
echo "  docker exec -it g1-bridge bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /odom'"
echo ""
echo "Save map (after SLAM):"
echo "  docker exec -it g1-bridge bash -c 'source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/my_map'"
echo ""
echo "Stop all:"
echo "  ./run_robot.sh --down"
echo ""
