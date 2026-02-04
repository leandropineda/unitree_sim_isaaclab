# G1 Navigation System Setup Guide

This guide provides comprehensive instructions for setting up SLAM and autonomous navigation on the Unitree G1 robot, both in simulation (Isaac Lab) and on the real robot (Jetson Orin NX).

**All components run in Docker containers** for reproducibility and easy deployment.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Component Overview](#2-component-overview)
3. [Prerequisites](#3-prerequisites)
4. [Quick Start (Docker)](#4-quick-start-docker)
5. [Simulation Setup (Isaac Lab)](#5-simulation-setup-isaac-lab)
6. [Real Robot Setup (Jetson Orin)](#6-real-robot-setup-jetson-orin)
7. [Running Navigation](#7-running-navigation)
8. [Troubleshooting](#8-troubleshooting)
9. [Component Reference](#9-component-reference)

---

## 1. System Architecture

The navigation system bridges Isaac Lab simulation (or real robot) with ROS2 Nav2:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         NAVIGATION SYSTEM ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌─────────────────────┐         ┌──────────────────────────────────────┐  │
│   │     SIMULATION      │         │           JETSON ORIN NX             │  │
│   │   (Your Workstation)│         │        (Real Robot / Sim)            │  │
│   │                     │         │                                      │  │
│   │  ┌───────────────┐  │         │  ┌──────────────────────────────┐   │  │
│   │  │  Isaac Lab    │  │         │  │    Isaac ROS (GPU)           │   │  │
│   │  │  Simulation   │  │         │  │                              │   │  │
│   │  │               │  │         │  │  ┌─────────────────────────┐ │   │  │
│   │  │  • G1 Robot   │  │   DDS   │  │  │ Visual SLAM (30Hz)      │ │   │  │
│   │  │  • RGB-D Cam  │◄─┼────────►│  │  │ • 6-DoF odometry        │ │   │  │
│   │  │  • Physics    │  │         │  │  │ • Loop closure          │ │   │  │
│   │  │               │  │         │  │  └─────────────────────────┘ │   │  │
│   │  └───────────────┘  │         │  │                              │   │  │
│   │         │           │         │  │  ┌─────────────────────────┐ │   │  │
│   │         │ Cameras   │         │  │  │ nvblox (10Hz)           │ │   │  │
│   │         ▼           │         │  │  │ • 3D TSDF mapping       │ │   │  │
│   │  ┌───────────────┐  │         │  │  │ • 2D costmap → Nav2     │ │   │  │
│   │  │  DDS Topics   │  │         │  │  └─────────────────────────┘ │   │  │
│   │  │               │  │         │  │                              │   │  │
│   │  │ rt/run_cmd    │  │         │  └──────────────────────────────┘   │  │
│   │  │ rt/lowstate   │  │         │              │                      │  │
│   │  └───────────────┘  │         │              │ ROS2 Topics          │  │
│   │         │           │         │              ▼                      │  │
│   └─────────┼───────────┘         │  ┌──────────────────────────────┐   │  │
│             │                     │  │         Nav2 Stack           │   │  │
│             │ DDS-ROS2 Bridge     │  │                              │   │  │
│             ▼                     │  │  • MPPI Controller           │   │  │
│   ┌─────────────────────┐         │  │  • Global Planner            │   │  │
│   │  dds_ros2_bridge.py │────────►│  │  • Local Costmap             │   │  │
│   │                     │◄────────│  │  • Recovery Behaviors        │   │  │
│   │  • /cmd_vel → DDS   │  ROS2   │  │                              │   │  │
│   │  • DDS → /odom      │  Topics │  └──────────────────────────────┘   │  │
│   └─────────────────────┘         │                                      │  │
│                                   └──────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Sensor Data**: RGB-D camera in simulation → Isaac ROS Visual SLAM + nvblox
2. **Localization**: Visual SLAM provides `/odom` (6-DoF pose)
3. **Mapping**: nvblox creates 3D TSDF map → 2D costmap for Nav2
4. **Planning**: Nav2 plans path using costmap
5. **Control**: Nav2 sends `/cmd_vel` → DDS bridge → `rt/run_command/cmd`
6. **Execution**: Robot locomotion policy executes velocity commands

---

## 2. Component Overview

### 2.1 Files Created/Modified

| File | Purpose | When to Debug |
|------|---------|---------------|
| `tasks/common_config/camera_configs.py` | RGB-D camera configuration | Camera not showing depth |
| `tasks/common_scene/base_scene_navigation.py` | Navigation scene definition | Scene loading issues |
| `tasks/g1_tasks/navigation_g1_29dof/` | Navigation task registration | Task not found errors |
| `navigation/dds_ros2_bridge.py` | DDS ↔ ROS2 communication | Topics not connecting |
| `navigation/config/g1_nav_params.yaml` | Nav2 parameters | Navigation behavior issues |
| `navigation/config/nvblox_params.yaml` | 3D mapping parameters | Mapping quality issues |
| `navigation/config/vslam_params.yaml` | Visual SLAM parameters | Odometry drift |
| `navigation/launch/nav2_bringup.launch.py` | Nav2 launch file | Nav2 startup failures |
| `navigation/launch/isaac_ros_navigation.launch.py` | Full stack launch | Integration issues |

### 2.2 New Tasks Registered

| Task Name | Description |
|-----------|-------------|
| `Isaac-Navigation-G129-Wholebody` | Basic navigation scene |
| `Isaac-Navigation-G129-Wholebody-Obstacles` | Navigation with extra obstacles |

### 2.3 New DDS/ROS2 Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `rt/run_command/cmd` | String (DDS) | ROS2 → Robot | Velocity commands |
| `/cmd_vel` | Twist (ROS2) | Nav2 → Bridge | Navigation commands |
| `/odom` | Odometry (ROS2) | Bridge → Nav2 | Robot odometry |
| `/camera/depth/*` | Image (ROS2) | Sim → SLAM/nvblox | Depth data |

---

## 3. Prerequisites

### 3.1 Simulation Workstation

```bash
# System requirements
- Ubuntu 22.04
- NVIDIA GPU (RTX 3080+ recommended)
- NVIDIA Driver 535+
- 32GB+ RAM

# Software
- Isaac Sim 5.0.0
- Isaac Lab v2.2.0
- Docker with NVIDIA Container Toolkit
```

### 3.2 Real Robot (Jetson Orin NX)

```bash
# Hardware
- Unitree G1 Edu Ultimate D
- Jetson Orin NX 16GB (100 TOPS)
- Intel RealSense D435

# Software
- JetPack 6.0+
- ROS2 Humble
- Isaac ROS packages
```

### 3.3 Install ROS2 Humble (if not installed)

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-nav2-bringup ros-humble-slam-toolbox

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.4 Install Isaac ROS (Jetson Orin)

```bash
# Follow NVIDIA's official guide:
# https://nvidia-isaac-ros.github.io/getting_started/

# Key packages needed:
# - isaac_ros_visual_slam
# - isaac_ros_nvblox
```

---

## 4. Quick Start (Docker)

The fastest way to get started is using the provided Docker setup.

### 4.1 Simulation (Workstation)

```bash
cd unitree_sim_isaaclab/docker

# Build images (first time only, ~30 minutes)
docker build -t unitree-sim:latest -f ../Dockerfile ..
docker build -t g1-navigation:x86 -f Dockerfile.navigation.x86 ..

# Run everything
./run_simulation.sh --all

# Or step by step:
./run_simulation.sh           # Basic: simulation + bridge
./run_simulation.sh --nav2    # Add Nav2 (SLAM mode, RViz)
./run_simulation.sh --rviz    # Add standalone RViz visualization

# Teleop in simulation
docker exec -it g1-simulation bash -c \
    'cd /home/code/unitree_sim_isaaclab && python send_commands_keyboard.py'

# Stop
./run_simulation.sh --down
```

### 4.2 Real Robot (Jetson Orin)

```bash
# On the Jetson Orin inside the G1:
cd unitree_sim_isaaclab/docker

# Build image (first time only, ~40 minutes on Jetson)
docker build -t g1-navigation:jetson -f Dockerfile.navigation.jetson ..

# Run navigation
./run_robot.sh                 # Basic: bridge + camera
./run_robot.sh --slam          # Add SLAM Toolbox (CPU)
./run_robot.sh --isaac         # Add Isaac ROS (GPU SLAM)
./run_robot.sh --nav2          # Add Nav2 (needs saved map, RViz)
./run_robot.sh --viz           # Standalone RViz (optional)

# Stop
./run_robot.sh --down
```

### 4.3 Container Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         SIMULATION (Workstation)                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────┐  │
│  │  unitree-sim:latest │    │  g1-navigation:x86  │    │ g1-navigation   │  │
│  │                     │    │                     │    │     :x86        │  │
│  │  • Isaac Lab        │◄──►│  • DDS-ROS2 Bridge  │◄──►│  • Nav2         │  │
│  │  • G1 Robot Sim     │DDS │  • ROS2 Humble      │ROS2│  • SLAM         │  │
│  │  • RGB-D Camera     │    │  • Odometry Pub     │    │  • Path Plan    │  │
│  └─────────────────────┘    └─────────────────────┘    └─────────────────┘  │
│        Container 1               Container 2               Container 3       │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                          REAL ROBOT (Jetson Orin)                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────┐  │
│  │  g1-navigation      │    │  g1-navigation      │    │ g1-navigation   │  │
│  │     :jetson         │    │     :jetson         │    │    :jetson      │  │
│  │                     │    │                     │    │                 │  │
│  │  • DDS-ROS2 Bridge  │    │  • Isaac ROS VSLAM  │    │  • Nav2         │  │
│  │  • RealSense Driver │    │  • Isaac ROS nvblox │    │  • MPPI Control │  │
│  │                     │    │  • GPU Accelerated  │    │                 │  │
│  └─────────────────────┘    └─────────────────────┘    └─────────────────┘  │
│        Container 1               Container 2               Container 3       │
│                    All containers use --network host for DDS                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Simulation Setup (Isaac Lab)

### 5.1 Build Docker Environment

```bash
# Clone repository (if not done)
git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git
cd unitree_sim_isaaclab

# Build Docker image
sudo docker build -t unitree-sim:latest -f Dockerfile .

# Download assets
./fetch_assets.sh
```

### 5.2 Launch Simulation

```bash
# Allow X11 forwarding
xhost +local:docker

# Run container
sudo docker run --gpus all -it --rm \
    --network host \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    unitree-sim:latest /bin/bash

# Inside container: Launch navigation task
python sim_main.py \
    --task Isaac-Navigation-G129-Wholebody \
    --robot_type g129 \
    --enable_wholebody_dds
```

### 5.3 Verify Simulation

1. **Check Isaac Sim GUI**: Robot should appear in warehouse scene
2. **Switch to main camera**: PerspectiveCamera → Cameras → PerspectiveCamera
3. **Verify DDS**: In another terminal, run `python send_commands_keyboard.py`
   - Robot should respond to WASD keys

---

## 6. Real Robot Setup (Jetson Orin)

### 6.1 Install Navigation Package

```bash
# On Jetson Orin
cd ~/workspace
git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git
cd unitree_sim_isaaclab

# Install dependencies
pip install -r requirements.txt
pip install unitree_sdk2py
```

### 6.2 Configure Isaac ROS

```bash
# Copy configuration files
mkdir -p ~/ros2_ws/src/g1_navigation/config
cp navigation/config/*.yaml ~/ros2_ws/src/g1_navigation/config/

# Build workspace
cd ~/ros2_ws
colcon build --packages-select g1_navigation
source install/setup.bash
```

### 6.3 Configure RealSense Camera

```bash
# Install RealSense ROS2 wrapper
sudo apt install ros-humble-realsense2-camera

# Launch camera
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_color:=true \
    depth_module.profile:=640x480x30
```

---

## 7. Running Navigation

### 7.1 Simulation (Isaac Lab)

#### Camera Data Architecture

In simulation, camera data flows through a shared memory bridge:

```
Isaac Lab Simulation          Navigation Container
┌─────────────────────┐      ┌──────────────────────────┐
│  front_camera       │      │  sim_camera_ros2_bridge  │
│  (RGB + Depth)      │ ──▶  │  - Reads shared memory   │
│                     │ shm  │  - Publishes to ROS2     │
│  camera_state.py    │      │                          │
│  writes to shm      │      │  Topics (RealSense fmt): │
└─────────────────────┘      │  /camera/color/image_raw │
                             │  /camera/depth/image_raw │
                             │  /camera/depth/color/pts │
                             └──────────────────────────┘
```

This publishes to the **same ROS2 topics as the real RealSense driver**,
enabling seamless switching between simulation and real robot.

#### 7.1.1 Docker bringup (recommended)
```bash
cd unitree_sim_isaaclab/docker

# Starts simulation + DDS bridge + camera bridge + Nav2 + RViz
./run_simulation.sh --nav2
```

The camera bridge automatically starts in the navigation container and
publishes depth data from the simulation to ROS2.

#### 7.1.2 GPU-first stack (Isaac ROS on workstation)
If Isaac ROS packages are installed in the navigation container, use the
GPU-accelerated stack for VSLAM + nvblox:
```bash
# Start simulation + bridge first (same as above)

# Launch GPU stack + Nav2 + RViz
ros2 launch navigation/launch/isaac_ros_navigation.launch.py \
    use_sim_time:=True
```

#### 7.1.3 CPU fallback (SLAM Toolbox)
```bash
ros2 launch navigation/launch/nav2_bringup.launch.py \
    use_sim_time:=True \
    slam:=True
```

> RViz is launched by default. Disable with `use_rviz:=False`.

#### 7.1.4 Mapping the environment
```bash
# Drive the robot around to build a map
python send_commands_keyboard.py

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/my_warehouse_map
```

#### 7.1.5 Autonomous navigation (with saved map)
```bash
ros2 launch navigation/launch/nav2_bringup.launch.py \
    use_sim_time:=True \
    map:=/path/to/my_warehouse_map.yaml
```

In RViz:
- Click **2D Pose Estimate** and set the initial pose
- Click **Nav2 Goal** to send a target

### 7.2 Real Robot (Jetson Orin)

> **Note on Real Robot Camera**: The real robot uses an **Intel RealSense D435**
> depth camera connected via USB. The camera driver runs in a dedicated container
> with privileged access to USB devices.

#### 7.2.1 Prerequisites
1. RealSense D435 connected to Jetson via USB 3.0
2. Verify camera is detected on the host:
   ```bash
   # On Jetson host (not in container)
   lsusb | grep -i intel
   # Should show: Intel Corp. RealSense D435
   ```

#### 7.2.2 GPU-first bringup (Isaac ROS)
```bash
# On the Jetson inside the G1
cd unitree_sim_isaaclab/docker

# Starts bridge + camera + Isaac ROS (GPU)
./run_robot.sh --isaac
```

#### 7.2.3 CPU fallback (SLAM Toolbox)
```bash
./run_robot.sh --slam
```

#### 7.2.4 Mapping the environment
```bash
# Drive the robot around with teleop, then save the map
docker exec -it g1-bridge bash -c \
  'source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/my_map'
```

#### 7.2.5 Autonomous navigation (with saved map)
```bash
# Launch Nav2 + RViz
./run_robot.sh --nav2
```

In RViz:
- Click **2D Pose Estimate** and set the initial pose
- Click **Nav2 Goal** to send a target

> RViz launches from Nav2 bringup. If you need a separate RViz instance,
> run `./run_robot.sh --viz`.

#### 7.2.6 Troubleshooting RealSense Camera

If the camera is not detected:
```bash
# Check USB devices on host
lsusb | grep -i intel

# Check /dev/video devices
ls /dev/video*

# Check dmesg for USB errors
dmesg | tail -50 | grep -i usb

# Install udev rules if needed
sudo apt install librealsense2-udev-rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Make sure you're using the `realsense` service from docker-compose, NOT running
the camera driver manually in a non-privileged container.

---

## 8. Troubleshooting

### 8.1 Common Issues

#### Issue: "Task not found: Isaac-Navigation-G129-Wholebody"

**Cause**: Task not registered properly

**Solution**:
```bash
# Verify task registration
python -c "import tasks.g1_tasks; print('OK')"

# Check __init__.py includes navigation task
cat tasks/g1_tasks/__init__.py | grep navigation
```

#### Issue: Robot doesn't respond to /cmd_vel

**Cause**: DDS bridge not running or misconfigured

**Debug steps**:
```bash
# 1. Check bridge is running
ps aux | grep dds_ros2_bridge

# 2. Verify ROS2 topics
ros2 topic list | grep cmd_vel
ros2 topic echo /cmd_vel

# 3. Check DDS connectivity
# In simulation terminal, you should see:
# [DDS] Received command: [0.5, 0.0, 0.0, 0.8]
```

#### Issue: No depth data in RViz

**Cause**: Camera not publishing depth or wrong topic

**Debug steps**:
```bash
# 1. Check camera topics
ros2 topic list | grep camera

# 2. Verify depth publishing
ros2 topic hz /camera/depth/image_rect_raw

# 3. Check camera_configs.py has depth enabled
grep -A5 "front_rgbd_camera" tasks/common_config/camera_configs.py
```

#### Issue: Visual SLAM drift/failure

**Cause**: Insufficient visual features or bad camera data

**Debug steps**:
```bash
# 1. Check VSLAM status
ros2 topic echo /visual_slam/tracking/slam_status

# 2. Verify camera is providing good images
ros2 run image_view image_view image:=/camera/color/image_raw

# 3. Ensure adequate lighting in simulation
# Adjust light intensity in base_scene_navigation.py
```

#### Issue: Nav2 fails to plan path

**Cause**: Costmap issues or unreachable goal

**Debug steps**:
```bash
# 1. Check costmap in RViz
# Add Local Costmap and Global Costmap displays

# 2. Verify robot footprint
grep robot_radius navigation/config/g1_nav_params.yaml

# 3. Check planner server
ros2 service call /compute_path_to_pose nav2_msgs/srv/ComputePathToPose
```

### 8.2 Performance Issues

#### Slow SLAM on Jetson

```bash
# 1. Lock GPU/CPU clocks
sudo jetson_clocks

# 2. Check GPU usage
tegrastats

# 3. Reduce VSLAM resolution in vslam_params.yaml
# image_height: 360  # Reduce from 480
# image_width: 480   # Reduce from 640
```

#### High latency in navigation

```bash
# 1. Reduce controller frequency
# In g1_nav_params.yaml:
# controller_frequency: 10.0  # Reduce from 20.0

# 2. Use simpler planner
# Change from NavFn to Theta*
```

### 8.3 Network/DDS Issues

#### DDS topics not visible between machines

```bash
# 1. Check same ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be same on all machines (default: 0)

# 2. Verify network connectivity
ping <other_machine_ip>

# 3. Check firewall
sudo ufw status
# May need to allow UDP ports 7400-7500
```

### 8.4 Docker-Specific Issues

#### Container can't access GPU

```bash
# Check NVIDIA runtime is installed
docker info | grep -i runtime

# If "nvidia" not listed, install NVIDIA Container Toolkit:
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

# Verify GPU access
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

#### X11 display issues (no GUI)

```bash
# Allow Docker to access X11
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY  # Should be :0 or :1

# If running over SSH, use X forwarding:
ssh -X user@host

# Alternative: Run headless
docker compose up simulation  # Add --headless to sim_main.py command
```

#### DDS communication between containers not working

```bash
# All containers must use --network host
# Check docker-compose.yml has:
#   network_mode: host

# Verify containers are on host network
docker inspect g1-simulation | grep NetworkMode
# Should show: "NetworkMode": "host"

# Check DDS is working inside container
docker exec -it g1-navigation bash -c \
    'source /opt/ros/humble/setup.bash && ros2 topic list'
```

#### Container fails to start on Jetson

```bash
# Check NVIDIA runtime is default
cat /etc/docker/daemon.json
# Should contain: "default-runtime": "nvidia"

# If not, add it:
sudo tee /etc/docker/daemon.json <<EOF
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
EOF
sudo systemctl restart docker
```

#### Build fails on Jetson (out of memory)

```bash
# Jetson has limited RAM - enable swap
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Build with limited parallelism
DOCKER_BUILDKIT=1 docker build --build-arg MAKEFLAGS="-j2" ...
```

#### Container can't access RealSense camera

```bash
# Run with privileged mode and USB access
docker run --privileged -v /dev:/dev ...

# Or add specific device
docker run --device=/dev/bus/usb ...

# Check camera is visible
docker exec -it g1-realsense bash -c 'rs-enumerate-devices'
```

---

## 9. Component Reference

### 9.1 camera_configs.py - RGB-D Camera

**Location**: `tasks/common_config/camera_configs.py`

**Purpose**: Defines camera sensors for navigation

**Key configurations**:
```python
@classmethod
def g1_front_rgbd_camera(cls) -> CameraCfg:
    """Front RGB-D camera for SLAM"""
    return CameraBaseCfg.get_camera_config(
        data_types=["rgb", "depth"],  # Enable both RGB and depth
        update_period=0.033,           # ~30Hz
        clipping_range=(0.1, 10.0),    # 10cm to 10m depth range
    )
```

**When to modify**:
- Change camera resolution (height, width)
- Adjust depth range (clipping_range)
- Change update rate (update_period)

### 9.2 base_scene_navigation.py - Scene Configuration

**Location**: `tasks/common_scene/base_scene_navigation.py`

**Purpose**: Defines the warehouse scene for navigation

**Key components**:
- `room_walls`: Warehouse structure
- `packing_table1/2`: Static obstacles
- `light`: Scene illumination

**When to modify**:
- Add/remove obstacles
- Change lighting (affects VSLAM)
- Modify scene size

### 9.3 navigation_g1_29dof_env_cfg.py - Environment Configuration

**Location**: `tasks/g1_tasks/navigation_g1_29dof/navigation_g1_29dof_env_cfg.py`

**Purpose**: Isaac Lab environment configuration

**Key settings**:
```python
self.decimation = 4          # Control at 50Hz (200Hz physics / 4)
self.episode_length_s = 300  # 5 minute episodes
self.sim.dt = 0.005          # 200Hz physics
```

**When to modify**:
- Change control frequency
- Adjust physics settings
- Modify episode length

### 9.4 dds_ros2_bridge.py - Communication Bridge

**Location**: `navigation/dds_ros2_bridge.py`

**Purpose**: Bidirectional bridge between Unitree DDS and ROS2

**Key functions**:
```python
def _cmd_vel_callback(self, msg: Twist):
    """Convert ROS2 Twist → Unitree [x, y, yaw, height]"""
    
def _publish_odometry(self):
    """Publish robot odometry to /odom"""
```

**When to modify**:
- Adjust velocity limits
- Change coordinate conventions
- Add additional topic bridges

### 9.5 g1_nav_params.yaml - Nav2 Parameters

**Location**: `navigation/config/g1_nav_params.yaml`

**Purpose**: Nav2 stack configuration for G1 humanoid

**Key sections**:
- `controller_server`: MPPI controller settings
- `local_costmap`: Obstacle avoidance parameters
- `global_costmap`: Path planning parameters
- `behavior_server`: Recovery behavior settings

**When to modify**:
- Tune velocity limits (`vx_max`, `vy_max`, `wz_max`)
- Adjust obstacle inflation (`inflation_radius`)
- Change goal tolerance (`xy_goal_tolerance`)

### 9.6 nvblox_params.yaml - 3D Mapping

**Location**: `navigation/config/nvblox_params.yaml`

**Purpose**: GPU-accelerated 3D mapping configuration

**Key settings**:
```yaml
voxel_size: 0.05              # 5cm resolution
integration_rate_hz: 10.0     # 10Hz depth integration
map_2d_update_rate_hz: 5.0    # 5Hz costmap updates
```

**When to modify**:
- Change resolution (voxel_size)
- Adjust performance vs quality tradeoff
- Configure depth camera topics

### 9.7 vslam_params.yaml - Visual SLAM

**Location**: `navigation/config/vslam_params.yaml`

**Purpose**: GPU-accelerated visual odometry

**Key settings**:
```yaml
enable_localization_n_mapping: True
enable_loop_closure: True
target_fps: 30
```

**When to modify**:
- Enable/disable loop closure
- Adjust processing rate
- Configure camera topics

---

## Quick Reference Commands

### Simulation (Isaac Lab)
```bash
# Start simulation + bridge + Nav2 + RViz
./run_simulation.sh --nav2

# Launch Nav2 (SLAM mode, RViz)
ros2 launch navigation/launch/nav2_bringup.launch.py use_sim_time:=True slam:=True

# Launch Nav2 (with map, RViz)
ros2 launch navigation/launch/nav2_bringup.launch.py use_sim_time:=True map:=/path/to/map.yaml

# Launch GPU stack (if Isaac ROS is available)
ros2 launch navigation/launch/isaac_ros_navigation.launch.py use_sim_time:=True

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Real Robot (Jetson Orin)
```bash
# Start GPU stack (bridge + camera + Isaac ROS)
./run_robot.sh --isaac

# Launch Nav2 + RViz
./run_robot.sh --nav2

# Save map
docker exec -it g1-bridge bash -c \
  'source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/my_map'
```

---

## Support

For issues specific to:
- **Isaac Lab/Sim**: Check NVIDIA forums and Isaac Lab GitHub
- **Nav2**: Check Nav2 documentation and GitHub issues
- **Isaac ROS**: Check NVIDIA Isaac ROS documentation
- **Unitree G1**: Check Unitree documentation and Discord
