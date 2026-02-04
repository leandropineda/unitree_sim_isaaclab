# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Navigation Module for Unitree G1 Robot

This module provides the bridge between:
- Isaac Lab simulation (or real robot) using Unitree DDS
- ROS2 navigation stack (Nav2, SLAM, etc.)

Components:
-----------
dds_ros2_bridge : DdsRos2Bridge
    Bidirectional bridge between Unitree DDS and ROS2 topics.
    Converts cmd_vel (Twist) to rt/run_command/cmd format.
    Publishes robot odometry to /odom topic.

Launch Files:
-------------
- isaac_ros_vslam.launch.py: GPU-accelerated Visual SLAM
- isaac_ros_nvblox.launch.py: GPU-accelerated 3D mapping
- nav2_bringup.launch.py: Full navigation stack
- navigation_sim.launch.py: Complete simulation + navigation

Configuration:
--------------
- g1_nav_params.yaml: Nav2 parameters tuned for G1 humanoid
- nvblox_params.yaml: 3D mapping parameters
- vslam_params.yaml: Visual SLAM parameters

Architecture:
-------------
    ┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
    │  Isaac Lab      │     │  DDS-ROS2        │     │  ROS2 Nav2      │
    │  Simulation     │◄───►│  Bridge          │◄───►│  Stack          │
    │  (or Real G1)   │     │                  │     │                 │
    └─────────────────┘     └──────────────────┘     └─────────────────┘
           │                        │                        │
           │ DDS                    │                        │ ROS2
           │ rt/run_command/cmd     │                        │ /cmd_vel
           │ rt/lowstate            │                        │ /odom
           └────────────────────────┴────────────────────────┘

Usage:
------
    # Start simulation with navigation task
    python sim_main.py --task Isaac-Navigation-G129-Wholebody \\
        --robot_type g129 --enable_wholebody_dds
    
    # Start DDS-ROS2 bridge (in another terminal)
    python -m navigation.dds_ros2_bridge
    
    # Start navigation stack (in another terminal)
    ros2 launch navigation/launch/nav2_bringup.launch.py
"""

__version__ = "1.0.0"
__author__ = "Unitree Robotics"

# Module exports
__all__ = [
    "DdsRos2Bridge",
]
