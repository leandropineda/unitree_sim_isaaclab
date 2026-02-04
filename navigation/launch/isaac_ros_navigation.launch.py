#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Isaac ROS Navigation Launch File for Unitree G1

This launch file brings up the complete GPU-accelerated navigation stack:
1. Isaac ROS Visual SLAM - GPU-accelerated visual odometry
2. Isaac ROS nvblox - GPU-accelerated 3D mapping
3. Nav2 - Path planning and control

This is designed for:
- Jetson Orin NX (100 TOPS GPU)
- Intel RealSense D435 RGB-D camera
- Both simulation (Isaac Lab) and real robot

The stack leverages the Jetson's GPU for:
- Real-time visual odometry (~30Hz)
- Dense 3D reconstruction (~10Hz)
- 2D costmap generation for Nav2

Usage:
------
    # Full navigation stack
    ros2 launch navigation/launch/isaac_ros_navigation.launch.py \\
        use_sim_time:=True

    # With existing map (skip SLAM)
    ros2 launch navigation/launch/isaac_ros_navigation.launch.py \\
        use_sim_time:=True \\
        map:=/path/to/map.yaml \\
        enable_slam:=False

Components:
-----------
    - isaac_ros_visual_slam: GPU visual odometry
    - isaac_ros_nvblox: GPU 3D mapping + costmap
    - nav2_bringup: Path planning and control
    - RViz: Visualization (optional)
    - DDS-ROS2 bridge: Unitree robot interface
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')
    launch_dir = os.path.join(pkg_dir, 'launch')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_vslam = LaunchConfiguration('enable_vslam')
    enable_nvblox = LaunchConfiguration('enable_nvblox')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    declare_enable_vslam = DeclareLaunchArgument(
        'enable_vslam',
        default_value='True',
        description='Enable Isaac ROS Visual SLAM'
    )
    
    declare_enable_nvblox = DeclareLaunchArgument(
        'enable_nvblox',
        default_value='True',
        description='Enable Isaac ROS nvblox mapping'
    )
    
    declare_enable_nav2 = DeclareLaunchArgument(
        'enable_nav2',
        default_value='True',
        description='Enable Nav2 navigation stack'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map file (empty for SLAM mode)'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz for visualization'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(config_dir, 'g1_navigation.rviz'),
        description='Path to RViz config file'
    )
    
    # =========================================================================
    # Isaac ROS Visual SLAM
    # =========================================================================
    # GPU-accelerated visual odometry for 6-DoF pose estimation
    
    vslam_params_file = os.path.join(config_dir, 'vslam_params.yaml')
    
    # Check if Isaac ROS is available
    try:
        isaac_ros_vslam_dir = get_package_share_directory('isaac_ros_visual_slam')
        vslam_available = True
    except Exception:
        vslam_available = False
    
    if vslam_available:
        vslam_node = Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam_node',
            parameters=[vslam_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('visual_slam/tracking/odometry', '/odom'),
            ],
            condition=IfCondition(enable_vslam),
        )
    else:
        # Placeholder node for systems without Isaac ROS
        vslam_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vslam_placeholder',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            condition=IfCondition(enable_vslam),
        )
        print("WARNING: isaac_ros_visual_slam not found. Using static TF.")
    
    # =========================================================================
    # Isaac ROS nvblox
    # =========================================================================
    # GPU-accelerated 3D mapping and costmap generation
    
    nvblox_params_file = os.path.join(config_dir, 'nvblox_params.yaml')
    
    try:
        isaac_ros_nvblox_dir = get_package_share_directory('isaac_ros_nvblox')
        nvblox_available = True
    except Exception:
        nvblox_available = False
    
    if nvblox_available:
        nvblox_node = Node(
            package='isaac_ros_nvblox',
            executable='nvblox_node',
            name='nvblox_node',
            parameters=[nvblox_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('depth/image', '/camera/depth/image_rect_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('color/image', '/camera/color/image_raw'),
                ('color/camera_info', '/camera/color/camera_info'),
            ],
            condition=IfCondition(enable_nvblox),
        )
    else:
        # Placeholder message for systems without nvblox
        nvblox_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nvblox_placeholder',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(enable_nvblox),
        )
        print("WARNING: isaac_ros_nvblox not found. 3D mapping disabled.")
    
    # =========================================================================
    # Nav2 Navigation Stack
    # =========================================================================
    # Include the Nav2 launch file
    
    nav2_params_file = os.path.join(config_dir, 'g1_nav_params.yaml')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'nav2_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav2_params_file,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            # Camera TF is already published in this launch file
            'publish_camera_tf': 'False',
        }.items(),
        condition=IfCondition(enable_nav2),
    )
    
    # =========================================================================
    # DDS-ROS2 Bridge
    # =========================================================================
    # Bridge between Unitree DDS and ROS2
    
    bridge_node = Node(
        package='python3',
        executable=os.path.join(pkg_dir, 'dds_ros2_bridge.py'),
        name='dds_ros2_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # =========================================================================
    # Static Transforms
    # =========================================================================
    # Define robot URDF transforms (base_link -> camera frames)
    
    # Camera transform (base_link -> camera_link)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        # Position camera at robot head height, facing forward
        # Adjust these values based on actual G1 camera mounting
        arguments=[
            '0.1',   # x: 10cm forward from base
            '0',     # y: centered
            '0.9',   # z: 90cm up (head height)
            '0',     # roll
            '0',     # pitch  
            '0',     # yaw
            'base_link',
            'camera_link'
        ],
    )
    
    # Depth camera optical frame
    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf_publisher',
        # Optical frame is rotated (z forward, x right, y down)
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',  # -90° roll, -90° yaw
            'camera_link',
            'camera_depth_optical_frame'
        ],
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_enable_vslam,
        declare_enable_nvblox,
        declare_enable_nav2,
        declare_map,
        declare_use_rviz,
        declare_rviz_config,
        
        # Static transforms
        camera_tf,
        camera_depth_tf,
        
        # Isaac ROS nodes
        vslam_node,
        nvblox_node,
        
        # Nav2 (delayed start to allow SLAM initialization)
        TimerAction(
            period=2.0,
            actions=[nav2_launch],
        ),
        
        # Note: DDS-ROS2 bridge should be started separately
        # to allow proper DDS initialization
    ])
