#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Nav2 Bringup Launch File for Unitree G1 Navigation

This launch file starts the Nav2 navigation stack configured for the G1 robot.
It can be used with both Isaac Lab simulation and the real robot.

Components launched:
- Nav2 BT Navigator (behavior tree-based navigation)
- Controller Server (MPPI controller for humanoid motion)
- Planner Server (global path planning)
- Local/Global Costmaps
- Behavior Server (recovery behaviors)
- Lifecycle Manager
- RViz (optional)

Usage:
------
    # With a saved map (localization mode)
    ros2 launch navigation/launch/nav2_bringup.launch.py \\
        use_sim_time:=True \\
        map:=/path/to/map.yaml

    # Without a map (for SLAM mode)
    ros2 launch navigation/launch/nav2_bringup.launch.py \\
        use_sim_time:=True \\
        slam:=True

    # Disable RViz (headless)
    ros2 launch navigation/launch/nav2_bringup.launch.py \\
        use_sim_time:=True \\
        slam:=True \\
        use_rviz:=False

    # If no SLAM/localization is running, publish a static map->odom TF
    ros2 launch navigation/launch/nav2_bringup.launch.py \\
        use_sim_time:=True \\
        slam:=True \\
        use_fake_map_tf:=True

    # Enable pointcloud -> LaserScan for SLAM/AMCL
    ros2 launch navigation/launch/nav2_bringup.launch.py \\
        use_sim_time:=True \\
        slam:=True \\
        use_pointcloud_to_laserscan:=True

Parameters:
-----------
    use_sim_time : bool
        Use simulation time (True for Isaac Lab, False for real robot)
    map : str
        Path to map YAML file (required for localization mode)
    params_file : str
        Path to Nav2 parameters file (defaults to g1_nav_params.yaml)
    slam : bool
        Launch SLAM instead of AMCL localization
    use_rviz : bool
        Launch RViz for visualization
    rviz_config : str
        Path to RViz config file
    use_fake_map_tf : bool
        Publish a static map->odom transform (debug fallback)
    use_pointcloud_to_laserscan : bool
        Convert depth pointcloud to /scan for SLAM/AMCL
    pointcloud_topic : str
        PointCloud2 topic used for scan conversion
    scan_topic : str
        LaserScan output topic
    scan_target_frame : str
        Target frame for LaserScan data (usually base_link)
    publish_camera_tf : bool
        Publish a static base_link->camera_link TF (if no URDF)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to this package's config directory
    # Note: In a standalone repo, we use relative paths
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_params_file = os.path.join(pkg_dir, 'config', 'g1_nav_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_fake_map_tf = LaunchConfiguration('use_fake_map_tf')
    use_pointcloud_to_laserscan = LaunchConfiguration('use_pointcloud_to_laserscan')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    scan_target_frame = LaunchConfiguration('scan_target_frame')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    camera_frame = LaunchConfiguration('camera_frame')
    camera_tf_x = LaunchConfiguration('camera_tf_x')
    camera_tf_y = LaunchConfiguration('camera_tf_y')
    camera_tf_z = LaunchConfiguration('camera_tf_z')
    camera_tf_roll = LaunchConfiguration('camera_tf_roll')
    camera_tf_pitch = LaunchConfiguration('camera_tf_pitch')
    camera_tf_yaw = LaunchConfiguration('camera_tf_yaw')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')
    scan_min_range = LaunchConfiguration('scan_min_range')
    scan_max_range = LaunchConfiguration('scan_max_range')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time (set False for real robot)'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map YAML file for localization'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to Nav2 parameters file'
    )
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Launch SLAM instead of localization'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start Nav2 stack'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz for visualization'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'g1_navigation.rviz'),
        description='Path to RViz config file'
    )

    declare_use_fake_map_tf = DeclareLaunchArgument(
        'use_fake_map_tf',
        default_value='False',
        description='Publish a static map->odom TF when no SLAM/localization is running'
    )

    declare_use_pointcloud_to_laserscan = DeclareLaunchArgument(
        'use_pointcloud_to_laserscan',
        default_value='True',
        description='Convert depth pointcloud to /scan for SLAM/AMCL'
    )

    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/camera/depth/color/points',
        description='PointCloud2 topic used for scan conversion'
    )

    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan output topic'
    )

    declare_scan_target_frame = DeclareLaunchArgument(
        'scan_target_frame',
        default_value='base_link',
        description='Target frame for LaserScan data'
    )

    declare_publish_camera_tf = DeclareLaunchArgument(
        'publish_camera_tf',
        default_value='True',
        description='Publish a static base_link->camera_link TF (if no URDF)'
    )

    declare_camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera link frame (parented to base_link)'
    )

    declare_camera_tf_x = DeclareLaunchArgument(
        'camera_tf_x',
        default_value='0.1',
        description='Camera X offset from base_link (meters)'
    )

    declare_camera_tf_y = DeclareLaunchArgument(
        'camera_tf_y',
        default_value='0.0',
        description='Camera Y offset from base_link (meters)'
    )

    declare_camera_tf_z = DeclareLaunchArgument(
        'camera_tf_z',
        default_value='0.9',
        description='Camera Z offset from base_link (meters)'
    )

    declare_camera_tf_roll = DeclareLaunchArgument(
        'camera_tf_roll',
        default_value='0.0',
        description='Camera roll offset (radians)'
    )

    declare_camera_tf_pitch = DeclareLaunchArgument(
        'camera_tf_pitch',
        default_value='0.0',
        description='Camera pitch offset (radians)'
    )

    declare_camera_tf_yaw = DeclareLaunchArgument(
        'camera_tf_yaw',
        default_value='0.0',
        description='Camera yaw offset (radians)'
    )

    declare_scan_min_height = DeclareLaunchArgument(
        'scan_min_height',
        default_value='0.1',
        description='Minimum height for points used in LaserScan (meters)'
    )

    declare_scan_max_height = DeclareLaunchArgument(
        'scan_max_height',
        default_value='1.5',
        description='Maximum height for points used in LaserScan (meters)'
    )

    declare_scan_min_range = DeclareLaunchArgument(
        'scan_min_range',
        default_value='0.1',
        description='Minimum range for LaserScan (meters)'
    )

    declare_scan_max_range = DeclareLaunchArgument(
        'scan_max_range',
        default_value='10.0',
        description='Maximum range for LaserScan (meters)'
    )
    
    # Try to use nav2_bringup if available, otherwise use inline nodes
    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        nav2_available = True
    except Exception:
        nav2_available = False
        print("WARNING: nav2_bringup not found. Using inline configuration.")
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    fake_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(use_fake_map_tf),
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            camera_tf_x,
            camera_tf_y,
            camera_tf_z,
            camera_tf_roll,
            camera_tf_pitch,
            camera_tf_yaw,
            'base_link',
            camera_frame,
        ],
        condition=IfCondition(publish_camera_tf),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', pointcloud_topic),
            ('scan', scan_topic),
        ],
        parameters=[{
            'target_frame': scan_target_frame,
            'transform_tolerance': 0.1,
            'min_height': scan_min_height,
            'max_height': scan_max_height,
            'range_min': scan_min_range,
            'range_max': scan_max_range,
            'use_inf': True,
        }],
        condition=IfCondition(use_pointcloud_to_laserscan),
    )

    launch_actions = [
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_slam,
        declare_autostart,
        declare_use_rviz,
        declare_rviz_config,
        declare_use_fake_map_tf,
        declare_use_pointcloud_to_laserscan,
        declare_pointcloud_topic,
        declare_scan_topic,
        declare_scan_target_frame,
        declare_publish_camera_tf,
        declare_camera_frame,
        declare_camera_tf_x,
        declare_camera_tf_y,
        declare_camera_tf_z,
        declare_camera_tf_roll,
        declare_camera_tf_pitch,
        declare_camera_tf_yaw,
        declare_scan_min_height,
        declare_scan_max_height,
        declare_scan_min_range,
        declare_scan_max_range,
    ]

    # Launch Nav2 using the standard bringup if available
    if nav2_available:
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
                'params_file': params_file,
                'slam': slam,
                'autostart': autostart,
            }.items()
        )

        launch_actions.extend([
            nav2_launch,
            camera_tf,
            pointcloud_to_laserscan,
            fake_map_tf,
            rviz_node,
        ])
        return LaunchDescription(launch_actions)
    
    # Fallback: Launch Nav2 nodes directly
    # This allows running without the full nav2_bringup package
    
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]
    
    # Controller server (MPPI for humanoid motion)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    
    # Planner server (global path planning)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    
    # Behavior server (recovery behaviors)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    
    # BT Navigator (behavior tree navigation)
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    
    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )
    
    launch_actions.extend([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        camera_tf,
        pointcloud_to_laserscan,
        fake_map_tf,
        rviz_node,
    ])

    return LaunchDescription(launch_actions)
