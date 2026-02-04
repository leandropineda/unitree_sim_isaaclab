# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
"""
Navigation scene configuration module

Provides a navigation-focused scene with:
- Open warehouse environment for mobile navigation
- No manipulation objects (focus on locomotion)
- Obstacles for path planning
- Optimized for SLAM and autonomous navigation testing

This scene is designed for:
1. SLAM mapping with Visual SLAM (isaac_ros_visual_slam)
2. 3D reconstruction with nvblox
3. Nav2 autonomous navigation testing
4. Sim-to-real transfer validation
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from tasks.common_config import CameraBaseCfg
import os

project_root = os.environ.get("PROJECT_ROOT")


@configclass
class NavigationSceneCfg(InteractiveSceneCfg):
    """Navigation scene configuration class
    
    Defines a warehouse environment optimized for autonomous navigation:
    - Large open space for path planning
    - Static obstacles (shelves, tables) for obstacle avoidance
    - Good visual features for Visual SLAM
    - Realistic lighting for depth camera operation
    
    Components:
    -----------
    room_walls : AssetBaseCfg
        The warehouse environment with walls, floor, and ceiling.
        Provides visual features for SLAM loop closure.
        
    navigation_obstacles : list[AssetBaseCfg]
        Static obstacles placed throughout the environment.
        Used for testing obstacle avoidance and path planning.
        
    light : AssetBaseCfg
        Dome lighting for consistent illumination.
        Important for RGB camera and Visual SLAM performance.
    """
    
    # 1. Warehouse environment - provides structure for SLAM
    room_walls = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Room",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/small_warehouse/small_warehouse_digital_twin.usd",
        ),
    )

    # 2. Navigation obstacles - tables as static obstacles
    packing_table1 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_1",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-2.35644, -3.45572, -0.2],
            rot=[0.70091, 0.0, 0.0, 0.71325]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable_2/PackingTable.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    packing_table2 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_2",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-3.97225, -4.3424, -0.2],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    # 3. Lighting - important for Visual SLAM and depth cameras
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(
            color=(0.75, 0.75, 0.75),
            intensity=3000.0
        ),
    )
    
    # 4. World camera for visualization/debugging
    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(-1.9, -5.0, 1.8),
        rot_offset=(-0.40614, 0.78544, 0.4277, -0.16986)
    )


@configclass
class NavigationSceneWithObstaclesCfg(NavigationSceneCfg):
    """Extended navigation scene with additional obstacles
    
    Adds more obstacles to the base navigation scene for:
    - More challenging path planning scenarios
    - Testing obstacle avoidance in cluttered environments
    - Validating costmap generation with dense obstacles
    """
    
    # Additional obstacle - box/crate
    obstacle_box1 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/ObstacleBox1",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-3.0, -2.0, 0.25],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.5, 0.5, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.6, 0.4, 0.2),
                metallic=0.0
            ),
        ),
    )
    
    obstacle_box2 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/ObstacleBox2",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-4.5, -3.5, 0.25],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.4, 0.6, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.4, 0.4, 0.6),
                metallic=0.0
            ),
        ),
    )
    
    # Barrel-like cylinder obstacle
    obstacle_barrel = AssetBaseCfg(
        prim_path="/World/envs/env_.*/ObstacleBarrel",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-2.5, -4.5, 0.4],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=sim_utils.CylinderCfg(
            radius=0.3,
            height=0.8,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.3, 0.5, 0.3),
                metallic=0.2
            ),
        ),
    )
    
    # Shelf-like obstacle
    obstacle_shelf = AssetBaseCfg(
        prim_path="/World/envs/env_.*/ObstacleShelf",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-5.0, -2.5, 0.75],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.4, 1.5, 1.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.5, 0.5, 0.5),
                metallic=0.3
            ),
        ),
    )
