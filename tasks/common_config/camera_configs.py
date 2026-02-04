# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
public camera configuration
include the basic configuration for different types of cameras, support scene-specific parameter customization
"""

import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
import os


@configclass
class CameraBaseCfg:
    """camera base configuration class
    
    provide the default configuration for different types of cameras, support scene-specific parameter customization
    """
    
    @classmethod
    def get_camera_config(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot/d435_link/front_cam",
        update_period: float = 0.02,
        height: int = 480,
        width: int =  640,
        focal_length: float = 7.6,
        focus_distance: float = 400.0,
        horizontal_aperture: float = 20.0,
        clipping_range: tuple = (0.1, 1.0e5),
        pos_offset: tuple = (0, 0.0, 0),
        rot_offset: tuple = (0.5, -0.5, 0.5, -0.5),
        data_types: list = None
    ) -> CameraCfg:
        """get the front camera configuration
        
        Args:
            prim_path: the path of the camera in the scene
            update_period: update period (seconds)
            height: image height (pixels)
            width: image width (pixels)
            focal_length: focal length
            focus_distance: focus distance
            horizontal_aperture: horizontal aperture
            clipping_range: clipping range (near clipping plane, far clipping plane)
            pos_offset: position offset (x, y, z)
            rot_offset: rotation offset quaternion
            data_types: data type list
            
        Returns:
            CameraCfg: camera configuration
        """
        if data_types is None:
            data_types = ["rgb"]

        return CameraCfg(
            prim_path=prim_path,
            update_period=update_period,
            height=height,
            width=width,
            data_types=data_types,
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=focal_length,
                focus_distance=focus_distance,
                horizontal_aperture=horizontal_aperture,
                clipping_range=clipping_range
            ),
            offset=CameraCfg.OffsetCfg(
                pos=pos_offset,
                rot=rot_offset,
                convention="ros"
            )
        )
    



@configclass
class CameraPresets:
    """camera preset configuration collection
    
    include the common camera configuration preset for different scenes
    """
    
    @classmethod
    def g1_front_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config()
    @classmethod
    def h12_front_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path = "/World/envs/env_.*/Robot/camera_link/front_cam")
    @classmethod
    def g1_world_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path="/World/envs/env_.*/Robot/d435_link/PerspectiveCamera_robot",
                                                    pos_offset=(-0.9, 0.0, 0.0),
                                                    rot_offset=( -0.51292,0.51292,-0.48674, 0.48674),
                                                    focal_length = 12,
                                                    horizontal_aperture=27)
    @classmethod
    def h12_world_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path="/World/envs/env_.*/Robot/camera_link/PerspectiveCamera_robot",
                                                    pos_offset=(-0.9, 0.0, 0.0),
                                                    rot_offset=( -0.51292,0.51292,-0.48674, 0.48674),
                                                    focal_length = 12,
                                                    horizontal_aperture=27)
    @classmethod
    def left_gripper_wrist_camera(cls) -> CameraCfg:
        """left wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(0.02541028, 0.045, 0.135),
            rot_offset=(-0.34202, 0.93969, 0, 0),
        )
    @classmethod
    def right_gripper_wrist_camera(cls) -> CameraCfg:
        """right wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.02541028, 0.045, 0.135),
            rot_offset=(-0.34202, 0.93969, 0, 0),
        ) 
    @classmethod
    def left_dex3_wrist_camera(cls) -> CameraCfg:
        """left wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_camera_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, -0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        )
    @classmethod
    def right_dex3_wrist_camera(cls) -> CameraCfg:
        """right wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_camera_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, 0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        ) 
    
    @classmethod
    def left_inspire_wrist_camera(cls) -> CameraCfg:
        """left wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_camera_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, -0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        )
    @classmethod
    def right_inspire_wrist_camera(cls) -> CameraCfg:
        """right wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_camera_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, 0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        )
    
    # ========================================================================
    # NAVIGATION CAMERAS - RGB-D cameras for SLAM and obstacle avoidance
    # ========================================================================
    
    @classmethod
    def g1_front_rgbd_camera(cls) -> CameraCfg:
        """Front RGB-D camera for navigation/SLAM (Intel RealSense D435 simulation)
        
        This camera provides both RGB and depth data for:
        - Visual SLAM (isaac_ros_visual_slam)
        - 3D mapping (isaac_ros_nvblox)
        - Obstacle detection for Nav2
        
        Matches the real G1's Intel RealSense D435 specifications.
        Depth range: 0.1m to 10.0m (practical indoor range)
        """
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/d435_link/front_cam",
            height=480,
            width=640,
            update_period=0.033,  # ~30Hz for SLAM
            data_types=["rgb", "depth"],  # RGB-D output
            focal_length=1.93,  # D435 focal length (mm)
            focus_distance=400.0,
            horizontal_aperture=3.896,  # D435 sensor width approximation
            clipping_range=(0.1, 10.0),  # 10cm to 10m depth range
            pos_offset=(0, 0.0, 0),
            rot_offset=(0.5, -0.5, 0.5, -0.5),
        )
    
    @classmethod
    def g1_front_depth_only_camera(cls) -> CameraCfg:
        """Front depth-only camera for lightweight obstacle detection
        
        Use this when you only need depth data (e.g., for costmap generation)
        without the overhead of RGB processing.
        """
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/d435_link/depth_cam",
            height=480,
            width=640,
            update_period=0.05,  # 20Hz for costmap updates
            data_types=["depth"],
            focal_length=1.93,
            focus_distance=400.0,
            horizontal_aperture=3.896,
            clipping_range=(0.1, 10.0),
            pos_offset=(0, 0.0, 0),
            rot_offset=(0.5, -0.5, 0.5, -0.5),
        )
    
    @classmethod
    def h12_front_rgbd_camera(cls) -> CameraCfg:
        """H1-2 front RGB-D camera for navigation/SLAM"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/camera_link/front_cam",
            height=480,
            width=640,
            update_period=0.033,
            data_types=["rgb", "depth"],
            focal_length=1.93,
            focus_distance=400.0,
            horizontal_aperture=3.896,
            clipping_range=(0.1, 10.0),
            pos_offset=(0, 0.0, 0),
            rot_offset=(0.5, -0.5, 0.5, -0.5),
        ) 