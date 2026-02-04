# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Navigation Environment Configuration for G1 Robot

This module defines the Isaac Lab environment configuration for G1 navigation tasks.
It provides:
- RGB-D camera for Visual SLAM and obstacle detection
- Wholebody locomotion control via DDS velocity commands
- Warehouse scene for realistic indoor navigation
- Contact sensors for fall detection

Key Features:
-------------
1. RGB-D Camera: Simulates Intel RealSense D435 with both RGB and depth
2. DDS Compatibility: Same velocity command format as real robot
3. Sim-to-Real: Designed to transfer to Jetson Orin with Isaac ROS

Usage:
------
    python sim_main.py --task Isaac-Navigation-G129-Wholebody \\
        --robot_type g129 --enable_wholebody_dds
"""

import torch
from dataclasses import MISSING

from pink.tasks import FrameTask

import isaaclab.envs.mdp as base_mdp
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import ContactSensorCfg

from . import mdp
from tasks.common_config import G1RobotPresets, CameraPresets
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager

# Import navigation scene
from tasks.common_scene.base_scene_navigation import NavigationSceneCfg, NavigationSceneWithObstaclesCfg


# =============================================================================
# Scene Configuration
# =============================================================================

@configclass
class NavigationSceneG129Cfg(NavigationSceneCfg):
    """Navigation scene with G1 robot (Inspire hand) and RGB-D camera
    
    Components:
    -----------
    robot : ArticulationCfg
        G1 29-DOF robot with Inspire 5-finger hand and wholebody locomotion.
        Positioned in the warehouse for navigation testing.
        
    contact_forces : ContactSensorCfg
        Contact sensor for detecting falls and collisions.
        Used for safety monitoring and episode termination.
        
    hand_contact_forces : ContactSensorCfg
        Tactile contact sensors on the Inspire hand fingertips.
        Used for grasp detection and manipulation feedback.
        
    front_camera : CameraCfg
        Intel RealSense D435 simulation providing RGB-D data.
        - RGB: For Visual SLAM feature tracking
        - Depth: For 3D mapping and obstacle detection
        
    left_wrist_camera / right_wrist_camera : CameraCfg
        Wrist-mounted cameras for manipulation tasks.
    """
    
    # G1 robot with Inspire 5-finger hand and wholebody locomotion
    # Initial position in the warehouse, facing forward
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_inspire_wholebody(
        init_pos=(-3.9, -2.81811, 0.8),
        init_rot=(1, 0, 0, 0)
    )
    
    # Contact sensor for fall detection and body collisions
    # Covers the entire robot including Inspire hands
    contact_forces = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*",
        history_length=10,
        track_air_time=True,
        debug_vis=False
    )
    
    # RGB-D camera for navigation (Visual SLAM + obstacle detection)
    # This is the primary sensor for navigation - provides both RGB and depth
    front_camera = CameraPresets.g1_front_rgbd_camera()
    
    # Inspire hand wrist cameras for manipulation
    left_wrist_camera = CameraPresets.left_inspire_wrist_camera()
    right_wrist_camera = CameraPresets.right_inspire_wrist_camera()


@configclass
class NavigationSceneG129ObstaclesCfg(NavigationSceneWithObstaclesCfg):
    """Navigation scene with Inspire hand and additional obstacles"""
    
    # G1 robot with Inspire 5-finger hand
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_inspire_wholebody(
        init_pos=(-3.9, -2.81811, 0.8),
        init_rot=(1, 0, 0, 0)
    )
    
    # Contact sensor for fall detection and body collisions
    contact_forces = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*",
        history_length=10,
        track_air_time=True,
        debug_vis=False
    )
    
    # RGB-D camera for navigation - provides both RGB and depth
    front_camera = CameraPresets.g1_front_rgbd_camera()
    
    # Inspire hand wrist cameras
    left_wrist_camera = CameraPresets.left_inspire_wrist_camera()
    right_wrist_camera = CameraPresets.right_inspire_wrist_camera()


# =============================================================================
# MDP Configuration
# =============================================================================

@configclass
class ActionsCfg:
    """Action configuration for navigation
    
    Actions control the robot joints. For wholebody navigation:
    - Leg joints are controlled by the RL locomotion policy
    - The policy receives velocity commands from DDS (Nav2 or teleoperation)
    - Arm joints can be held at default positions or controlled separately
    """
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=True
    )


@configclass
class ObservationsCfg:
    """Observation configuration for navigation
    
    Observations include:
    - Robot joint states: For locomotion policy
    - Camera images: RGB-D data for SLAM and obstacle detection
    
    Note: The actual SLAM processing happens outside Isaac Lab,
    in the Isaac ROS nodes running on the same machine or Jetson.
    """
    
    @configclass
    class PolicyCfg(ObsGroup):
        """Policy observation group
        
        Contains all observations needed for the locomotion policy.
        Camera data is also exposed for external SLAM/navigation systems.
        """
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        camera_image = ObsTerm(func=mdp.get_camera_image)
        
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False
    
    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    """Termination conditions for navigation
    
    Navigation episodes terminate when:
    - Robot falls (safety concern)
    - Robot leaves navigation bounds (simulation constraint)
    
    Note: Unlike manipulation tasks, navigation doesn't have a "success"
    termination. The task runs continuously until manually reset.
    """
    # Robot fall detection
    robot_fell = DoneTerm(func=mdp.robot_fell)


@configclass
class RewardsCfg:
    """Reward configuration for navigation
    
    Navigation rewards are minimal since we're not training a navigation
    policy (Nav2 handles planning). The locomotion policy has its own
    reward structure.
    """
    reward = RewTerm(func=mdp.compute_reward, weight=1.0)


@configclass
class EventCfg:
    """Event configuration for navigation
    
    Events handle:
    - Scene reset when episode terminates
    - Robot pose reset to initial position
    """
    pass


# =============================================================================
# Environment Configuration
# =============================================================================

@configclass
class NavigationG129WholebodyEnvCfg(ManagerBasedRLEnvCfg):
    """Complete environment configuration for G1 navigation
    
    This configuration creates an Isaac Lab environment suitable for:
    1. SLAM mapping with RGB-D camera
    2. Autonomous navigation with Nav2
    3. Teleoperation testing
    4. Sim-to-real validation
    
    The environment exposes:
    - RGB-D camera data (via shared memory or DDS)
    - Robot state (joint positions, velocities)
    - Velocity command interface (DDS rt/run_command/cmd)
    
    Example Usage:
    --------------
    # Launch simulation
    python sim_main.py --task Isaac-Navigation-G129-Wholebody \\
        --robot_type g129 --enable_wholebody_dds
    
    # In another terminal, run keyboard teleop
    python send_commands_keyboard.py
    
    # Or connect Nav2 via DDS-ROS2 bridge
    python navigation/dds_ros2_bridge.py
    """
    
    # Scene configuration
    scene: NavigationSceneG129Cfg = NavigationSceneG129Cfg(
        num_envs=1,
        env_spacing=2.5,
        replicate_physics=True
    )
    
    # MDP configuration
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events = EventCfg()
    commands = None
    rewards: RewardsCfg = RewardsCfg()
    curriculum = None
    
    def __post_init__(self):
        """Post-initialization configuration
        
        Sets simulation parameters optimized for navigation:
        - 200Hz physics (5ms timestep) for stable locomotion
        - 50Hz control (decimation=4) matching DDS update rate
        - Long episodes (navigation is continuous)
        """
        # Control frequency
        self.decimation = 4  # 50Hz control from 200Hz physics
        self.episode_length_s = 300.0  # 5 minutes per episode (navigation is long)
        
        # Physics settings
        self.sim.dt = 0.005  # 200Hz physics
        self.scene.contact_forces.update_period = self.sim.dt
        self.sim.render_interval = self.decimation
        
        # PhysX settings for stable locomotion
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        
        # Ground friction for stable walking
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.friction_combine_mode = "max"
        self.sim.physics_material.restitution_combine_mode = "max"
        
        # Event manager for scene resets
        self.event_manager = SimpleEventManager()
        
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device)
            )
        ))


@configclass
class NavigationG129WholebodyObstaclesEnvCfg(NavigationG129WholebodyEnvCfg):
    """Navigation environment with additional obstacles
    
    Same as base navigation environment but with more obstacles for:
    - Testing obstacle avoidance
    - Validating local costmap generation
    - Challenging path planning scenarios
    """
    
    scene: NavigationSceneG129ObstaclesCfg = NavigationSceneG129ObstaclesCfg(
        num_envs=1,
        env_spacing=2.5,
        replicate_physics=True
    )
