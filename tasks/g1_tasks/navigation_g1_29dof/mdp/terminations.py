# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Navigation task termination conditions.

For navigation tasks, we typically don't terminate episodes based on
task completion (unlike manipulation tasks). Instead, termination may occur:
- If the robot falls (stability check)
- If the robot leaves the navigation area bounds
- Manual termination via reset command

These conditions ensure safe operation and episode management.
"""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg


def robot_fell(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Check if the robot has fallen based on pelvis height.
    
    The G1 robot's pelvis should maintain a minimum height during normal
    operation. If the pelvis drops below a threshold, the robot has likely
    fallen and the episode should terminate.
    
    Args:
        env: The Isaac Lab environment instance
        
    Returns:
        Boolean tensor indicating which environments have fallen robots
    """
    # Get robot root state (pelvis position)
    robot = env.scene["robot"]
    root_pos = robot.data.root_pos_w
    
    # Check if pelvis height is below minimum (robot has fallen)
    # G1 standing height is ~0.8m, fallen would be < 0.4m
    min_height = 0.4
    has_fallen = root_pos[:, 2] < min_height
    
    return has_fallen


def out_of_bounds(
    env: ManagerBasedRLEnv,
    bounds_min: tuple = (-10.0, -10.0),
    bounds_max: tuple = (10.0, 10.0)
) -> torch.Tensor:
    """Check if the robot has left the navigation area.
    
    This prevents the robot from wandering too far from the intended
    navigation area, which could cause issues with the simulation or
    represent unrealistic scenarios.
    
    Args:
        env: The Isaac Lab environment instance
        bounds_min: Minimum (x, y) coordinates of navigation area
        bounds_max: Maximum (x, y) coordinates of navigation area
        
    Returns:
        Boolean tensor indicating which environments have out-of-bounds robots
    """
    robot = env.scene["robot"]
    root_pos = robot.data.root_pos_w
    
    # Check x and y bounds
    out_x = (root_pos[:, 0] < bounds_min[0]) | (root_pos[:, 0] > bounds_max[0])
    out_y = (root_pos[:, 1] < bounds_min[1]) | (root_pos[:, 1] > bounds_max[1])
    
    return out_x | out_y


# Export termination functions
__all__ = [
    "robot_fell",
    "out_of_bounds",
]
