# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Navigation task reward functions.

For teleoperation and autonomous navigation (Nav2), rewards are not strictly
necessary since we're not training an RL policy for navigation. However,
these reward functions can be useful for:
- Monitoring navigation quality
- Training navigation-specific behaviors
- Evaluating sim-to-real transfer

The main locomotion policy (wholebody) handles its own rewards for walking.
"""

import torch
from isaaclab.envs import ManagerBasedRLEnv


def compute_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Compute navigation reward (placeholder).
    
    For navigation with Nav2, rewards are typically not used since the
    navigation stack handles path planning and execution. This function
    returns a constant reward for compatibility with the RL environment
    interface.
    
    Args:
        env: The Isaac Lab environment instance
        
    Returns:
        Reward tensor (zeros for navigation - no RL training)
    """
    return torch.zeros(env.num_envs, device=env.device)


def velocity_tracking_reward(
    env: ManagerBasedRLEnv,
    target_vel: tuple = (0.0, 0.0, 0.0)
) -> torch.Tensor:
    """Reward for tracking target velocity commands.
    
    Can be used to evaluate how well the robot follows velocity commands
    from Nav2 or teleoperation.
    
    Args:
        env: The Isaac Lab environment instance
        target_vel: Target (vx, vy, vyaw) velocities
        
    Returns:
        Reward based on velocity tracking error
    """
    robot = env.scene["robot"]
    root_vel = robot.data.root_lin_vel_w
    root_ang_vel = robot.data.root_ang_vel_w
    
    # Compute velocity error
    vel_error = torch.zeros(env.num_envs, device=env.device)
    vel_error += (root_vel[:, 0] - target_vel[0]) ** 2  # x velocity
    vel_error += (root_vel[:, 1] - target_vel[1]) ** 2  # y velocity
    vel_error += (root_ang_vel[:, 2] - target_vel[2]) ** 2  # yaw velocity
    
    # Convert error to reward (negative error)
    reward = torch.exp(-vel_error)
    
    return reward


# Export reward functions
__all__ = [
    "compute_reward",
    "velocity_tracking_reward",
]
