# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
G1 Navigation Task Module

This module registers the navigation task for the G1 robot.
The task is designed for:
- SLAM mapping with RGB-D camera
- Autonomous navigation with Nav2
- Sim-to-real transfer to Jetson Orin

Task: Isaac-Navigation-G129-Wholebody
- Uses wholebody locomotion (RL policy controls legs)
- RGB-D front camera for Visual SLAM and obstacle detection
- DDS velocity commands compatible with real robot
- Warehouse environment for realistic navigation scenarios
"""

import gymnasium as gym

from . import navigation_g1_29dof_env_cfg


# Register the navigation task
gym.register(
    id="Isaac-Navigation-G129-Wholebody",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": navigation_g1_29dof_env_cfg.NavigationG129WholebodyEnvCfg,
    },
    disable_env_checker=True,
)

# Alternative task with additional obstacles for more challenging scenarios
gym.register(
    id="Isaac-Navigation-G129-Wholebody-Obstacles",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": navigation_g1_29dof_env_cfg.NavigationG129WholebodyObstaclesEnvCfg,
    },
    disable_env_checker=True,
)
