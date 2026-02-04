# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Navigation task observation functions.

This module provides observation functions specific to navigation:
- Robot body joint states for locomotion
- RGB-D camera data for SLAM and obstacle detection
- Robot base pose/velocity for odometry

These observations are designed to match what's available on the real robot
to ensure sim-to-real transfer compatibility.
"""

from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states
from tasks.common_observations.camera_state import get_camera_image

# Export all observation functions
__all__ = [
    "get_robot_boy_joint_states",
    "get_camera_image",
]
