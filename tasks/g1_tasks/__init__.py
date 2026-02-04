
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""Unitree G1 robot task module
contains various task implementations for the G1 robot, such as pick and place, motion control, etc.

Task Categories:
----------------
1. Pick and Place Tasks (Joint control, fixed base):
   - Isaac-PickPlace-Cylinder-G129-Dex1-Joint
   - Isaac-PickPlace-Cylinder-G129-Dex3-Joint
   - Isaac-PickPlace-RedBlock-G129-Dex1-Joint
   - etc.

2. Wholebody Tasks (Mobile base + manipulation):
   - Isaac-Move-Cylinder-G129-Dex1-Wholebody
   - Isaac-Move-Cylinder-G129-Dex3-Wholebody
   - etc.

3. Navigation Tasks (Mobile base, RGB-D for SLAM):
   - Isaac-Navigation-G129-Wholebody
   - Isaac-Navigation-G129-Wholebody-Obstacles
"""

# use relative import
from . import pick_place_cylinder_g1_29dof_dex3
from . import pick_place_cylinder_g1_29dof_dex1
from . import pick_place_cylinder_g1_29dof_inspire

from . import pick_place_redblock_g1_29dof_dex1
from . import pick_place_redblock_g1_29dof_dex3
from . import stack_rgyblock_g1_29dof_dex1
from . import stack_rgyblock_g1_29dof_dex3
from . import stack_rgyblock_g1_29dof_inspire
from . import pick_redblock_into_drawer_g1_29dof_dex1
from . import pick_redblock_into_drawer_g1_29dof_dex3
from . import pick_place_redblock_g1_29dof_inspire
from . import move_cylinder_g1_29dof_dex1_wholebody
from . import move_cylinder_g1_29dof_dex3_wholebody
from . import move_cylinder_g1_29dof_inspire_wholebody

# Navigation tasks (SLAM, autonomous navigation)
from . import navigation_g1_29dof

# export all modules
__all__ = [
        # Pick and Place tasks
        "pick_place_cylinder_g1_29dof_dex3", "pick_place_cylinder_g1_29dof_dex1", 
        "pick_place_redblock_g1_29dof_dex1", "pick_place_redblock_g1_29dof_dex3", 
        "stack_rgyblock_g1_29dof_dex1", "stack_rgyblock_g1_29dof_dex3", 
        "stack_rgyblock_g1_29dof_inspire",
        "pick_redblock_into_drawer_g1_29dof_dex1","pick_redblock_into_drawer_g1_29dof_dex3",
        "pick_place_redblock_g1_29dof_inspire",
        "pick_place_cylinder_g1_29dof_inspire",
        # Wholebody tasks (mobile manipulation)
        "move_cylinder_g1_29dof_dex1_wholebody",
        "move_cylinder_g1_29dof_dex3_wholebody",
        "move_cylinder_g1_29dof_inspire_wholebody",
        # Navigation tasks (SLAM, autonomous navigation)
        "navigation_g1_29dof",
]