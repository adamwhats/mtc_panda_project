"""
Defines classes for storing task information
"""
from dataclasses import dataclass
from typing import Any, NamedTuple

from geometry_msgs.msg import PoseStamped

#########################
#       Motion tasks    #                          
#########################

class joint_target(NamedTuple):
    """
    Stores the target joint values for a move_joint task
    """
    q1: float
    q2: float
    q3: float
    q4: float
    q5: float
    q6: float
    q7: float


@dataclass
class motion_task:
    """
    Base class for storing information for moveit planning
    """
    goal: Any
    planner: str
    vel_scale: float = 0.5
    acc_scale: float = 0.3


@dataclass
class move_configuration(motion_task):
    """
    Stores information for a moveit motion to a named configuration
    """
    goal: str


@dataclass
class move_pose(motion_task):
    """
    Stores information for a moveit motion to a pose in task space
    """
    goal: PoseStamped


@dataclass
class move_joint(motion_task):
    """
    Stores information for a moveit motion to a pose in joint space   
    """
    goal: joint_target


#########################
#       Modbus tasks    #
#########################

class modbus_task(NamedTuple):
    """
    States of each coil of the modbus IO in the MTC Panda Cell
    """
    c1: bool = False
    c2: bool = False
    c3: bool = False
    c4: bool = False
    c5: bool = False
    c6: bool = False
    c7: bool = False
    c8: bool = False
    c9: bool = False
    c10: bool = False
    c11: bool = False
    c12: bool = False


if __name__ == '__main__':
    pass