import enum
from typing import List, Optional

import numpy as np

from soccer_msgs.msg import RobotState
from soccer_strategy.ball import Ball
from soccer_strategy.obstacle import Obstacle


class Robot:
    """
    Root class for robot used by 2D, 3D and 3D friendly robots
           Robot
         /        \
    RobotObserved   Robot Controlled
                    /                  \
             Robot Controlled 2D     Robot Controlled 3D

    Robot Controlled 2D is only used for the 2D simulation and should be used with game_engine_2d
    Robot Controlled 3D is only used for the real robot and webots simulation and used with game_engine_3d
    """

    class Team(enum.IntEnum):
        UNKNOWN = 0
        FRIENDLY = 1
        OPPONENT = 2

    class TeamColor(enum.IntEnum):
        UNKNOWN_TEAM = 0
        BLUE = 1
        RED = 2

    class Role(enum.IntEnum):
        UNASSIGNED = RobotState.ROLE_UNASSIGNED
        GOALIE = RobotState.ROLE_GOALIE
        STRIKER = RobotState.ROLE_STRIKER
        LEFT_WING = RobotState.ROLE_LEFT_WING
        RIGHT_WING = RobotState.ROLE_RIGHT_WING

    # Status flow diagram
    # https://drive.google.com/file/d/107bBTbxOhYO6FOo8361nhH7ug5c5HON9/view?usp=sharing
    class Status(enum.IntEnum):
        DISCONNECTED = RobotState.STATUS_DISCONNECTED
        DETERMINING_SIDE = RobotState.STATUS_DETERMINING_SIDE
        READY = RobotState.STATUS_READY
        LOCALIZING = RobotState.STATUS_LOCALIZING
        WALKING = RobotState.STATUS_WALKING
        TERMINATING_WALK = RobotState.STATUS_TERMINATING_WALK
        KICKING = RobotState.STATUS_KICKING
        FALLEN_FRONT = RobotState.STATUS_FALLEN_FRONT
        FALLEN_BACK = RobotState.STATUS_FALLEN_BACK
        FALLEN_SIDE = RobotState.STATUS_FALLEN_SIDE
        GETTING_BACK_UP = RobotState.STATUS_GETTING_BACK_UP
        PENALIZED = RobotState.STATUS_PENALIZED
        STOPPED = RobotState.STATUS_STOPPED

        FALLEN_BACK_LEFT = RobotState.STATUS_FALLEN_BACK
        FALLEN_BACK_RIGHT = RobotState.STATUS_FALLEN_BACK

    def __init__(
        self,
        robot_id=0,
        team=Team.UNKNOWN,
        role=Role.UNASSIGNED,
        status=Status.DISCONNECTED,
        position=np.array([0, 0, 0]),
    ):
        """
        Initializes the robot structure

        :param robot_id: Integer representing the robot's id
        :param team: Which team the robot is on
        :param role: Which role on the field
        :param status: What is the current status of the robot
        :param position: X, Y position of the robot where Y is the short length of the field and X is the long length
        """
        self.team = team
        self.role = role
        self.status = status
        self.position: np.ndarray = position
        self.localized: bool = False
        self.robot_id: int = robot_id
        self.observed_ball: Optional[Ball] = None
        self.observed_obstacles: List[Obstacle] = []
