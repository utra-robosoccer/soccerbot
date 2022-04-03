import enum

import numpy
from ball import Ball

try:
    from soccer_msgs.msg import RobotState
except:
    from soccer_msgs.fake_msg import RobotState
# Root class for robot used by 2D, 3D and 3D friendly robots
class Robot:
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
        TRAJECTORY_IN_PROGRESS = RobotState.STATUS_TRAJECTORY_IN_PROGRESS
        PENALIZED = RobotState.STATUS_PENALIZED
        STOPPED = RobotState.STATUS_STOPPED

    def __init__(
        self,
        robot_id=0,
        team=Team.UNKNOWN,
        role=Role.UNASSIGNED,
        status=Status.DISCONNECTED,
        position=numpy.array([0, 0, 0]),
    ):
        self.team = team
        self.role = role
        self.status = status
        self.position = position
        self.robot_id = robot_id
        self.observed_ball = Ball()
