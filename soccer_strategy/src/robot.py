import enum

import numpy

from ball import Ball

try:
    from soccer_msgs.msg import RobotState
except:
    class RobotState:
        ROLE_UNASSIGNED = 1
        ROLE_GOALIE = 2
        ROLE_STRIKER = 3
        ROLE_LEFT_WING = 4
        ROLE_RIGHT_WING = 5
        STATUS_DISCONNECTED = 6
        STATUS_DETERMINING_SIDE = 7
        STATUS_READY = 8
        STATUS_LOCALIZING = 9
        STATUS_WALKING = 10
        STATUS_TERMINATING_WALK = 11
        STATUS_KICKING = 12
        STATUS_FALLEN_FRONT = 13
        STATUS_FALLEN_BACK = 14
        STATUS_KICKING = 15
        STATUS_TRAJECTORY_IN_PROGRESS = 16
        STATUS_PENALTY = 17
        STATUS_OUT_OF_BOUNDS = 18
        STATUS_STOPPED = 19

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

    # https://drive.google.com/file/d/107bBTbxOhYO6FOo8361nhH7ug5c5HON9/view?usp=sharing
    class Role(enum.IntEnum):
        UNASSIGNED = RobotState.ROLE_UNASSIGNED
        GOALIE = RobotState.ROLE_GOALIE
        STRIKER = RobotState.ROLE_STRIKER
        LEFT_WING = RobotState.ROLE_LEFT_WING
        RIGHT_WING = RobotState.ROLE_RIGHT_WING

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
        FALLEN_SIDE = RobotState.STATUS_KICKING
        TRAJECTORY_IN_PROGRESS = RobotState.STATUS_TRAJECTORY_IN_PROGRESS
        PENALTY = RobotState.STATUS_PENALTY
        OUT_OF_BOUNDS = RobotState.STATUS_OUT_OF_BOUNDS
        STOPPED = RobotState.STATUS_STOPPED


    def __init__(self, robot_id=0, team=Team.UNKNOWN, role=Role.UNASSIGNED, status=Status.DISCONNECTED, position=numpy.array([0, 0, 0])):
        self.team = team
        self.role = role
        self.status = status
        self.position = position
        self.robot_id = robot_id
        self.observed_ball = Ball()
