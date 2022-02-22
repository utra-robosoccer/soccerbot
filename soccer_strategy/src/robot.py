import enum
from ball import Ball

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
        UNASSIGNED = 0
        GOALIE = 1
        STRIKER = 2
        LEFT_WING = 3
        RIGHT_WING = 4

    class Status(enum.IntEnum):
        DISCONNECTED = 0
        READY = 1
        LOCALIZING = 2
        WALKING = 3
        TERMINATING_WALK = 4
        KICKING = 5
        FALLEN_FRONT = 6
        FALLEN_BACK = 7
        FALLEN_SIDE = 8
        PENALTY = 9
        OUT_OF_BOUNDS = 10
        TRAJECTORY_IN_PROGRESS = 11
        STOPPED = 12  # Game controller

    def __init__(self, robot_id=0, team=Team.UNKNOWN, role=Role.UNASSIGNED, status=Status.DISCONNECTED, position=None):
        self.team = team
        self.role = role
        self.status = status
        self.position = position
        self.robot_id = robot_id
        self.observed_ball = Ball(None)
