import enum
import tf
from ball import Ball
import rospy
import tf.transformations

from soccer_msgs.msg import TeamData


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
        KICKING = 4
        FALLEN_FRONT = 5
        FALLEN_BACK = 6
        FALLEN_SIDE = 7
        PENALTY = 8
        OUT_OF_BOUNDS = 9
        TRAJECTORY_IN_PROGRESS = 10
        STOPPED = 11  # Game controller

    def __init__(self, robot_id=0, team=Team.UNKNOWN, role=Role.UNASSIGNED, status=Status.DISCONNECTED, position=None):
        self.team = team
        self.role = role
        self.status = status
        self.position = position
        self.robot_id = robot_id
        self.observed_ball = Ball(None)


class RobotTeamMate(Robot):
    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED, status=Robot.Status.DISCONNECTED):
        super().__init__(robot_id, team, role, status)

        self.team_data_sub = rospy.Subscriber("/robot" + str(self.robot_id) + "/team_data", TeamData, self.team_data_callback)

    def team_data_callback(self, data):
        self.robot_id = data.robot_id
        robot_trans = data.robot_position.pose.position
        robot_rot = data.robot_position.pose.orientation

        robot_eul = tf.transformations.euler_from_quaternion([robot_rot.w, robot_rot.x, robot_rot.y, robot_rot.z])
        self.position = [robot_trans.x, robot_trans.y, robot_eul[2]]
        self.covariance = data.robot_position.covariance

        # Ball information
        ball_trans = data.robot_position.pose.position
        self.observed_ball.position = [ball_trans.x, ball_trans.y]

    def set_navigation_position(self, position):
        pass