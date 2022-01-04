import enum
import rospy
import tf.transformations
from soccer_msgs.msg import TeamData


class Team_Data_Robot():
    class Team(enum.IntEnum):
        UNKNOWN_TEAM = 0
        BLUE = 1
        RED = 2

    def __init__(self):
        self.team = Team_Data_Robot.Team.UNKNOWN_TEAM
        self.player_id = 0
        self.position = []
        self.covariance = []


class Team_Data_Ball():
    def __init__(self):
        self.position = []
        self.covariance = []


class Team_Data():
    def __init__(self):
        self.team_data_sub = rospy.Subscriber("/team_data", TeamData, self.team_data_callback)

        self.robots = {
            1: Team_Data_Robot(),
            2: Team_Data_Robot(),
            3: Team_Data_Robot(),
            4: Team_Data_Robot(),
        }

        self.ball = Team_Data_Ball()


    def team_data_callback(self, data):
        if data.robot_id in self.robots.keys():
            self.robots[data.robot_id].player_id = data.robot_id
            robot_trans = data.robot_position.pose.position
            robot_rot = data.robot_position.pose.orientation

            robot_eul = tf.transformations.euler_from_quaternion(robot_rot)
            self.robots[data.robot_id].position = [robot_trans.x, robot_trans.y, robot_eul[2]]
            self.robots[data.robot_id].covariance = data.robot_position.covariance
        else:
            rospy.loginfo(f"{data.robot_id} is not a valid robot id")

        #todo filter ball info
        ball_trans = data.robot_position.pose.position
        self.robots[data.robot_id].position = [ball_trans.x, ball_trans.y]
        self.robots[data.robot_id].covariance = data.robot_position.covariance


