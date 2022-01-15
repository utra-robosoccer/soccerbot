import enum
import rospy
import tf.transformations
from soccer_msgs.msg import TeamData
from soccer_msgs.msg import GameState


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

    def is_known(self):
        if self.position is None:
            return False
        return len(self.position) > 0


class Team_Data():
    #TODO print stuff to make sure robots are communicating
    def __init__(self):
        self.team_data_sub = rospy.Subscriber("/team_data", TeamData, self.team_data_callback)

        self.robots = {
            1: Team_Data_Robot(),
            2: Team_Data_Robot(),
            3: Team_Data_Robot(),
            4: Team_Data_Robot(),
        }

        self.ball = Team_Data_Ball()
        self.team_color = None
        self.is_first_half = None
        self.secondary_state = None

    def set_gamestate(self, gameState):
        self.team_color = gameState.teamColor
        self.is_first_half = gameState.firstHalf
        self.secondary_state = gameState.secondaryState


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
        self.ball.position = [ball_trans.x, ball_trans.y]
        self.ball.covariance = data.robot_position.covariance


#should make a team_data superclass
class TeamData2D:
    def __init__(self):
        self.robots = {
            1: Team_Data_Robot(),
            2: Team_Data_Robot(),
            3: Team_Data_Robot(),
            4: Team_Data_Robot()
        }
        self.ball = Team_Data_Ball()

    def team_data_callback(self, data):
        robot = data[0]
        ball = data[1]
        if robot.robot_id in self.robots.keys():
            self.robots[robot.robot_id].player_id = robot.robot_id
            self.robots[robot.robot_id].position = robot.position
            self.robots[robot.robot_id].covariance = None

        self.ball.position = ball.position
        self.ball.covariance = None