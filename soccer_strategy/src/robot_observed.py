import tf
import rospy
import tf.transformations
from soccer_msgs.msg import TeamData
from robot import Robot


class RobotObserved(Robot):
    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED, status=Robot.Status.DISCONNECTED):
        super().__init__(robot_id, team, role, status)

        self.team_data_sub = rospy.Subscriber("/robot" + str(self.robot_id) + "/status", TeamData, self.team_data_callback)

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
