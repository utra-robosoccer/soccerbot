import tf
import rospy
import tf.transformations
from soccer_msgs.msg import RobotState
from robot import Robot


class RobotObserved(Robot):
    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED, status=Robot.Status.DISCONNECTED):
        super().__init__(robot_id=robot_id, team=team, role=role, status=status)

        self.robot_state_subscriber = rospy.Subscriber("/robot" + str(self.robot_id) + "/state", RobotState, self.robot_state_callback)

    def robot_state_callback(self, r: RobotState):
        self.robot_id = r.player_id
        self.status = r.status
        self.role = r.role
        eul = tf.transformations.euler_from_quaternion([r.pose.orientation.x, r.pose.orientation.y, r.pose.orientation.z, r.pose.orientation.w])
        self.position = [r.pose.position.x, r.pose.position.y, eul[2]]

        self.observed_ball.position[0] = r.ball_pose.x
        self.observed_ball.position[1] = r.ball_pose.y

    def set_navigation_position(self, position):
        pass
