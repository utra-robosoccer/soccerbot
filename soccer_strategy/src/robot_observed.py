import tf
import rospy
import tf.transformations
from soccer_msgs.msg import RobotState
from robot import Robot


class RobotObserved(Robot):
    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED, status=Robot.Status.DISCONNECTED):
        super().__init__(robot_id, team, role, status)

        self.robot_state_subscriber = rospy.Subscriber("/robot" + str(self.robot_id) + "/state", RobotState, self.robot_state_callback)

    def robot_state_callback(self, r):
        self.status = r.status
        self.role = r.role

    def set_navigation_position(self, position):
        pass
