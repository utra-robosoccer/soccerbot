from soccer_geometry.camera import Camera
import rospy
from soccer_msgs.msg import RobotState

class Detector:

    def __init__(self):
        self.robot_name = rospy.get_namespace()[1:-1]
        self.camera = Camera(self.robot_name)
        self.camera.reset_position()

        self.robot_state_subscriber = rospy.Subscriber("state", RobotState,
                                                               self.robot_state_callback)
        self.robot_state = RobotState()
        self.robot_state.status = RobotState.STATUS_DISCONNECTED
        pass

    def robot_state_callback(self, robot_state: RobotState):
        self.robot_state = robot_state

