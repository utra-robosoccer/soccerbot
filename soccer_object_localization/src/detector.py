from soccer_geometry.camera import Camera
from sensor_msgs.msg import JointState
import rospy

class Detector:

    def __init__(self):
        self.robot_name = rospy.get_namespace()[1:]  # remove '/'
        self.camera = Camera(self.robot_name)
        pass


