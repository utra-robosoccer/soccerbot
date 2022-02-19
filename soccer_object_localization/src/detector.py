from soccer_geometry.camera import Camera
import rospy
from std_msgs.msg import Bool

class Detector:

    def __init__(self):
        self.robot_name = rospy.get_namespace()[1:-1]  # remove '/'
        self.camera = Camera(self.robot_name)
        self.camera.reset_position()

        self.trajectory_complete_subscriber = rospy.Subscriber("trajectory_complete", Bool,
                                                               self.trajectory_complete_callback)
        self.trajectory_complete = True
        pass

    def trajectory_complete_callback(self, trajectory_complete: Bool):
        self.trajectory_complete = trajectory_complete.data

