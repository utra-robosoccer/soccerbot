from soccer_geometry.camera import Camera
import rospy

class Detector:

    def __init__(self):
        self.robot_name = rospy.get_namespace()[1:-1]  # remove '/'
        self.camera = Camera(self.robot_name)
        self.camera.reset_position()
        pass


