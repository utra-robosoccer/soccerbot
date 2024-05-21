import cv2

from soccer_common.camera import Camera
from soccer_msgs.msg import RobotState


class Detector:
    def __init__(self):
        self.camera = Camera()
        self.camera.reset_position()

        self.robot_state = RobotState()
        self.robot_state.status = RobotState.STATUS_DISCONNECTED

    def robot_state_callback(self, robot_state: RobotState):
        self.robot_state = robot_state

    def circular_mask(self, radius: int):
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius, radius))
