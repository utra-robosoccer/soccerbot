import cv2
import numpy as np
from sensor_msgs.msg import Image

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

    def grass_mask(self, image: Image, h: int):
        # Grass Mask
        # Hue > 115 needed
        grass_only = cv2.inRange(image, (35, 85, 0), (115, 255, 255))
        grass_only = cv2.vconcat([np.zeros((h + 1, grass_only.shape[1]), dtype=grass_only.dtype), grass_only])

        # Use odd numbers for all circular masks otherwise the line will shift location
        grass_only_0 = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, self.circular_mask(5))
        grass_only_1 = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, self.circular_mask(5))
        grass_only_2 = cv2.morphologyEx(grass_only_1, cv2.MORPH_OPEN, self.circular_mask(21))
        grass_only_3 = cv2.morphologyEx(grass_only_2, cv2.MORPH_CLOSE, self.circular_mask(61))

        grass_only_morph = cv2.morphologyEx(grass_only_3, cv2.MORPH_ERODE, self.circular_mask(9))
        grass_only_flipped = cv2.bitwise_not(grass_only)

        return grass_only, grass_only_0, grass_only_1, grass_only_2, grass_only_3, grass_only_morph, grass_only_flipped
