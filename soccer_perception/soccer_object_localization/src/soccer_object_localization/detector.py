import cv2
import numpy as np
from sensor_msgs.msg import Image

from soccer_common.perception.camera_calculations import CameraCalculations


class Detector:
    """
    Base class for object-localization  to group common functionality together.
    """

    def __init__(self):
        self.camera = CameraCalculations()
        self.camera.reset_position()
        # TODO remove all mentions of state for strategy

    @staticmethod
    def circular_mask(radius: int):
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius, radius))

    def grass_mask(self, image: Image, h: int, debug: bool = False):
        # Grass Mask
        # Hue > 115 needed
        grass_only = cv2.inRange(image, (35, 85, 0), (115, 255, 255))

        # Use odd numbers for all circular masks otherwise the line will shift location
        grass_only_0 = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, self.circular_mask(5))
        grass_only_1 = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, self.circular_mask(5))
        grass_only_2 = cv2.morphologyEx(grass_only_1, cv2.MORPH_OPEN, self.circular_mask(21))
        grass_only_3 = cv2.morphologyEx(grass_only_2, cv2.MORPH_CLOSE, self.circular_mask(61))

        grass_only_morph = cv2.morphologyEx(grass_only_3, cv2.MORPH_ERODE, self.circular_mask(9))
        grass_only_flipped = cv2.bitwise_not(grass_only)

        if debug:
            cv2.imshow("grass_only", grass_only)
            cv2.imwrite("/tmp/grass_only.png", grass_only)
            cv2.imshow("grass_only_0", grass_only_0)
            cv2.imwrite("/tmp/grass_only_0.png", grass_only_0)
            cv2.imshow("grass_only_1", grass_only_1)
            cv2.imwrite("/tmp/grass_only_1.png", grass_only_1)
            cv2.imshow("grass_only_2", grass_only_2)
            cv2.imwrite("/tmp/grass_only_2.png", grass_only_2)
            cv2.imshow("grass_only_3", grass_only_3)
            cv2.imwrite("/tmp/grass_only_3.png", grass_only_3)
            cv2.imshow("grass_only_morph", grass_only_morph)
            cv2.imwrite("/tmp/grass_only_morph.png", grass_only_morph)
            cv2.imshow("grass_only_flipped", grass_only_flipped)
            cv2.imwrite("/tmp/grass_only_flipped.png", grass_only_flipped)

        return grass_only_morph, grass_only_flipped
