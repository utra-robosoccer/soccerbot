import os

import cv2
import numpy as np
import rospy

from soccer_localization.src.field_lines_ukf import FieldLinesUKF

if __name__ == "__main__":
    f = FieldLinesUKF()
    rospy.spin()
