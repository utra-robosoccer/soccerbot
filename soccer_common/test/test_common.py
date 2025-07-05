from unittest import TestCase

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo

from soccer_common.transformation import Transformation


class TestCommon(TestCase):
    def test_transformation(self):
        t = Transformation(quaternion=[0, 0, 1, 0]) @ Transformation(position=[1, 0, 0])
        assert np.all(t.quaternion == [0, 0, 1, 0])
        assert np.all(t.position == [-1, 0, 0])
