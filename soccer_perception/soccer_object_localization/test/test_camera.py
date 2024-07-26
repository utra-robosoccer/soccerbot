import math
import os
from unittest import TestCase

from sensor_msgs.msg import CameraInfo

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation


# TODO fix unit test
class TestCamera(TestCase):
    def test_camera_find_floor_coordinate(self):
        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = Camera()
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p2 = c.findFloorCoordinate([360 / 2, 240 / 2])
        self.assertAlmostEqual(p2[0], 0.5, delta=0.005)
        self.assertAlmostEqual(p2[1], 0, delta=0.005)
        self.assertAlmostEqual(p2[2], 0, delta=0.005)

    def test_camera_find_camera_coordinate(self):
        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = Camera()
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p2 = c.findCameraCoordinate([0.5, 0, 0])
        self.assertAlmostEqual(p2[0], 360 / 2, delta=0.5)
        self.assertAlmostEqual(p2[1], 240 / 2, delta=0.5)

    def test_camera_find_camera_coordinate_2(self):
        p = Transformation([0, 0, 0.5], euler=[0, 0, 0])
        c = Camera()
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p3 = c.findCameraCoordinate([0.5, 0, 0.5])
        self.assertAlmostEqual(p3[0], 360 / 2, delta=0.5)
        self.assertAlmostEqual(p3[1], 240 / 2, delta=0.5)
