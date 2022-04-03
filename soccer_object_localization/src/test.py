import math
from unittest import TestCase

import rospy

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation


class Test(TestCase):
    def setUp(self) -> None:
        rospy.init_node("camera_test")
        pass

    def test_camera_find_floor_coordinate(self):
        q = Transformation.get_quaternion_from_euler([0, math.pi / 4, 0])
        p = Transformation([0, 0, 0.5], q)
        c = Camera("robot1")
        c.pose = p
        c.resolution_x = 360
        c.resolution_y = 240

        p2 = c.findFloorCoordinate([360 / 2, 240 / 2])
        self.assertAlmostEqual(p2[0], 0.5)
        self.assertAlmostEqual(p2[1], 0)
        self.assertAlmostEqual(p2[2], 0)

    def test_camera_find_camera_coordinate(self):
        q = Transformation.get_quaternion_from_euler([0, math.pi / 4, 0])
        p = Transformation([0, 0, 0.5], q)
        c = Camera("robot1")
        c.pose = p
        c.resolution_x = 360
        c.resolution_y = 240

        p2 = c.findCameraCoordinate([0.5, 0, 0])
        self.assertAlmostEqual(p2[0], 360 / 2)
        self.assertAlmostEqual(p2[1], 240 / 2)

    def test_camera_find_camera_coordinate_2(self):
        q = Transformation.get_quaternion_from_euler([0, 0, 0])
        p = Transformation([0, 0, 0.5], q)
        c = Camera("robot1")
        c.pose = p
        c.resolution_x = 360
        c.resolution_y = 240

        p3 = c.findCameraCoordinate([0.5, 0, 0.5])
        self.assertAlmostEqual(p3[0], 360 / 2)
        self.assertAlmostEqual(p3[1], 240 / 2)
