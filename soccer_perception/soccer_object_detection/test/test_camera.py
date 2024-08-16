import math
from unittest import TestCase

from sensor_msgs.msg import CameraInfo
from soccer_object_detection.camera.camera_calculations import CameraCalculations

from soccer_common.transformation import Transformation


# TODO fix unit test
class TestCamera(TestCase):
    def test_camera_find_floor_coordinate(self):
        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = CameraCalculations()
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p2 = c.find_floor_coordinate([360 / 2, 240 / 2])
        self.assertAlmostEqual(p2[0], 0.5, delta=0.005)
        self.assertAlmostEqual(p2[1], 0, delta=0.005)
        self.assertAlmostEqual(p2[2], 0, delta=0.005)

    def test_camera_find_camera_coordinate(self):
        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = CameraCalculations()
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p2 = c.find_camera_coordinate([0.5, 0, 0])
        self.assertAlmostEqual(p2[0], 360 / 2, delta=0.5)
        self.assertAlmostEqual(p2[1], 240 / 2, delta=0.5)

    def test_camera_find_camera_coordinate_2(self):
        p = Transformation([0, 0, 0.5], euler=[0, 0, 0])
        c = CameraCalculations()
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p3 = c.find_camera_coordinate([0.5, 0, 0.5])
        self.assertAlmostEqual(p3[0], 360 / 2, delta=0.5)
        self.assertAlmostEqual(p3[1], 240 / 2, delta=0.5)

    def test_calculate_bounding_boxes_from_ball(self):
        # rospy.init_node("test")

        for cam_angle in [0, 0.1, -0.1]:

            for cam_position in [[0, 0, 0], [0, 0, 0.1], [0, 0, -0.1]]:
                p = Transformation(cam_position, euler=[cam_angle, 0, 0])
                c = CameraCalculations()
                c.pose = p
                ci = CameraInfo()
                ci.height = 360
                ci.width = 240
                c.camera_info = ci

                positions = [[0.5, 0, 0.1], [0.5, 0, 0], [0.5, 0, 0.1]]
                for position in positions:
                    ball_pose = Transformation(position)
                    ball_radius = 0.07

                    bounding_boxes = c.calculate_bounding_boxes_from_ball(ball_pose, ball_radius)
                    # [[135.87634651355825, 75.87634651355823], [224.12365348644175, 164.12365348644175]]
                    position = c.calculate_ball_from_bounding_boxes(ball_radius, bounding_boxes)

                    self.assertAlmostEqual(position.position[0], ball_pose.position[0], delta=0.001)
                    self.assertAlmostEqual(position.position[1], ball_pose.position[1], delta=0.001)
                    self.assertAlmostEqual(position.position[2], ball_pose.position[2], delta=0.001)
