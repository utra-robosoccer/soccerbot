import math
import unittest
from unittest import TestCase

from soccer_common.mock_ros import mock_ros

mock_ros()

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation


class Test(TestCase):
    @unittest.mock.patch("soccer_common.camera.TransformListener")
    def test_camera_find_floor_coordinate(self, mock_tf_listener):

        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = Camera("robot1")
        c.pose = p
        c.resolution_x = 360
        c.resolution_y = 240

        p2 = c.findFloorCoordinate([360 / 2, 240 / 2])
        self.assertAlmostEqual(p2[0], 0.5)
        self.assertAlmostEqual(p2[1], 0)
        self.assertAlmostEqual(p2[2], 0)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    def test_camera_find_camera_coordinate(self, mock_tf_listener):
        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = Camera("robot1")
        c.pose = p
        c.resolution_x = 360
        c.resolution_y = 240

        p2 = c.findCameraCoordinate([0.5, 0, 0])
        self.assertAlmostEqual(p2[0], 360 / 2)
        self.assertAlmostEqual(p2[1], 240 / 2)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    def test_camera_find_camera_coordinate_2(self, mock_tf_listener):
        p = Transformation([0, 0, 0.5], euler=[0, 0, 0])
        c = Camera("robot1")
        c.pose = p
        c.resolution_x = 360
        c.resolution_y = 240

        p3 = c.findCameraCoordinate([0.5, 0, 0.5])
        self.assertAlmostEqual(p3[0], 360 / 2)
        self.assertAlmostEqual(p3[1], 240 / 2)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    def test_calculate_bounding_boxes_from_ball(self, mock_tf_listener):

        for cam_angle in [0, 0.1, -0.1]:
            for cam_position in [[0, 0, 0], [0, 0, 0.1], [0, 0, -0.1]]:
                p = Transformation(cam_position, euler=[cam_angle, 0, 0])
                c = Camera("robot1")
                c.pose = p
                c.resolution_x = 360
                c.resolution_y = 240

                positions = [[0.5, 0, 0.1], [0.5, 0, 0], [0.5, 0, 0.1]]
                for position in positions:
                    ball_pose = Transformation(position)
                    ball_radius = 0.07

                    bounding_boxes = c.calculateBoundingBoxesFromBall(ball_pose, ball_radius)
                    # [[135.87634651355825, 75.87634651355823], [224.12365348644175, 164.12365348644175]]
                    position = c.calculateBallFromBoundingBoxes(ball_radius, bounding_boxes)

                    self.assertAlmostEqual(position.position[0], ball_pose.position[0], delta=0.001)
                    self.assertAlmostEqual(position.position[1], ball_pose.position[1], delta=0.001)
                    self.assertAlmostEqual(position.position[2], ball_pose.position[2], delta=0.001)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    def test_fieldine_detection(self, mock_tf_listener):
        from unittest.mock import MagicMock

        from detector_fieldline import DetectorFieldline
        from sensor_msgs.msg import Image

        Camera.reset_position = MagicMock()
        d = DetectorFieldline()
        import os

        import cv2
        from cv2 import Mat
        from cv_bridge import CvBridge

        cvbridge = CvBridge()
        # img: Mat = cv2.imread(os.path.join(test_path, file_name))  # ground truth box = (68, 89) (257, 275)
        # img_msg: Image = cvbridge.cv2_to_imgmsg(img)
        # d.image_callback(img)
        # pass
