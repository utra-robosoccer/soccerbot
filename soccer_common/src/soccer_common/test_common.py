from unittest import TestCase
from unittest.mock import MagicMock

import numpy as np
import tf2_ros

tf2_ros.TransformListener = MagicMock()
import rospy

rospy.Time.now = MagicMock(return_value=0)

from sensor_msgs.msg import CameraInfo

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation


class Test(TestCase):
    def test_transformation(self):
        t = Transformation(quaternion=[0, 0, 1, 0]) @ Transformation(position=[1, 0, 0])
        assert np.all(t.quaternion == [0, 0, 1, 0])
        assert np.all(t.position == [-1, 0, 0])

    def test_calculate_bounding_boxes_from_ball(self):
        for cam_angle in [0, 0.1, -0.1]:

            for cam_position in [[0, 0, 0], [0, 0, 0.1], [0, 0, -0.1]]:
                p = Transformation(cam_position, euler=[cam_angle, 0, 0])
                c = Camera("robot1")
                c.pose = p
                ci = CameraInfo()
                ci.height = 360
                ci.width = 240
                c.camera_info = ci

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
