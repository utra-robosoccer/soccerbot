from unittest import TestCase
from unittest.mock import MagicMock

import tf2_ros

tf2_ros.TransformListener = MagicMock()
import rospy

rospy.Time.now = MagicMock(return_value=0)

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation


class Test(TestCase):
    def test_calculate_bounding_boxes_from_ball(self):
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
