import json
import math
import os
import random
import time
from unittest import TestCase

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from tf import TransformListener

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation


class Test(TestCase):
    def test_calculate_bounding_boxes_from_ball(self):
        rospy.init_node("test_soccer_bounding_boxes")

        for cam_angle in [0, 0.1, -0.1]:
            q = Transformation.get_quaternion_from_euler([cam_angle, 0, 0])

            for cam_position in [[0, 0, 0], [0, 0, 0.1], [0, 0, -0.1]]:
                p = Transformation(cam_position, q)
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

                    self.assertAlmostEqual(position.get_position()[0], ball_pose.get_position()[0], delta=0.001)
                    self.assertAlmostEqual(position.get_position()[1], ball_pose.get_position()[1], delta=0.001)
                    self.assertAlmostEqual(position.get_position()[2], ball_pose.get_position()[2], delta=0.001)
