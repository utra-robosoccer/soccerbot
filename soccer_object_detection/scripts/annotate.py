import math
import random
from time import sleep
from unittest import TestCase

import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.spatial.transform import Rotation as R
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

class Test(TestCase):

    def wrapToPi(num: float) -> float:
        rem = (num + np.pi) % (2 * np.pi) - np.pi
        return rem

    def setUp(self) -> None:
        rospy.init_node("soccer_control")

    def set_ball_position(self, x, y):
        ballPublisher = rospy.Publisher("/reset_ball", Pose, queue_size=1, latch=True)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0

        ballPublisher.publish(p)

        pass

    def reset_robot_position(self, x, y, theta, head_angle_1, head_angle_2):
        resetPublisher = rospy.Publisher("/reset_robot", Pose, queue_size=1, latch=True)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0

        r = R.from_euler(seq='ZYX', angles=[0, 0, theta], degrees=False)
        q = r.as_quat()

        p.orientation[0] = q[0]
        p.orientation[1] = q[1]
        p.orientation[2] = q[2]
        p.orientation[3] = q[3]

        resetPublisher.publish(p)

    def test_annotate_ball(self, num_samples=10000):

        field_width = 3.5 #m
        field_height = 4.5
        camera_height = 0.4
        ball_radius = 0.05

        for i in range(num_samples):
            robot_x = random.randrange(field_width * 1000) / 1000
            robot_y = random.randrange(field_height * 1000) / 1000
            robot_theta = random.randrange(math.pi * 1000) / 1000

            ball_x = random.randrange(field_width * 1000) / 1000
            ball_y = random.randrange(field_height * 1000) / 1000

            '''
            y              -B
            ^          -    |
            |      -        |
            |  -            |
            R---------------|---> x
            '''

            ball_robot_theta = math.atan2(ball_y - robot_y, ball_x - robot_x)


            head_angle_1 = random.gauss(0, 10)
            head_angle_2 = 0


            self.set_ball_position(robot_x, robot_y)
            self.reset_robot_position(robot_x, robot_y, ball_robot_theta, head_angle_1, head_angle_2)

