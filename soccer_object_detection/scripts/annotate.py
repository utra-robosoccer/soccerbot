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
    def wrapToPi(num: float) -> float:
        rem = (num + np.pi) % (2 * np.pi) - np.pi
        return rem

    def set_ball_position(self, x, y):
        ballPublisher = rospy.Publisher("/reset_ball", Pose, queue_size=1, latch=True)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0

        ballPublisher.publish(p)

        pass

    def reset_robot_position(self, x, y, theta, head_angle_1, head_angle_2):
        resetPublisher = rospy.Publisher("/robot1/reset_robot", Pose, queue_size=1, latch=True)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0

        r = R.from_euler("ZYX", [theta, 0, 0], degrees=False)
        q = r.as_quat()

        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        resetPublisher.publish(p)

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

    # 1. Run roslaunch soccerbot soccerbot_multi.launch competition:=false fake_localization:=true
    # If complaining missing opencv make sure LD_LIBRARY_PATH env is set
    def test_annotate_ball(self, num_samples=10000):
        os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_pycontrol'")
        os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_trajectories'")
        j = 0

        rospy.init_node("soccer_annotate")
        self.camera = Camera("robot1")

        field_width = 2.5  # m
        field_height = 1.5
        ball_radius = 0.07  # 0.0753 # 0.07 # 0.0962 # 0.0783

        while not rospy.is_shutdown() and not self.camera.ready():
            print("Waiting for camera info")

        tf_listener = TransformListener()

        for i in range(num_samples):
            robot_x = random.uniform(-field_height, field_height)
            robot_y = random.uniform(-field_width, field_width)

            robot_position = [robot_x, robot_y]
            robot_theta = random.uniform(-math.pi, math.pi)

            ball_distance_offset = random.uniform(0.8, 3)
            ball_angle_offset = random.uniform(-math.pi / 5, math.pi / 5)

            ball_offset = [
                math.cos(robot_theta + ball_angle_offset) * ball_distance_offset,
                math.sin(robot_theta + ball_angle_offset) * ball_distance_offset,
                0,
            ]

            ball_position = [ball_offset[0] + robot_position[0], ball_offset[1] + robot_position[1], 0.0783]

            head_angle_1 = 0
            head_angle_2 = 0

            print(robot_position, robot_theta)
            self.reset_robot_position(robot_position[0], robot_position[1], robot_theta, head_angle_1, head_angle_2)
            time.sleep(0.5)
            self.set_ball_position(ball_position[0], ball_position[1])
            time.sleep(0.5)
            # Calculate the frame in the camera
            image_msg = rospy.wait_for_message("/robot1/camera/image_raw", Image)

            # Save the image
            rgb_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            camera_info_K = np.array(self.camera.camera_info.K).reshape([3, 3])
            camera_info_D = np.array(self.camera.camera_info.D)
            image = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

            # Annotate the image automatically
            # Set the camera
            while not rospy.is_shutdown():
                try:
                    (ball_position, rot) = tf_listener.lookupTransform("world", "robot1/ball", image_msg.header.stamp)
                    header = tf_listener.getLatestCommonTime("world", "robot1/ball")
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn_throttle(2, "Cannot find ball transform")

            self.camera.reset_position(publish_basecamera=False, from_world_frame=True, timestamp=image_msg.header.stamp)
            label = self.camera.calculateBoundingBoxesFromBall(Transformation(ball_position), ball_radius)

            # Draw the rectangle
            pt1 = (round(label[0][0]), round(label[0][1]))
            pt2 = (round(label[1][0]), round(label[1][1]))
            image_rect = cv2.rectangle(image.copy(), pt1, pt2, color=(0, 0, 0), thickness=1)

            # cv2.imshow("ball", image_rect)
            # key = cv2.waitKey(0)
            # print(key)
            # if key != 32:
            #     continue

            jsonPath = "../images/bb_img_{}.json".format(j)
            with open(jsonPath, "w") as f:
                json.dump((pt1, pt2), f)
            filePath = "../images/img_{}.jpg".format(j)
            if os.path.exists(filePath):
                os.remove(filePath)
            cv2.imwrite(filePath, image)
            filePath2 = "../images/img_border_{}.jpg".format(j)
            if os.path.exists(filePath2):
                os.remove(filePath2)
            cv2.imwrite(filePath2, image_rect)

            j = j + 1
