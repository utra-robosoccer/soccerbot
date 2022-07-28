import math
import os
import unittest
from unittest import TestCase
from unittest.mock import MagicMock

import pytest

from soccer_common.mock_ros import mock_ros

mock_ros()

from sensor_msgs.msg import CameraInfo

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation
from soccer_msgs.msg import RobotState


class TestObjectLocalization(TestCase):
    @unittest.mock.patch("soccer_common.camera.TransformListener")
    @unittest.mock.patch("soccer_common.camera.rospy.Time.now")
    def test_camera_find_floor_coordinate(self, mock_tf_listener, now):
        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = Camera("robot1")
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p2 = c.findFloorCoordinate([360 / 2, 240 / 2])
        self.assertAlmostEqual(p2[0], 0.5)
        self.assertAlmostEqual(p2[1], 0)
        self.assertAlmostEqual(p2[2], 0)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    @unittest.mock.patch("soccer_common.camera.rospy.Time.now")
    def test_camera_find_camera_coordinate(self, mock_tf_listener, now):
        p = Transformation([0, 0, 0.5], euler=[0, math.pi / 4, 0])
        c = Camera("robot1")
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p2 = c.findCameraCoordinate([0.5, 0, 0])
        self.assertAlmostEqual(p2[0], 360 / 2)
        self.assertAlmostEqual(p2[1], 240 / 2)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    @unittest.mock.patch("soccer_common.camera.rospy.Time.now")
    def test_camera_find_camera_coordinate_2(self, mock_tf_listener, now):
        p = Transformation([0, 0, 0.5], euler=[0, 0, 0])
        c = Camera("robot1")
        c.pose = p
        ci = CameraInfo()
        ci.height = 240
        ci.width = 360
        c.camera_info = ci

        p3 = c.findCameraCoordinate([0.5, 0, 0.5])
        self.assertAlmostEqual(p3[0], 360 / 2)
        self.assertAlmostEqual(p3[1], 240 / 2)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    @unittest.mock.patch("soccer_common.camera.rospy.Time.now")
    def test_calculate_bounding_boxes_from_ball(self, mock_tf_listener, now):
        from sensor_msgs.msg import CameraInfo

        for cam_angle in [0, 0.1, -0.1]:
            for cam_position in [[0, 0, 0], [0, 0, 0.1], [0, 0, -0.1]]:
                p = Transformation(cam_position, euler=[cam_angle, 0, 0])
                c = Camera("robot1")
                c.pose = p
                ci = CameraInfo()
                ci.height = 240
                ci.width = 360
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

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    @unittest.mock.patch("soccer_common.camera.rospy.Time.now")
    def test_fieldine_detection(self, mock_tf_listener, now):
        from detector_fieldline import DetectorFieldline
        from sensor_msgs.msg import CameraInfo, Image

        Camera.reset_position = MagicMock()
        Camera.ready = MagicMock()
        d = DetectorFieldline()
        d.robot_state.status = RobotState.STATUS_READY
        d.image_publisher.get_num_connections = MagicMock(return_value=1)
        # d.publish_point_cloud = True
        # d.point_cloud_publisher.get_num_connections = MagicMock(return_value=1)

        import cv2
        from cv2 import Mat
        from cv_bridge import CvBridge

        cvbridge = CvBridge()

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../images"
        for file_name in os.listdir(test_path):
            # file_name = "img0_-0.6805418953941712_-0.19657546078618005_-1.049088190788794.png"

            print(file_name)
            img: Mat = cv2.imread(os.path.join(test_path, file_name))

            c = CameraInfo()
            c.height = img.shape[0]
            c.width = img.shape[1]
            d.camera.camera_info = c

            img_msg: Image = cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
            d.image_publisher.publish = MagicMock()
            d.image_callback(img_msg, debug=False)

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)

                if d.image_publisher.publish.call_count != 0:
                    img_out = cvbridge.imgmsg_to_cv2(d.image_publisher.publish.call_args[0][0])
                    cv2.imshow("After", img_out)

                cv2.waitKey(0)

    @pytest.mark.skip
    def test_hsv_filter(self):
        import cv2
        import numpy as np

        def nothing(x):
            pass

        # Create a window
        cv2.namedWindow("image")

        # create trackbars for color change
        cv2.createTrackbar("HMin", "image", 0, 179, nothing)  # Hue is from 0-179 for Opencv
        cv2.createTrackbar("SMin", "image", 0, 255, nothing)
        cv2.createTrackbar("VMin", "image", 0, 255, nothing)
        cv2.createTrackbar("HMax", "image", 0, 179, nothing)
        cv2.createTrackbar("SMax", "image", 0, 255, nothing)
        cv2.createTrackbar("VMax", "image", 0, 255, nothing)

        # Set default value for MAX HSV trackbars.
        cv2.setTrackbarPos("HMax", "image", 179)
        cv2.setTrackbarPos("SMax", "image", 255)
        cv2.setTrackbarPos("VMax", "image", 255)

        # Initialize to check if HSV min/max value changes
        hMin = sMin = vMin = hMax = sMax = vMax = 0
        phMin = psMin = pvMin = phMax = psMax = pvMax = 0

        img = cv2.imread("../images/img14_-0.4107905429521215_2.4511358573282696_-0.8450607115290163.png")
        output = img
        waitTime = 33

        while 1:

            # get current positions of all trackbars
            hMin = cv2.getTrackbarPos("HMin", "image")
            sMin = cv2.getTrackbarPos("SMin", "image")
            vMin = cv2.getTrackbarPos("VMin", "image")

            hMax = cv2.getTrackbarPos("HMax", "image")
            sMax = cv2.getTrackbarPos("SMax", "image")
            vMax = cv2.getTrackbarPos("VMax", "image")

            # Set minimum and max HSV values to display
            lower = np.array([hMin, sMin, vMin])
            upper = np.array([hMax, sMax, vMax])

            # Create HSV Image and threshold into a range.
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            output = cv2.bitwise_and(img, img, mask=mask)

            # Print if there is a change in HSV value
            if (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax):
                print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin, sMin, vMin, hMax, sMax, vMax))
                phMin = hMin
                psMin = sMin
                pvMin = vMin
                phMax = hMax
                psMax = sMax
                pvMax = vMax

            # Display output image
            cv2.imshow("image", output)

            # Wait longer to prevent freeze for videos.
            if cv2.waitKey(waitTime) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()
