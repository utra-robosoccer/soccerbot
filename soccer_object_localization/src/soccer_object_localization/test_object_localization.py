import math
import os
import unittest
from pathlib import Path
from unittest import TestCase
from unittest.mock import MagicMock

import gdown
import pytest

from soccer_common.mock_ros import mock_ros

mock_ros()

from sensor_msgs.msg import CameraInfo

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation
from soccer_msgs.msg import RobotState


def download_dataset(url, folder_name):
    # Collect the dataset from the web (https://drive.google.com/drive/u/2/folders/1amhnKBSxHFzkH7op1SusShJckcu-jiUn)
    src_path = os.path.dirname(os.path.realpath(__file__))
    test_path = src_path + f"/../../images/{folder_name}"
    bag_path = Path(test_path)

    if not bag_path.is_dir():
        print(f"Dataset not found at {test_path}. Downloading ...")
        os.makedirs(bag_path)

        zipfilepath = test_path + "/dataset.zip"
        gdown.download(url=url, output=zipfilepath, quiet=False)
        import zipfile

        with zipfile.ZipFile(zipfilepath, "r") as zip_ref:
            zip_ref.extractall(test_path)
        os.remove(zipfilepath)


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
        self.assertAlmostEqual(p2[0], 0.5, delta=0.005)
        self.assertAlmostEqual(p2[1], 0, delta=0.005)
        self.assertAlmostEqual(p2[2], 0, delta=0.005)

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
        self.assertAlmostEqual(p2[0], 360 / 2, delta=0.5)
        self.assertAlmostEqual(p2[1], 240 / 2, delta=0.5)

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
        self.assertAlmostEqual(p3[0], 360 / 2, delta=0.5)
        self.assertAlmostEqual(p3[1], 240 / 2, delta=0.5)

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
    def test_fieldline_detection(self, mock_tf_listener, now):
        from sensor_msgs.msg import CameraInfo, Image

        from soccer_object_localization.detector_fieldline import DetectorFieldline

        download_dataset(url="https://drive.google.com/uc?id=1nJX6ySks_a7mESvCm3sNllmJTNpm-x2_", folder_name="fieldlines")

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
        test_path = src_path + "/../../images/goal_net"
        for file_name in os.listdir(test_path):
            file_name = "img160_-1.452993567956688_-3.15_0.7763055830612666.png"

            print(file_name)
            img: Mat = cv2.imread(os.path.join(test_path, file_name))

            c = CameraInfo()
            c.height = img.shape[0]
            c.width = img.shape[1]
            d.camera.camera_info = c

            img_msg: Image = cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
            d.image_publisher.publish = MagicMock()
            d.image_callback(img_msg, debug=True)

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)
                cv2.imwrite("/tmp/before.png", img)

                if d.image_publisher.publish.call_count != 0:
                    img_out = cvbridge.imgmsg_to_cv2(d.image_publisher.publish.call_args[0][0])
                    cv2.imshow("After", img_out)
                    cv2.imwrite("/tmp/after.png", img_out)

                cv2.waitKey(0)

    @unittest.mock.patch("soccer_common.camera.TransformListener")
    @unittest.mock.patch("soccer_common.camera.rospy.Time.now")
    def test_goalpost_detection(self, mock_tf_listener, now):
        import numpy as np

        # Draws viewcone lines from the robot's position to the left or right goaline to check whether either post is
        # visible to the robot
        def check_net(robot_x, robot_y, robot_yaw, check_left_net=True):
            camera_fov = 1.39626  # rads
            left_net_coords = [[-4.5, 1.3], [-4.5, -1.3]]
            right_net_coords = [[4.5, 1.3], [4.5, -1.3]]
            if check_left_net:
                net_coords = left_net_coords
                net_str = "Left net"
            else:  # Check right net
                net_coords = right_net_coords
                net_str = "Right net"

            x_dist = net_coords[0][0] - robot_x
            top_post_y = net_coords[0][1]
            bottom_post_y = net_coords[1][1]

            # Get slope of viewcone edge lines
            vcone_l_edge_slope = math.tan(robot_yaw + camera_fov / 2)
            vcone_r_edge_slope = math.tan(robot_yaw - camera_fov / 2)

            vcone_l_edge_goal_line_y = vcone_l_edge_slope * x_dist + robot_y
            vcone_r_edge_goal_line_y = vcone_r_edge_slope * x_dist + robot_y

            # TODO Need some intelligence for dealing with the viewcone line intersection being behind the robot:
            # ex: https://www.desmos.com/calculator/b9lndsb1bl, black line has positive slope
            # maybe check if POI is greater than abs(robot_y)?
            y_u_bound = max(vcone_r_edge_goal_line_y, vcone_l_edge_goal_line_y)
            y_l_bound = min(vcone_r_edge_goal_line_y, vcone_l_edge_goal_line_y)

            is_net_visible = False
            if y_l_bound <= top_post_y <= y_u_bound:
                print(f"{net_str}'s top post in viewcone")
                is_net_visible = True
            if y_l_bound <= bottom_post_y <= y_u_bound:
                print(f"{net_str}'s bottom post in viewcone")
                is_net_visible = True

            return is_net_visible

        def check_left_net(robot_pose):
            robot_x, robot_y, robot_yaw = robot_pose
            is_net_visible = check_net(robot_x, robot_y, robot_yaw, check_left_net=True)
            visible_net = "LEFT" if is_net_visible else None
            return visible_net

        def check_right_net(robot_pose):
            robot_x, robot_y, robot_yaw = robot_pose
            is_net_visible = check_net(robot_x, robot_y, robot_yaw, check_left_net=False)
            visible_net = "RIGHT" if is_net_visible else None
            return visible_net

        # Determines which net is visible, if any, returning "RIGHT", "LEFT" or None
        def determine_net_side(robot_x, robot_y, robot_yaw):
            robot_pose = [robot_x, robot_y, robot_yaw]
            if robot_y <= 0:
                if robot_yaw > np.pi / 2:
                    visible_net = check_left_net(robot_pose)
                elif robot_yaw < np.pi / 2:
                    visible_net = check_right_net(robot_pose)
                else:  # robot_yaw == np.pi / 2
                    if robot_x >= 0:
                        visible_net = check_right_net(robot_pose)
                    else:  # robot_x < 0:
                        visible_net = check_left_net(robot_pose)

            else:  # robot_y > 0
                if robot_yaw > -np.pi / 2:
                    visible_net = check_right_net(robot_pose)
                elif robot_yaw < -np.pi / 2:
                    visible_net = check_left_net(robot_pose)
                else:  # robot_yaw == -np.pi / 2
                    if robot_x >= 0:
                        visible_net = check_right_net(robot_pose)
                    else:  # robot_x < 0:
                        visible_net = check_left_net(robot_pose)

            return visible_net

        from sensor_msgs.msg import CameraInfo, Image

        from soccer_object_localization.detector_goalpost import DetectorGoalPost

        download_dataset(url="https://drive.google.com/uc?id=17qdnW7egoopXHvakiNnUUufP2MOjyZ18", folder_name="goal_net")

        Camera.reset_position = MagicMock()
        Camera.ready = MagicMock()
        d = DetectorGoalPost()
        d.robot_state.status = RobotState.STATUS_DETERMINING_SIDE
        d.image_publisher.get_num_connections = MagicMock(return_value=1)

        import cv2
        from cv2 import Mat
        from cv_bridge import CvBridge

        cvbridge = CvBridge()

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/goal_net"
        for file_name in os.listdir(test_path):
            # file_name = "img160_-1.452993567956688_-3.15_0.7763055830612666.png"

            print(f"Loading {file_name} from goal_net dataset")
            file_name_no_ext = os.path.splitext(file_name)[0]
            x, y, yaw = file_name_no_ext.split("_")[1:]
            print(f"Parsed (x, y, yaw): ({x}, {y}, {yaw}) from filename.")
            visible_net = determine_net_side(float(x), float(y), float(yaw))
            print(f"{visible_net} is visible")

            img: Mat = cv2.imread(os.path.join(test_path, file_name))

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)

            c = CameraInfo()
            c.height = img.shape[0]
            c.width = img.shape[1]
            d.camera.camera_info = c

            img_msg: Image = cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
            d.image_publisher.publish = MagicMock()
            d.image_callback(img_msg, debug=True)

            if "DISPLAY" in os.environ:
                if d.image_publisher.publish.call_count != 0:
                    img_out = cvbridge.imgmsg_to_cv2(d.image_publisher.publish.call_args[0][0])
                    cv2.imshow("After", img_out)

                cv2.waitKey(0)

    # @pytest.mark.skip
    @unittest.mock.patch("soccer_common.camera.TransformListener")
    @unittest.mock.patch("soccer_common.camera.rospy.Time.now")
    def test_goalpost_detection_tune(self, mock_tf_listener, now):
        from sensor_msgs.msg import CameraInfo, Image

        from soccer_object_localization.detector_goalpost import DetectorGoalPost

        download_dataset(url="https://drive.google.com/uc?id=17qdnW7egoopXHvakiNnUUufP2MOjyZ18", folder_name="goal_net")

        Camera.reset_position = MagicMock()
        Camera.ready = MagicMock()
        d = DetectorGoalPost()

        import cv2
        import numpy as np
        from cv2 import Mat

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/goal_net"
        file_name = "img160_-1.452993567956688_-3.15_0.7763055830612666.png"

        print(f"Loading {file_name} from goal_net dataset")
        file_name_no_ext = os.path.splitext(file_name)[0]
        x, y, yaw = file_name_no_ext.split("_")[1:]
        print(f"Parsed (x, y, yaw): ({x}, {y}, {yaw}) from filename.")

        img: Mat = cv2.imread(os.path.join(test_path, file_name))

        c = CameraInfo()
        c.height = img.shape[0]
        c.width = img.shape[1]
        d.camera.camera_info = c

        # Create a window
        cv2.namedWindow("image")

        def nothing(x):
            pass

        # create trackbars for hough line parameters
        default_vline_angle_tol_deg = 3
        default_theta = np.pi / 180
        default_threshold = 50
        default_min_line_length = 30
        default_max_line_gap = 10
        cv2.createTrackbar("vLineAngleTolDeg", "image", 0, 15, nothing)
        cv2.createTrackbar("threshold", "image", 0, 255, nothing)
        cv2.createTrackbar("minLineLength", "image", 0, 250, nothing)
        cv2.createTrackbar("maxLineGap", "image", 0, 20, nothing)

        # Set default value for MAX HSV trackbars.
        cv2.setTrackbarPos("vLineAngleTolDeg", "image", default_vline_angle_tol_deg)
        cv2.setTrackbarPos("threshold", "image", default_threshold)
        cv2.setTrackbarPos("minLineLength", "image", default_min_line_length)
        cv2.setTrackbarPos("maxLineGap", "image", default_max_line_gap)

        while True:
            # get current positions of all trackbars
            vline_angle_tol_deg = cv2.getTrackbarPos("vLineAngleTolDeg", "image")
            threshold = cv2.getTrackbarPos("threshold", "image")
            min_line_length = cv2.getTrackbarPos("minLineLength", "image")
            max_line_gap = cv2.getTrackbarPos("maxLineGap", "image")

            img_out = d.get_vlines_from_img(
                img,
                debug=False,
                angle_tol_deg=vline_angle_tol_deg,
                hough_theta=default_theta,
                hough_threshold=threshold,
                hough_min_line_length=min_line_length,
                hough_max_line_gap=max_line_gap,
            )
            cv2.imshow("image", img_out)

            if cv2.waitKey(33) & 0xFF == ord("n"):
                file_name = "newfile"
                break

    # @pytest.mark.skip
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

        img = cv2.imread("../../images/goal_net/img160_-1.452993567956688_-3.15_0.7763055830612666.png")
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
