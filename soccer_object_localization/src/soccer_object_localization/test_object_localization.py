import os

import cv2
import pytest
import rospy
from cv2 import Mat
from cv_bridge import CvBridge

from soccer_object_localization.detector_goalpost import DetectorGoalPost

os.environ["ROS_NAMESPACE"] = "/robot1"

import math
from unittest import TestCase
from unittest.mock import MagicMock

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation
from soccer_common.utils import download_dataset, wrapToPi
from soccer_msgs.msg import RobotState
from soccer_object_localization.detector_fieldline import DetectorFieldline


class TestObjectLocalization(TestCase):
    def test_camera_find_floor_coordinate(self):
        rospy.init_node("test")
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

    def test_camera_find_camera_coordinate(self):
        rospy.init_node("test")
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

    def test_camera_find_camera_coordinate_2(self):
        rospy.init_node("test")
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

    def test_calculate_bounding_boxes_from_ball(self):
        rospy.init_node("test")

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

    def test_fieldline_detection(self):
        rospy.init_node("test")

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/fieldlines"
        download_dataset(url="https://drive.google.com/uc?id=1nJX6ySks_a7mESvCm3sNllmJTNpm-x2_", folder_path=test_path)

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
            d.image_callback(img_msg, debug=False)

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)
                cv2.imwrite("/tmp/before.png", img)

                if d.image_publisher.publish.call_count != 0:
                    img_out = cvbridge.imgmsg_to_cv2(d.image_publisher.publish.call_args[0][0])
                    cv2.imshow("After", img_out)
                    cv2.imwrite("/tmp/after.png", img_out)

                cv2.waitKey(0)
        cv2.destroyAllWindows()

    def test_goalpost_detection(self):

        """
        Returns whether a point at a given field coordinate is visible to the robot
        """
        rospy.init_node("test")

        def get_point_visibility(robot_pose, point_coords):
            robot_x, robot_y, robot_yaw = robot_pose
            point_x, point_y = point_coords

            point_yaw = math.atan2(point_y - robot_y, point_x - robot_x)
            camera_fov = 1.39626  # rads

            # Both yaw angles are between -pi and pi
            delta_yaw = wrapToPi(point_yaw - robot_yaw)

            # Check if the point is within the view cone
            # No equals case as the point wouldn't be fully visible
            is_point_visible = -camera_fov / 2.0 < delta_yaw < camera_fov / 2.0

            return is_point_visible

        """
            Returns a dictionary that stores booleans indicating whether each post is visible
            Visual reference: https://www.desmos.com/calculator/b9lndsb1bl
            Example: both posts of the left net are visible
            visible_posts = {
                "NEG_X_NET": {
                    "POS_Y_POST": True,
                    "NEG_Y_POST": True
                },
                "POS_X_NET": {
                    "POS_Y_POST": False,
                    "NEG_Y_POST": False
                }
            }
        """

        def get_visible_posts(robot_x, robot_y, robot_yaw):
            visible_posts = {"NEG_X_NET": {"POS_Y_POST": True, "NEG_Y_POST": True}, "POS_X_NET": {"POS_Y_POST": False, "NEG_Y_POST": False}}

            net_coords = {
                "NEG_X_NET": {"POS_Y_POST": [-4.5, 1.3], "NEG_Y_POST": [-4.5, -1.3]},
                "POS_X_NET": {"POS_Y_POST": [4.5, 1.3], "NEG_Y_POST": [4.5, -1.3]},
            }

            for net in net_coords.keys():
                post_coords = net_coords[net]
                for post in post_coords.keys():
                    visible_posts[net][post] = get_point_visibility((robot_x, robot_y, robot_yaw), net_coords[net][post])

            return visible_posts

        # Setup test environment:

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/goal_net"
        download_dataset(url="https://drive.google.com/uc?id=17qdnW7egoopXHvakiNnUUufP2MOjyZ18", folder_path=test_path)

        Camera.reset_position = MagicMock()
        Camera.ready = MagicMock()
        d = DetectorGoalPost()
        d.robot_state.status = RobotState.STATUS_DETERMINING_SIDE
        d.camera.pose = Transformation(position=[0, 0, 0.46])
        d.image_publisher.get_num_connections = MagicMock(return_value=1)

        cvbridge = CvBridge()

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/goal_net"

        # Loop through test images
        for file_name in os.listdir(test_path):
            # file_name = "img173_-0.852141317992289_3.15_-1.7125376246657054.png"

            print(f"Loading {file_name} from goal_net dataset")
            file_name_no_ext = os.path.splitext(file_name)[0]
            x, y, yaw = file_name_no_ext.split("_")[1:]
            yaw = wrapToPi(float(yaw))
            if yaw < 0:
                yaw = (yaw + np.pi) % (np.pi)

            d.camera.pose.orientation_euler = [yaw, 0, 0]
            print(f"Parsed (x, y, yaw): ({x}, {y}, {yaw}) from filename.")
            visible_posts = get_visible_posts(float(x), float(y), float(yaw))
            for net in visible_posts.keys():
                for post in visible_posts[net].keys():
                    if visible_posts[net][post]:
                        print(f"{net}, {post} is visible")

            img: Mat = cv2.imread(os.path.join(test_path, file_name))

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)
                cv2.waitKey(0)

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

    @pytest.mark.skip
    def test_goalpost_detection_tune(self):
        rospy.init_node("test")

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/goal_net"
        download_dataset(url="https://drive.google.com/uc?id=17qdnW7egoopXHvakiNnUUufP2MOjyZ18", folder_path=test_path)

        Camera.reset_position = MagicMock()
        Camera.ready = MagicMock()
        d = DetectorGoalPost()

        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/goal_net"
        file_name = "img341_0.960470105738711_-3.15_1.3585673890175494.png"

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
        default_threshold = 30
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

            vertical_lines, img_out = d.get_vlines_from_img(
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
