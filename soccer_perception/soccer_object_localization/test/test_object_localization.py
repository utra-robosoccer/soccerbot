import os
import time

import rospy
from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_object_localization.detector_fieldline_ros import DetectorFieldlineRos
from soccer_object_localization.detector_goalpost import DetectorGoalPost

from soccer_common.perception.camera_calculations_ros import CameraCalculationsRos

# from soccer_object_detection.object_detect_node import ObjectDetectionNode

# ROS
# from soccer_object_detection.test_object_detection import IoU
# from soccer_object_localization.detector_objects import DetectorObjects

os.environ["ROS_NAMESPACE"] = "/robot1"

import math
from unittest import TestCase
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

# import rosbag
# import rospy
# import tf2_ros
from cv2 import Mat
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from soccer_object_localization.detector_fieldline import DetectorFieldline

from soccer_common.transformation import Transformation
from soccer_common.utils import download_dataset, wrapToPi
from soccer_msgs.msg import GameState, RobotState

# from soccer_object_localization.detector_goalpost import DetectorGoalPost


# TODO fix unit test
class TestObjectLocalization(TestCase):

    def test_fieldline_detection(self):
        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/fieldlines"
        download_dataset(url="https://drive.google.com/uc?id=1nJX6ySks_a7mESvCm3sNllmJTNpm-x2_", folder_path=test_path)

        d = DetectorFieldline()

        for file_name in os.listdir(test_path):
            # file_name = "img160_-1.452993567956688_-3.15_0.7763055830612666.png"

            print(file_name)
            img: Mat = cv2.imread(os.path.join(test_path, file_name))

            lines_only = d.image_filter(img, debug=False)
            points3d = d.img_to_points(lines_only)
            print(points3d)  # TODO need way to vis and verify

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)
                # cv2.imwrite("/tmp/before.png", img) # TODO why

                cv2.imshow("After", lines_only)

                cv2.waitKey(0)
        cv2.destroyAllWindows()

    def test_fieldline_detection_cam(self):
        d = DetectorFieldline()

        cap = cv2.VideoCapture(4)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            img = cv2.resize(frame, dsize=(640, 480))

            lines_only = d.image_filter(img, debug=False)

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)
                # cv2.imwrite("/tmp/before.png", img) # TODO why

                cv2.imshow("After", lines_only)

                cv2.waitKey(0)
        cv2.destroyAllWindows()

    def test_fieldline_detection_vid(self):
        d = DetectorFieldline()

        src_path = os.path.dirname(os.path.realpath(__file__))
        print(src_path + "../../soccer_object_detection/videos/2023-07-08-124521.webm")
        cap = cv2.VideoCapture(src_path + "/../../soccer_object_detection/videos/2023-07-08-124521.webm")
        if not cap.isOpened():
            print("Cannot open camera")
            exit()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            img = cv2.resize(frame, dsize=(640, 480))
            lines_only = d.image_filter(img, debug=False)  # 0.05
            points3d = d.img_to_points(lines_only)
            print(points3d)  # TODO need way to vis and verify

            if "DISPLAY" in os.environ:
                cv2.imshow("Before", img)
                # cv2.imwrite("/tmp/before.png", img) # TODO why

                cv2.imshow("After", lines_only)

                cv2.waitKey(1)
        cv2.destroyAllWindows()

    # def test_robot_detection(self):
    #     src_path = os.path.dirname(os.path.realpath(__file__))
    #     test_path = src_path + "/../../../soccer_object_detection/images/simulation"
    #     download_dataset("https://drive.google.com/uc?id=11nN58j8_PBoLNRAzOEdk7fMe1UK1diCc", folder_path=test_path)
    #
    #     # ROS
    #     # rospy.init_node("test")
    #
    #     Camera.reset_position = MagicMock()
    #
    #     src_path = os.path.dirname(os.path.realpath(__file__))
    #     model_path = src_path + "/../../../soccer_object_detection/models/half_5.pt"
    #
    #     n = ObjectDetectionNode(model_path=model_path)
    #     n.robot_state.status = RobotState.STATUS_READY
    #     n.game_state.gameState = GameState.GAMESTATE_PLAYING
    #
    #     Camera.reset_position = MagicMock()
    #     Camera.ready = MagicMock()
    #     do = DetectorObjects()
    #     do.robot_state.status = RobotState.STATUS_READY
    #
    #     cvbridge = CvBridge()
    #     for file_name in os.listdir(f"{test_path}/images"):
    #         print(file_name)
    #         img: Mat = cv2.imread(os.path.join(f"{test_path}/images", file_name))  # ground truth box = (68, 89) (257, 275)
    #         img_original_size = img.size
    #         img = cv2.resize(img, dsize=(640, 480))
    #         img_msg: Image = cvbridge.cv2_to_imgmsg(img)
    #
    #         # Mock the detections
    #         n.pub_detection = MagicMock()
    #         n.pub_boundingbox = MagicMock()
    #         n.pub_detection.get_num_connections = MagicMock(return_value=1)
    #         n.pub_boundingbox.get_num_connections = MagicMock(return_value=1)
    #         n.pub_detection.publish = MagicMock()
    #         n.pub_boundingbox.publish = MagicMock()
    #
    #         ci = CameraInfo()
    #         ci.height = img.shape[0]
    #         ci.width = img.shape[1]
    #         n.camera.camera_info = ci
    #         do.camera.camera_info = ci
    #         n.camera.pose = Transformation(position=[0, 0, 0.46], orientation_euler=[0, np.pi / 8, 0])
    #         do.camera.pose = n.camera.pose
    #
    #         # n.get_model_output(img_msg)
    #         detection_image = n.get_model_output(img)  # send the image directly
    #
    #         # Check assertion
    #         # if n.pub_boundingbox.publish.call_args is not None:
    #         #     bounding_boxes = n.pub_boundingbox.publish.call_args[0][0]
    #         #     do.objectDetectorCallback(bounding_boxes)
    #
    #         if "DISPLAY" in os.environ:
    #             # mat = cvbridge.imgmsg_to_cv2(n.pub_detection.publish.call_args[0][0])
    #             # cv2.imshow("Image", mat)
    #             # cv2.imshow("Image", img)
    #             cv2.imshow("Detections", detection_image)
    #             cv2.waitKey()
    #
    #     cv2.destroyAllWindows()
