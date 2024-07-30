import os

import rospy
from soccer_object_localization.detector_fieldline_ros import DetectorFieldlineRos

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
class TestObjectLocalizationRos(TestCase):
    # TODO Is this necessary or should we have tests like traj to bo on the real robot

    def test_fieldline_detection_ros(self):
        rospy.init_node("test")
        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/fieldlines"
        download_dataset(url="https://drive.google.com/uc?id=1nJX6ySks_a7mESvCm3sNllmJTNpm-x2_", folder_path=test_path)

        # TODO should do on real world img set
        # TODO we shouldnt need this
        CameraCalculationsRos.reset_position = MagicMock()
        CameraCalculationsRos.ready = MagicMock()

        d = DetectorFieldlineRos()

        # ROS
        d.image_publisher.get_num_connections = MagicMock(return_value=1)
        d.publish_point_cloud = True
        d.point_cloud_publisher.get_num_connections = MagicMock(return_value=1)

        cvbridge = CvBridge()

        for file_name in os.listdir(test_path):
            # file_name = "img160_-1.452993567956688_-3.15_0.7763055830612666.png"

            print(file_name)
            img: Mat = cv2.imread(os.path.join(test_path, file_name))

            # ROS
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
