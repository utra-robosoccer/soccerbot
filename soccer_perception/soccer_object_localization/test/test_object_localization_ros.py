import os
from os.path import expanduser
from unittest import TestCase
from unittest.mock import MagicMock

import cv2
import rospy
from cv2 import Mat
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from soccer_object_detection.camera.camera_calculations_ros import CameraCalculationsRos
from soccer_object_localization.detector_fieldline_ros import DetectorFieldlineRos

from soccer_common.utils import download_dataset

os.environ["ROS_NAMESPACE"] = "/robot1"


# TODO fix unit test
class TestObjectLocalizationRos(TestCase):
    # TODO Is this necessary or should we have tests like traj to bo on the real robot

    def test_fieldline_detection_ros(self):
        rospy.init_node("test")
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        test_path = src_path + "data/images/fieldlines"

        download_dataset("https://drive.google.com/uc?id=1nJX6ySks_a7mESvCm3sNllmJTNpm-x2_", folder_path=test_path)

        # TODO should do on real world img set
        # TODO we shouldnt need this
        CameraCalculationsRos.reset_position = MagicMock()
        d = DetectorFieldlineRos()

        # ROS
        d.image_publisher.get_num_connections = MagicMock(return_value=1)
        d.publish_point_cloud = True  # TODO is this really needed
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
