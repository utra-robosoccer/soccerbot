import os
import os.path
from os.path import expanduser
from unittest import TestCase
from unittest.mock import MagicMock

import cv2
import numpy as np
import rospy
from cv2 import Mat
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from soccer_object_detection.object_detect_node import Label
from soccer_object_detection.object_detect_node_ros import ObjectDetectionNodeRos

from soccer_common.perception.camera_calculations_ros import CameraCalculationsRos
from soccer_common.utils import download_dataset
from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_perception.soccer_object_detection.test.utils import check_bounding_box

set_rosparam_from_yaml_file()


class TestObjectDetectionRos(TestCase):
    # TODO should these just be simple usage
    def test_object_detection_node_ros(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        test_path = src_path + "data/images/simulation"

        download_dataset("https://drive.google.com/uc?id=11nN58j8_PBoLNRAzOEdk7fMe1UK1diCc", folder_path=test_path)

        model_path = src_path + "soccer_object_detection/models/half_5.pt"
        rospy.init_node("test")

        CameraCalculationsRos.reset_position = MagicMock()
        n = ObjectDetectionNodeRos(model_path=model_path)

        cvbridge = CvBridge()
        for file_name in os.listdir(f"{test_path}/images"):
            print(file_name)
            img: Mat = cv2.imread(os.path.join(f"{test_path}/images", file_name))  # ground truth box = (68, 89) (257, 275)
            img = cv2.resize(img, dsize=(640, 480))

            img_msg: Image = cvbridge.cv2_to_imgmsg(img)

            # Mock the detections
            n.pub_detection = MagicMock()
            n.pub_boundingbox = MagicMock()
            n.pub_detection.get_num_connections = MagicMock(return_value=1)
            n.pub_boundingbox.get_num_connections = MagicMock(return_value=1)
            n.pub_detection.publish = MagicMock()
            n.pub_boundingbox.publish = MagicMock()

            n.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            n.callback(img_msg)

            with open(os.path.join(f"{test_path}/labels", file_name.replace("PNG", "txt"))) as f:
                lines = f.readlines()

            if "DISPLAY" in os.environ:
                mat = cvbridge.imgmsg_to_cv2(n.pub_detection.publish.call_args[0][0])
                # cv2.imshow("Image", mat)
                # cv2.waitKey(1000)
                # cv2.destroyAllWindows()

            # Check assertion
            if n.pub_boundingbox.publish.call_args is not None:
                for bounding_box in n.pub_boundingbox.publish.call_args[0][0].bounding_boxes:
                    if bounding_box.probability >= n.CONFIDENCE_THRESHOLD and int(bounding_box.Class) in [Label.BALL.value, Label.ROBOT.value]:
                        best_iou = check_bounding_box(bounding_box, lines, n.camera.camera_info.width, n.camera.camera_info.height)

                        self.assertGreater(best_iou, 0.05, f"bounding boxes are off by too much! Image= {file_name} Best IOU={best_iou}")
                        if best_iou < 0.5:
                            rospy.logwarn(f"bounding boxes lower than 0.5 Image= {file_name} Best IOU={best_iou}")

                        # if "DISPLAY" in os.environ:
                        # cv2.rectangle(
                        #     img=mat,
                        #     pt1=(best_dimensions[0], best_dimensions[1]),
                        #     pt2=(best_dimensions[2], best_dimensions[3]),
                        #     color=(255, 255, 255),
                        # )
                        # if bounding_box.obstacle_detected is True:
                        #     cv2.circle(mat, (bounding_box.xbase, bounding_box.ybase), 0, (0, 255, 255), 3)

            if "DISPLAY" in os.environ:
                cv2.imshow("Image", mat)
                cv2.waitKey()
                cv2.destroyAllWindows()
