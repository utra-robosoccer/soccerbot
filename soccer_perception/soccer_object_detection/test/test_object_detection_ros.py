import math
import os
import os.path
import pickle
import sys
import time
from unittest import TestCase
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
import rospy
import tf2_ros
import yaml
from cv2 import Mat
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from soccer_object_detection.object_detect_node import Label, ObjectDetectionNode
from soccer_object_detection.object_detect_node_ros import ObjectDetectionNodeRos

from soccer_common import Transformation
from soccer_common.perception.camera_calculations_ros import CameraCalculationsRos
from soccer_common.utils import download_dataset, wrapToPi
from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_msgs.msg import GameState, RobotState

set_rosparam_from_yaml_file()


def IoU(boxA, boxB):
    # determine the (x, y)-coordinates of the intersection rectangle
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    # compute the area of intersection rectangle
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
    # compute the area of both the prediction and ground-truth
    # rectangles
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)
    # return the intersection over union value
    return iou


class TestObjectDetectionRos(TestCase):

    def test_object_detection_node_ros(self):
        src_path = os.path.dirname(os.path.realpath(__file__))
        test_path = src_path + "/../../images/simulation"
        download_dataset("https://drive.google.com/uc?id=11nN58j8_PBoLNRAzOEdk7fMe1UK1diCc", folder_path=test_path)

        model_path = src_path + "/../models/half_5.pt"
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
                        bounding_boxes = [
                            bounding_box.xmin,
                            bounding_box.ymin,
                            bounding_box.xmax,
                            bounding_box.ymax,
                        ]

                        best_iou = 0
                        best_dimensions = None
                        for line in lines:
                            info = line.split(" ")
                            label = int(info[0])
                            if label != int(bounding_box.Class):
                                continue

                            x = float(info[1])
                            y = float(info[2])
                            width = float(info[3])
                            height = float(info[4])

                            xmin = int((x - width / 2) * n.camera.camera_info.width)
                            ymin = int((y - height / 2) * n.camera.camera_info.height)
                            xmax = int((x + width / 2) * n.camera.camera_info.width)
                            ymax = int((y + height / 2) * n.camera.camera_info.height)
                            ground_truth_boxes = [xmin, ymin, xmax, ymax]
                            iou = IoU(bounding_boxes, ground_truth_boxes)
                            if iou > best_iou:
                                best_iou = iou
                                best_dimensions = ground_truth_boxes

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
