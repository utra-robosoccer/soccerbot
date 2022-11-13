import os.path
from unittest import TestCase
from unittest.mock import MagicMock

import cv2
from cv2 import Mat

from soccer_common.mock_ros import mock_ros
from soccer_msgs.msg import GameState, RobotState

mock_ros()


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


class Test(TestCase):
    def test_object_detection_node(self):
        import numpy as np
        import rospy
        import tf2_ros
        from sensor_msgs.msg import Image

        from soccer_common import Camera

        Camera.reset_position = MagicMock()
        tf2_ros.TransformListener = MagicMock()
        rospy.Time.now = MagicMock(return_value=0)
        from sensor_msgs.msg import CameraInfo

        from soccer_object_detection.object_detect_node import ObjectDetectionNode

        src_path = os.path.dirname(os.path.realpath(__file__))
        model_path = src_path + "/small_model/July14.pt"
        test_path = src_path + "/test_image"

        from cv_bridge import CvBridge

        cvbridge = CvBridge()
        for file_name in os.listdir(test_path):
            img: Mat = cv2.imread(os.path.join(test_path, file_name))  # ground truth box = (68, 89) (257, 275)
            img_msg: Image = cvbridge.cv2_to_imgmsg(img)

            n = ObjectDetectionNode(model_path=model_path)
            n.pub_detection = MagicMock()
            n.pub_boundingbox = MagicMock()
            n.pub_detection.get_num_connections = MagicMock(return_value=1)
            n.pub_boundingbox.get_num_connections = MagicMock(return_value=1)
            n.pub_detection.publish = MagicMock()
            n.pub_boundingbox.publish = MagicMock()

            n.robot_state.status = RobotState.STATUS_READY
            n.game_state.gameState = GameState.GAMESTATE_PLAYING

            ci = CameraInfo()
            ci.height = img.shape[0]
            ci.width = img.shape[1]
            n.camera.camera_info = ci
            n.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            n.callback(img_msg)

            # Extract ground truth
            ground_truth_boxes = file_name[:-4].split("_")[1:]  # strip name and .png
            ground_truth_boxes = list(map(int, ground_truth_boxes))

            # Check assertion
            self.assertGreater(n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].probability, n.CONFIDENCE_THRESHOLD)
            bounding_boxes = [
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].xmin,
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].ymin,
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].xmax,
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].ymax,
            ]
            iou = IoU(bounding_boxes, ground_truth_boxes)
            self.assertGreater(iou, 0.8, "bounding boxes are off by too much!")

            if "DISPLAY" in os.environ:
                mat = cvbridge.imgmsg_to_cv2(n.pub_detection.publish.call_args[0][0])
                cv2.imshow("res", mat)
                cv2.waitKey(1)
                cv2.destroyAllWindows()
