import unittest
from os.path import expanduser

import numpy as np

from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_msgs.msg import BoundingBox, BoundingBoxes

REAL_TIME = True

class TestBallLocalization(unittest.TestCase):
    def test_ball_localization(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        bbs_msg = BoundingBoxes()
        bbs_msg.bounding_boxes.append(BoundingBox())
        bbs_msg.bounding_boxes[0].xmin = 289
        bbs_msg.bounding_boxes[0].ymin = 153
        bbs_msg.bounding_boxes[0].xmax = 333
        bbs_msg.bounding_boxes[0].ymax = 197
        bbs_msg.bounding_boxes[0].Class = "0"
        bbs_msg.bounding_boxes[0].probability = 0.9100908
        bbs_msg.bounding_boxes[0].xbase = 311
        bbs_msg.bounding_boxes[0].ybase = 194

        camera_z_height = 0.48848283290863037
        camera_orientation = [0.051347, 0.61632, 0.04393]

        ball_estim = [1.4448, 0.027018, 0.69474]
        ball_actual = [0.98048, 0.056295, -0.2648]

        ball_found = False
        for box in bbs_msg.bounding_boxes:
            if box.Class == "0":
                ball_found = True

        if ball_found:
            ball_pos = detect.get_ball_position(bbs_msg, camera_z_height, camera_orientation)
            # ball_pos = Transformation(position=ball_pos_euler, euler=[0, 0, 0])
            print(
                f"floor pos: {np.round(ball_pos.position, 5).tolist()}  " f"ball: {ball_estim}",
                flush=True,
            )
            detected_ball_pos = np.round(ball_pos.position, 5).tolist()
            self.assertAlmostEqual(detected_ball_pos[0], ball_estim[0], delta=0.001)
            self.assertAlmostEqual(detected_ball_pos[1], ball_estim[1], delta=0.001)
            self.assertAlmostEqual(detected_ball_pos[2], ball_estim[2], delta=0.001)