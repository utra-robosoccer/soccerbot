#!/usr/bin/env python3

import enum

import numpy as np
import torch
from cv2 import Mat

from soccer_common import CameraCalculations
from soccer_msgs.msg import BoundingBox, BoundingBoxes


# TODO should be somewhere else
class Label(enum.IntEnum):
    # Defines output channels of model
    # Refer to class name enumeration in soccer_object_detection/config/Torso21.yaml
    BALL = 0
    GOALPOST = 1
    ROBOT = 2
    L_INTERSECTION = 3
    T_INTERSECTION = 4
    X_INTERSECTION = 5
    TOPBAR = 6


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    MAGENTA = "\u001b[35m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class ObjectDetectionNode:
    """
    Class for classifying ball, robot, and publish bounding boxes
    input: 480x640x4 bgra8 -> output: 3x200x150
    """

    def __init__(self, model_path):

        self.CONFIDENCE_THRESHOLD = 0.75
        self.cover_horizon_up_threshold = 30

        torch.hub._brvalidate_not_a_forked_repo = (
            lambda a, b, c: True
        )  # https://discuss.pytorch.org/t/help-for-http-error-403-rate-limit-exceeded/125907
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=model_path)

        # ROS
        if torch.cuda.is_available():
            self.model.cuda()  # TODO does this do anything

        self.camera = CameraCalculations()
        self.camera.reset_position()

    def get_model_output(self, image: Mat) -> [Mat, BoundingBoxes]:
        # webots: 480x640x4pixels

        # cover horizon to help robot ignore things outside field
        # TODO do we need a cover horizon?
        h = max(self.camera.calculate_horizon_cover_area() - self.cover_horizon_up_threshold, 0)
        # h = 0
        if image is not None:
            # 1. preprocess image
            img = image[:, :, :3]  # get rid of alpha channel
            img = img[..., ::-1]  # convert bgr to rgb
            img = img[max(0, h + 1) :, :]
            # 2. inference

            results = self.model(img)

            # TODO should be a func
            bbs_msg = BoundingBoxes()
            id = 0
            for prediction in results.xyxy[0]:
                x1, y1, x2, y2, confidence, img_class = prediction.cpu().numpy()  # TODO cuda?
                y1 += h + 1
                y2 += h + 1
                if img_class in [label.value for label in Label] and confidence > self.CONFIDENCE_THRESHOLD:
                    bb_msg = BoundingBox()
                    bb_msg.xmin = round(x1)  # top left of bounding box
                    bb_msg.ymin = round(y1)
                    bb_msg.xmax = round(x2)  # bottom right of bounding box
                    bb_msg.ymax = round(y2)
                    bb_msg.probability = confidence
                    bb_msg.id = id
                    bb_msg.Class = str(int(img_class))
                    # TODO Joanne look the pixels of the image in addition to the bounding box,
                    #  calculate likely foot coordinate xy
                    if bb_msg.Class == "2":

                        # --- simple version just draw the box in bottom ratio of box to detect feet position---
                        # only look at bottom 1/3 of bounding box (assumption: bounding box is of a standing robot)
                        # Calculate ymin value to start checking for black pixels
                        if bb_msg.ymax < self.camera.resolution_y - 5:
                            temp_ymin = round(bb_msg.ymax * 0.85 + bb_msg.ymin * 0.15)
                            midpoint = [(bb_msg.xmax + bb_msg.xmin) / 2, (bb_msg.ymax + temp_ymin) / 2]
                            bb_msg.ybase = round(midpoint[1])
                            bb_msg.xbase = round(midpoint[0])
                            bb_msg.obstacle_detected = True
                    bbs_msg.bounding_boxes.append(bb_msg)
                    id += 1

            detection_image = np.squeeze(results.render())
            # TODO needed for cover horizon but that might not be needed
            detection_image = np.concatenate((np.zeros((h + 1, self.camera.camera_info.width, 3), detection_image.dtype), detection_image))
            detection_image = detection_image[..., ::-1]

            return detection_image, bbs_msg


if __name__ == "__main__":
    pass
