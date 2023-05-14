#!/usr/bin/env python3

import enum
import os

import numpy as np
from matplotlib import pyplot as plt
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from argparse import ArgumentParser

import cv2
import rospy
import torch
from cv_bridge import CvBridge
from rospy import ROSException
from sensor_msgs.msg import Image

from soccer_common.camera import Camera
from soccer_msgs.msg import BoundingBox, BoundingBoxes, GameState, RobotState


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


class ObjectDetectionNode(object):

    """
    Detect ball, robot
    publish bounding boxes
    input: 480x640x4 bgra8 -> output: 3x200x150
    """

    def __init__(self, model_path):
        self.SOCCER_BALL = 0
        self.CONFIDENCE_THRESHOLD = rospy.get_param("~ball_confidence_threshold", 0.75)

        torch.hub._validate_not_a_forked_repo = (
            lambda a, b, c: True
        )  # https://discuss.pytorch.org/t/help-for-http-error-403-rate-limit-exceeded/125907
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=model_path)
        if torch.cuda.is_available():
            rospy.loginfo(f"{bcolors.OKGREEN}Using CUDA for object detection{bcolors.ENDC}")
            self.model.cuda()
        else:
            rospy.logwarn("Not using CUDA")

        self.robot_name = rospy.get_namespace()[1:-1]  # remove '/'
        self.camera = Camera(self.robot_name)
        self.camera.reset_position()

        # Params
        self.br = CvBridge()

        self.pub_detection = rospy.Publisher("detection_image", Image, queue_size=1, latch=True)
        self.pub_boundingbox = rospy.Publisher("object_bounding_boxes", BoundingBoxes, queue_size=1, latch=True)
        self.image_subscriber = rospy.Subscriber(
            "camera/image_raw", Image, self.callback, queue_size=1, buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/)
        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.robot_state_callback)
        self.robot_state = RobotState()
        self.game_state_subscriber = rospy.Subscriber("gamestate", GameState, self.game_state_callback)
        self.game_state = GameState()

    def game_state_callback(self, game_state: GameState):
        self.game_state = game_state

    def robot_state_callback(self, robot_state: RobotState):
        self.robot_state = robot_state

    def callback(self, msg: Image):
        # webots: 480x640x4pixels
        if self.robot_state.status not in [
            RobotState.STATUS_LOCALIZING,
            RobotState.STATUS_READY,
            RobotState.ROLE_UNASSIGNED,
        ]:
            return

        if self.game_state.gameState != GameState.GAMESTATE_PLAYING:
            return

        rospy.loginfo_once("Object Detection Receiving image")
        # width x height x channels (bgra8)
        image = self.br.imgmsg_to_cv2(msg)
        self.camera.reset_position(timestamp=msg.header.stamp)

        # cover horizon to help robot ignore things outside field
        cover_horizon_up_threshold = rospy.get_param("cover_horizon_up_threshold", 30)
        h = max(self.camera.calculateHorizonCoverArea() - cover_horizon_up_threshold, 0)

        if image is not None:
            # 1. preprocess image
            img = image[:, :, :3]  # get rid of alpha channel
            img = img[..., ::-1]  # convert bgr to rgb
            img = img[max(0, h + 1) :, :]
            # 2. inference

            results = self.model(img)

            bbs_msg = BoundingBoxes()
            id = 0
            for prediction in results.xyxy[0]:
                x1, y1, x2, y2, confidence, img_class = prediction.cpu().numpy()
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
                    bbs_msg.bounding_boxes.append(bb_msg)
                    id += 1

                    # TODO Joanne look the pixels of the image in addition to the bounding box,
                    #  calculate likely foot coordinate xy
                    if bb_msg.Class == "2":

                        # --- simple version just draw the box in bottom 1/3 of box to detect feet position---
                        # only look at bottom 1/3 of bounding box (assumption: bounding box is of a standing robot)
                        # Calculate ymin value to start checking for black pixels
                        temp_ymin = round(bb_msg.ymin + ((bb_msg.ymax - bb_msg.ymin) / 4) * 3)
                        midpoint = [(bb_msg.xmax + bb_msg.xmin) / 2, (bb_msg.ymax + temp_ymin) / 2]
                        bb_msg.ybase = round(midpoint[1])
                        bb_msg.xbase = round(midpoint[0])

                        cv2.rectangle(image, (bb_msg.xmin, temp_ymin), (bb_msg.xmax, bb_msg.ymax), (255, 0, 0), 3)

                        cv2.circle(image, (bb_msg.xbase, bb_msg.ybase), 5, (0, 255, 255), 3)
                        cv2.imshow("test", image)
                        cv2.waitKey()
                        # ----------------------------------------

                        # gray_pixel_img = np.dot(img[..., :3], [0.299, 0.587, 0.114])
                        thresh = 128
                        # make the image greyscale so that we can also account for pixels may not exactly be black
                        # (0,0,0) due to lighting situatioins
                        # img_grayscale = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                        # cv2.imshow('Gray image', img_grayscale)
                        # cv2.waitKey()
                        # global thresholding?
                        # thresh = cv2.threshold(img_segments, 0, 255, cv2.THRESH_BINARY)[1]

                        # can also try adaptive thresholding if fixed thresholding doesn't perform well
                        # thresh is an image that contains only black and white pixels
                        # grayscale_final = cv2.adaptiveThreshold(img_grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                        #                                         cv2.THRESH_BINARY, 11, 2)
                        # cv2.imshow('final gray image', grayscale_final)
                        # cv2.waitKey()
                        # only look at bottom 1/3 of bounding box (assumption: bounding box is of a standing robot)
                        # identify pixels that are black
                        # black_pixels = []
                        # Calculate ymin value to start checking for black pixels
                        # temp_ymin = round(bb_msg.ymin + ((bb_msg.ymax - bb_msg.ymin) / 4) * 3)
                        # cv2.rectangle(grayscale_final, (bb_msg.xmin, bb_msg.ymin-h), (bb_msg.xmax, bb_msg.ymax-h),
                        #              (255, 0, 0), 3)
                        # cv2.imshow("test1", grayscale_final)
                        # cv2.waitKey()

                        # # Convert the image to the HSV color space
                        # hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
                        # # Define the range of green color in HSV
                        # lower_green = np.array([25, 52, 72])
                        # upper_green = np.array([102, 255, 255])
                        # # Threshold the image to get only green pixels
                        # mask = cv2.inRange(hsv, lower_green, upper_green)
                        #
                        # # Replace the green pixels with white in the entire image
                        # hsv[mask > 0] = [0, 0, 255]
                        #
                        # # Convert HSV back to BGR color space
                        # img_no_green = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                        # cv2.imshow('img_no_green', img_no_green)
                        # cv2.waitKey(0)
                        #
                        # # Write the modified image to a NumPy array
                        # img_array = np.asarray(img_no_green)
                        # cv2.imshow('Modified Image', img_array)
                        # cv2.waitKey(0)
                        #
                        # # for i in range(bb_msg.xmin, bb_msg.xmax):
                        # #     for j in range(temp_ymin, bb_msg.ymax):
                        # #         if mask[i][j] == 255:
                        # #             img[i][j] = [255, 255, 255]
                        # img_grayscale = cv2.cvtColor(img_array, cv2.COLOR_RGB2GRAY)
                        # # Show the result
                        # cv2.imshow('result', img_grayscale)
                        # cv2.waitKey(0)
                        #
                        # grayscale_final = cv2.adaptiveThreshold(img_grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                        #                                         cv2.THRESH_BINARY, 11, 2)
                        # cv2.imshow('final gray image', grayscale_final)
                        # cv2.waitKey()
                        # cv2.destroyAllWindows()

                        # only look at bottom 1/3 of bounding box (assumption: bounding box is of a standing robot)
                        # identify pixels that are black
                        # black_pixels = []
                        # Calculate ymin value to start checking for black pixels
                        # temp_ymin = round(bb_msg.ymin + ((bb_msg.ymax - bb_msg.ymin) / 4) * 3)
                        # cv2.rectangle(grayscale_final, (bb_msg.xmin, temp_ymin), (bb_msg.xmax, bb_msg.ymax,),
                        #               (255, 0, 0), 3)

                        # crop_img = grayscale_final[bb_msg.xmin: bb_msg.xmax, temp_ymin:bb_msg.ymax,]
                        # cv2.imshow('crop image', grayscale_final)
                        # cv2.waitKey()

                        # for row in range(bb_msg.xmin, bb_msg.xmax):
                        #     for col in range(temp_ymin, bb_msg.ymax):
                        #         if grayscale_final[row][col] == 0:
                        #             # find black pixels in the bottom 1/3 of the bounding box
                        #             black_pixels.append((row, col))
                        #
                        # black_pixel_ymin = min(black_pixels, key=lambda x: x[1])[1]
                        # black_pixel_ymax = max(black_pixels, key=lambda x: x[1])[1]
                        #
                        # black_pixel_xmin = min(black_pixels, key=lambda x: x[0])[0]
                        # black_pixel_xmax = max(black_pixels, key=lambda x: x[0])[0]
                        #
                        # cv2.rectangle(image, (black_pixel_xmin, black_pixel_ymin), (black_pixel_xmax, black_pixel_ymax),
                        #               (255, 0, 0), 3)
                        #
                        # midpoint = [(black_pixel_xmax + black_pixel_xmin)/2, (black_pixel_ymax + black_pixel_ymin)/2]
                        # bb_msg.ybase = round(midpoint[1])
                        # bb_msg.xbase = round(midpoint[0])
                        # # draw xbase ybase on image
                        # cv2.circle(image, (bb_msg.xbase, bb_msg.ybase), 5, (0, 255, 255), 3)
                        # cv2.imshow("test", image)
                        # cv2.waitKey()

            bbs_msg.header = msg.header
            try:
                if self.pub_detection.get_num_connections() > 0:
                    detection_image = np.squeeze(results.render())
                    detection_image = np.concatenate((np.zeros((h + 1, msg.width, 3), detection_image.dtype), detection_image))

                    detection_image = detection_image[..., ::-1]  # convert rgb to bgr
                    self.pub_detection.publish(self.br.cv2_to_imgmsg(detection_image, encoding="bgr8"))

                if self.pub_boundingbox.get_num_connections() > 0 and len(bbs_msg.bounding_boxes) > 0:
                    self.pub_boundingbox.publish(bbs_msg)

            except ROSException as re:
                print(re)
                exit(0)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--model", dest="model_path", default="../../models/July14.pt", help="pytorch model")
    parser.add_argument("--num-feat", dest="num_feat", default=10, help="specify model size of the neural network")
    args, unknown = parser.parse_known_args()

    rospy.init_node("object_detector")
    my_node = ObjectDetectionNode(args.model_path)

    try:
        rospy.spin()
    except ROSException as rx:
        exit(0)
