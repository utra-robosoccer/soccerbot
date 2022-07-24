#!/usr/bin/env python3

import os

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from argparse import ArgumentParser

import cv2
import rospy
import torch
from cv_bridge import CvBridge
from model import Label
from rospy import ROSException
from sensor_msgs.msg import Image

from soccer_common.camera import Camera
from soccer_msgs.msg import BoundingBox, BoundingBoxes, RobotState


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

        rospy.loginfo_once("Object Detection Receiving image")
        # width x height x channels (bgra8)
        image = self.br.imgmsg_to_cv2(msg)
        self.camera.reset_position(timestamp=msg.header.stamp)

        # cover horizon to help robot ignore things outside field
        cover_horizon_up_threshold = rospy.get_param("cover_horizon_up_threshold", 30)
        h = self.camera.calculateHorizonCoverArea() - cover_horizon_up_threshold
        cv2.rectangle(image, [0, 0], [self.camera.resolution_x, max(0, h)], [0, 165, 255], cv2.FILLED)

        if image is not None:
            # 1. preprocess image
            img = image[:, :, :3]  # get rid of alpha channel
            img = img[..., ::-1]  # convert bgr to rgb
            img = img[max(0, h + 1) : -1, :]
            # 2. inference

            results = self.model(img)

            bbs_msg = BoundingBoxes()
            for prediction in results.xyxy[0]:
                x1, y1, x2, y2, confidence, img_class = prediction.cpu().numpy()
                y1 += h + 1
                y2 += h + 1
                if img_class == self.SOCCER_BALL and confidence > self.CONFIDENCE_THRESHOLD:
                    bb_msg = BoundingBox()
                    bb_msg.xmin = round(x1)
                    bb_msg.ymin = round(y1)
                    bb_msg.xmax = round(x2)
                    bb_msg.ymax = round(y2)
                    bb_msg.probability = confidence
                    bb_msg.id = Label.BALL.value
                    bb_msg.Class = "ball"
                    bbs_msg.bounding_boxes.append(bb_msg)

            bbs_msg.header = msg.header
            try:
                if self.pub_detection.get_num_connections() > 0:
                    results.render()
                    self.pub_detection.publish(self.br.cv2_to_imgmsg(results.imgs[0], encoding="bgr8"))

                if self.pub_boundingbox.get_num_connections() > 0 and len(bbs_msg.bounding_boxes) > 0:
                    self.pub_boundingbox.publish(bbs_msg)

            except ROSException as re:
                print(re)
                exit(0)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--model", dest="model_path", default="outputs/model3_feat10", help="pytorch model")
    parser.add_argument("--num-feat", dest="num_feat", default=10, help="specify model size of the neural network")
    args, unknown = parser.parse_known_args()

    rospy.init_node("object_detector")
    my_node = ObjectDetectionNode(args.model_path)

    try:
        rospy.spin()
    except ROSException as rx:
        exit(0)
