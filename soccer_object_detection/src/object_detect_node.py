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
from model import CNN, Label, find_batch_bounding_boxes, init_weights
from rospy import ROSException
from sensor_msgs.msg import Image

from soccer_common.camera import Camera
from soccer_msgs.msg import RobotState
from soccer_object_detection.msg import BoundingBox, BoundingBoxes

SOCCER_BALL = 32


class ObjectDetectionNode(object):
    """
    Detect ball, robot
    publish bounding boxes
    input: 480x640x4 bgra8 -> output: 3x200x150
    """

    def __init__(self, model_path, num_feat):
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s")

        if torch.cuda.is_available():
            rospy.logwarn("Using CUDA for object detection")
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
        global SOCCER_BALL
        # webots: 480x640x4pixels
        if self.robot_state.status not in [
            RobotState.STATUS_LOCALIZING,
            RobotState.STATUS_READY,
            RobotState.ROLE_UNASSIGNED,
        ]:
            return

        rospy.loginfo_throttle(60, "Recieved Image")
        # width x height x channels (bgra8)
        image = self.br.imgmsg_to_cv2(msg)
        self.camera.reset_position(timestamp=msg.header.stamp)

        # cover horizon to help robot ignore things outside field
        h = self.camera.calculateHorizonCoverArea()
        cv2.rectangle(image, [0, 0], [640, h + 30], [0, 165, 255], cv2.FILLED)

        if image is not None:
            # 1. preprocess image
            img = image[:, :, :3]  # get rid of alpha channel
            img = img[..., ::-1]  # convert bgr to rgb

            # 2. inference
            results = self.model(img)

            bbs_msg = BoundingBoxes()
            for prediction in results.xyxy[0]:
                x1, y1, x2, y2, confidence, img_class = prediction.cpu().numpy()
                if img_class == SOCCER_BALL:
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
                    self.pub_detection.publish(self.br.cv2_to_imgmsg(results.imgs[0]))

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
    my_node = ObjectDetectionNode(args.model_path, args.num_feat)

    try:
        rospy.spin()
    except ROSException as rx:
        exit(0)
