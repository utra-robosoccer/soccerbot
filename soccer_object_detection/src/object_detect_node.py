#!/usr/bin/python3

import os

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from argparse import ArgumentParser

import cv2
import gluoncv
import mxnet as mx
import numpy as np
import rospy
import torch
import util
from cv_bridge import CvBridge
from gluoncv import data, model_zoo, utils
from matplotlib import pyplot as plt
from model import CNN, Label, find_batch_bounding_boxes, init_weights
from rospy import ROSException
from sensor_msgs.msg import Image

from soccer_common.camera import Camera
from soccer_msgs.msg import RobotState
from soccer_object_detection.msg import BoundingBox, BoundingBoxes


class ObjectDetectionNode(object):
    """
    Detect ball, robot
    publish bounding boxes
    input: 480x640x4 bgra8 -> output: 3x200x150
    """

    def __init__(self, model_path, num_feat):
        self.model = model_zoo.get_model("yolo3_mobilenet1.0_coco", pretrained=True)
        self.model.reset_class(classes=["sports ball"], reuse_weights=["sports ball"])

        self.robot_name = rospy.get_namespace()[1:-1]  # remove '/'
        self.camera = Camera(self.robot_name)
        self.camera.reset_position()

        # Params
        self.br = CvBridge()

        self.pub_detection = rospy.Publisher("detection_image", Image, queue_size=1)
        self.pub_boundingbox = rospy.Publisher("object_bounding_boxes", BoundingBoxes, queue_size=1)
        self.image_subscriber = rospy.Subscriber(
            "camera/image_raw", Image, self.callback, queue_size=1, buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/)
        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.robot_state_callback)
        self.robot_state = RobotState()

    def robot_state_callback(self, robot_state: RobotState):
        self.robot_state = robot_state

    def callback(self, msg: Image):
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
        h = self.camera.calculateHorizonCoverArea()
        cv2.rectangle(image, [0, 0], [640, h + 30], [0, 165, 255], cv2.FILLED)

        if image is not None:
            img = image[:, :, :3]  # get rid of alpha channel
            # scale = 480 / 300
            # dim = (int(640 / scale), 300)
            # img = cv2.resize(img, dsize=dim, interpolation=cv2.INTER_AREA)
            #
            # w, h = 400, 300
            # y, x, _ = img.shape  # 160, 213
            # x_offset = x / 2 - w / 2
            # y_offset = y / 2 - h / 2
            #
            # crop_img = img[int(y_offset):int(y_offset + h), int(x_offset):int(x_offset + w)]
            #
            # img_torch = util.cv_to_torch(crop_img)
            #
            # img_norm = img_torch / 255  # model expects normalized img

            # outputs, _ = self.model(torch.tensor(np.expand_dims(img_norm, axis=0)).float())
            # bbxs = find_batch_bounding_boxes(outputs)[0]

            x = mx.nd.array(img)
            x, orig_img = data.transforms.presets.rcnn.transform_test([x])
            box_ids, scores, bbxs = self.model(x)

            if bbxs is None:
                return

            if box_ids[0][0][0].asscalar() < 0:
                return

            bbs_msg = BoundingBoxes()
            bb_msg = BoundingBox()
            ball_bb = bbxs[0][0]

            xratio = msg.width / orig_img.shape[1]
            yratio = msg.height / orig_img.shape[0]

            bb_msg.xmin = round(ball_bb[0].asscalar() * xratio)
            bb_msg.ymin = round(ball_bb[1].asscalar() * yratio)
            bb_msg.xmax = round(ball_bb[2].asscalar() * xratio)
            bb_msg.ymax = round(ball_bb[3].asscalar() * yratio)
            bb_msg.id = Label.BALL.value
            bb_msg.Class = "ball"
            bbs_msg.bounding_boxes.append(bb_msg)

            bbs_msg.header = msg.header
            try:
                if self.pub_boundingbox.get_num_connections() > 0 and len(bbs_msg.bounding_boxes) > 0:
                    self.pub_boundingbox.publish(bbs_msg)
            except ROSException as re:
                print(re)
                exit(0)

            if self.pub_detection.get_num_connections() > 0:
                img = utils.viz.cv_plot_bbox(orig_img, bbxs[0], scores[0], box_ids[0], class_names=self.model.classes, linewidth=1)
                self.pub_detection.publish(self.br.cv2_to_imgmsg(img))


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--model", dest="model_path", default="outputs/model3_feat10", help="pytorch model")
    parser.add_argument("--num-feat", dest="num_feat", default=10, help="specify model size of the neural network")
    args, unknown = parser.parse_known_args()

    rospy.init_node("object_detector")
    my_node = ObjectDetectionNode(args.model_path, args.num_feat)
    rospy.spin()
