#!/usr/bin/python3
import os
import sys
import rospy
from soccer_geometry.camera import Camera

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from argparse import ArgumentParser
from sensor_msgs.msg import Image
import std_msgs
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from model import CNN, init_weights
import util
from model import find_batch_bounding_boxes, Label
from soccer_object_detection.msg import BoundingBox, BoundingBoxes

class ObjectDetectionNode(object):
    '''
    Detect ball, robot
    publish bounding boxes
    input: 480x640x4 bgra8 -> output: 3x200x150
    '''

    def __init__(self, model_path, num_feat):
        self.robot_name = rospy.get_namespace()[1:-1]  # remove '/'
        self.camera = Camera(self.robot_name)
        self.camera.reset_position()

        # Params
        self.br = CvBridge()

        self.pub_detection = rospy.Publisher('detection_image', Image, queue_size=10)
        self.pub_boundingbox = rospy.Publisher('object_bounding_boxes', BoundingBoxes, queue_size=10)
        self.image_subscriber = rospy.Subscriber("camera/image_raw",Image, self.callback, queue_size=1)

        self.model = CNN(kernel=3, num_features=int(num_feat))

        self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
        self.model.eval()

    def callback(self, msg: Image):
        # width x height x channels (bgra8)
        image = self.br.imgmsg_to_cv2(msg)
        self.camera.reset_position(timestamp=msg.header.stamp)
        h = self.camera.calculateHorizonCoverArea()
        cv2.rectangle(image, [0, 0], [640, h], [0, 165, 255], cv2.FILLED)

        if image is not None:
            img = image[:, :, :3]  # get rid of alpha channel
            scale = 480 / 300
            dim = (int(640 / scale), 300)
            img = cv2.resize(img, dsize=dim, interpolation=cv2.INTER_AREA)

            w, h = 400, 300
            y, x, _ = img.shape  # 160, 213
            x_offset = x / 2 - w / 2
            y_offset = y / 2 - h / 2

            crop_img = img[int(y_offset):int(y_offset + h), int(x_offset):int(x_offset + w)]

            img_torch = util.cv_to_torch(crop_img)

            img_norm = img_torch / 255  # model expects normalized img

            outputs, _ = self.model(torch.tensor(np.expand_dims(img_norm, axis=0)).float())
            bbxs = find_batch_bounding_boxes(outputs)[0]

            if bbxs is None:
                return

            bbs_msg = BoundingBoxes()

            for ball_bb in bbxs[Label.BALL.value]:
                bb_msg = BoundingBox()
                bb_msg.xmin = round((ball_bb[0] + x_offset) * scale)
                bb_msg.ymin = round((ball_bb[1] + y_offset) * scale)
                bb_msg.xmax = round((ball_bb[2] + x_offset) * scale)
                bb_msg.ymax = round((ball_bb[3] + y_offset) * scale)
                bb_msg.id = Label.BALL.value
                bb_msg.Class = 'ball'
                bbs_msg.bounding_boxes.append(bb_msg)

            big_enough_robot_bbxs = []
            for robot_bb in bbxs[Label.ROBOT.value]:
                bb_msg = BoundingBox()
                bb_msg.xmin = round((robot_bb[0] + x_offset) * scale)
                bb_msg.ymin = round((robot_bb[1] + y_offset) * scale)
                bb_msg.xmax = round((robot_bb[2] + x_offset) * scale)
                bb_msg.ymax = round((robot_bb[3] + y_offset) * scale)
                bb_msg.id = Label.ROBOT.value
                bb_msg.Class = 'robot'
                # ignore small boxes
                if (bb_msg.xmax - bb_msg.xmin) * (bb_msg.ymax - bb_msg.ymin) > 1000:
                    bbs_msg.bounding_boxes.append(bb_msg)
                    big_enough_robot_bbxs.append(robot_bb)

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            bbs_msg.header = header
            bbs_msg.image_header = msg.header
            self.pub_boundingbox.publish(bbs_msg)

            if self.pub_detection.get_num_connections() > 0:
                img_torch = util.draw_bounding_boxes(img_torch, big_enough_robot_bbxs, (0, 0, 255))
                img_torch = util.draw_bounding_boxes(img_torch, bbxs[Label.BALL.value], (255, 0, 0))
                img = util.torch_to_cv(img_torch)
                self.pub_detection.publish(self.br.cv2_to_imgmsg(img))


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("--model", dest="model_path", help="pytorch model")
    parser.add_argument("--num-feat", dest="num_feat", help="specify model size of the neural network")
    args, unknown = parser.parse_known_args()

    rospy.init_node("object_detector")
    my_node = ObjectDetectionNode(args.model_path, args.num_feat)
    rospy.spin()
