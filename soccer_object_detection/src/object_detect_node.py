#!/usr/bin/python3
import os
import sys
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
from sensor_msgs.msg import Image
import std_msgs
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from model import CNN, init_weights
from my_dataset import initialize_loader
from train import Trainer
import util
import torchvision
from model import find_batch_bounding_boxes, Label
from soccer_object_detection.msg import BoundingBox, BoundingBoxes

class ObjectDetectionNode(object):
    '''
    Detect ball, robot
    publish bounding boxes
    input: 480x640x4 bgra8 -> output: 3x200x150
    '''

    def __init__(self, model_path):
        # Params
        self.image = None
        self.image_header = None
        self.br = CvBridge()

        self.pub_detection = rospy.Publisher('detection_image', Image, queue_size=10)
        self.pub_boundingbox = rospy.Publisher('object_bounding_boxes', BoundingBoxes, queue_size=10)
        rospy.Subscriber("camera/image_raw",Image, self.callback)

        self.model = CNN(kernel=3, num_features=8)

        if os.getenv('COMPETITION', 'false') == 'true':
            self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
        else:
            self.model.load_state_dict(torch.load(model_path))

        self.model.eval()

    def callback(self, msg):
        # width x height x channels (bgra8)
        self.image_header = msg.header
        self.image = self.br.imgmsg_to_cv2(msg)

    def start(self):
        loop_rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():

            if self.image is not None:
                img = self.image[:,:,:3] # get rid of alpha channel

                dim = (640//3, 480//3) # (213, 160)
                img = cv2.resize(img, dsize=dim, interpolation=cv2.INTER_AREA)

                w, h = 200, 150
                y, x, _ = img.shape # 160, 213
                x_offset = x/2 - w/2
                y_offset = y/2 - h/2

                crop_img = img[int(y_offset):int(y_offset+h), int(x_offset):int(x_offset+w)]

                img_torch = util.cv_to_torch(crop_img)

                img_norm = img_torch / 255  # model expects normalized img

                outputs, _ = self.model(torch.tensor(np.expand_dims(img_norm, axis=0)).float())
                bbxs = find_batch_bounding_boxes(outputs)[0]

                if bbxs is None:
                    continue

                bbs_msg = BoundingBoxes()
                bb_msg = BoundingBox()
                for ball_bb in bbxs[Label.BALL.value]:
                    bb_msg.xmin = int((ball_bb[0] + x_offset) * 3)
                    bb_msg.ymin = int((ball_bb[1] + y_offset) * 3)
                    bb_msg.xmax = int((ball_bb[2] + x_offset) * 3)
                    bb_msg.ymax = int((ball_bb[3] + y_offset) * 3)
                    bb_msg.id = Label.BALL.value
                    bb_msg.Class = 'ball'
                bbs_msg.bounding_boxes = [bb_msg]

                big_enough_robot_bbxs = []
                for robot_bb in bbxs[Label.ROBOT.value]:
                    bb_msg = BoundingBox()
                    bb_msg.xmin = int((robot_bb[0] + x_offset) * 3)
                    bb_msg.ymin = int((robot_bb[1] + y_offset) * 3)
                    bb_msg.xmax = int((robot_bb[2] + x_offset) * 3)
                    bb_msg.ymax = int((robot_bb[3] + y_offset) * 3)
                    bb_msg.id = Label.ROBOT.value
                    bb_msg.Class = 'robot'
                    # ignore small boxes
                    if (bb_msg.xmax - bb_msg.xmin)*(bb_msg.ymax - bb_msg.ymin) > 1000:
                        bbs_msg.bounding_boxes.append(bb_msg)
                        big_enough_robot_bbxs.append(robot_bb)

                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                bbs_msg.header = header
                bbs_msg.image_header = self.image_header

                img_torch = util.draw_bounding_boxes(img_torch, big_enough_robot_bbxs, (0, 0, 255))
                img_torch = util.draw_bounding_boxes(img_torch, bbxs[Label.BALL.value], (255, 0, 0))
                img = util.torch_to_cv(img_torch)

                self.pub_boundingbox.publish(bbs_msg)
                self.pub_detection.publish(self.br.cv2_to_imgmsg(img))

            try:
                loop_rate.sleep()
            except rospy.exceptions.ROSInterruptException as ex:
                print(ex)
                exit(0)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: my_node.py <path_to_pytorch_model>")
    else:
        myargv = rospy.myargv(argv=sys.argv)
        rospy.init_node("object_detector")
        my_node = ObjectDetectionNode(sys.argv[1])
        my_node.start()