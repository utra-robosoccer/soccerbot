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
        self.loop_rate = rospy.Rate(10) # Hz
        self.pub_detection = rospy.Publisher('detection_image', Image, queue_size=10)
        self.pub_boundingbox = rospy.Publisher('object_bounding_boxes', BoundingBoxes, queue_size=10)
        rospy.Subscriber("camera/image_raw",Image, self.callback)

        self.model = CNN(kernel=3, num_features=8)
        self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
        self.model.eval()

    def callback(self, msg):
        # width x height x channels (bgra8)
        self.image_header = msg.header
        self.image = self.br.imgmsg_to_cv2(msg)

    def start(self):
        while not rospy.is_shutdown():
            br = CvBridge()
            if self.image is not None:
                img = self.image[:,:,:3] # get rid of alpha channel

                dim = (640//3, 480//3) # (213, 160)
                img = cv2.resize(img, dsize=dim, interpolation=cv2.INTER_CUBIC)

                w, h = 200, 150
                y, x, _ = img.shape # 160, 213
                x_offset = x/2 - w/2
                y_offset = y/2 - h/2

                crop_img = img[int(y_offset):int(y_offset+h), int(x_offset):int(x_offset+w)]

                img_torch = util.cv_to_torch(crop_img)

                outputs, _ = self.model(torch.tensor(np.expand_dims(img_torch, axis=0)).float())
                bounding_boxes = find_batch_bounding_boxes(outputs)[0]

                img_torch = util.draw_bounding_boxes(img_torch, bounding_boxes[Label.BALL.value], (255, 0, 0))
                img_torch = util.draw_bounding_boxes(img_torch, bounding_boxes[Label.ROBOT.value], (0, 0, 255))

                img = util.torch_to_cv(img_torch)

                if bounding_boxes is None:
                    continue

                bbs_msg = BoundingBoxes()
                ball_box_msg = BoundingBox()
                robot_box_msg = BoundingBox()
                for ball_box in bounding_boxes[Label.BALL.value]:
                    ball_box_msg.xmin = int((ball_box[0] + x_offset) * 3)
                    ball_box_msg.ymin = int((ball_box[1] + y_offset) * 3)
                    ball_box_msg.xmax = int((ball_box[2] + x_offset) * 3)
                    ball_box_msg.ymax = int((ball_box[3] + y_offset) * 3)
                    ball_box_msg.id = Label.BALL.value
                    ball_box_msg.Class = 'ball'

                for robot_box in bounding_boxes[Label.ROBOT.value]:
                    robot_box_msg.xmin = int((robot_box[0]+x_offset) * 3)
                    ball_box_msg.ymin = int((robot_box[1] + y_offset) * 3)
                    ball_box_msg.xmax = int((robot_box[2] + x_offset) * 3)
                    ball_box_msg.ymax = int((robot_box[3] + y_offset) * 3)
                    ball_box_msg.id = Label.ROBOT.value
                    ball_box_msg.Class = 'robot'

                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                bbs_msg.header = header
                bbs_msg.image_header = self.image_header
                bbs_msg.bounding_boxes = [ball_box_msg, robot_box_msg]

                self.pub_boundingbox.publish(bbs_msg)
                self.pub_detection.publish(br.cv2_to_imgmsg(img))
            self.loop_rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: my_node.py <path_to_pytorch_model>")
    else:
        myargv = rospy.myargv(argv=sys.argv)
        rospy.init_node("object_detector")
        my_node = ObjectDetectionNode(sys.argv[1])
        my_node.start()