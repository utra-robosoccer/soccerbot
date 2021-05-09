import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
from sensor_msgs.msg import Image
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

class ObjectDetectionNode(object):
    '''
    Detect ball, robot
    publish bounding boxes
    input: 480x640x4 bgra8 -> output: 3x200x150
    '''

    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(10) # Hz
        self.pub = rospy.Publisher('detection_image', Image, queue_size=10)
        rospy.Subscriber("camera/image_raw",Image, self.callback)

        self.model = CNN(kernel=3, num_features=8)
        self.model.load_state_dict(torch.load('outputs/model_ker3_feat8'))
        self.model.eval()

    def callback(self, msg):
        # width x height x channels (bgra8)
        self.image = self.br.imgmsg_to_cv2(msg)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            br = CvBridge()
            if self.image is not None:
                img = self.image[:,:,:3] # get rid of alpha channel
                rospy.loginfo(img.shape)

                dim = (640//3, 480//3) # (213, 160)
                img = cv2.resize(img, dsize=dim, interpolation=cv2.INTER_CUBIC)
                rospy.loginfo(img.shape)

                w, h = 200, 150
                y, x, _ = img.shape # 160, 213
                x = x/2 - w/2
                y = y/2 - h/2

                crop_img = img[int(y):int(y+h), int(x):int(x+w)]
                rospy.loginfo(crop_img.shape)

                img_torch = util.cv_to_torch(crop_img)

                outputs, _ = self.model(torch.tensor(np.expand_dims(img_torch, axis=0)).float())
                bbxs = find_batch_bounding_boxes(outputs)[0]
                # img = util.draw_bounding_boxes(img, bbxs[Label.ROBOT.value], (0, 0, 255))
                img_torch = util.draw_bounding_boxes(img_torch, bbxs[Label.BALL.value], (255, 0, 0))

                img = util.torch_to_cv(img_torch)

                self.pub.publish(br.cv2_to_imgmsg(img))
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("object_detector")
    my_node = ObjectDetectionNode()
    my_node.start()