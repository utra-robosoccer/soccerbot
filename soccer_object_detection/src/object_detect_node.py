import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

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
        self.pub = rospy.Publisher('imagetimer', Image, queue_size=10)
        rospy.Subscriber("robot1/camera/image_raw",Image, self.callback)

    def callback(self, msg):
        # width x height x channels (bgra8)
        self.image = self.br.imgmsg_to_cv2(msg)

    def start(self):
        rospy.loginfo("Timing images")
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

                self.pub.publish(br.cv2_to_imgmsg(crop_img))
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = ObjectDetectionNode()
    my_node.start()