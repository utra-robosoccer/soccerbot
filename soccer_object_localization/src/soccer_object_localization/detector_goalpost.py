#!/usr/bin/env python3

import os
import time

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from soccer_msgs.msg import RobotState
from soccer_object_localization.detector import Detector
from soccer_object_localization.detector_fieldline import DetectorFieldline


class DetectorGoalPost(Detector):
    def __init__(self):
        super().__init__()

        self.image_subscriber = rospy.Subscriber(
            "camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/)
        self.image_publisher = rospy.Publisher("camera/goal_image", Image, queue_size=1)
        cv2.setRNGSeed(12345)
        pass

    def image_callback(self, img: Image, debug=False):
        t_start = time.time()

        if self.robot_state.status is not RobotState.STATUS_DETERMINING_SIDE:
            return

        if not self.camera.ready():
            return

        self.camera.reset_position(timestamp=img.header.stamp)

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        image_blurred = cv2.bilateralFilter(image, 9, 75, 75)
        image_hsv = cv2.cvtColor(src=image_blurred, code=cv2.COLOR_BGR2HSV)
        image_hsv_filter = cv2.inRange(image_hsv, (0, 0, 150), (255, 50, 255))

        h = self.camera.calculateHorizonCoverArea()
        image_crop = image_hsv[h + 1 :, :, :]
        grass_only = cv2.inRange(image_crop, (35, 85, 0), (115, 255, 255))
        grass_only = cv2.vconcat([np.zeros((h + 1, grass_only.shape[1]), dtype=grass_only.dtype), grass_only])

        # Use odd numbers for all circular masks otherwise the line will shift location
        grass_only_0 = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, self.circular_mask(5))
        grass_only_1 = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, self.circular_mask(5))
        grass_only_2 = cv2.morphologyEx(grass_only_1, cv2.MORPH_OPEN, self.circular_mask(21))
        grass_only_3 = cv2.morphologyEx(grass_only_2, cv2.MORPH_CLOSE, self.circular_mask(61))

        grass_only_morph = cv2.morphologyEx(grass_only_3, cv2.MORPH_ERODE, self.circular_mask(9))
        grass_only_flipped = cv2.bitwise_not(grass_only_morph)

        image_bw = cv2.bitwise_and(image, image, mask=image_hsv_filter)
        image_bw = cv2.bitwise_and(image_bw, image_bw, mask=grass_only_flipped)
        image_bw = cv2.cvtColor(image_bw, cv2.COLOR_BGR2GRAY)
        image_bw_eroded = cv2.morphologyEx(image_bw, cv2.MORPH_ERODE, self.circular_mask(5))
        image_edges = cv2.Canny(image_bw_eroded, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(
            image_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=100,
            minLineLength=100,
            maxLineGap=10,
        )
        for x1, y1, x2, y2 in lines[0]:
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if debug:
            cv2.imshow("image_blurred", image_blurred)
            cv2.imshow("image_hsv", image_hsv)
            cv2.imshow("image_hsv_filter", image_hsv_filter)
            cv2.imshow("grass_only", grass_only_flipped)

            cv2.imshow("image_bw", image_bw)
            cv2.imshow("image_bw_eroded", image_bw_eroded)
            cv2.imshow("image_edges", image_edges)
            cv2.imshow("image_hough", image)

            cv2.waitKey(0)

        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(image_hsv)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        t_end = time.time()
        rospy.loginfo_throttle(60, "GoalPost detection rate: " + str(t_end - t_start))


if __name__ == "__main__":
    rospy.init_node("detector_goalpost")
    fieldline_detector = DetectorGoalPost()
    rospy.spin()
