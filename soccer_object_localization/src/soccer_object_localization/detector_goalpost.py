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
        vertical_lines, image_out = self.get_vlines_from_img(image, debug=debug)

        is_net_detected, image_out = DetectorGoalPost.detect_net_in_vlines(vertical_lines, image)

        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(image_out)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        t_end = time.time()
        rospy.loginfo_throttle(60, "GoalPost detection rate: " + str(t_end - t_start))

    def get_vlines_from_img(
        self, image, debug=False, angle_tol_deg=3, hough_theta=np.pi / 180, hough_threshold=50, hough_min_line_length=30, hough_max_line_gap=10
    ):
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
            image_edges, rho=1, theta=hough_theta, threshold=hough_threshold, minLineLength=hough_min_line_length, maxLineGap=hough_max_line_gap
        )
        if debug:
            image_hough = image.copy()
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image_hough, (x1, y1), (x2, y2), (0, 255, 0), 2)

        angle_tol_rad = np.radians(angle_tol_deg)  # +-deg from vertical
        image_out = image.copy()

        vertical_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            abs_slope = abs((y2 - y1) / (x2 - x1))  # Use abs so all angles in first 2 quadrants
            abs_line_angle = math.atan(abs_slope)
            if abs(abs_line_angle - np.pi / 2) < angle_tol_rad:
                vertical_lines.append(line[0])
                cv2.line(image_out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        if debug:
            cv2.imshow("image_blurred", image_blurred)
            cv2.imshow("image_hsv", image_hsv)
            cv2.imshow("image_hsv_filter", image_hsv_filter)
            cv2.imshow("grass_only", grass_only_flipped)

            cv2.imshow("image_bw", image_bw)
            cv2.imshow("image_bw_eroded", image_bw_eroded)
            cv2.imshow("image_edges", image_edges)
            cv2.imshow("image_hough", image_hough)

            cv2.waitKey(0)
        return vertical_lines, image_out

    @staticmethod
    def cluster_vlines(vlines, max_y_gap, min_x_gap):
        """Arrange sorted vertical line data into groups where successive elements
        must have similar y values and a gap between their x values
        Based on: https://stackoverflow.com/questions/14783947/grouping-clustering-numbers-in-python
        """
        groups = [[vlines[0]]]
        for d in vlines[1:]:
            is_x1s_dissimilar = abs(d["x1"] - groups[-1][-1]["x1"]) >= min_x_gap
            is_y1s_similar = abs(d["y1"] - groups[-1][-1]["y1"]) <= max_y_gap
            is_y2s_similar = abs(d["y2"] - groups[-1][-1]["y2"]) <= max_y_gap
            is_x2s_dissimilar = abs(d["x2"] - groups[-1][-1]["x2"]) >= min_x_gap
            if is_x1s_dissimilar and is_y1s_similar and is_x2s_dissimilar and is_y2s_similar:
                groups[-1].append(d)
            else:
                groups.append([d])
        return groups

    @staticmethod
    def detect_net_in_vlines(vertical_lines, base_img, length_similarity_threshold=20, min_post_separation=10):
        # Check for a net by looking for lines that start and end at similar y values and are a sufficient x distance
        # apart
        is_net_in_vlines = False
        # TODO remove dict conversion
        vlines = []
        for line in vertical_lines:
            x1, y1, x2, y2 = line
            if y1 > y2:  # want y1 to be the smaller y value when we sort
                (x1, y1, x2, y2) = (x2, y2, x1, y1)
            vlines.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2})
        vlines_sorted_by_y1 = sorted(vlines, key=lambda d: d["y1"])
        clustered_vlines = DetectorGoalPost.cluster_vlines(vlines_sorted_by_y1, length_similarity_threshold, min_post_separation)
        net_line_img = base_img
        for v_line_group in clustered_vlines:
            if len(v_line_group) > 1:
                for d in v_line_group:
                    cv2.line(net_line_img, (d["x1"], d["y1"]), (d["x2"], d["y2"]), (255, 0, 0), 2)
                is_net_in_vlines = True
        return is_net_in_vlines, net_line_img


if __name__ == "__main__":
    rospy.init_node("detector_goalpost")
    fieldline_detector = DetectorGoalPost()
    rospy.spin()
