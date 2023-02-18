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

        if self.robot_state.status not in [RobotState.STATUS_DETERMINING_SIDE, RobotState.STATUS_LOCALIZING]:
            return

        if not self.camera.ready():
            return

        self.camera.reset_position(timestamp=img.header.stamp)

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        vertical_lines, image_out = self.get_vlines_from_img(image, debug=debug)

        # Get vertical line in 3D position to the camera assuming that the lower point of the line is on the grass
        closest_distance = np.inf
        closest_line = None
        closest_line_relative_position_post_to_robot = None
        min_horizon_pixel = self.camera.calculateHorizonCoverArea()

        for line in vertical_lines:
            if line[1] > line[3]:
                lower_point = [line[0], line[1]]
            else:
                lower_point = [line[2], line[3]]

            if lower_point[1] < min_horizon_pixel:
                continue

            relative_position_post_to_robot = self.camera.findFloorCoordinate(lower_point)
            if np.linalg.norm(relative_position_post_to_robot) < closest_distance:
                closest_distance = np.linalg.norm(relative_position_post_to_robot)
                closest_line = line
                closest_line_relative_position_post_to_robot = relative_position_post_to_robot

        if closest_line is not None:
            cv2.line(image_out, (closest_line[0], closest_line[1]), (closest_line[2], closest_line[3]), (255, 0, 0), 2)
            self.br.sendTransform(
                closest_line_relative_position_post_to_robot,
                (0, 0, 0, 1),
                img.header.stamp,
                self.robot_name + "/goal_post",
                self.robot_name + "/base_footprint",
            )
        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(image_out)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        t_end = time.time()
        rospy.loginfo_throttle(60, "GoalPost detection rate: " + str(t_end - t_start))

    """
        Retrieves the vertical lines in image by isolating the grass in the field, removing it and then uses
        Canny and Hough to detect the lines using hough_theta, hough_threshold, hough_min_line_length,
        hough_max_line_gap and returns lines that are within angle_tol_deg of vertical.
        Returns vertical lines as a list of lists where each line's coordinates are stored as:
        [x_start, y_start, x_end, y_end]
        and the original image with the vertical lines drawn on it
    """

    def get_vlines_from_img(
        self, image, debug=False, angle_tol_deg=3, hough_theta=np.pi / 180, hough_threshold=30, hough_min_line_length=30, hough_max_line_gap=10
    ):
        # Isolate and remove field from image
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

        # Isolate all lines using Canny edge detection and Hough lines with the provided settings
        image_edges = cv2.Canny(image_bw_eroded, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(
            image_edges, rho=1, theta=hough_theta, threshold=hough_threshold, minLineLength=hough_min_line_length, maxLineGap=hough_max_line_gap
        )
        if debug:
            image_hough = image.copy()
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image_hough, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Isolate vertical lines
        angle_tol_rad = np.radians(angle_tol_deg)  # +-deg from vertical
        image_out = image.copy()

        vertical_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            abs_line_angle = math.atan2(y2 - y1, x2 - x1)
            if abs(abs_line_angle - np.pi / 2) < angle_tol_rad:
                vertical_lines.append(line[0])
                cv2.line(image_out, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if debug and "DISPLAY" in os.environ:
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


if __name__ == "__main__":
    rospy.init_node("detector_goalpost")
    fieldline_detector = DetectorGoalPost()
    rospy.spin()
