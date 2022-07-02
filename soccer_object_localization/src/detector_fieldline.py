#!/usr/bin/env python3

import os
import time

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE

from soccer_msgs.msg import RobotState

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from detector import Detector
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Header


class DetectorFieldline(Detector):
    CANNY_THRESHOLD_1 = rospy.get_param("~CANNY_THRESHOLD_1", 50)
    CANNY_THRESHOLD_2 = rospy.get_param("~CANNY_THRESHOLD_2", 1000)
    HOUGH_RHO = rospy.get_param("~HOUGH_RHO", 1)
    HOUGH_THETA = rospy.get_param("~HOUGH_THETA", np.pi / 180)
    HOUGH_THRESHOLD = rospy.get_param("~HOUGH_THRESHOLD", 50)
    HOUGH_MIN_LINE_LENGTH = rospy.get_param("~HOUGH_MIN_LINE_LENGTH", 50)
    HOUGH_MAX_LINE_GAP = rospy.get_param("~HOUGH_MAX_LINE_GAP", 50)

    field_line_detection_hsv = rospy.get_param("~field_line_detection_hsv", 65)
    binary_threshold_start = rospy.get_param("~binary_threshold_start", 127)
    binary_threshold_end = rospy.get_param("~binary_threshold_end", 255)

    def __init__(self):
        super().__init__()

        self.initial_pose_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose_callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(
            "camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/)
        self.image_publisher = rospy.Publisher("camera/line_image", Image, queue_size=1)
        self.point_cloud_publisher = rospy.Publisher("field_point_cloud", PointCloud2, queue_size=1)
        self.publish_point_cloud = False
        cv2.setRNGSeed(12345)
        pass

    def initial_pose_callback(self, initial_pose: PoseWithCovarianceStamped):
        self.publish_point_cloud = True

    def image_callback(self, img: Image):

        t_start = time.time()

        if self.robot_state.status not in [
            RobotState.STATUS_READY,
            RobotState.STATUS_LOCALIZING,
            RobotState.STATUS_WALKING,
            RobotState.STATUS_DETERMINING_SIDE,
        ]:
            return

        if not self.camera.ready():
            return

        pts = []

        self.camera.reset_position(publish_basecamera=True, timestamp=img.header.stamp)
        rospy.loginfo_once("Started Publishing Fieldlines")

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        hsv = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2HSV)
        h = self.camera.calculateHorizonCoverArea()
        cv2.rectangle(image, [0, 0], [640, h], [0, 0, 0], cv2.FILLED)

        # Field line detection

        mask2 = cv2.inRange(hsv, (0, 0, 255 - DetectorFieldline.field_line_detection_hsv), (255, DetectorFieldline.field_line_detection_hsv, 255))
        out = cv2.bitwise_and(image, image, mask=mask2)

        kernel = np.ones((7, 7), np.uint8)
        out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel)

        cdst = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)

        retval, dst = cv2.threshold(cdst, DetectorFieldline.binary_threshold_start, DetectorFieldline.binary_threshold_end, cv2.THRESH_BINARY)

        edges = cv2.Canny(dst, DetectorFieldline.CANNY_THRESHOLD_1, DetectorFieldline.CANNY_THRESHOLD_2)

        lines = cv2.HoughLinesP(
            edges,
            rho=DetectorFieldline.HOUGH_RHO,
            theta=DetectorFieldline.HOUGH_THETA,
            threshold=DetectorFieldline.HOUGH_THRESHOLD,
            minLineLength=DetectorFieldline.HOUGH_MIN_LINE_LENGTH,
            maxLineGap=DetectorFieldline.HOUGH_MAX_LINE_GAP,
        )

        ccdst = cv2.cvtColor(cdst, cv2.COLOR_GRAY2RGB)

        if lines is None:
            return

        for l in lines:
            x1, y1, x2, y2 = l[0]
            # computing magnitude and angle of the line
            if x2 == x1:
                angle = 0
            else:
                angle = np.rad2deg(np.arctan((y2 - y1) / (x2 - x1)))

            pt1 = (x1, y1)
            pt2 = (x2, y2)
            if abs(angle) < 10:  # Horizontal # I dont remember what happens here I think its getting all different types of line
                cv2.line(ccdst, pt1, pt2, (255, 0, 0), thickness=3, lineType=cv2.LINE_AA)
            elif abs(abs(angle) - 90) < 10:  # Vertical
                cv2.line(ccdst, pt1, pt2, (0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
            else:
                cv2.line(ccdst, pt1, pt2, (0, 0, 255), thickness=3, lineType=cv2.LINE_AA)

            if (pt2[0] - pt1[0]) == 0:
                continue
            slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            b = y1 - slope * x1

            for i in range(x1, x2, 10):
                y = slope * i + b
                pt = [i, y]
                pts.append(pt)

        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(ccdst)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        if self.publish_point_cloud and self.point_cloud_publisher.get_num_connections() > 0:
            points3d = []
            for p in pts:
                points3d.append(self.camera.findFloorCoordinate(p))
            # Publish fieldlines in laserscan format
            header = Header()
            header.stamp = img.header.stamp
            header.frame_id = self.robot_name + "/base_camera"
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
            if self.point_cloud_publisher.get_num_connections() > 0:
                self.point_cloud_publisher.publish(point_cloud_msg)

        t_end = time.time()
        rospy.loginfo_throttle(60, "Fieldline detection rate: " + str(t_end - t_start))


if __name__ == "__main__":
    rospy.init_node("detector_fieldline")
    fieldline_detector = DetectorFieldline()
    rospy.spin()
