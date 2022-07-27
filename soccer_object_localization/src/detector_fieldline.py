#!/usr/bin/env python3
import os
import time

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE

from soccer_common.transformation import Transformation
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
    HOUGH_RHO = 1
    HOUGH_THETA = np.pi / 180
    HOUGH_THRESHOLD = 50
    HOUGH_MIN_LINE_LENGTH = 50
    HOUGH_MAX_LINE_GAP = 50

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

    def image_callback(self, img: Image, debug=False):

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

        self.camera.reset_position(timestamp=img.header.stamp)
        rospy.loginfo_once("Started Publishing Fieldlines")

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        h = self.camera.calculateHorizonCoverArea()
        image_crop = image[h + 1 : -1, :, :]

        hsv = cv2.cvtColor(src=image_crop, code=cv2.COLOR_BGR2HSV)

        if debug:
            cv2.imshow("CVT Color", image_crop)
            cv2.waitKey(0)

        # Grass Mask
        grass_only = cv2.inRange(hsv, (35, 100, 0), (85, 255, 255))
        grass_only = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        if debug:
            cv2.imshow("Grass Only", grass_only)
            cv2.waitKey(0)

        grass_only = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, np.ones((30, 30), np.uint8))

        if debug:
            cv2.imshow("Grass Only", grass_only)
            cv2.waitKey(0)

        new_image = cv2.bitwise_and(hsv, hsv, mask=grass_only)
        lines_in_field = cv2.inRange(new_image, (0, 0, 110), (255, 100, 255))
        lines_in_field = cv2.morphologyEx(lines_in_field, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        lines_in_field = cv2.morphologyEx(lines_in_field, cv2.MORPH_GRADIENT, np.ones((2, 2), np.uint8))

        lines_in_field = cv2.vconcat([np.zeros((h + 2, lines_in_field.shape[1]), dtype=lines_in_field.dtype), lines_in_field])

        if debug:
            cv2.imshow("Lines in field", lines_in_field)
            cv2.waitKey(0)

        lines = cv2.HoughLinesP(
            lines_in_field,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=70,
            maxLineGap=30,
        )

        if lines is None:
            # Need to publish an empty point cloud to start the robot
            if self.publish_point_cloud and self.point_cloud_publisher.get_num_connections() > 0:
                # Publish fieldlines in laserscan format
                header = Header()
                header.stamp = img.header.stamp
                header.frame_id = self.robot_name + "/odom"
                point_cloud_msg = pcl2.create_cloud_xyz32(header, [])
                self.point_cloud_publisher.publish(point_cloud_msg)
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
            if abs(angle) < 10:  # Horizontal
                cv2.line(image, pt1, pt2, (255, 0, 0), thickness=1, lineType=cv2.LINE_AA)
            elif abs(abs(angle) - 90) < 10:  # Vertical
                cv2.line(image, pt1, pt2, (0, 255, 0), thickness=1, lineType=cv2.LINE_AA)
            else:
                cv2.line(image, pt1, pt2, (0, 0, 255), thickness=1, lineType=cv2.LINE_AA)

            if (pt2[0] - pt1[0]) == 0:
                continue
            slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            b = y1 - slope * x1

            for i in range(x1, x2, 10):
                y = slope * i + b
                pt = [i, y]
                pts.append(pt)

        if debug:
            cv2.imshow("Final Lines", image)
            cv2.waitKey(0)

        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(image)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        if self.publish_point_cloud and self.point_cloud_publisher.get_num_connections() > 0:
            points3d = []
            for p in pts:
                camToPoint = Transformation(self.camera.findFloorCoordinate(p))
                points3d.append(camToPoint.position)

            # Publish fieldlines in laserscan format
            header = Header()
            header.stamp = img.header.stamp
            header.frame_id = self.robot_name + "/odom"
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
            self.point_cloud_publisher.publish(point_cloud_msg)

        t_end = time.time()
        rospy.loginfo_throttle(60, "Fieldline detection rate: " + str(t_end - t_start))


if __name__ == "__main__":
    rospy.init_node("detector_fieldline")
    fieldline_detector = DetectorFieldline()
    rospy.spin()
