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
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Header

from soccer_object_localization.detector import Detector


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

        def circular_mask(radius: int):
            mask = np.ones((radius, radius), np.uint8)
            for i in range(mask.shape[0]):
                for j in range(mask.shape[1]):
                    if np.sqrt((i - radius / 2) ** 2 + (j - radius / 2) ** 2) > radius / 2:
                        mask[i, j] = 0
            return mask

        # Grass Mask
        # Hue > 115 needed
        grass_only = cv2.inRange(hsv, (35, 85, 0), (115, 255, 255))
        grass_only = cv2.vconcat([np.zeros((h + 2, grass_only.shape[1]), dtype=grass_only.dtype), grass_only])

        grass_only_0 = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, circular_mask(4))
        grass_only_1 = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, circular_mask(4))
        grass_only_2 = cv2.morphologyEx(grass_only_1, cv2.MORPH_OPEN, circular_mask(20))
        grass_only_3 = cv2.morphologyEx(grass_only_2, cv2.MORPH_CLOSE, circular_mask(60))

        grass_only_morph = cv2.morphologyEx(grass_only_3, cv2.MORPH_ERODE, circular_mask(8))
        grass_only_flipped = cv2.bitwise_not(grass_only)

        lines_only = cv2.bitwise_and(grass_only_flipped, grass_only_flipped, mask=grass_only_morph)
        lines_only = cv2.morphologyEx(lines_only, cv2.MORPH_CLOSE, circular_mask(4))

        if debug:
            cv2.imshow("grass_only", grass_only)
            cv2.imshow("grass_only_0", grass_only_0)
            cv2.imshow("grass_only_1", grass_only_1)
            cv2.imshow("grass_only_2", grass_only_2)
            cv2.imshow("grass_only_3", grass_only_3)
            cv2.imshow("grass_only_morph", grass_only_morph)
            cv2.imshow("grass_only_flipped", grass_only_flipped)
            cv2.imshow("lines_only", lines_only)
            cv2.waitKey(0)

        # No line detection simply publish all white points
        pts_x, pts_y = np.where(lines_only == 255)

        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(lines_only)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        if self.publish_point_cloud and self.point_cloud_publisher.get_num_connections() > 0:
            points3d = []

            i = 0
            for px, py in zip(pts_y, pts_x):
                i = i + 1
                if i % 30 == 0:
                    camToPoint = Transformation(self.camera.findFloorCoordinate([px, py]))
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
