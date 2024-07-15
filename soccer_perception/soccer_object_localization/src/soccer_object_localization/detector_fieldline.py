#!/usr/bin/env python3
import os
import time

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from tf import TransformBroadcaster

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
from soccer_object_localization.detector import Detector
from std_msgs.msg import Bool, Header


class DetectorFieldline(Detector):
    def __init__(self):
        super().__init__()

        self.initial_pose_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose_callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(
            "camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/)
        self.image_publisher = rospy.Publisher("camera/line_image", Image, queue_size=1)
        self.point_cloud_publisher = rospy.Publisher("field_point_cloud", PointCloud2, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()

        self.point_cloud_max_distance = rospy.get_param("point_cloud_max_distance", 5)
        self.point_cloud_spacing = rospy.get_param("point_cloud_spacing", 30)
        self.publish_point_cloud = False
        self.ground_truth = False

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

        if self.ground_truth:
            if not self.publish_point_cloud:
                self.camera.reset_position(timestamp=img.header.stamp)
            else:
                self.camera.reset_position(timestamp=img.header.stamp, from_world_frame=True, camera_frame="/camera_gt")
        else:
            self.camera.reset_position(timestamp=img.header.stamp)
        # Uncomment for ground truth
        rospy.loginfo_once("Started Publishing Fieldlines")

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        h = self.camera.calculateHorizonCoverArea()
        if h + 1 >= self.camera.resolution_y:
            return
        image_crop = image[h + 1 :, :, :]
        # image_crop_blurred = cv2.GaussianBlur(image_crop, (3, 3), 0)
        image_crop_blurred = cv2.bilateralFilter(image, 9, 75, 75)

        hsv = cv2.cvtColor(src=image_crop_blurred, code=cv2.COLOR_BGR2HSV)

        if debug:
            cv2.imshow("CVT Color", image_crop)
            cv2.imshow("CVT Color Contrast", image_crop_blurred)
            cv2.waitKey(0)

        # Grass Mask
        # Hue > 115 needed
        grass_only = cv2.inRange(hsv, (35, 85, 0), (115, 255, 255))
        grass_only = cv2.vconcat([np.zeros((h + 1, grass_only.shape[1]), dtype=grass_only.dtype), grass_only])

        # Use odd numbers for all circular masks otherwise the line will shift location
        grass_only_0 = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, self.circular_mask(5))
        grass_only_1 = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, self.circular_mask(5))
        grass_only_2 = cv2.morphologyEx(grass_only_1, cv2.MORPH_OPEN, self.circular_mask(21))
        grass_only_3 = cv2.morphologyEx(grass_only_2, cv2.MORPH_CLOSE, self.circular_mask(61))

        grass_only_morph = cv2.morphologyEx(grass_only_3, cv2.MORPH_ERODE, self.circular_mask(9))
        grass_only_flipped = cv2.bitwise_not(grass_only)

        lines_only = cv2.bitwise_and(grass_only_flipped, grass_only_flipped, mask=grass_only_morph)
        lines_only = cv2.morphologyEx(lines_only, cv2.MORPH_CLOSE, self.circular_mask(5))

        if debug:
            cv2.imshow("grass_only", grass_only)
            cv2.imwrite("/tmp/grass_only.png", grass_only)
            cv2.imshow("grass_only_0", grass_only_0)
            cv2.imwrite("/tmp/grass_only_0.png", grass_only_0)
            cv2.imshow("grass_only_1", grass_only_1)
            cv2.imwrite("/tmp/grass_only_1.png", grass_only_1)
            cv2.imshow("grass_only_2", grass_only_2)
            cv2.imwrite("/tmp/grass_only_2.png", grass_only_2)
            cv2.imshow("grass_only_3", grass_only_3)
            cv2.imwrite("/tmp/grass_only_3.png", grass_only_3)
            cv2.imshow("grass_only_morph", grass_only_morph)
            cv2.imwrite("/tmp/grass_only_morph.png", grass_only_morph)
            cv2.imshow("grass_only_flipped", grass_only_flipped)
            cv2.imwrite("/tmp/grass_only_flipped.png", grass_only_flipped)
            cv2.imshow("lines_only", lines_only)
            cv2.imwrite("/tmp/lines_only.png", lines_only)
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
                if i % self.point_cloud_spacing == 0:
                    camToPoint = Transformation(self.camera.findFloorCoordinate([px, py]))

                    # Exclude points too far away
                    if camToPoint.norm_squared < self.point_cloud_max_distance**2:
                        points3d.append(camToPoint.position)

            # Publish straight base link
            self.tf_broadcaster.sendTransform(
                self.camera.pose_base_link_straight.position,
                self.camera.pose_base_link_straight.quaternion,
                img.header.stamp,
                self.robot_name + "/base_footprint_straight",
                self.robot_name + "/odom",
            )

            # Publish fieldlines in laserscan format
            header = Header()
            header.stamp = img.header.stamp
            header.frame_id = self.robot_name + "/base_footprint_straight"
            if self.ground_truth:
                if not self.publish_point_cloud:
                    header.frame_id = self.robot_name + "/base_footprint_straight"
                else:
                    header.frame_id = "world"
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
            self.point_cloud_publisher.publish(point_cloud_msg)

        t_end = time.time()
        rospy.loginfo_throttle(60, "Fieldline detection rate: " + str(t_end - t_start))


if __name__ == "__main__":
    rospy.init_node("detector_fieldline")
    fieldline_detector = DetectorFieldline()
    rospy.spin()
