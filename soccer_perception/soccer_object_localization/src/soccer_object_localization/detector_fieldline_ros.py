#!/usr/bin/env python3
import os
import time

from cv2 import Mat
from self.impl.tcpros_base import DEFAULT_BUFF_SIZE
from soccer_object_detection.camera.camera_calculations_ros import CameraCalculationsRos
from soccer_object_localization.detector_fieldline import DetectorFieldline
from tf import TransformBroadcaster

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rclpy
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header


class DetectorFieldlineRos(DetectorFieldline):
    """
    Ros bridge for detecting fieldlines.
    """

    def __init__(self):
        super().__init__()
        self.robot_name = self.get_namespace()[1:-1]
        # TODO fix this up
        self.camera = CameraCalculationsRos(self.robot_name)
        self.camera.reset_position()
        # self.initial_pose_create_subscription = self.create_subscription("initialpose", PoseWithCovarianceStamped,
        #                                                 self.initial_pose_callback, queue_size=1)
        self.image_create_subscription = self.create_subscription(
            "camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-create_subscription-lag-despite-queue-1/)
        self.image_create_publisher = self.create_publisher("camera/line_image", Image, queue_size=1)
        self.point_cloud_create_publisher = self.create_publisher("field_point_cloud", PointCloud2, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()

        self.point_cloud_max_distance = self.get_param("point_cloud_max_distance", 5)
        self.point_cloud_spacing = self.get_param("point_cloud_spacing", 30)

    # TODO look into
    # def initial_pose_callback(self, initial_pose: PoseWithCovarianceStamped):
    #     self.publish_point_cloud = True

    def image_callback(self, img: Image, debug=False):

        t_start = time.time()

        # if self.robot_state.status not in [
        #     RobotState.STATUS_READY,
        #     RobotState.STATUS_LOCALIZING,
        #     RobotState.STATUS_WALKING,
        #     RobotState.STATUS_DETERMINING_SIDE,
        # ]:
        #     return
        #
        # TODO shouldnt need since this is only called if we get an image
        # if not self.camera.ready:
        #     return
        # TODO ros
        if self.ground_truth:
            if not self.publish_point_cloud:
                self.camera.reset_position(timestamp=img.header.stamp)
            else:
                self.camera.reset_position(timestamp=img.header.stamp, from_world_frame=True, camera_frame="/camera_gt")
        else:
            self.camera.reset_position(timestamp=img.header.stamp)
        # Uncomment for ground truth
        self.get_logger().info_once("Started Publishing Fieldlines")

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        lines_only = self.image_filter(image, debug)

        # ROS
        if self.image_create_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(lines_only)
            img_out.header = img.header
            self.image_create_publisher.publish(img_out)

        self.pub_pointcloud(lines_only, img.header.stamp)

        t_end = time.time()
        self.get_logger().error(60, "Fieldline detection rate: " + str(t_end - t_start))

    def pub_pointcloud(self, lines_only: Mat, stamp: self.Time):
        # TODO should this be a super or a new func?
        points3d = self.img_to_points(lines_only)

        # TODO own function
        if self.publish_point_cloud and self.point_cloud_create_publisher.get_num_connections() > 0:

            # Publish straight base link TODO why is this here
            self.tf_broadcaster.sendTransform(
                self.camera.pose_base_link_straight.position,
                self.camera.pose_base_link_straight.quaternion,
                stamp,
                self.robot_name + "/base_footprint_straight",
                self.robot_name + "/odom",
            )

            # TODO ros
            # Publish fieldlines in laserscan format
            header = Header()
            header.stamp = stamp
            header.frame_id = self.robot_name + "/base_footprint_straight"
            if self.ground_truth:
                if not self.publish_point_cloud:
                    header.frame_id = self.robot_name + "/base_footprint_straight"
                else:
                    header.frame_id = "world"
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
            self.point_cloud_create_publisher.publish(point_cloud_msg)


if __name__ == "__main__":
    self.init_node("detector_fieldline")
    fieldline_detector = DetectorFieldlineRos()
    self.spin()
