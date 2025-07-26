import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from soccer_pycontrol.model.sensors import Sensors
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from soccer_common import Transformation


class SensorsROS(Sensors):
    def __init__(self, node, ns: str = ""):
        # TODO rework later
        self.node = node
        self.imu_msg = None
        self.odom_msg = None
        self.imu_create_subscription = self.node.create_subscription(Imu, ns + "imu_filtered", self.imu_callback, qos_profile=10)
        self.imu_ready = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.pose_create_subscription = self.node.create_subscription(Odometry, ns + "base_pose_ground_truth", self.odom_callback, qos_profile=10)

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU information

        :param msg: IMU Message
        """
        self.imu_msg = msg
        self.imu_ready = True
        # TODO rework imu its a bit weird

    def odom_callback(self, msg: Odometry):
        """
        Callback function for IMU information

        :param msg: IMU Message
        """
        self.odom_msg = msg

    def get_imu(self):
        """
        Gets the IMU at the IMU link location.

        :return: calculated orientation of the center of the torso of the robot
        """

        assert self.imu_ready
        return Transformation(
            [0, 0, 0],
            [
                self.imu_msg.orientation.x,
                self.imu_msg.orientation.y,
                self.imu_msg.orientation.z,
                self.imu_msg.orientation.w,
            ],
        ).orientation_euler

    def get_foot_pressure_sensors(self, floor):
        # TODO subscribe to foot pressure sensors
        pass

    def get_pose(self):
        # Transformation(pose=self.odom_msg.pose.pose)
        # return Transformation(pose=self.odom_msg.pose.pose)
        try:
            t = self.tf_buffer.lookup_transform("base_link", "left_foot", rclpy.time.Time())
            return Transformation(
                position=[0, 0, t.transform.translation.z],
                quaternion=[t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w],
            )
        except TransformException as ex:
            self.node.get_logger().info(f'Could not transform {"base_link"} to {"left_foot"}: {ex}')
            return

    def get_ball(self):
        try:
            t1 = self.tf_buffer.lookup_transform("robot1/base_footprint_gt", "world", rclpy.time.Time())

            t2 = self.tf_buffer.lookup_transform("robot1/ball_gt", "world", rclpy.time.Time())
            trans1 = [t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z]
            trans2 = [t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z]
            trans = (np.array(trans2) - np.array(trans1)).tolist()
            return Transformation(position=trans, orientation=t2.transform.rotation)
        except TransformException as ex:
            self.node.get_logger().error("Unable to find transformation from world to ")

    def get_height(self):

        try:
            t = self.tf_buffer.lookup_transform("head", "left_foot", rclpy.time.Time())
            temp = Transformation(
                position=[0, 0, t.transform.translation.z],
                quaternion=[t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w],
            )
            temp.orientation_euler = 1 * temp.orientation_euler
            return temp
        except TransformException as ex:
            self.node.get_logger().error("Unable to find transformation from world to ")
            return Transformation()

    def get_global_height(self):
        try:
            t = self.tf_buffer.lookup_transform("robot1/camera_gt", "robot1/base_footprint_gt", rclpy.time.Time())
            return Transformation(
                position=[t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
                quaternion=[t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w],
            )
        except TransformException as ex:
            self.node.get_logger().error(5, "Unable to find transformation from world to ")
            pass
