import math
import os
import threading
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import scipy
import serial
import yaml
from geometry_msgs.msg import Vector3, TransformStamped
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from soccer_common import Transformation


class Transform(Node):
    def __init__(self):
        Node.__init__(self, "transform_imu")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_msg = None
        self.imu_create_subscription = self.create_subscription(Imu, "imu_filtered", self.imu_callback, qos_profile=10)

        self.imu_ready = False
        # Start the thread
        # self._lock = threading.Lock()
        # self._read_lock = threading.Lock()
        #
        # serial_thread = threading.Thread(target=self.firmware_update_loop)
        # serial_thread.start()


        # self._timer = self.create_timer(1 / 100, self.firmware_update_loop)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU information

        :param msg: IMU Message
        """
        self.imu_msg = msg
        self.imu_ready = True
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = "imu_link"

        t.transform.translation.x = float(0)
        t.transform.translation.y = float(0)
        t.transform.translation.z = float(0)

        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        # self.tf_broadcaster.sendTransform(t)

        # try:
        #     t = self.tf_buffer.lookup_transform("imu_link", "base_link", rclpy.time.Time())
        #     temp = Transformation(position=[0, 0, t.transform.translation.z], quaternion=[t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        #     temp.orientation_euler = 1 * temp.orientation_euler
        #     self.get_logger().error(f"{temp.orientation_euler[1]}")
        #     return temp
        # except TransformException as ex:
        #     self.get_logger().error("Unable to find transformation from world to ")
        #     return Transformation()

        # TODO rework imu its a bit weird
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

    # def firmware_update_loop(self):





def main():
    rclpy.init()
    node = Transform()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
