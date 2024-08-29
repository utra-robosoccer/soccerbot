import os

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from soccer_pycontrol.model.sensors import Sensors

from soccer_common import Transformation


class SensorsROS(Sensors):
    def __init__(self, ns: str = ""):
        # TODO rework later
        self.imu_msg = None
        self.odom_msg = None
        self.imu_subscriber = rospy.Subscriber(ns + "imu_filtered", Imu, self.imu_callback, queue_size=1)
        self.imu_ready = False
        self.tf_listener = tf.TransformListener()
        self.pose_subscriber = rospy.Subscriber(ns + "base_pose_ground_truth", Odometry, self.odom_callback, queue_size=1)

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
        return Transformation(pose=self.odom_msg.pose.pose)
