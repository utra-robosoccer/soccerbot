#!/usr/bin/env python3
import os

from sensor_msgs.msg import Imu

from soccer_common import Transformation

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
import tf
from std_msgs.msg import Float64

torso_height = 0.334
imu = Imu()
imu.orientation.w = 1

def torso_height_callback(height: Float64):
    global torso_height
    torso_height = height.data

def imu_callback(imu_msg: Imu):
    global imu
    imu = imu_msg


if __name__ == "__main__":
    rospy.init_node("soccer_base_footprint_to_torso")
    rospy.Subscriber("torso_height", Float64, torso_height_callback)
    rospy.Subscriber("imu_filtered", Imu, imu_callback)
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()

        height_transform = Transformation(position=[0, 0, torso_height])
        imu_transform = Transformation(quaternion=[imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
        orientation_euler = imu_transform.orientation_euler
        orientation_euler[0] = 0
        orientation_euler[1] = 0
        imu_transform.orientation_euler = orientation_euler

        base_link_to_torso = imu_transform @ height_transform

        br.sendTransform(
            base_link_to_torso.position,
            base_link_to_torso.quaternion,
            imu.header.stamp,
            os.environ["ROS_NAMESPACE"] + "/torso",
            os.environ["ROS_NAMESPACE"] + "/base_footprint",
        )

        try:
            rospy.sleep(0.05)
        except rospy.exceptions.ROSInterruptException:
            break
