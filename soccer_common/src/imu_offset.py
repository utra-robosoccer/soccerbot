#!/usr/bin/env python3

from sensor_msgs.msg import Imu
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

rospy.init_node("imu_offset")
offset = rospy.get_param("~offset")

p = rospy.Publisher("imu_corrected", Imu, queue_size=10)

def correct_imu(imu_msg: Imu):
    euler = euler_from_quaternion([imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z])
    euler_offset = euler[2] + offset
    q = quaternion_from_euler(euler[0], euler[1], euler_offset)
    imu_msg.orientation.w = q[0]
    imu_msg.orientation.x = q[1]
    imu_msg.orientation.y = q[2]
    imu_msg.orientation.z = q[3]
    p.publish(imu_msg)
    pass
s = rospy.Subscriber("imu_filtered", Imu, correct_imu)
rospy.spin()