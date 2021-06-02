#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Empty
from tf.transformations import euler_from_quaternion
from enum import Enum

Kp = 5
Kd = 250
Ki = 0.001
desired = 0.0
integral = 0
derivative = 0
lasterror = 0
last_F = 0
start = False


def imu_callback(msg, pub):
    global lasterror, integral, derivative, last_F
    global start
    q = msg.orientation
    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    competition = rospy.get_param("competition")
    # Simulation
    pub_all_motor = rospy.Publisher("all_motor", JointState, queue_size=10)

    # Competition/ Real
    left_motor_publishers = rospy.Publisher("left_arm_motor_0/command", Float64, queue_size=1)
    right_motor_publishers = rospy.Publisher("right_arm_motor_0/command", Float64, queue_size=1)
    # PID
    error = desired - pitch
    # if not (0.02 > error > -0.02):
    #     integral += error
    derivative = error - lasterror
    # if not (0.00005 > derivative > -0.00005):
    #     derivative = 0
    angle = Float64()
    F = (Kp * error) + (Ki * integral) + (Kd * derivative)
    if F > 1.57:
        F = 1.57
    elif F < -1.57:
        F = -1.57
    # if 0.001 > derivative > -0.001:
    #     F = F-0.1
    angle.data = F
    last_F = F
    lasterror = error
    # print(" Error: ", round(error, 4), " F: ", round(angle.data, 4), " Integral: ", round(integral, 4),
    # " Derivative: ", round(derivative, 4))
    left_motor_publishers.publish(angle)
    right_motor_publishers.publish(angle)
    # We want to publish once on state transition


def start_walk():
    pass


def stop_walk():
    pass


def main():
    rospy.init_node('balance')
    rospy.sleep(1.0)
    r = rospy.Rate(10)
    while not rospy.has_param("competition"):
        r.sleep()

    rospy.Subscriber('imu_filtered', Imu, imu_callback)
    rospy.Subscriber('start_walking', Empty, start_walk)
    rospy.Subscriber('terminate_walking', Empty, stop_walk)
    rospy.Subscriber('completed_walking', Empty, stop_walk)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#
# import numpy as np
# from vispy.plot import Fig
#
# fig = Fig()
# ax_left = fig[0, 0]
#
# data = [[0,2],[1,4]]
# ax_left.plot(data)
# data = [[0,0],[1,3]]
# ax_left.plot(data)
# data = [[0,1],[1,5]]
# ax_left.plot(data)
# fig.show(run=True)
