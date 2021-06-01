#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from enum import Enum


class State(Enum):
    FALL_FRONT = 'fell_front'
    FALL_BACK = 'fell_back'
    STANDING = 'standing'


ANGLE_THRESHOLD = 1  # in radians
ROBOT_STATE = State.STANDING
FIRST_STANDING = None
STANDING_THRESHOLD = 10
PUBLISHED_STANDING = False
Kp = 5
Kd = 250
Ki = 0.001
desired = 0.0
integral = 0
derivative = 0
lasterror = 0
last_F = 0


def imu_callback(msg, pub):
    global ROBOT_STATE, ANGLE_THRESHOLD, FIRST_STANDING, STANDING_THRESHOLD, PUBLISHED_STANDING
    global lasterror, integral, derivative, last_F
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
    # print(" Error: ", round(error, 4), " F: ", round(angle.data, 4), " Integral: ", round(integral, 4), " Derivative: ",
    #       round(derivative, 4))
    left_motor_publishers.publish(angle)
    right_motor_publishers.publish(angle)
    # We want to publish once on state transition
    if pitch > ANGLE_THRESHOLD and ROBOT_STATE == State.STANDING:
        ROBOT_STATE = State.FALL_FRONT
        PUBLISHED_STANDING = False
        FIRST_STANDING = None
        pub.publish(ROBOT_STATE.value)

    if pitch < -ANGLE_THRESHOLD and ROBOT_STATE == State.STANDING:
        ROBOT_STATE = State.FALL_BACK
        PUBLISHED_STANDING = False
        FIRST_STANDING = None
        pub.publish(ROBOT_STATE.value)

    if abs(pitch) < ANGLE_THRESHOLD and not PUBLISHED_STANDING:
        ROBOT_STATE = State.STANDING
        if FIRST_STANDING == None:
            print('Pose is vertical')
            FIRST_STANDING = rospy.Time.now()
        if rospy.Time.now() - FIRST_STANDING > rospy.Duration(STANDING_THRESHOLD):
            pub.publish(ROBOT_STATE.value)
            FIRST_STANDING = None
            PUBLISHED_STANDING = True


def main():
    rospy.init_node('balance')
    rospy.sleep(1.0)
    r = rospy.Rate(10)
    while not rospy.has_param("competition"):
        r.sleep()
    pub = rospy.Publisher('fall_state', String, queue_size=10, latch=True)
    rospy.Subscriber('imu_filtered', Imu, imu_callback, pub)
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
