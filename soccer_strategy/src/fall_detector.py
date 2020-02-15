#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from enum import Enum

class State(Enum):
    FALL_FRONT = 'fell_front'
    FALL_BACK = 'fell_back'
    STANDING = 'standing'

ANGLE_THRESHOLD = 1 # in radians
ROBOT_STATE = State.STANDING


def imu_callback(msg, pub):
    global ROBOT_STATE, ANGLE_THRESHOLD
    q = msg.orientation
    roll , pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # We want to publish once on state transition
    if pitch > ANGLE_THRESHOLD and ROBOT_STATE == State.STANDING:
        ROBOT_STATE = State.FALL_FRONT
        pub.publish(ROBOT_STATE.value)

    if pitch < -ANGLE_THRESHOLD and ROBOT_STATE == State.STANDING:
        ROBOT_STATE = State.FALL_BACK
        pub.publish(ROBOT_STATE.value)

    if abs(pitch) < ANGLE_THRESHOLD and ROBOT_STATE in [State.FALL_BACK, State.FALL_FRONT]:
        ROBOT_STATE = State.STANDING
        pub.publish(ROBOT_STATE.value)

def main():
    rospy.init_node('fall_detector')
    pub = rospy.Publisher('fall_state', String, queue_size=10, latch=True)
    rospy.Subscriber('imu', Imu, imu_callback, pub)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
