#!/usr/bin/env python3
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
FIRST_STANDING = None
STANDING_THRESHOLD = 10
PUBLISHED_STANDING = False

def imu_callback(msg, pub):
    global ROBOT_STATE, ANGLE_THRESHOLD, FIRST_STANDING, STANDING_THRESHOLD, PUBLISHED_STANDING
    q = msg.orientation
    roll , pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # We want to publish once on state transition
    if pitch > ANGLE_THRESHOLD and ROBOT_STATE == State.STANDING:
        get_up = rospy.Publisher('command', String, queue_size=1, latch=True)
        ROBOT_STATE = State.FALL_FRONT
        PUBLISHED_STANDING = False
        FIRST_STANDING = None
        pub.publish(ROBOT_STATE.value)
        msg = String()
        msg.data = "getupfront"
        get_up.publish(msg)

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
    rospy.init_node('fall_detector')
    pub = rospy.Publisher('fall_state', String, queue_size=10, latch=True)
    rospy.Subscriber('imu_data', Imu, imu_callback, pub)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
