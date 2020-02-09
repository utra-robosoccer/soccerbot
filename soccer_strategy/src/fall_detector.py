#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

angle_threshold = 1 # in radians

def imu_callback(msg, pub):
    q = msg.orientation
    roll , pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    if pitch > angle_threshold:
        message = String()
        message.data = "I_fell_front"
        pub.publish(message)
    if pitch < -angle_threshold:
        message = String()
        message.data = "I_fell_back"
        pub.publish(message)

def main():
    rospy.init_node('soccer_strategy', anonymous=True)
    pub = rospy.Publisher('fall_state', String, queue_size=10)
    rospy.Subscriber('imu', Imu, imu_callback, pub)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
