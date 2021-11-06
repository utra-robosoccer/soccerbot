#!/usr/bin/env python3
import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
import tf
from std_msgs.msg import Float64

torso_height = 0.334
def torso_height_callback(height: Float64):
    global torso_height
    torso_height = height.data


if __name__ == '__main__':
    rospy.init_node("soccer_base_footprint_to_torso")
    rospy.Subscriber("torso_height", Float64, torso_height_callback)
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, torso_height),
                         [0, 0, 0, 1],
                         rospy.Time.now(),
                         os.environ["ROS_NAMESPACE"] + "/torso",
                         os.environ["ROS_NAMESPACE"] + "/base_footprint")

        try:
            rospy.sleep(0.05)
        except rospy.exceptions.ROSInterruptException:
            break