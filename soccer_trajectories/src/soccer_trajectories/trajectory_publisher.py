#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64


def publish_trajectories(trajectory):
    publishers = {joint: rospy.Publisher("/{}/command".format(joint), Float64, queue_size=10) for joint in
                  trajectory.joints()}
    
    rospy.init_node("trajectory_publisher")
    while not rospy.get_rostime():
        pass
    rate = rospy.Rate(10)  # 10hz
    start = rospy.get_rostime()
    now = rospy.get_rostime()
    delta = (now - start).to_sec()

    while not rospy.is_shutdown() and delta < trajectory.total_time:
        now = rospy.get_rostime()
        delta = (now - start).to_sec()
        for joint, setpoint in trajectory.get_setpoint(delta).iteritems():
            publishers[joint].publish(setpoint)
        rate.sleep()
