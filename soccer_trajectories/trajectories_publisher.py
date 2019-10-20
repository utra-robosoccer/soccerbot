#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64

joints = ["left_ankle_foot",
          "left_bicep_forearm",
          "left_calve_ankle",
          "left_hip_front_thigh",
          "left_hip_side_hip_front",
          "left_thigh_calve",
          "neck_head",
          "right_ankle_foot",
          "right_bicep_forearm",
          "right_calve_ankle",
          "right_hip_front_thigh",
          "right_hip_side_hip_front",
          "right_thigh_calve",
          "torso_left_bicep",
          "torso_left_hip_side",
          "torso_neck",
          "torso_right_bicep",
          "torso_right_hip_side"]


def get_setpoints(timestamp):
    setpoints = {joint: 0.0 for joint in joints}
    setpoints["left_hip_front_thigh"] = 1.0
    return setpoints

def publish_trajectories():
    # TODO: Is this a good queue size?
    publishers = {joint: rospy.Publisher("/{}/command".format(joint), Float64, queue_size=10) for joint in joints}
    
    rospy.init_node("trajectory_publisher")
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        setpoints = get_setpoints(None)
        rospy.loginfo(str(setpoints))
        for joint in setpoints:
            publishers[joint].publish(setpoints[joint])
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_trajectories()
    except rospy.ROSInterruptException:
        pass
