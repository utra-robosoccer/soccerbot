#!/usr/bin/env python3
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import numpy as np
import rospy

from soccer_pycontrol.soccerbot_controller_ros import SoccerbotControllerRos

np.set_printoptions(precision=3)


if __name__ == "__main__":

    rospy.init_node("soccer_control")
    rospy.loginfo("Initializing Soccer Control")
    walker = SoccerbotControllerRos()
    rospy.loginfo("Starting Control Loop")
    try:
        walker.run()
    except rospy.exceptions.ROSException as ex:
        exit(0)
