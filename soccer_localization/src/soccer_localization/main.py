#!/usr/bin/env python3
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import rospy

from soccer_localization.field_lines_ukf_ros import FieldLinesUKFROS

if __name__ == "__main__":
    rospy.init_node("soccer_localization")
    rospy.loginfo("Initializing Soccer Localization")

    f = FieldLinesUKFROS()
    rospy.spin()
