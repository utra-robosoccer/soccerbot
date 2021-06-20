#!/usr/bin/env python3
import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
import soccerbot_controller_ros


if __name__ == '__main__':

    rospy.init_node("soccer_control")
    rospy.logwarn("Initializing Soccer Control")
    rospy.sleep(5)
    walker = soccerbot_controller_ros.SoccerbotControllerRos()
    rospy.logwarn("Starting Control Loop")
    walker.run()
