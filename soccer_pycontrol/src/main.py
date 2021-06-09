#!/usr/bin/env python3
import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
import soccerbot_controller_ros

import matplotlib as plt
#plt.use('tkagg')

USE_RL_CONTROL=False

if __name__ == '__main__':

    rospy.init_node("soccer_control")

    if USE_RL_CONTROL:
        import soccerbot_controller_ros_rl
        walker = soccerbot_controller_ros_rl.SoccerbotControllerRosRl()
        walker.run()
    else:
        walker = soccerbot_controller_ros.SoccerbotControllerRos()
        walker.run()
