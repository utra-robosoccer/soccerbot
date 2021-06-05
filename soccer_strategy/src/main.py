#!/usr/bin/env python3

from game_engine_ros import GameEngineRos
import rospy

RUN_IN_ROS = True

if __name__ == '__main__':

    rospy.init_node("soccer_strategy")
    rospy.sleep(20)

    g = GameEngineRos()
    g.run()