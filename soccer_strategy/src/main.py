#!/usr/bin/env python3
from game_engine_ros import GameEngineRos
import rospy

if __name__ == '__main__':
    rospy.init_node("soccer_strategy")
    rospy.logerr("Waiting")
    rospy.sleep(10)

    g = GameEngineRos()
    g.run()