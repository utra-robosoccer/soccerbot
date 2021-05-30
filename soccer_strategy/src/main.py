#!/usr/bin/env python3
import sys
from game_engine import GameEngine

RUN_IN_ROS = True

if __name__ == '__main__':


    if (len(sys.argv) > 2 and sys.argv[1] == '__name:=soccer_strategy') or RUN_IN_ROS:
        import rospy
        from game_engine_ros import GameEngineRos

        rospy.init_node("soccer_strategy")
        r = rospy.Rate(10)
        while not rospy.has_param("walking_engine_ready"):
            r.sleep()

        g = GameEngineRos()
        g.run()
    else:
        g = GameEngine()
        g.run()
