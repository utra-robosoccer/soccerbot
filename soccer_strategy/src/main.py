#!/usr/bin/env python3
import rospy

from game_engine_3d import GameEngine3D

if __name__ == '__main__':
    rospy.init_node("soccer_strategy")
    g = GameEngine3D()
    g.run()

