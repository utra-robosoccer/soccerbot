#!/usr/bin/env python3
import sys
import time
from game_engine import GameEngine

RUN_IN_ROS = False

if __name__ == '__main__':
    if (len(sys.argv) > 2 and sys.argv[1] == '__name:=soccer_strategy') and RUN_IN_ROS:
        import rospy
        from game_engine_ros import GameEngineRos
        rospy.init_node("soccer_strategy")
        g = GameEngineRos()
        g.run()
    else:
        DISPLAY_GAME = False
        NUM_GAMES = 10
        num_friendly_wins = 0
        tot_friendly_pts = 0
        num_opponent_wins = 0
        tot_opponent_pts = 0
        ts = time.time()
        for i in range(NUM_GAMES):
            g = GameEngine(display=DISPLAY_GAME)
            friendly_pts, opponent_pts = g.run()
            tot_friendly_pts += friendly_pts
            tot_opponent_pts += opponent_pts
            if friendly_pts > opponent_pts:
                num_friendly_wins += 1
            elif friendly_pts < opponent_pts:
                num_opponent_wins += 1
        tf = time.time()
        num_ties = NUM_GAMES - num_friendly_wins - num_opponent_wins
        print('\nFINAL SCORES')
        print(f'Friendly: {num_friendly_wins}, opponent: {num_opponent_wins}, ties: {num_ties}')
        print(f'Friendly points: {tot_friendly_pts}, Opponent points: {tot_opponent_pts}')
        print(f'Avg match time: {(tf - ts) / NUM_GAMES:.2f}s')
