import math

import numpy as np
import py_trees.composites
import rospy
from Behaviours import *

from soccer_common.utils import wrapTo2Pi
from soccer_msgs.msg import GameState
from soccer_strategy.robot import Robot
from soccer_strategy.strategy.strategy import (
    Strategy,
    get_back_up,
    update_average_ball_position,
)
from soccer_strategy.strategy.utils import Utility
from soccer_strategy.team import Team


class StrategyDummer(Strategy):
    """
    A More basic strategy, we are not even thnking that much
    """

    def __init__(self):
        self.time_of_end_of_action = rospy.Time.now()
        self.update_frequency = 1
        super(StrategyDummer, self).__init__()

    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().step_strategy(friendly_team, opponent_team, game_state)

        this_robot = self.get_current_robot(friendly_team)
        tree = self.create_tree(self, friendly_team, opponent_team, game_state, this_robot)
        tree.tick_once()

    def create_tree(self, friendly_team: Team, opponent_team: Team, game_state: GameState, robot: Robot):
        root = py_trees.composites.Sequence("Dummber BT", memory=False)

        # Base layer
        non_action_decider = py_trees.composites.Parallel("non_action_decider", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
        non_action_decider.add_children(Not_In_Action(self, robot=robot, time=self.time_of_end_of_action))
        non_action_decider.add_children(Game_Status(self, game_state, robot=robot))

        root.add_child(non_action_decider)
        root.add_child(Seen_Ball_2sec(self, friendly_team))

        # movement behaviors
        movement_selector = py_trees.composites.Selector("movement_selector", memory=True)

        return root
