import sys
import unittest
from unittest import TestCase
from unittest.mock import MagicMock

sys.modules["rospy"] = MagicMock()
sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
import rospy

rospy.Time = MagicMock()


from game_engine_2d import GameEngine2D
from strategy.decision_tree.strategy_decision_tree import StrategyDecisionTree
from strategy.decision_tree_lookahead.strategy_decision_tree_lookahead import (
    StrategyDecisionTreeLookhead,
)
from strategy.strategy_dummy import StrategyDummy
from strategy.strategy_stationary import StrategyStationary


class Test(TestCase):
    def setUp(self) -> None:
        super().setUpClass()
        self.display = True
        if "pytest" in sys.argv[0]:
            self.display = False

    def test_dummy_strategy(self):
        friendly_wins = 0
        opponent_wins = 0
        for i in range(1):
            g = GameEngine2D(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy)
            friendly_points, opponent_points = g.run()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f"Friendly: {friendly_wins}, opponent: {opponent_wins}")

    #@unittest.skip("Under Development")
    def test_formation_strategy(self):
        #
        # rospy.init_node("dummy_strategy")
        friendly_wins = 0
        opponent_wins = 0
        for i in range(10):
            g = GameEngine2D(display=self.display, team_1_strategy=StrategyDecisionTree, team_2_strategy=StrategyStationary)
            friendly_points, opponent_points = g.run()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f"Friendly: {friendly_wins}, opponent: {opponent_wins}")
