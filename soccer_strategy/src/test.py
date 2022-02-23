from unittest import TestCase
from game_engine import GameEngine
from strategy.strategy_dummy import StrategyDummy
from strategy.decision_tree_lookahead.strategy_decision_tree_lookahead import StrategyDecisionTreeLookhead

class Test(TestCase):
    def test_dummy_strategy(self):
        friendly_wins = 0
        opponent_wins = 0
        for i in range(10):
            g = GameEngine()
            g.team1.strategy = StrategyDummy()
            g.team2.strategy = StrategyDummy()

            friendly_points, opponent_points = g.run_loop()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f'Friendly: {friendly_wins}, opponent: {opponent_wins}')

    def test_formation_strategy(self):
        friendly_wins = 0
        opponent_wins = 0
        for i in range(10):
            g = GameEngine()
            g.team1.strategy = StrategyDecisionTreeLookhead()
            g.team2.strategy = StrategyDecisionTreeLookhead()

            friendly_points, opponent_points = g.run_loop()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f'Friendly: {friendly_wins}, opponent: {opponent_wins}')