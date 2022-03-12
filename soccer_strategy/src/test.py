from unittest import TestCase
from game_engine_2d import GameEngine2D
from strategy.strategy_dummy import StrategyDummy
from strategy.strategy_stationary import StrategyStationary
from strategy.decision_tree_lookahead.strategy_decision_tree_lookahead import StrategyDecisionTreeLookhead

class Test(TestCase):
    def test_dummy_strategy(self):
        friendly_wins = 0
        opponent_wins = 0
        for i in range(10):
            g = GameEngine2D(team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy)

            friendly_points, opponent_points = g.run()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f'Friendly: {friendly_wins}, opponent: {opponent_wins}')

    def test_formation_strategy(self):
        friendly_wins = 0
        opponent_wins = 0
        for i in range(10):
            g = GameEngine2D()
            g.team1.strategy = StrategyDecisionTreeLookhead()
            g.team2.strategy = StrategyDecisionTreeLookhead()

            friendly_points, opponent_points = g.run()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f'Friendly: {friendly_wins}, opponent: {opponent_wins}')