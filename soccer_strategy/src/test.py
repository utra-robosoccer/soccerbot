from unittest import TestCase
from game_engine import GameEngine
from strategy.dummy_strategy import DummyStrategy
from strategy.decision_tree.formation_strategy import FormationDecisionTreeStrategy

class Test(TestCase):
    def test_dummy_strategy(self):
        friendly_wins = 0
        opponent_wins = 0
        for i in range(10):
            g = GameEngine()
            g.team1.strategy = DummyStrategy()
            g.team2.strategy = DummyStrategy()

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
            g.team1.strategy = FormationDecisionTreeStrategy()
            g.team2.strategy = FormationDecisionTreeStrategy()

            friendly_points, opponent_points = g.run_loop()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f'Friendly: {friendly_wins}, opponent: {opponent_wins}')