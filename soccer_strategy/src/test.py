from unittest import TestCase
from game_engine import GameEngine


class Test(TestCase):
    def test_game(self):
        friendly_wins = 0
        opponent_wins = 0
        for i in range(10):
            g = GameEngine()
            friendly_points, opponent_points = g.run()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f'Friendly: {friendly_wins}, opponent: {opponent_wins}')
