from soccer_msgs.msg import GameState
from soccer_strategy.strategy.strategy import Strategy
from soccer_strategy.strategy.utils import Utility
from soccer_strategy.team import Team


class StrategyFinished(Strategy):
    """
    Strategy used when the game is finished
    """

    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().step_strategy(friendly_team, opponent_team, game_state)
        Utility.stop_all_robots(friendly_team)
