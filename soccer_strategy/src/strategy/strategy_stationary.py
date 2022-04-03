from ball import Ball
from strategy.strategy import Strategy, get_back_up
from team import Team

try:
    from soccer_msgs.msg import GameState
except:
    from soccer_msgs.fake_msg import GameState


class StrategyStationary(Strategy):
    @get_back_up
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        pass
