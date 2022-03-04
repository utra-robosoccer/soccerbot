from strategy.strategy import Strategy, get_back_up
from team import Team
from ball import Ball
try:
    from soccer_msgs.msg import GameState
except:
    class GameState:
        GAMESTATE_PLAYING = 1
        STATE_NORMAL = 2


class StrategyStationary(Strategy):

    @get_back_up
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        pass
