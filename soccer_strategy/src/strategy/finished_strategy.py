from strategy.strategy import Strategy
from team import Team
from soccer_msgs.msg import GameState


class FinishedStrategy(Strategy):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        self.stop_all_robots(friendly_team)
