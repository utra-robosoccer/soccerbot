from strategy.strategy import Strategy, get_back_up
from team import Team
from soccer_msgs.msg import GameState
from strategy.interfaces.actions import Actions

class StrategySet(Strategy):

    @get_back_up
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        Actions.stop_all_robots(friendly_team.robots)
