from strategy.strategy import Strategy, get_back_up
from team import Team
from soccer_msgs.msg import GameState
from robot import Robot
from strategy.interfaces.actions import Actions

class StrategyReady(Strategy):
    @get_back_up
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        Actions.resume_all_robots(friendly_team.robots)
        this_robot = self.get_current_robot(friendly_team)
        Actions.navigation_to_position(this_robot, friendly_team.formations["initial"][this_robot.role])
