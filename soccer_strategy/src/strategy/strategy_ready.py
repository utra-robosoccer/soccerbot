from strategy.strategy import Strategy, get_back_up
from team import Team
from soccer_msgs.msg import GameState
from robot import Robot
from strategy.interfaces.actions import Actions
import numpy as np

class StrategyReady(Strategy):
    def __init__(self):
        super().__init__()
        self.update_frequency = 5

    @get_back_up
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        Actions.resume_all_robots(friendly_team.robots)
        this_robot = self.get_current_robot(friendly_team)

        navigation_position = friendly_team.formations["initial"][this_robot.role]
        if np.linalg.norm(this_robot.position[0:2] - navigation_position[0:2]) > 0.25:
            Actions.navigation_to_position(this_robot, navigation_position)
