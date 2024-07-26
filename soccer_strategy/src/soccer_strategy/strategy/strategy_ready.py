import numpy as np

from soccer_common.utils import wrapToPi
from soccer_msgs.msg import GameState
from soccer_strategy.old.robot import Robot
from soccer_strategy.old.team import Team
from soccer_strategy.strategy.strategy import Strategy, get_back_up
from soccer_strategy.strategy.utils import Utility


class StrategyReady(Strategy):
    """
    Strategy used when the robot is getting ready, basically move towards the formation position
    """

    def __init__(self):
        super().__init__()
        self.update_frequency = 1

    @get_back_up
    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().step_strategy(friendly_team, opponent_team, game_state)

        Utility.resume_all_robots(friendly_team.robots)
        this_robot = self.get_current_robot(friendly_team)

        if this_robot.status != Robot.Status.WALKING:  # TODO use dynamic walking to adjust position
            navigation_position = np.array(friendly_team.formations["ready"][this_robot.role], dtype=float)
            if np.linalg.norm(this_robot.position[0:2] - navigation_position[0:2]) > 0.10:
                this_robot.set_navigation_position(navigation_position)
            elif abs(wrapToPi(this_robot.position[2] - navigation_position[2])) > 0.2:  # Rotate only
                navigation_position[0:2] = this_robot.position[0:2]
                this_robot.set_navigation_position(navigation_position)
            else:
                self.complete = True
