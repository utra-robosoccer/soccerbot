import abc
from robot_controlled_3d import RobotControlled3D

from soccer_msgs.msg import GameState
from team import Team

class Strategy():
    def __init__(self):
        self.update_frequency = 1

    @abc.abstractmethod
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        raise NotImplementedError

    def get_current_robot(self, friendly_team: Team) -> RobotControlled3D:
        for robot in friendly_team.robots:
            if type(robot) is RobotControlled3D:
                return robot
        raise AssertionError
