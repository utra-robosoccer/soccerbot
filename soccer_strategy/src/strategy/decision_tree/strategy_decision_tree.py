import enum
from strategy.strategy import Strategy
from team import Team
try:
    from soccer_msgs.msg import GameState
except:
    from soccer_msgs.fake_msg import GameState
from copy import deepcopy
import abc
import numpy as np


class StrategyDecisionTree(Strategy):
    #can we just give pointers to friendly_teeam, opponent_team, and game_state here so we don't need to keep passing them?
    def __init__(self):
        super().__init__()
        self.current_formation = Formations.DEFENSIVE

    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        self.current_formation = self.decide_formation()

    # something so there can be different formation deciders like a super defensive biased one
    def decide_formation(self):
        if self.team_data.ball.is_known() and self.team_data.ball.position[0] < 0:  # ball in oppents side
            return Formations.ATTACKING
        else:
            return Formations.DEFENSIVE


class FieldPosition:
    # can add other stuff like shape later
    def __init__(self, center):
        self.center = center


class FieldPositions:
    CENTER_STRIKER = FieldPosition(np.array([0, 0]))
    GOALIE = FieldPosition(np.array([4.5, 0]))
    RIGHT_WING = FieldPosition(np.array([-2, 3]))
    LEFT_WING = FieldPosition(np.array([-2, -3]))
    RIGHT_BACK = FieldPosition(np.array([3.5, 2]))
    LEFT_BACK = FieldPosition(np.array([3.5, -2]))
    CENTER_BACK = FieldPosition(np.array([3, 0]))


# formations can be in a human readable data file?
class Formation:
    def __init__(self, positions):
        # array where 0 position is position for robot 0 (goalie) I guess?
        #TODO: change positions to dictionary with robot_id
        self.positions = positions

    def closest_position(self, target):
        a = [distance(position.center, target) for position in self.positions]
        return np.argmin(a) + 1


# should have a utils class for geometry calculations, geometry file/class?
def distance(o1, o2):
    return np.linalg.norm(o1 - o2)


class Formations:
    #don't go to role
    DEFENSIVE = Formation(
        [FieldPositions.GOALIE, FieldPositions.LEFT_BACK, FieldPositions.RIGHT_BACK, FieldPositions.CENTER_BACK])
    ATTACKING = Formation(
        [FieldPositions.GOALIE, FieldPositions.LEFT_WING, FieldPositions.CENTER_STRIKER, FieldPositions.RIGHT_WING])

