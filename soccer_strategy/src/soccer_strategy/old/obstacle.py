import enum

import numpy as np


class Obstacle:
    class Team(enum.IntEnum):
        UNKNOWN = 0
        FRIENDLY = 1
        OPPONENT = 2

    def __init__(self, position=np.array([0, 0]), team=Team.UNKNOWN, probability: float = 1):
        self.position = position
        self.team = team
        self.probability = probability
