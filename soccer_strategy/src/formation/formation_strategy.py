import math
import numpy as np
import rospy
import enum

import config as config
from strategy.strategy import Strategy
# from soccer_msgs.msg import GameState
from robot import Robot


class FieldPosition:
    # can add other stuff like shape later
    def __init__(self, center):
        self.center = center



# formations can be in a human readable data file?
class Formation:
    def __init__(self, positions):
        # array where 0 position is position for robot 0 (goalie) I guess?
        # TODO: change positions to dictionary with robot_id
        self.positions = []
        for p in positions:
            self.positions.append(FieldPosition(p))

    def closest_position(self, target):
        a = [distance(position.center, target) for position in self.positions]
        return np.argmin(a) + 1



# should have a utils class for geometry calculations, geometry file/class?
def distance(o1, o2):
    return np.linalg.norm(o1 - o2)


class Formations:

    def __init__(self, team_data):
        self.DEFENSIVE = Formation(
            config.get_all_formation_positions(
                team_data.field_side,
                team_data.is_first_half,
                "defence",
            )
        )
        self.MIDFIELD = Formation(
            config.get_all_formation_positions(
                team_data.field_side,
                team_data.is_first_half,
                "midfield",
            )
        )
        self.ATTACKING = Formation(
            config.get_all_formation_positions(
                team_data.field_side,
                team_data.is_first_half,
                "attack",
            )
        )

# class FormationSet:
#     def __init__(self, formations):
#         self.formations = formations
#
#     def decide_formation(self, robots, ball, game_properties):
#         raise NotImplementedError("Please Implement this method")
#
# class NormalFormationSet(FormationSet):
#     def __init__(self):
#         super()

class FormationStrategy:

    def __init__(self, team_data):
        self.team_data = team_data
        self.enemy_goal_position = None

    def update_strategy(self, robot):
        self.decide_formation()
        self.act_individual(robot)

    def decide_formation(self):
        raise NotImplementedError("please implement")

    def act_individual(self, robot):
        raise NotImplementedError("please implement")


