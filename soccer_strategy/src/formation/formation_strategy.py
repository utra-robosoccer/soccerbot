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
        # TODO: change positions to dictionary with robot_id
        self.positions = positions

    def closest_position(self, target):
        a = [distance(position.center, target) for position in self.positions]
        return np.argmin(a) + 1


# should have a utils class for geometry calculations, geometry file/class?
def distance(o1, o2):
    return np.linalg.norm(o1 - o2)


class Formations:
    # don't go to role
    DEFENSIVE = Formation(
        [FieldPositions.GOALIE, FieldPositions.LEFT_BACK, FieldPositions.RIGHT_BACK, FieldPositions.CENTER_BACK])
    ATTACKING = Formation(
        [FieldPositions.GOALIE, FieldPositions.LEFT_WING, FieldPositions.CENTER_STRIKER, FieldPositions.RIGHT_WING])


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

    def __init__(self, default_formation, team_data):
        self.formation = default_formation
        self.team_data = team_data
        self.enemy_goal_position = None

    def update_strategy(self, robot):
        self.formation = self.decide_formation()
        self.act_individual(robot)

    def decide_formation(self):
        raise NotImplementedError("please implement")

    def act_individual(self, robot):
        raise NotImplementedError("please implement")

    def update_goal_position(self, field_side, firstHalf):
        goal_position = config.position_map_goal(
            config.ENEMY_GOAL_POSITION,
            field_side,
            firstHalf,
            1  # game_properties.secondary_state == GameState.STATE_PENALTYSHOOT #is pentalty shot
        )

        if abs(self.team_data.ball.get_position()[1]) < 1.0:
            goal_position[1] = self.team_data.ball.get_position()[1]
        self.enemy_goal_position = goal_position
