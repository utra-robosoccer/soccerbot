import math
import numpy as np
import rospy
import enum
from strategy import Strategy
from robot import Robot
from team import Team
from ball import Ball
from soccer_msgs.msg import GameState

import config as config

from tree.tree_node import TreeNode
from actions.action import Action


class DecisionTree:
    def __init__(self, root: TreeNode):
        self.root = root

    def execute(self, robot, team_data):
        curr = self.root
        while not isinstance(curr, Action):
            if curr.execute(robot, team_data):
                curr = curr.children[0]
            else:
                curr = curr.children[1]
        # action
        curr.execute(robot, team_data)


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

class FormationStrategy(Strategy):

    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        self.decide_formation(friendly_team)

        for robot in friendly_team.robots:
            robot.role

    def update_strategy(self, robot):
        pass

    # something so there can be different formation deciders like a super defensive biased one
    def decide_formation(self, friendly_team: Team):
        formation = Formations(self.team_data)
        if self.team_data.ball.is_known():
            if self.team_data.ball.position[0] < -1:  # ball in oppents side
                self.team_data.formation = formation.ATTACKING
            elif self.team_data.ball.position[0] < 1:
                self.team_data.formation = formation.MIDFIELD
            else:
                self.team_data.formation = formation.DEFENSIVE
        else:
            self.team_data.formation = formation.DEFENSIVE

    def act_individual(self, robot):
        self.decision_tree.execute(robot, self.team_data)



