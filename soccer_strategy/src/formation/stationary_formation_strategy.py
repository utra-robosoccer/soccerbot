import numpy as np
import math
import rospy

# 3D
# from robot_ros import Robot

from formation.formation_strategy import FormationStrategy, Formations
import config


class StationaryFormationStrategy(FormationStrategy):
    def __init__(self, team_data):
        super().__init__(Formations.DEFENSIVE, team_data)
        self.decision_tree = config.StationaryDecisionTree

    def decide_formation(self):
        return

    def act_individual(self, robot):
        self.decision_tree.execute(robot, self.team_data)
