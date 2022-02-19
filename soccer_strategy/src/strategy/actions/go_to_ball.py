from actions.action import Action
import numpy as np
import math
import config
from robot import Robot
from navigation import Navigation

class GoToBall(Action):
    def execute(self, robot, team_data):
        # navigate robot behind the ball
        Navigation.navigate_to_position_with_offset(robot, team_data)




