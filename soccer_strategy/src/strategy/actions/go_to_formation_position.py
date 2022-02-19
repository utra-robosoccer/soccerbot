from actions.action import Action
import numpy as np


class GoToFormationPosition(Action):
    # returns true or false
    def execute(self, robot, team_data):
        #TODO this fails when robot ID doesnt match, this is pre bad in general, should make a change to formation
        goal = team_data.formation.positions[robot.robot_id - 1].center
        # TODO add this if to robot.set_navigation_position
        # if not np.allclose(goal, robot.goal_position):
        robot.set_navigation_position(np.array(goal))
