import math
import numpy as np

from robot import Robot


class Strategy:
    def __init__(self):
        pass

    def update_team_strategy(self, robots, ball, teamcolor, is_first_half, secondaryState, opponent_team=False):
        friendly = []
        opponent = []
        for robot in robots:
            if robot.team == Robot.Team.FRIENDLY or robot.Team == Robot.Team.FRIENDLY:
                friendly.append(robot)
            else:
                opponent.append(robot)
        if not opponent_team:
            self.update_next_strategy(friendly, opponent, ball, teamcolor, is_first_half, secondaryState)
        else:
            self.update_next_strategy(opponent, friendly, ball, teamcolor, is_first_half, secondaryState)

    def update_next_strategy(self, friendly, opponent, ball, teamcolor, is_first_half, secondaryState):
        raise NotImplementedError

    def check_ball_avaliable(self, ball):
        if ball.get_position() is None:
            print("No ball position available")
            return False
        else:
            return True

    def who_has_the_ball(self, robots, ball):
        closest_dist = math.inf
        current_closest = None
        for robot in robots:
            if robot.status != Robot.Status.READY and robot.status != Robot.Status.WALKING:
                continue

            # if robot.relocalization_timeout > 0:
            #     robot.relocalization_timeout = robot.relocalization_timeout - 1
            #     continue

            dist = np.linalg.norm(ball.get_position()[0:2] - robot.get_position()[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest
