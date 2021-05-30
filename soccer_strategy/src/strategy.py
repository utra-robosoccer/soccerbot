import math
import numpy as np
from robot import Robot

class Strategy:
    def __init__(self):
        pass

    def update_both_team_strategy(self, robots, ball):
        friendly = []
        opponent = []
        for robot in robots:
            if robot.team == Robot.Team.FRIENDLY or robot.Team == Robot.Team.FRIENDLY:
                friendly.append(robot)
            else:
                opponent.append(robot)

        self.update_next_strategy(friendly, opponent, ball)

    def update_next_strategy(self, friendly, opponent, ball):
        raise NotImplementedError


class StationaryStrategy(Strategy):
    def update_next_strategy(self, friendly, opponent, ball):
        return

class DummyStrategy(Strategy):

    def who_has_the_ball(self, robots, ball):
        closest_dist = math.inf
        current_closest = None
        for robot in robots:
            if robot.status != Robot.Status.READY:
                continue

            dist = np.linalg.norm(ball.get_position()[0:2] - robot.get_position()[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest

    def update_next_strategy(self, friendly, opponent, ball):
        # Guess who has the ball
        current_closest = self.who_has_the_ball(friendly, ball)

        if current_closest == None:
            return

        a = current_closest.get_position()
        b = ball.get_position()
        if np.linalg.norm(current_closest.get_position()[0:2] - ball.get_position()) < 0.2:
            # Stop moving
            current_closest.set_navigation_position(current_closest.get_position())

            if current_closest.team == Robot.Team.FRIENDLY:
                opponent_goal = np.array([0, 4.5])
            else:
                opponent_goal = np.array([0, -4.5])

            # Kick the ball towards the goal
            delta = opponent_goal - ball.get_position()
            unit = delta / np.linalg.norm(delta)

            current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)
            current_closest.status = Robot.Status.KICKING
        else:
            current_closest.set_navigation_position(np.append(ball.get_position(), 0))
            current_closest.status = Robot.Status.WALKING

class PassStrategy(DummyStrategy):

    def get_closest_teammate(self, player, team):
        closest_dist = math.inf
        current_closest = None
        for robot in team:
            if robot == player:
                continue
            dist = np.linalg.norm(player.get_position()[0:2] - robot.get_position()[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest

    def update_next_strategy(self, friendly, opponent, ball):
        # Guess who has the ball
        current_closest = self.who_has_the_ball(friendly, ball)

        a = current_closest.get_position()
        b = ball.get_position()
        if np.linalg.norm(current_closest.get_position()[0:2] - ball.get_position()) < 0.2:
            # Stop moving
            current_closest.set_navigation_position(current_closest.get_position())

            # Kick the ball towards the goal
            closest_teammate = self.get_closest_teammate(current_closest, friendly)
            delta_teammate = closest_teammate.get_position()[0:2] - current_closest.get_position()[0:2]
            dist_to_teammate = np.linalg.norm(delta_teammate)
            opponent_goal = current_closest.get_opponent_net_position()
            dist_to_goal = np.linalg.norm(opponent_goal - current_closest.get_position()[0:2])
            delta_goal = opponent_goal - ball.get_position()
            if dist_to_goal > dist_to_teammate and np.dot(delta_teammate, delta_goal) > 0:
                unit = delta_teammate / np.linalg.norm(delta_teammate)
            else:
                unit = delta_goal / np.linalg.norm(delta_goal)

            current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)
            current_closest.status = Robot.Status.KICKING
        else:
            # if current_closest.status != Robot.Status.READY:
            #     return
            current_closest.set_navigation_position(np.append(ball.get_position(), 0))
            current_closest.status = Robot.Status.WALKING
