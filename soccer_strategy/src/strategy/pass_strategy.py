import math
import numpy as np

from strategy.dummy_strategy import DummyStrategy
from robot import Robot

# BROKEN DO NOT USE
class PassStrategy(DummyStrategy):

    def get_closest_teammate(self, player, team):
        closest_dist = math.inf
        current_closest = None
        for robot in team:
            if robot == player:
                continue
            dist = np.linalg.norm(player.position[0:2] - robot.position[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest

    def update_next_strategy(self, friendly, opponent, ball):
        if not self.check_ball_available(ball):
            return

        # Guess who has the ball
        current_closest = self.who_has_the_ball(friendly, ball)

        a = current_closest.position
        b = ball.position
        if np.linalg.norm(current_closest.position[0:2] - ball.position) < 0.2:
            # Stop moving
            current_closest.set_navigation_position(current_closest.position)

            # Kick the ball towards the goal
            closest_teammate = self.get_closest_teammate(current_closest, friendly)
            delta_teammate = closest_teammate.position[0:2] - current_closest.position[0:2]
            dist_to_teammate = np.linalg.norm(delta_teammate)
            opponent_goal = current_closest.get_opponent_net_position()
            dist_to_goal = np.linalg.norm(opponent_goal - current_closest.position[0:2])
            delta_goal = opponent_goal - ball.position
            if dist_to_goal > dist_to_teammate and np.dot(delta_teammate, delta_goal) > 0:
                unit = delta_teammate / np.linalg.norm(delta_teammate)
            else:
                unit = delta_goal / np.linalg.norm(delta_goal)

            current_closest.status = Robot.Status.KICKING
            current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)

        else:
            current_closest.set_navigation_position(np.append(ball.position, 0))