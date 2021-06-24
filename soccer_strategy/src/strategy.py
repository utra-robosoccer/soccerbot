import math
import numpy as np
from robot import Robot

class Strategy:
    def __init__(self):
        pass

    def update_friendly_strategy(self, robots, ball, teamcolor, is_first_half):
        friendly = []
        opponent = []
        for robot in robots:
            if robot.team == Robot.Team.FRIENDLY or robot.Team == Robot.Team.FRIENDLY:
                friendly.append(robot)
            else:
                opponent.append(robot)

        self.update_next_strategy(friendly, opponent, ball, teamcolor, is_first_half)

    def update_next_strategy(self, friendly, opponent, ball):
        raise NotImplementedError

    def check_ball_avaliable(self, ball):
        if ball.position is None:
            print("No ball position available")
            return False
        else:
            return True

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

    def update_next_strategy(self, friendly, opponent, ball, teamcolor, is_first_half):
        if not self.check_ball_avaliable(ball):
            return

        # Guess who has the ball
        current_closest = self.who_has_the_ball(friendly, ball)
        if current_closest == None or not current_closest.send_nav:
            return
        if abs(ball.get_position()[0]) > 3.5 or abs(ball.get_position()[1]) > 5:
            return
        print("Here for ball")
        print(current_closest.get_position())

        position = [round(ball.get_position()[0], 2),
                    round(ball.get_position()[1], 2), round(current_closest.get_position()[2], 2)]
        print(position)
        print("X: ", ball.get_position()[1], "Y: ", -ball.get_position()[0])
        print(np.linalg.norm(current_closest.get_position()[0:2] - ball.get_position()[0:2]))
        if np.linalg.norm(current_closest.get_position()[0:2] - ball.get_position()) < 0.14:
            # Stop moving

            if teamcolor == 1: #blue
                if is_first_half == 1:
                    opponent_goal = np.array([0, 4.5])
                else:
                    opponent_goal = np.array([0, -4.5])
            else:
                if is_first_half == 1:
                    opponent_goal = np.array([0, -4.5])
                else:
                    opponent_goal = np.array([0, 4.5])


            # Kick the ball towards the goal
            delta = opponent_goal - ball.get_position()
            unit = delta / np.linalg.norm(delta)

            # current_closest.status = Robot.Status.KICKING
            # current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)
        else:
            # current_closest.set_navigation_position(position)
            pass

# BROKEN DO NOT USE
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
        if not self.check_ball_avaliable(ball):
            return

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

            current_closest.status = Robot.Status.KICKING
            current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)

        else:
            current_closest.set_navigation_position(np.append(ball.get_position(), 0))


class FreekickStrategy(DummyStrategy):
    # preparation if we are the kicking team
    def update_kicking_strategy(self, friendly, ball):
        if not self.check_ball_avaliable(ball):
            return False

        non_kicker = []
        for robot in friendly:
            if robot.designated_kicker:
                kicker = self.who_has_the_ball(friendly, ball)
            else:
                non_kicker.append(robot)

        if kicker == None:
            # should not happen
            return

        # todo move non-kicking robots

        if np.linalg.norm(kicker.get_position()[0:2] - ball.get_position()) < 0.2:
            # Stop moving
            kicker.set_navigation_position(kicker.get_position())
            return True
        else:
            kicker.set_navigation_position(np.append(ball.get_position(), 0))
            return False

    # kicker actually kick the ball
    def execute_kicking(self, friendly, ball):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            if robot.designated_kicker:
                kicker = self.who_has_the_ball(friendly, ball)

        if kicker is None:
            return

        if np.linalg.norm(kicker.get_position()[0:2] - ball.get_position()) < 0.2:
            kicker.set_navigation_position(kicker.get_position())
            kicker.status = Robot.Status.KICKING
        else:
            kicker.set_navigation_position(np.append(ball.get_position(), 0))

    # preparation if we are not the kicking team
    def update_non_kicking_strategy(self, friendly, ball, teamcolor, is_first_half):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            if teamcolor == 1: #blue
                if is_first_half == 1:
                    own_goal = np.array([0, -4.5])
                    angle = 0
                else:
                    own_goal = np.array([0, 4.5])
                    angle = 3.14
            elif teamcolor == 0: #red
                if is_first_half == 1:
                    own_goal = np.array([0, 4.5])
                    angle = 3.14
                else:
                    own_goal = np.array([0, -4.5])
                    angle = 0

            if robot.role == Robot.Role.LEFT_MIDFIELD:
                nav_pose = ball.get_position() + np.array([0.5, 0])
                robot.set_navigation_position(np.append(nav_pose, angle))
            if robot.role == Robot.Role.RIGHT_MIDFIELD:
                nav_pose = ball.get_position() + np.array([-0.5, 0])
                robot.set_navigation_position(np.append(nav_pose, angle))
            if robot.role == Robot.Role.GOALIE:
                nav_pose = own_goal
                robot.set_navigation_position(np.append(nav_pose, angle))

        #todo make is so that all robot stay within the field boundary

class PenaltykickStrategy(FreekickStrategy):
    # preparation if we are not the kicking team
    def update_non_kicking_strategy(self, friendly, ball, teamcolor, is_first_half):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            if is_first_half == 1:
                if robot.role == Robot.Role.LEFT_MIDFIELD:
                    if teamcolor == 1:
                        nav_pose = np.array([-1, -1, 0])
                    else:
                        nav_pose = np.array([-1, 1, 3.14])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.RIGHT_MIDFIELD:
                    if teamcolor == 1:
                        nav_pose = np.array([1, -1, 0])
                    else:
                        nav_pose = np.array([1, 1, 3.14])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.GOALIE:
                    if teamcolor == 1:
                        nav_pose = np.array([0, -4.5, 0])
                    else:
                        nav_pose = np.array([0, 4.5, 3.14])
                    robot.set_navigation_position(nav_pose)
            # second half
            else:
                if robot.role == Robot.Role.LEFT_MIDFIELD:
                    if teamcolor == 1:
                        nav_pose = np.array([-1, 1, 3.14])
                    else:
                        nav_pose = np.array([-1, -1, 0])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.RIGHT_MIDFIELD:
                    if teamcolor == 1:
                        nav_pose = np.array([1, 1, 3.14])
                    else:
                        nav_pose = np.array([1, -1, 0])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.GOALIE:
                    if teamcolor == 1:
                        nav_pose = np.array([0, 4.5, 3.14])
                    else:
                        nav_pose = np.array([0, -4.5, 0])
                    robot.set_navigation_position(nav_pose)
