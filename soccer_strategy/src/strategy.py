import math
import numpy as np
import rospy

from robot import Robot
from soccer_msgs.msg import GameState
class Strategy:
    def __init__(self):
        pass

    def update_friendly_strategy(self, robots, ball, teamcolor, is_first_half,secondaryState):
        friendly = []
        opponent = []
        for robot in robots:
            if robot.team == Robot.Team.FRIENDLY or robot.Team == Robot.Team.FRIENDLY:
                friendly.append(robot)
            else:
                opponent.append(robot)

        self.update_next_strategy(friendly, opponent, ball, teamcolor, is_first_half,secondaryState)

    def update_next_strategy(self, friendly, opponent, ball):
        raise NotImplementedError

    def check_ball_avaliable(self, ball):
        if ball.get_position() is None:
            print("No ball position available")
            return False
        else:
            return True

class StationaryStrategy(Strategy):
    def update_next_strategy(self, friendly, opponent, ball):
        return

HAVENT_SEEN_THE_BALL_TIMEOUT = 30
class DummyStrategy(Strategy):

    def __init__(self):
        self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
        super(DummyStrategy, self).__init__()

    def who_has_the_ball(self, robots, ball):
        closest_dist = math.inf
        current_closest = None
        for robot in robots:
            if robot.status != Robot.Status.READY:
                continue

            if robot.relocalization_timeout > 0:
                robot.relocalization_timeout = robot.relocalization_timeout - 1
                continue

            dist = np.linalg.norm(ball.get_position()[0:2] - robot.get_position()[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest

    def wrapToPi(self, num: float) -> float:
        rem = (num + np.pi) % (2 * np.pi) - np.pi
        return rem

    def update_next_strategy(self, friendly, opponent, ball, teamcolor, is_first_half, secondaryState):

        if self.check_ball_avaliable(ball):

            if secondaryState == GameState.STATE_PENALTYSHOOT:
                goal_position = np.array([0, -4.5])
            else:
                if teamcolor == 0:
                    if is_first_half == 1:
                        goal_position = np.array([0, 5])
                    else:
                        goal_position = np.array([0, -5])
                else:
                    if is_first_half == 1:
                        goal_position = np.array([0, -5])
                    else:
                        goal_position = np.array([0, 5])

            if abs(ball.get_position()[0]) < 1.0:
                goal_position[0] = ball.get_position()[0]

            if abs(ball.get_position()[0]) < 3.5 and abs(ball.get_position()[1]) < 5:
                self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
                current_closest = self.who_has_the_ball(friendly, ball) # Guess who has the ball
                if current_closest is not None and current_closest.send_nav:

                    distance_of_player_goal_to_ball = 0.1
                    ball_position = ball.get_position()
                    player_position = current_closest.get_position()[0:2]
                    player_angle = current_closest.get_position()[2]

                    diff = ball_position - goal_position
                    diff_unit = diff / np.linalg.norm(diff)
                    diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

                    destination_position = ball_position + diff_unit * distance_of_player_goal_to_ball
                    distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)
                    if distance_of_player_to_ball < 0.4:
                        navigation_bias = 0.6
                    else:
                        navigation_bias = 1.1
                    diff = destination_position - player_position
                    destination_position_biased = player_position + diff * navigation_bias

                    destination_position_biased = [destination_position_biased[0], destination_position_biased[1], diff_angle]


                    print("Position of closest player")
                    print(player_position)
                    print("Ball Position")
                    print(ball_position)
                    print("Destination Position")
                    print(destination_position)
                    print("Distance between player and ball")
                    print(distance_of_player_to_ball)

                    player_vector = [math.cos(player_angle), math.sin(player_angle)]
                    player_to_ball_vector = ball_position - player_position
                    cross = np.cross(player_to_ball_vector, player_vector)

                    print("Robot ball angle")
                    print(distance_of_player_to_ball)
                    if distance_of_player_to_ball < 0.19 and abs(cross) > 0.15:
                        print("robot ball ange too large, unable to kick")

                    if distance_of_player_to_ball < 0.19 and abs(cross) < 0.15:
                        if cross > 0.03:
                            # right foot
                            current_closest.kick_with_right_foot = True
                        else:
                            current_closest.kick_with_right_foot = False

                        delta = goal_position - ball.get_position()
                        unit = delta / np.linalg.norm(delta)

                        current_closest.status = Robot.Status.KICKING
                        current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)
                    else:
                        current_closest.set_navigation_position(destination_position_biased)
                        pass

        # If player is not facing the right direction, and not seeing the ball, then face the goal
        self.havent_seen_the_ball_timeout = self.havent_seen_the_ball_timeout - 1
        # print(self.havent_seen_the_ball_timeout)
        for player in friendly:
            if player.status != Robot.Status.READY:
                continue

            player_angle = player.get_position()[2]
            player_position = player.get_position()[0:2]
            #
            # player_to_goal = goal_position - player_position
            # player_to_goal_angle = math.atan2(player_to_goal[1], player_to_goal[0])
            #
            # # Not facing the goal
            # if abs(self.wrapToPi(player_angle - player_to_goal_angle)) > math.pi / 2:
            #     facing_goalposition = [player_position[0], player_position[1], player_to_goal_angle]
            #     player.set_navigation_position(facing_goalposition)
            #     pass

            # Haven't seen the ball timeout
            if self.havent_seen_the_ball_timeout == 0:
                rospy.loginfo("Havent seen the ball for a while. Rototating robot " + player.robot_name
                              )
                self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
                turn_position = [player_position[0], player_position[1], player_angle + math.pi]
                player.set_navigation_position(turn_position)



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
            if teamcolor == 0: #blue
                if is_first_half == 1:
                    own_goal = np.array([0, -4.5])
                    angle = 0
                else:
                    own_goal = np.array([0, 4.5])
                    angle = 3.14
            elif teamcolor == 1: #red
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
                    if teamcolor == 0:
                        nav_pose = np.array([-1, -1, 0])
                    else:
                        nav_pose = np.array([-1, 1, 3.14])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.RIGHT_MIDFIELD:
                    if teamcolor == 0:
                        nav_pose = np.array([1, -1, 0])
                    else:
                        nav_pose = np.array([1, 1, 3.14])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.GOALIE:
                    if teamcolor == 0:
                        nav_pose = np.array([0, -4.5, 0])
                    else:
                        nav_pose = np.array([0, 4.5, 3.14])
                    robot.set_navigation_position(nav_pose)
            # second half
            else:
                if robot.role == Robot.Role.LEFT_MIDFIELD:
                    if teamcolor == 0:
                        nav_pose = np.array([-1, 1, 3.14])
                    else:
                        nav_pose = np.array([-1, -1, 0])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.RIGHT_MIDFIELD:
                    if teamcolor == 0:
                        nav_pose = np.array([1, 1, 3.14])
                    else:
                        nav_pose = np.array([1, -1, 0])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.GOALIE:
                    if teamcolor == 0:
                        nav_pose = np.array([0, 4.5, 3.14])
                    else:
                        nav_pose = np.array([0, -4.5, 0])
                    robot.set_navigation_position(nav_pose)
