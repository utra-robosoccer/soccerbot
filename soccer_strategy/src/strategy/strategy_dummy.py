import math
#import rospy
#rospy only used here for rospy.loginfo
import time

import rospy

from strategy.strategy import Strategy, get_back_up, update_average_ball_position
from strategy.utils import *
from team import Team
from strategy.interfaces.evaluations import Evaluations

try:
    from soccer_msgs.msg import GameState
except:
    from soccer_msgs.fake_msg import GameState

HAVENT_SEEN_THE_BALL_TIMEOUT = 9
GRADIENT_UPDATE_INTERVAL_LENGTH = 0.5

ALPHA = 0.5
BETA = 0.5
EPS = 0.5


class Thresholds:
    POSSESSION = 0.2  # How close player has to be to ball to have possession
    GOALIE_ANGLE = 5  # How close goalie has to be to defense line
    PASS = -1  # Maximum distance between players trying to pass to each other
    OBSTACLE = 1  # Size of player obstacles
    PASSING = 2  # distance for obstacle detection when moving to a position


class StrategyDummy(Strategy):

    def __init__(self):
        self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
        self.update_frequency = 1
        super(StrategyDummy, self).__init__()

    @get_back_up
    @update_average_ball_position
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        this_robot = self.get_current_robot(friendly_team)

        if friendly_team.average_ball_position.position is not None:
            self.havent_seen_the_ball_timeout = min(HAVENT_SEEN_THE_BALL_TIMEOUT, self.havent_seen_the_ball_timeout + 1)

            # generate goal pose
            goal_position = friendly_team.enemy_goal_position
            ball = friendly_team.average_ball_position

            if abs(ball.position[1]) < 3.5 and abs(ball.position[0]) < 5:

                current_closest = Evaluations.who_has_the_ball(friendly_team.robots, ball)  # Guess who has the ball
                if current_closest is None:
                    pass
                elif current_closest.robot_id == this_robot.robot_id:
                    if this_robot.can_kick(ball, goal_position):
                        delta = goal_position - ball.position[0:2]
                        unit = delta / np.linalg.norm(delta)

                        this_robot.status = Robot.Status.KICKING
                        this_robot.set_kick_velocity(unit * this_robot.max_kick_speed)
                    else:
                        if (time.time() - this_robot.navigation_goal_localized_time) < 2 and this_robot.status != Robot.Status.WALKING:
                            ball_position = ball.position[0:2]
                            player_position = this_robot.position[0:2]
                            diff = ball_position - goal_position
                            diff_unit = diff / np.linalg.norm(diff)
                            diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

                            distance_of_player_goal_to_ball = 0.1
                            destination_position = ball_position + diff_unit * distance_of_player_goal_to_ball

                            navigation_bias = 1
                            diff = destination_position - player_position
                            # nav bias offset nav goal to be behind the ball
                            destination_position_biased = player_position + diff * navigation_bias

                            # nav goal behind the ball
                            destination_position_biased = [destination_position_biased[0],
                                                           destination_position_biased[1],
                                                           diff_angle]
                            np.set_printoptions(precision=3)
                            print(
                                "Player {}: Navigation | Destination position biased {}".format(current_closest.robot_id, destination_position_biased))
                            this_robot.set_navigation_position(destination_position_biased) # TODO dynamic walking
        else:
            # If player is not facing the right direction, and not seeing the ball, then face the goal
            print("Player {}: Cannot find the ball".format(this_robot.robot_id))

            if this_robot.status not in [Robot.Status.WALKING, Robot.Status.KICKING]:
                self.havent_seen_the_ball_timeout = self.havent_seen_the_ball_timeout - 1

            player_angle = this_robot.position[2]
            player_position = this_robot.position[0:2]

            # Haven't seen the ball timeout
            if self.havent_seen_the_ball_timeout < 0 and this_robot.status != Robot.Status.WALKING:
                self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
                print("Player {}: Rotating to locate ball".format(this_robot.robot_id))

                turn_position = [player_position[0], player_position[1], player_angle + math.pi * 0.9]
                this_robot.set_navigation_position(turn_position)


    def move_player_to(self, player, destination_position):
        # Path planning with obstacle avoidance via potential functions
        # Source:
        # - http://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf
        obstacles = np.array(player.get_detected_obstacles())
        player_position = np.array(player.position[0:2])
        goal_pos = np.array(destination_position[0:2])

        if distance_between(player_position, goal_pos) > EPS:
            grad = grad_att(ALPHA, player_position, goal_pos)
            if len(obstacles) > 0:
                r_rep = 2 * Thresholds.POSSESSION
                d_rep = float('inf')
                obs_rep = None
                for obs in obstacles:
                    dist = distance_between(obs[0:2], player_position)
                    if dist < d_rep:
                        d_rep = dist
                        obs_rep = obs
                        # grad -= grad_rep(self._beta, r_rep, dist, obs, self._player_pos)
                grad -= grad_rep(BETA, r_rep, d_rep, obs_rep, player_position)
            # Perturb out of local minima
            angle_rand = np.random.uniform(low=-np.pi / 12, high=np.pi / 12)
            rotation_rand = np.array([[np.cos(angle_rand), -np.sin(angle_rand)],
                                      [np.sin(angle_rand), np.cos(angle_rand)]])
            grad_perturbed = rotation_rand @ grad
            # Gradient descent update
            goal_pos = player_position - GRADIENT_UPDATE_INTERVAL_LENGTH * grad_perturbed

        # Update robot state
        diff = player_position - goal_pos
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        # send player to new position
        player.set_navigation_position(np.append(goal_pos, diff_angle))
        print(str(goal_pos) + "  " + str(diff_angle))
