import math

# import rospy
# rospy only used here for rospy.loginfo
import time

import rospy
from strategy.interfaces.actions import Actions
from strategy.interfaces.evaluations import Evaluations
from strategy.strategy import Strategy, get_back_up, update_average_ball_position
from strategy.utils import *
from team import Team

from soccer_msgs.msg import GameState

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
        self.time_of_end_of_action = rospy.Time.now()
        self.update_frequency = 1
        super(StrategyDummy, self).__init__()

    @get_back_up
    @update_average_ball_position
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().update_next_strategy(friendly_team, opponent_team, game_state)

        this_robot = self.get_current_robot(friendly_team)

        if this_robot.status in [
            Robot.Status.WALKING,
            Robot.Status.KICKING,
            Robot.Status.FALLEN_BACK,
            Robot.Status.FALLEN_FRONT,
            Robot.Status.FALLEN_SIDE,
            Robot.Status.TRAJECTORY_IN_PROGRESS,
        ]:
            self.time_of_end_of_action = rospy.Time.now()

        if friendly_team.average_ball_position.position is not None:

            # generate goal pose
            goal_position = friendly_team.enemy_goal_position
            ball = friendly_team.average_ball_position

            current_closest = Evaluations.who_has_the_ball(friendly_team.robots, ball)  # Guess who has the ball
            if current_closest is None:
                pass
            elif current_closest.robot_id == this_robot.robot_id:
                exit(0)
                if this_robot.can_kick(ball, goal_position):
                    if this_robot.status in [Robot.Status.READY]:

                        delta = goal_position - ball.position[0:2]
                        unit = delta / np.linalg.norm(delta)

                        this_robot.status = Robot.Status.KICKING
                        this_robot.set_kick_velocity(unit * this_robot.max_kick_speed)
                        this_robot.kick()
                else:
                    # Ball localized, move to ball
                    if (time.time() - this_robot.navigation_goal_localized_time) < 2 and this_robot.status != Robot.Status.WALKING:
                        Actions.navigate_to_position_with_offset(this_robot, np.array(ball.position[0:2]), goal_position)

            pass
        elif rospy.Time.now() - this_robot.observed_ball.last_observed_time_stamp > rospy.Duration(
            10
        ) and rospy.Time.now() - self.time_of_end_of_action > rospy.Duration(10):
            if this_robot.status not in [Robot.Status.WALKING, Robot.Status.KICKING]:
                player_angle = this_robot.position[2]
                player_position = this_robot.position[0:2]

                # Haven't seen the ball timeout
                time.sleep(1000)
                print("Player {}: Rotating to locate ball".format(this_robot.robot_id))

                turn_position = [player_position[0], player_position[1], player_angle + math.pi * 0.8]
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
                d_rep = float("inf")
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
            # fmt: off
            rotation_rand = np.array([[np.cos(angle_rand), -np.sin(angle_rand)],
                                      [np.sin(angle_rand), np.cos(angle_rand)]])
            # fmt: on
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
