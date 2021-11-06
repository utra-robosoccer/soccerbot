from abc import ABC
from enum import IntEnum
import itertools
import math
from strategy.strategy import Strategy
from strategy.utils import *

import numpy as np
from shapely.geometry import LineString, Point, Polygon

from ball import Ball
from robot import Robot

np.random.seed(2)

# how much the robot move per gradient descent update
GRADIENT_UPDATE_INTERVAL_LENGTH = 0.5
Field.init()

class Thresholds:
    POSSESSION = 0.2  # How close player has to be to ball to have possession
    GOALIE_ANGLE = 5  # How close goalie has to be to defense line
    PASS = -1  # Maximum distance between players trying to pass to each other
    OBSTACLE = 1  # Size of player obstacles
    PASSING = 2  # distance for obstacle detection when moving to a position

class PhysConsts:
    DELTA_T = -1

    @staticmethod
    def init(d_t):
        PhysConsts.DELTA_T = d_t

    @staticmethod
    def get_friction_coeff():
        coeff = PhysConsts.DELTA_T / (1 - Ball.FRICTION_COEFF)
        return coeff


class PlayerStrategy(ABC, Strategy):

    def __init__(self, player, ball, alpha=0.5, beta=0.5, eps=0.5): # 0.5, 0.5 0.5
        super().__init__()
        self._player = player
        self._player_pos = player.get_position()[0:2]
        self._team = self._player.team
        self._ball = ball
        self._ball_pos = ball.get_position()
        self._alpha = alpha  # multiplier for attractive field
        self._beta = beta  # multiplier for repulsive field
        self._eps = eps  # threshold distance where robot go directly to the ball
        self._net = Field.NET[self._team]
        if self._team == Robot.Team.FRIENDLY:
            self._opponent_net = Field.NET[Robot.Team.OPPONENT]
            self._opponent_net_line = Field.NET_LINE[Robot.Team.OPPONENT]
        else:
            self._opponent_net = Field.NET[Robot.Team.FRIENDLY]
            self._opponent_net_line = Field.NET_LINE[Robot.Team.FRIENDLY]
        self._net_area = Field.NET_AREA[self._team]
        self._net_line = Field.NET_LINE[self._team]
        self._Y_SIGN = -1 if self._net.y < 0 else 1
        self._all_robots = None
        self.target_pos = None

    def _distance_to(self, pos):
        return np.linalg.norm(self._player_pos - pos)

    def has_possession(self):
        return self._distance_to(self._ball_pos) <= Thresholds.POSSESSION

    def _compute_obstacles(self, ref_pos, thresh):
        """
        Computes the locations of obstacles (both friendly and opponent robots)
        within distance thresh of position ref_pos
        """
        obstacles = []
        for robot in self._all_robots:
            if robot == self._player:
                continue  # A robot is not its own obstacle
            pos = robot.get_position()[0:2]
            dist = distance_between(ref_pos, pos)
            # TODO: what about obstacles outside this radius? Do they matter?
            if dist <= thresh:
                obstacles.append(pos)
        return obstacles

    def _is_closest_to_pos(self, pos, friendlies, opponents):
        """
        Gets the player who is closest to the specified position
        """
        min_dist = float('inf')
        closest_to_ball = None
        for robot in itertools.chain(friendlies, opponents):
            robot_pos = robot.get_position()[0:2]
            dist = distance_between(pos, robot_pos)
            if dist < min_dist:
                min_dist = dist
                closest_to_ball = robot
        return self._player == closest_to_ball

    def _is_closest_to_ball_dest(self, friendlies, opponents):
        ball_dest = self._est_ball_dest()
        return self._is_closest_to_pos(ball_dest, friendlies, opponents)

    def _is_closest_to_ball(self, friendlies, opponents):
        return self._is_closest_to_pos(self._ball_pos, friendlies, opponents)

    def _get_closest_teammate(self, friendlies):
        """
        Return the teammate closest to the given one, and the distance between
        them
        """
        closest_teammate = None
        min_dist = float('inf')
        for robot in friendlies:
            if robot.role != self._player.role:
                dist = self._distance_to(robot.get_position()[0:2])
                if dist < min_dist:
                    min_dist = dist
                    closest_teammate = robot
        return closest_teammate, min_dist

    def _move_player_to(self, pos):
        assert self._all_robots is not None, 'all_robots not set!'
        self.target_pos = pos
        # Path planning with obstacle avoidance via potential functions
        # Source:
        # - http://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf
        thresh = Thresholds.PASSING  # Seems reasonable for avoidance
        obstacles = self._compute_obstacles(self._player_pos, thresh)
        goal_pos = pos
        if self._distance_to(goal_pos) > self._eps:
            grad = grad_att(self._alpha, self._player_pos, pos)
            if len(obstacles) > 0:
                r_rep = 2 * Thresholds.POSSESSION
                d_rep = float('inf')
                obs_rep = None
                for obs in obstacles:
                    dist = self._distance_to(obs)
                    if dist < d_rep:
                        d_rep = dist
                        obs_rep = obs
                        # grad -= grad_rep(self._beta, r_rep, dist, obs, self._player_pos)
                grad -= grad_rep(self._beta, r_rep, d_rep, obs_rep, self._player_pos)
            # Perturb out of local minima
            angle_rand = np.random.uniform(low=-np.pi / 12, high=np.pi / 12)
            rotation_rand = np.array([[np.cos(angle_rand), -np.sin(angle_rand)],
                                      [np.sin(angle_rand), np.cos(angle_rand)]])
            grad_perturbed = rotation_rand @ grad
            # Gradient descent update
            goal_pos = self._player_pos - GRADIENT_UPDATE_INTERVAL_LENGTH * grad_perturbed

        # Update robot state
        diff = self._player_pos - goal_pos
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        # todo: redo pathing
        self._player.set_navigation_position(np.append(goal_pos, diff_angle))
        self._player.status = Robot.Status.WALKING
        print(str(goal_pos) + "  " + str(diff_angle))

    # move player with 5 seperate descents, does not work well
    """def _move_player_to(self, pos):
        assert self._all_robots is not None, 'all_robots not set!'
        self.target_pos = pos
        # Path planning with obstacle avoidance via potential functions
        # Source:
        # - http://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf
        thresh = Thresholds.PASSING  # Seems reasonable for avoidance
        obstacles = self._compute_obstacles(self._player_pos, thresh)
        goal_pos = pos
        initial_pos = self._player_pos
        for p in range(1, 5):
            # if too far to go directly to goal_pos
            if self._distance_to(goal_pos) > self._eps:
                grad = grad_att(self._alpha, initial_pos, pos)
                if len(obstacles) > 0:
                    r_rep = 2 * Thresholds.POSSESSION
                    d_rep = float('inf')
                    obs_rep = None
                    for obs in obstacles:
                        dist = self._distance_to(obs)
                        if dist < d_rep:
                            d_rep = dist
                            obs_rep = obs
                            # grad -= grad_rep(self._beta, r_rep, dist, obs, self._player_pos)
                    grad -= grad_rep(self._beta, r_rep, d_rep, obs_rep, self._player_pos)
                # Perturb out of local minima
                angle_rand = np.random.uniform(low=-np.pi / 12, high=np.pi / 12)
                rotation_rand = np.array([[np.cos(angle_rand), -np.sin(angle_rand)],
                                          [np.sin(angle_rand), np.cos(angle_rand)]])
                grad_perturbed = rotation_rand @ grad
                # Gradient descent update
                goal_pos = self._player_pos - GRADIENT_UPDATE_INTERVAL_LENGTH/10 * grad_perturbed

                # get robot angle at goal position
                diff = self._player_pos - goal_pos
                diff_unit = diff / np.linalg.norm(diff)
                diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])
                # update new initial position for hte descent update
                initial_pos = goal_pos

            # go directly to goal_pos
            else:
                goal_pos = pos
                # get robot angle at goal position
                diff = self._player_pos - goal_pos
                diff_unit = diff / np.linalg.norm(diff)
                diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])
                break

        # todo: redo pathing
        self._player.set_navigation_position(np.append(goal_pos, diff_angle))
        self._player.status = Robot.Status.WALKING
        print(str(goal_pos) + "  " + str(diff_angle))"""

    # get list of potential field vector around the robots
    def get_potential_field_vector(self):
        if self.target_pos is None:
            return None

        thresh = Thresholds.PASSING  # Seems reasonable for avoidance
        obstacles = self._compute_obstacles(self._player_pos, thresh)
        return_list = [[], []]

        for x_sample, y_sample in itertools.product(np.linspace(-1, 1, 15), np.linspace(-1, 1, 15)):
            sample_point = np.array([self._player_pos[0] + x_sample, self._player_pos[1] + y_sample])

            grad = grad_att(self._alpha, sample_point, self.target_pos)
            if len(obstacles) > 0:
                r_rep = 2 * Thresholds.POSSESSION
                d_rep = float('inf')
                obs_rep = None
                for obs in obstacles:
                    dist = np.linalg.norm(sample_point - obs)
                    if dist < d_rep:
                        d_rep = dist
                        obs_rep = obs
                grad -= grad_rep(self._beta, r_rep, d_rep, obs_rep, sample_point)
            # Gradient descent update
            potential_field_vector = - grad

            unit_field_vector = potential_field_vector / np.linalg.norm(potential_field_vector) / 10

            return_list[0].append(sample_point)
            return_list[1].append(unit_field_vector)
        return return_list

    def _pursue_ball(self):
        self._move_player_to(self._ball_pos)

    def _est_ball_dest(self):
        """
        Uses equations of motion to guess, based on the ball's current position
        and velocity, what position it will rest at
        """
        coeff = PhysConsts.get_friction_coeff()
        vel = self._ball.get_velocity()
        ball_dest = self._ball_pos + vel * coeff
        return ball_dest

    def _kick_ball(self, pos, kick_speed_perc=100.0):
        """Kick ball towards position (full-speed kick)"""
        dir = unit_vec(pos - self._ball_pos)
        # TODO: rotate player to within some epsilon of the direction before finally kicking
        mag = self._player.max_kick_speed * kick_speed_perc / 100.0
        self._player.set_kick_velocity(dir * mag)
        self._player.status = Robot.Status.KICKING

    def _pass_ball(self, teammate):
        """Pass ball to teammate"""
        teammate_pos = teammate.get_position()[0:2]
        coeff = PhysConsts.get_friction_coeff()
        pass_dist = self._distance_to(teammate_pos)
        magnitude = 100 * min(pass_dist / coeff, 1)
        self._kick_ball(teammate_pos, kick_speed_perc=magnitude)

    def is_open(self, opponents):
        """Return True if the player is open, otherwise False"""
        player_is_open = True
        min_dist = float('inf')
        for robot in opponents:
            dist = self._distance_to(robot.get_position()[0:2])
            if dist < min_dist:
                min_dist = dist
                if min_dist < Thresholds.POSSESSION:
                    player_is_open = False
                    break
        return player_is_open

    def _pass_to_offense(self, friendlies):
        # TODO: implement this in an intelligent manner
        closest_teammate, dist = self._get_closest_teammate(friendlies)
        if closest_teammate is not None:
            self._pass_ball(closest_teammate)



class ScoreStrategy(PlayerStrategy):
    """
    Objective: score on the other team
    """

    def __init__(self, player, ball, alpha=0.5, beta=0.5, eps=0.5):
        super().__init__(player, ball, alpha, beta, eps)

    def update_next_strategy(self, friendlies, opponents, ball, game_properties, plot_vector=False):
        self._all_robots = list(itertools.chain(friendlies, opponents))
        if self.has_possession():
            # Try kicking to nearest point on goal line
            # TODO: account for kick angle uncertainty; play it safe
            opp_net_line = np.array(self._opponent_net_line)
            goal_pos, dist = closest_point_on_line(
                self._player_pos, opp_net_line[Field.X_COORD],
                opp_net_line[Field.Y_COORD]
            )
            self._kick_ball(goal_pos)
        elif self._is_closest_to_ball_dest(friendlies, []):
            # TODO: ^ not the best way. Should ideally have a function like
            # _can_intercept_ball_quickest(). Figuring out how to compute
            # that needs work though
            ball_dest = self._est_ball_dest()
            self._move_player_to(ball_dest)

        if plot_vector:
            return self.get_potential_field_vector()


class TeamStrategy(Strategy):

    def __init__(self, delta_t):
        super().__init__()
        self._dt = delta_t
        PhysConsts.init(self._dt)

    # change plot_vector value to allow potential vectors to be printed
    def update_next_strategy(self, friendlies, opponents, ball, game_properties, plot_vector=False):
        strats = []
        combined_field_vectors = []

        for robot in friendlies:
            # Goalie
            if robot.role == Robot.Role.GOALIE:
                strats.append(ScoreStrategy(robot, ball))
            elif robot.role == Robot.Role.LEFT_MIDFIELD:
                strats.append(ScoreStrategy(robot, ball))
            elif robot.role == Robot.Role.RIGHT_MIDFIELD:
                strats.append(ScoreStrategy(robot, ball))
            elif robot.role == Robot.Role.STRIKER:
                strats.append(ScoreStrategy(robot, ball))

        # if no plotting needed
        if not plot_vector:
            for strat in strats:
                strat.update_next_strategy(friendlies, opponents, ball, game_properties)
        else:
            for strat in strats:
                field_vectors = strat.update_next_strategy(friendlies, opponents, ball, game_properties, plot_vector)
                # todo muticolor for different robots
                combined_field_vectors.append(field_vectors)
            return {"potential_field_vectors": combined_field_vectors}
