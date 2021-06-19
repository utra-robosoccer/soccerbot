from abc import ABC
from enum import IntEnum
import itertools
import math


import numpy as np
from shapely.geometry import LineString, Point, Polygon

from ball import Ball
from robot import Robot

np.random.seed(2)


class Strategy:
    def __init__(self):
        pass

    def reset(self):
        pass

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
            dist = np.linalg.norm(ball.get_position()[0:2] - robot.get_position()[0:2])
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
            opponent_goal = current_closest.get_opponent_net_position()
            delta = opponent_goal - ball.get_position()
            unit = delta / np.linalg.norm(delta)

            current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)
            current_closest.status = Robot.Status.KICKING
        else:
            # if current_closest.status != Robot.Status.READY:
            #     return
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


###############################################################################
class GameState(IntEnum):
    INIT = 1
    OPPONENT_POSSESSION = 2
    FRIENDLY_POSSESSION = 3


class Field:
    X_COORD = 0
    Y_COORD = 1
    MIDFIELD = np.array([0, 0])
    WIDTH = 6
    LEFT_EDGE = -3
    RIGHT_EDGE = 3
    LENGTH = 9
    TOP_EDGE = 4.5
    BOTTOM_EDGE = -4.5
    GOAL_WIDTH = 2.6
    GOAL_AREA_WIDTH = 3
    GOAL_AREA_LENGTH = 1
    PENALTY_AREA_LENGTH = 2
    NET = {}
    NET_AREA = {}
    NET_LINE = {}

    @staticmethod
    def init():
        hw = Field.GOAL_WIDTH / 2
        # Friendly net
        bottom = Field.BOTTOM_EDGE
        top = Field.BOTTOM_EDGE + Field.GOAL_AREA_LENGTH
        goal_coords = [(-hw, bottom), (hw, bottom), (-hw, top), (hw, top)]
        Field.NET[Robot.Team.FRIENDLY] = Point(0, Field.BOTTOM_EDGE)
        Field.NET_AREA[Robot.Team.FRIENDLY] = Polygon(goal_coords)
        Field.NET_LINE[Robot.Team.FRIENDLY] = LineString([(-hw, bottom), (hw, bottom)])
        # Opponent net
        bottom = Field.TOP_EDGE - Field.GOAL_AREA_LENGTH
        top = Field.TOP_EDGE
        goal_coords = [(-hw, bottom), (hw, bottom), (-hw, top), (hw, top)]
        Field.NET[Robot.Team.OPPONENT] = Point(0, Field.TOP_EDGE)
        Field.NET_AREA[Robot.Team.OPPONENT] = Polygon(goal_coords)
        Field.NET_LINE[Robot.Team.OPPONENT] = LineString([(-hw, top), (hw, top)])
Field.init()


class PhysConsts:
    DELTA_T = -1

    @staticmethod
    def init(d_t):
        PhysConsts.DELTA_T = d_t

    @staticmethod
    def get_friction_coeff():
        coeff = PhysConsts.DELTA_T / (1 - Ball.FRICTION_COEFF)
        return coeff


class Thresholds:
    POSSESSION = 0.2  # How close player has to be to ball to have possession
    GOALIE_ANGLE = 5 # How close goalie has to be to defense line
    PASS = -1 # Maximum distance between players trying to pass to each other
    OBSTACLE = -1 # Size of player obstacles

    @staticmethod
    def init():
        Thresholds.OBSTACLE = 2 * Thresholds.POSSESSION # TODO: add robot_width / 2

    @staticmethod
    def passing(robot):
        val = robot.max_kick_speed * PhysConsts.get_friction_coeff()
        return val


def unit_vec(v):
    return v / np.linalg.norm(v)


def distance_between(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)


def closest_point_on_line(pos, pt1, pt2):
    """
    Orthogonal projection of position pos onto the line from pt1 to pt2, as
    well as the distance to this point

    Source:
    - https://blender.stackexchange.com/questions/94464/finding-the-closest-point-on-a-line-defined-by-two-points
    """
    line_diff = pt2 - pt1
    # Compute distance
    proj = np.dot(pos, line_diff) / np.dot(line_diff, line_diff)
    dist = np.linalg.norm(line_diff * proj - pos)
    # Compute crit pos
    n = line_diff / np.linalg.norm(line_diff)
    ap = pos - pt1
    t = ap.dot(n)
    crit_pos = pt1 + t * n
    return crit_pos, dist


def tangent_lines_to_circle_through_pt(c, r, pt):
    """
    Given a circle centered at c with radius r, this function computes the
    points on the circle which lie on the tangent lines passing through the
    point pt

    First, we translate all points to the origin so that we only have to solve
    this one instance of the problem. The idea is to then write the equation of
    the circle, and substitute in y = k*x for some k. We then find the k-values
    that result in a repeated root for the quadratic circle equation. We
    substitute each of these k-values in to get the two different x coordinates
    then finally substitute the x,k pairs into y = k*x to get both points of
    interest.

    Returns left point, right point

    Source:
    - https://math.stackexchange.com/questions/543496/how-to-find-the-equation-of-a-line-tangent-to-a-circle-that-passes-through-a-g
    """
    x_c, y_c = c - pt # Translate to origin and read out coordinates

    # Find values of k that result in a tangent line
    a_k = (r**2 - x_c**2)
    b_k = 2 * x_c * y_c
    c_k = (r**2 - y_c**2)
    disc = b_k**2 - 4 * a_k * c_k
    k1 = (-b_k - np.sqrt(disc)) / (2 * a_k)
    k2 = (-b_k + np.sqrt(disc)) / (2 * a_k)

    # Compute x values corresponding to each k
    a_x1 = 1 + k1**2
    b_x1 = 2 * (x_c + k1 * y_c)
    a_x2 = 1 + k2**2
    b_x2 = 2 * (x_c + k2 * y_c)
    c_x = x_c**2 + y_c**2 - r**2
    disc_x1 = b_x1**2 - 4 * a_x1 * c_x
    disc_x2 = b_x2**2 - 4 * a_x2 * c_x
    x1 = (-b_x1 - np.sqrt(max(disc_x1, 0))) / (2 * a_x1)
    x2 = (-b_x2 - np.sqrt(max(disc_x2, 0))) / (2 * a_x2)

    # Compute y values corresponding to each x value
    y1 = k1 * x1
    y2 = k2 * x2

    # Compute final points + translate back to original coordinates
    pt1 = np.array([x1, y1]) + pt
    pt2 = np.array([x2, y2]) + pt

    # Determine which point is "left" and which is "right". The "left" point
    # always has the greater slope
    if k1 > k2:
        ptl = pt1
        ptr = pt2
    else:
        ptl = pt2
        ptr = pt1
    return ptl, ptr


def func_att(alpha, x2, x1):
    return 0.5 * alpha * distance_between(x2, x1)


def grad_att(alpha, x2, x1):
    """Attractive gradient"""
    return alpha * (x2 - x1)


def func_rep(beta, r_rep, d_rep):
    val = 0
    if d_rep <= r_rep:
        t1 =  (1 / d_rep) - (1 / r_rep)
        val = 0.5 * beta * t1**2
    return val


def grad_rep(beta, r_rep, d_rep, x2, x1):
    """Repulsive gradient"""
    val = 0
    if d_rep <= r_rep:
        t1 = (1 / r_rep) - (1 / d_rep)
        t2 = (x2 - x1) / (2 * distance_between(x2, x1)**3)
        val = beta * t1 * t2
    return val


class PlayerStrategy(ABC, Strategy):

    def __init__(self, player, ball, alpha=0.5, beta=0.5, eps=0.5):
        super().__init__()
        self._player = player
        self._player_pos = player.get_position()[0:2]
        self._team = self._player.team
        self._ball = ball
        self._ball_pos = ball.get_position()
        self._alpha = alpha
        self._beta = beta
        self._eps = eps
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
                continue # A robot is not its own obstacle
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
        # Path planning with obstacle avoidance via potential functions
        # Source:
        # - http://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf
        thresh = Thresholds.passing(self._player) # Seems reasonable for avoidance
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
            angle_rand = np.random.uniform(low=-np.pi/12, high=np.pi/12)
            rotation_rand = np.array([[np.cos(angle_rand), -np.sin(angle_rand)],
                                      [np.sin(angle_rand), np.cos(angle_rand)]])
            grad_perturbed = rotation_rand @ grad
            # Gradient descent update
            step = 0.3  # TODO: line search for step size
            goal_pos = self._player_pos - step * grad_perturbed
        # Update robot state
        self._player.set_navigation_position(np.append(goal_pos, 0))
        self._player.status = Robot.Status.WALKING

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

    # def _dribble_ball(self, pos):
    #     """
    #     Dribble ball towards position (lower-speed kick)
    #     """
    #     self._kick_ball(pos, kick_speed_perc=70)

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
        self._pass_ball(closest_teammate)


class GoalieStrategy(PlayerStrategy):
    """
    Objective: defend the net against the opponent team
    """

    def __init__(self, player, ball, alpha=0.5, beta=0.5, eps=0.5):
        super().__init__(player, ball, alpha, beta, eps)

    def _shot_on_net(self):
        if not self._ball.is_moving():
            return False
        # If ball is moving, then check whether it's moving towards net. To do
        # this, we create a line between the ball's current position and its
        # estimated final position. We then query whether this line intersects
        # with the net we are trying to defend
        ball_dest = self._est_ball_dest()
        ball_path = LineString([self._ball_pos, ball_dest])
        return ball_path.intersects(self._net_line)

    def _defend_shot(self):
        """
        Moves the player near the point where the ball's velocity vector
        intersects the goal line
        """
        vel = self._ball.get_velocity()
        ball_dest = self._ball_pos + 2 * Field.LENGTH * unit_vec(vel)  # Far away point along ball's direction
        ball_path = LineString([self._ball_pos, ball_dest])
        pt = ball_path.intersection(self._net_line)
        pos = np.array(pt)
        self._move_player_to(pos)

    def _defend_net(self):
        # Compute how close we are to good defense position
        net_pos = np.array(self._net)
        goalie_to_ball_vec = net_pos - self._ball_pos
        player_to_ball_vec = self._player_pos - self._ball_pos
        num = np.dot(goalie_to_ball_vec, player_to_ball_vec)
        denom = np.linalg.norm(goalie_to_ball_vec) * np.linalg.norm(player_to_ball_vec)
        angle_deg = np.arccos(num / denom) * 180 / np.pi
        if np.abs(angle_deg) < Thresholds.GOALIE_ANGLE:
            # We are now in a good angular position to defend the net. We now
            # need to choose a good distance to position ourselves at
            crit_pos = net_pos
            ty = Field.PENALTY_AREA_LENGTH / (Field.LENGTH / 2)
            if np.sign(self._ball_pos[Field.Y_COORD]) == self._Y_SIGN:
                ty /= 2 # Ball is on our half of field
            crit_pos[Field.Y_COORD] = (1 - ty) * crit_pos[Field.Y_COORD]
        else:
            # If we're not in a good angular position to defend, then we try to
            # accomplish that first
            crit_pos, dist = closest_point_on_line(
                self._player_pos, self._ball_pos, net_pos
            )
        self._move_player_to(crit_pos)

    def update_next_strategy(self, friendlies, opponents):
        self._all_robots = list(itertools.chain(friendlies, opponents))
        if self.has_possession():
            # Return ball to offensive players
            self._pass_to_offense(friendlies)
        elif self._shot_on_net():
            # Intercept shot on net
            self._defend_shot()
        elif self._is_closest_to_ball_dest(friendlies, opponents):
            # Intercept nearby ball
            # Assumes ALL robots have same speed
            self._pursue_ball()
        else:
            # Defend along ball-goal line
            self._defend_net()


class ScoreStrategy(PlayerStrategy):
    """
    Objective: score on the other team
    """

    def __init__(self, player, ball, alpha=0.5, beta=0.5, eps=0.5):
        super().__init__(player, ball, alpha, beta, eps)

    def update_next_strategy(self, friendlies, opponents):
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


class OpenStrategy(PlayerStrategy):
    """
    Objective: find and move to an open position (to receive passes)
    """

    def __init__(self, player, ball, alpha=0.5, beta=0.5, eps=0.5):
        super().__init__(player, ball, alpha, beta, eps)

    def _get_feasible_region_angles(self, obstacles):
        """
        Returns a list of angles defining feasible regions for the purposes of
        passing the ball. Each element of the list is a pair
        (theta_left, theta_right).

        Assumes that no obstacles overlap with the ball
        """
        # First, find the tangent points
        r_obs = Thresholds.OBSTACLE
        regs = []
        for obs in obstacles:
            ptl, ptr = tangent_lines_to_circle_through_pt(obs, r_obs, self._ball_pos)
            theta_l = np.arctan2(ptl[Field.Y_COORD], ptl[Field.X_COORD])
            theta_r = np.arctan2(ptr[Field.Y_COORD], ptr[Field.X_COORD])
            angles = (theta_l, theta_r)
            for angle in angles:
                if angle < 0:
                    angle += 2 * np.pi
            regs.append(angles)
        # Now find the feasible regions
        regs.sort(key = lambda x: x[0])  # Sort by ascending theta_l
        feas_regs = []
        if len(regs) == 1:
            theta_l, theta_r = regs[0]
            feas_regs.append((theta_r, theta_l))
        else:
            i = 0
            while i < len(regs) - 1:
                theta_li, theta_ri = regs[i]
                theta_lip1, theta_rip1 = regs[i + 1]
                if theta_rip1 > theta_li:
                    # Only feasible if non-overlapping
                    feas_regs.append((theta_rip1, theta_li))
                i += 1
            # Handle edge case for first & last regions
            theta_l1, theta_r1 = regs[0]
            theta_lend, theta_rend = regs[-1]
            if theta_lend < theta_r1:
                feas_regs.append((theta_r1, theta_lend))
        return feas_regs

    def _compute_goal_pos(self, obstacles):
        """
        Computes the "open" position to move towards. Should be within line of
        sight of the ball, and away from opponents
        """
        # 1. Check feasibility. Need the distance threshold to be slightly
        # larger than the actual threshold, because later on the calculations
        # will get messed up if the obstacles are too close to the player
        is_infeasible = False
        thresh = 1.05 * Thresholds.OBSTACLE
        for obs in obstacles:
            dist = distance_between(self._ball_pos, obs)
            if dist <= thresh:
                # Ball is overlapping with an obstacle
                is_infeasible = True
                break
        # 2. If infeasible or no obstacles, set goal position to point on pass
        # circle closest to the goal line. Otherwise, we have to account for
        # obstacles
        opp_net_line = np.array(self._opponent_net_line)
        net_pos, dist = closest_point_on_line(
            self._player_pos, opp_net_line[Field.X_COORD],
            opp_net_line[Field.Y_COORD]
        )
        gb_unit_vec = unit_vec(net_pos - self._ball_pos)
        thresh = Thresholds.passing(self._player) # TODO: should be the player with possession
        if is_infeasible or len(obstacles) == 0:
            goal_pos = thresh * gb_unit_vec + self._ball_pos
        else:
            # 3. Compute feasible regions, each defined by a left and right
            # point
            feas_regs = self._get_feasible_region_angles(obstacles)

            # 4. If no feasible regions, ensure the player is nearby even if
            # there's no way to pass to them right now. Otherwise, need to
            # search for best region
            if len(feas_regs) == 0:
                delta = self._ball_pos - self._player_pos
                goal_pos = thresh * unit_vec(delta)
                goal_pos += self._player_pos
            else:
                # 5. For each feasible region, compute the minimum distance
                # between the net position and region, as well as the point
                # that minimizes this distance
                dists = []
                gb_angle = np.arctan2(
                    gb_unit_vec[Field.Y_COORD], gb_unit_vec[Field.X_COORD]
                )
                if gb_angle < 0:
                    gb_angle += 2 * np.pi
                for theta_l, theta_r in feas_regs:
                    gb_angle_tmp = gb_angle
                    if theta_l < theta_r:
                        # Rotate coords if needed
                        theta_l -= theta_r
                        theta_r -= theta_r
                        gb_angle_tmp -= theta_r
                    if theta_l >= gb_angle_tmp >= theta_r:
                        pt = thresh * gb_unit_vec + self._ball_pos
                        dmin = distance_between(net_pos, pt)
                    elif gb_angle>= theta_l:
                        pt = thresh * np.array([np.cos(theta_l), np.sin(theta_l)]) + self._ball_pos
                        dmin = distance_between(net_pos, pt)
                    else:
                        pt = thresh * np.array([np.cos(theta_r), np.sin(theta_r)]) + self._ball_pos
                        dmin = distance_between(net_pos, pt)
                    dists.append((dmin, pt))

                # 6. From the previously-computed set of points, choose the one
                # with minimum distance to the net
                dists.sort(key = lambda x: x[0])
                dmin, pt = dists[0]
                goal_pos = pt
        return goal_pos

    def update_next_strategy(self, friendlies, opponents):
        self._all_robots = list(itertools.chain(friendlies, opponents))
        # Compute goal position by avoiding obstacles near the ball, trying
        # to remain within line of sight of the ball, and remaining away from
        # the ball but within passing distance
        thresh = Thresholds.passing(self._player)
        obstacles = self._compute_obstacles(self._ball_pos, thresh)
        x_g = self._compute_goal_pos(obstacles)
        self._move_player_to(x_g)


class TeamStrategy(Strategy):

    def __init__(self, delta_t):
        super().__init__()
        self._dt = delta_t
        self.reset()
        PhysConsts.init(self._dt)
        Thresholds.init()
        self._cnt = 0

    def reset(self):
        pass

    def update_next_strategy(self, friendlies, opponents, ball):
        # Update substrategies
        # TODO: maybe TeamStrategy could decide which strategy each player
        # should use depending on game conditions. Could maybe also modify
        # strategy parameters...hmm...
        self._cnt += 1
        strats = []
        ball_pos = ball.get_position()
        dist_to_ball = []
        for robot in friendlies:
            if robot.role == Robot.Role.GOALIE:
                strats.append(GoalieStrategy(robot, ball))
            else:
                robot_pos = robot.get_position()[0:2]
                assert not np.isnan(robot_pos).any(), f'nan position on iter {self._cnt}'
                dist = distance_between(robot_pos, ball_pos)
                dist_to_ball.append((dist, robot))
        # The two players closest to the ball will use ScoreStrategy, and the
        # one furthest away will use OpenStrategy
        dist_to_ball.sort(key = lambda x: x[0])
        cnt = 0
        for dist, robot in dist_to_ball:
            strat = ScoreStrategy(robot, ball)
            if cnt >= 2:
                strat = OpenStrategy(robot, ball)
            strats.append(strat)
            cnt += 1
        # Update!
        for strat in strats:
            strat.update_next_strategy(friendlies, opponents)
