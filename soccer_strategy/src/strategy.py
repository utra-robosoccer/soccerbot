from abc import ABC
from enum import IntEnum
import itertools
import math
import numpy as np
from operator import itemgetter
from shapely.geometry import LineString, Point, Polygon

from robot import Robot


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


# TODO: sync this across Ball and GameEngine
class PhysConsts:
    DELTA_T = 0.1
    BALL_FRIC_COEFF = 0.8


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
    def init_nets():
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


Field.init_nets()


class Thresholds:
    POSSESSION = 0.2  # How close player has to be to ball to have possession
    GOALIE_ANGLE = 5 # How close goalie has to be to defense line


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


class PlayerStrategy(Strategy):

    def __init__(self, player, ball):
        super().__init__()
        self._player = player
        self._player_pos = player.get_position()[0:2]
        self._team = self._player.team
        self._ball = ball
        self._ball_pos = ball.get_position()
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

    def _distance_to(self, pos):
        return np.linalg.norm(self._player_pos - pos)

    def _has_possession(self):
        return self._distance_to(self._ball_pos) <= Thresholds.POSSESSION

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
        self._player.set_navigation_position(np.append(pos, 0))
        self._player.status = Robot.Status.WALKING

    def _pursue_ball(self):
        self._move_player_to(self._ball_pos)

    def _est_ball_dest(self):
        """
        Uses equations of motion to guess, based on the ball's current position
        and velocity, what position it will rest at
        """
        coeff = PhysConsts.DELTA_T / (1 - PhysConsts.BALL_FRIC_COEFF)
        vel = self._ball.get_velocity()
        ball_dest = self._ball_pos + vel * coeff
        return ball_dest

    def _kick_ball(self, pos, kick_speed_perc=100.0):
        """Kick ball towards position (full-speed kick)"""
        dir = unit_vec(pos - self._ball_pos)
        mag = self._player.max_kick_speed * kick_speed_perc / 100.0
        self._player.set_kick_velocity(dir * mag)
        self._player.status = Robot.Status.KICKING

    def _pass_ball(self, teammate):
        """Pass ball to teammate"""
        teammate_pos = teammate.get_position()[0:2]
        coeff = PhysConsts.DELTA_T / (1 - PhysConsts.BALL_FRIC_COEFF)
        pass_dist = self._distance_to(teammate_pos)
        magnitude = 100 * min(pass_dist / coeff, 1)
        self._kick_ball(teammate_pos, kick_speed_perc=magnitude)

    # def _dribble_ball(self, pos):
    #     """
    #     Dribble ball towards position (lower-speed kick)
    #     """
    #     self._kick_ball(pos, kick_speed_perc=70)
    #
    # def _is_open(self, opponents):
    #     """Return True if the player is open, otherwise False"""
    #     player_is_open = True
    #     min_dist = float('inf')
    #     for robot in opponents:
    #         dist = self._distance_to(robot.get_position()[0:2])
    #         if dist < min_dist:
    #             min_dist = dist
    #             if min_dist < Thresholds.OPEN_THRESH:
    #                 player_is_open = False
    #                 break
    #     return player_is_open

    def _pass_to_offense(self, friendlies):
        # TODO: implement this in an intelligent manner
        closest_teammate, dist = self._get_closest_teammate(friendlies)
        self._pass_ball(closest_teammate)


class GoalieStrategy(PlayerStrategy):

    def __init__(self, player, ball):
        super().__init__(player, ball)

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
        if self._has_possession():
            # Return ball to offensive players
            self._pass_to_offense(friendlies)
        elif self._shot_on_net():
            # Intercept shot on net
            self._defend_shot()
        elif self._is_closest_to_ball_dest(friendlies, opponents):
            # Intercept nearby ball
            self._pursue_ball()
        else:
            # Defend along ball-goal line
            self._defend_net()


class ScoreStrategy(PlayerStrategy):

    def __init__(self, player, ball):
        super().__init__(player, ball)

    def update_next_strategy(self, friendlies, opponents):
        if self._has_possession():
            # Try kicking to nearest point on goal line
            opp_net_line = np.array(self._opponent_net_line)
            goal_pos, dist = closest_point_on_line(
                self._player_pos, opp_net_line[Field.X_COORD],
                opp_net_line[Field.Y_COORD]
            )
            self._kick_ball(goal_pos)
        elif self._is_closest_to_ball_dest(friendlies, []):
            ball_dest = self._est_ball_dest()
            self._move_player_to(ball_dest)


class TeamStrategy(Strategy):

    def __init__(self):
        super().__init__()
        self.reset()

    def reset(self):
        pass

    def update_next_strategy(self, friendlies, opponents, ball):
        # Update substrategies
        # TODO: maybe TeamStrategy could decide which strategy each player
        # should use depending on game conditions. Could maybe also modify
        # strategy parameters...hmm...
        strats = []
        for robot in friendlies:
            if robot.role == Robot.Role.GOALIE:
                strats.append(GoalieStrategy(robot, ball))
            else:
                strats.append(ScoreStrategy(robot, ball))
        for strat in strats:
            strat.update_next_strategy(friendlies, opponents)
