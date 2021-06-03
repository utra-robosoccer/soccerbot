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
    OPEN = 0.6  # How far the closest opponent must be in order for a player to be considered "open"
    PASS = 1.5  # How close you have to be to teammate before passing
    SCORE = 2.25  # How close you have to be before trying to score
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
        self._net_area = Field.NET_AREA[self._team]
        self._net_line = Field.NET_LINE[self._team]
        self._Y_SIGN = -1 if self._net.y < 0 else 1

    def _distance_to(self, pos):
        return np.linalg.norm(self._player_pos - pos)

    def _has_possession(self):
        return self._distance_to(self._ball_pos) <= Thresholds.POSSESSION

    def _is_closest_to_ball(self, friendlies, opponents):
        min_dist = float('inf')
        closest_to_ball = None
        for robot in itertools.chain(friendlies, opponents):
            robot_pos = robot.get_position()[0:2]
            dist = distance_between(self._ball_pos, robot_pos)
            if dist < min_dist:
                if closest_to_ball == self._player:
                    # If we were previously the closest player but are no
                    # no longer, then we can cease the search
                    closest_to_ball = None
                    break
                min_dist = dist
                closest_to_ball = robot
        return self._player == closest_to_ball

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

    def _should_intercept_ball(self, friendlies, opponents):
        # TODO: take equations of motion of ball into account. For example, if
        # we're the closest to the ball but it's moving away from us, then
        # maybe we shouldn't be going for it after all
        retval = self._is_closest_to_ball(friendlies, opponents)
        retval |= self._shot_on_net()
        return retval

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
        net_pos = np.array([self._net.x, self._net.y])
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
            crit_pos, dist = closest_point_on_line(self._player_pos, self._ball_pos, net_pos)
        self._move_player_to(crit_pos)

    def update_next_strategy(self, friendlies, opponents):
        if self._has_possession():
            # Return ball to offensive players
            self._pass_to_offense(friendlies)
        elif self._should_intercept_ball(friendlies, opponents):
            # Intercept ball
            # TODO: employ caching or state to avoid redundant computation
            if self._shot_on_net():
                self._defend_shot()
            else:
                self._pursue_ball()
        else:
            # Position along ball-goal line...at a certain distance
            self._defend_net()


class TeamStrategy(Strategy):

    def __init__(self):
        super().__init__()
        self.game_state = GameState.INIT
        self.reset()

    def reset(self):
        self._has_setup = False
        self.game_state = GameState.INIT
        self.dist_to_ball = {}
        self.team_in_pos = None
        self.team = {}
        self.closest_friendly_to_ball = None
        self.friendly_team = None
        self.friendly_net = None
        self.opponent_net = None

    def _update_ball_info(self, friendly, opponent, ball):
        """
        Computes distance between each robot and the ball, and determines
            (1) which team has possession of the ball
            (2) which robot has possession of the ball
        where possession is defined as "being closest to the ball"
        """
        ball_pos = ball.get_position()[0:2]
        min_dist = float('inf')
        min_friendly_dist = float('inf')
        self.closest_friendly_to_ball = None
        self.team_in_pos = None
        for robot in itertools.chain(friendly, opponent):
            dist = np.linalg.norm(
                ball_pos - robot.get_position()[0:2]
            )
            self.dist_to_ball[robot] = dist
            if dist < min_dist:
                # Find overall closest player to determine which team has
                # possession
                min_dist = dist
                if min_dist <= Thresholds.POSSESSION:
                    self.team_in_pos = robot.team
            if robot.team == self.friendly_team and dist < min_friendly_dist:
                # Find friendly player closest to ball
                min_friendly_dist = dist
                self.closest_friendly_to_ball = robot

    def _populate_team_map(self, friendly):
        """
        Populates a data structure that makes it convenient to access different
        team members based on their function
        """
        for robot in friendly:
            role = robot.role
            self.team[role] = robot

    # def get_closest_opponent(self, this_role, opponents):
    #     """
    #     Return the oponnent closest to the given player, and the distance
    #     between them
    #     """
    #     closest_opponent = None
    #     min_dist = float('inf')
    #     for robot in opponents:
    #         dist = self.dist_to_player(this_role, robot.get_position()[0:2])
    #         if dist < min_dist:
    #             min_dist = dist
    #             closest_opponent = robot
    #     return closest_opponent, min_dist
    #
    # def get_teammates_in_ascending_order(self, this_role):
    #     """
    #     Return list of teammates closest to the given one in ascending order of
    #     distance
    #     """
    #     info = []
    #     for role, robot in self.team.items():
    #         if role != this_role:
    #             dist = self.dist_to_player(this_role, robot.get_position()[0:2])
    #             info.append((robot, dist))
    #     info.sort(key=itemgetter(1))
    #     robot = [e[0] for e in info]
    #     dists = [e[1] for e in info]
    #     return robot, dists

    def update_next_strategy(self, friendlies, opponents, ball):
        # 1. Setup
        if not self._has_setup:
            self._has_setup = True
            self.friendly_team = friendlies[0].team
            self.friendly_net = np.array(friendlies[0].get_net_position())
            self.opponent_net = np.array(friendlies[0].get_opponent_net_position())

        # 2. Measurements
        self._update_ball_info(friendlies, opponents, ball)
        self._populate_team_map(friendlies)

        # 3. Update game state state
        if self.team_in_pos != None:
            # If any player has possession of the ball, we update state. If no
            # player has possession, then we maintain the previous game state
            # (e.g., could be due to a pass)
            if self.team_in_pos == self.friendly_team:
                self.game_state = GameState.FRIENDLY_POSSESSION
            elif self.team_in_pos != self.friendly_team:
                self.game_state = GameState.OPPONENT_POSSESSION
            else:
                raise ValueError('Strategy state is invalid!')

        # 4. Update substrategies
        gs = GoalieStrategy(self.team[Robot.Role.GOALIE], ball)
        gs.update_next_strategy(friendlies, opponents)

        # 5. Take action based on current game state
        print(self.game_state)
        if self.game_state == GameState.INIT:
            pass
        elif self.game_state == GameState.FRIENDLY_POSSESSION:
            pass
        elif self.game_state == GameState.OPPONENT_POSSESSION:
            pass
        else:
            raise ValueError('Game state is invalid!')
