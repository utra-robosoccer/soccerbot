from enum import IntEnum
import itertools
import math
import numpy as np
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

class Field:
    X_COORD = 0
    Y_COORD = 1
    MIDFIELD = np.array([0, 0])
    LEFT_EDGE = -3.5
    RIGHT_EDGE = 3.5

class TeamStrategy(Strategy):

    def __init__(self):
        super().__init__()
        self.game_state = GameState.INIT
        self.POSSESSION_THRESH = 0.2
        self.MAX_GOALIE_DIST = 1
        self.reset()

    def reset(self):
        self.game_state = GameState.INIT
        self.dist_to_ball = {}
        self.team_in_pos = None
        self.team = {}
        self.closest_to_ball = None

    def _update_ball_info(self, friendly, opponent, ball):
        """
        Computes distance between each robot and the ball, and determines
            (1) which team has possession of the ball
            (2) which robot has possession of the ball
        where possession is defined as "being closest to the ball"
        """
        ball_pos = ball.get_position()[0:2]
        min_dist = float('inf')
        self.closest_to_ball = None
        self.team_in_pos = None
        for robot in itertools.chain(friendly, opponent):
            dist = np.linalg.norm(
                ball_pos - robot.get_position()[0:2]
            )
            self.dist_to_ball[robot] = dist
            if dist < min_dist:
                min_dist = dist
                self.closest_to_ball = robot
                if min_dist <= self.POSSESSION_THRESH:
                    self.team_in_pos = robot.team

    def _populate_team_map(self, friendly):
        """
        Populates a data structure that makes it convenient to access different
        team members based on their function
        """
        for robot in friendly:
            role = robot.role
            self.team[role] = robot

    def move_player_to(self, role, pos):
        """Wraps position update. Pos is 2D (x, y)"""
        robot = self.team[role]
        if role == Robot.Role.GOALIE:
            # Ensure goalie doesn't exceed its max dist from the goal by
            # projecting onto circle if needed
            #
            # Source:
            # - https://math.stackexchange.com/questions/127613/closest-point-on-circle-edge-from-point-outside-inside-the-circle
            dist_to_goal = np.linalg.norm(self.friendly_net - pos)
            ratio = dist_to_goal / self.MAX_GOALIE_DIST
            if ratio >= 1:
                direction = pos - self.friendly_net
                direction = self.MAX_GOALIE_DIST * direction / np.linalg.norm(direction)
                pos = self.friendly_net + direction
        robot.set_navigation_position(np.append(pos, 0))
        robot.status = Robot.Status.WALKING

    def _compute_move_to_crit_line(self, ball, goal_pos, role):
        """
        Orthogonal projection of robot's position onto the line between the
        ball and the goal

        Source:
        - https://blender.stackexchange.com/questions/94464/finding-the-closest-point-on-a-line-defined-by-two-points
        """
        robot_pos = self.team[role].get_position()[0:2]
        ball_pos = ball.get_position()[0:2]
        line_diff = goal_pos - ball_pos
        # Compute distance
        proj = np.dot(robot_pos, line_diff) / np.dot(line_diff, line_diff)
        dist = np.linalg.norm(line_diff * proj - robot_pos)
        # Compute crit pos
        n = line_diff / np.linalg.norm(line_diff)
        ap = robot_pos - ball_pos
        t = ap.dot(n)
        crit_pos = ball_pos + t * n
        return crit_pos, dist

    def dist_to_goal(self, pos):
        return np.linalg.norm(self.friendly_net - pos)

    def dist_to_player(self, role, pos):
        return np.linalg.norm(self.team[role].get_position()[0:2] - pos)

    def update_next_strategy(self, friendly, opponent, ball):
        # 1. Setup
        self.friendly_team = friendly[0].team
        self.friendly_net = np.array(friendly[0].get_net_position())
        self.opponent_net = np.array(friendly[0].get_opponent_net_position())

        # 2. Measurements
        self._update_ball_info(friendly, opponent, ball)
        self._populate_team_map(friendly)

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

        # 4. Take action based on current game state
        print(self.game_state)
        if self.game_state == GameState.INIT:
            # 5. Send players to default positions
            # 5.1 Move striker back to intercept opponent's kick
            t = 0.6
            striker_pos = t * Field.MIDFIELD + (1 - t) * self.friendly_net
            self.move_player_to(Robot.Role.STRIKER, striker_pos)

            # 5.2 Move right midfield into offensive position
            right_mf_pos = self.team[Robot.Role.RIGHT_MIDFIELD].get_position()[0:2]
            right_mf_pos[Field.Y_COORD] = Field.MIDFIELD[Field.Y_COORD]
            self.move_player_to(Robot.Role.RIGHT_MIDFIELD, right_mf_pos)

            # 5.3 Move left midfield into defensive position
            left_mf_pos = 0.5 * Field.MIDFIELD + 0.5 * self.friendly_net
            self.move_player_to(Robot.Role.LEFT_MIDFIELD, left_mf_pos)

            # 5.4 Move goalie to defensive position
            goalie_pos = 0.2 * Field.MIDFIELD + 0.8 * self.friendly_net
            self.move_player_to(Robot.Role.GOALIE, goalie_pos)
        elif self.game_state == GameState.FRIENDLY_POSSESSION:
            pass
        elif self.game_state == GameState.OPPONENT_POSSESSION:
            # 6.1 Compute default defense position for each player
            player_moves = {}
            # 6.1.1 Compute distance to critical defense line
            ball_pos = ball.get_position()[0:2]
            crit_moves = {}
            min_dist = float('inf')
            closest_to_crit = None
            for role in Robot.Role:
                # Critical move is:
                #  1. move towards the critical line, if that puts the player
                #     in the way of the ball
                #  2. move towards the ball if it's already gone past us
                crit_pos, dist = self._compute_move_to_crit_line(ball, self.friendly_net, role)
                if self.dist_to_goal(crit_pos) < self.dist_to_goal(ball_pos):
                    crit_move = crit_pos
                    crit_dist = dist
                else:
                    crit_move = ball_pos
                    crit_dist = self.dist_to_player(role, ball_pos)
                crit_moves[role] = crit_move
                if crit_dist < min_dist and role != Robot.Role.GOALIE:
                    min_dist = crit_dist
                    closest_to_crit = role

            # 6.1.2 Striker
            ty = 0.6
            striker_pos = ty * Field.MIDFIELD + (1 - ty) * self.friendly_net
            tx = 0.6
            striker_pos[Field.X_COORD] = tx * Field.LEFT_EDGE
            player_moves[Robot.Role.STRIKER] = striker_pos

            # 6.1.3 Right midfield
            ty = 0.6
            right_mf_pos = ty * Field.MIDFIELD + (1 - ty) * self.friendly_net
            tx = 0.6
            right_mf_pos[Field.X_COORD] = tx * Field.RIGHT_EDGE
            player_moves[Robot.Role.RIGHT_MIDFIELD] = right_mf_pos

            # 6.1.4 Left midfield
            player_moves[Robot.Role.LEFT_MIDFIELD] = crit_moves[Robot.Role.LEFT_MIDFIELD]

            # 6.1.5 Goalie
            player_moves[Robot.Role.GOALIE] = crit_moves[Robot.Role.GOALIE]

            # 6.1.6 Execute updates. Move all players to their default position
            # except for the one closest to the critical line
            for role, move in player_moves.items():
                if role != closest_to_crit:
                    self.move_player_to(role, move)
                else:
                    self.move_player_to(role, ball_pos)
        else:
            raise ValueError('Game state is invalid!')
