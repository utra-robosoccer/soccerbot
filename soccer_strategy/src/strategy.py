from enum import IntEnum
import itertools
import math
import numpy as np
from operator import itemgetter
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
        self.POSSESSION_THRESH = 0.2 # How close player has to be to ball to have possession
        self.OPEN_THRESH = 0.6 # How far the closest opponent must be in order for a player to be considered "open"
        self.PASS_THRESH = 1.5 # How close you have to be to teammate before passing
        self.GOAL_THRESH = 2 # How close you have to be before trying to score
        self.MAX_GOALIE_DIST = 1.5
        self.reset()

    def reset(self):
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
                if min_dist <= self.POSSESSION_THRESH:
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
        elif role == Robot.Role.LEFT_MIDFIELD:
            # Don't let left midfielder go too far up the field
            sign = -1 if self.team[role].get_position()[Field.Y_COORD] < 0 else 1
            ty = 0.7
            farthest = ty * Field.MIDFIELD + (1 - ty) * self.friendly_net
            yval = max(
                np.abs(pos[Field.Y_COORD]),
                np.abs(farthest[Field.Y_COORD])
            )
            pos[Field.Y_COORD] = sign * yval
        robot.set_navigation_position(np.append(pos, 0))
        robot.status = Robot.Status.WALKING

    def _compute_move_to_crit_line(self, ball, goal_pos, role):
        """
        Orthogonal projection of robot's position onto the line between the
        ball and the goal position

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

    def _compute_crit_move(self, ball, goal_pos, role):
        """
        Critical move is:
            1. move towards the critical line, if that puts the player in the
               way of the ball
            2. move towards the ball along the critical line, if we are already
               on the critical line
            3. move towards the ball if it's already gone past us
        """
        crit_pos, dist = self._compute_move_to_crit_line(ball, goal_pos, role)
        # If we are already "on" the critical line, then we want to
        # move along it towards the ball. We check this by computing
        # the angle between the goal-ball vector and the player-ball
        # vector
        ball_pos = ball.get_position()
        goalb_vec = self.friendly_net - ball_pos
        playerb_vec = self.team[role].get_position()[0:2] - ball_pos
        num = np.dot(goalb_vec, playerb_vec)
        denom = np.linalg.norm(goalb_vec) * np.linalg.norm(playerb_vec)
        angle_deg = np.arccos(num / denom) * 180 / np.pi
        ANGLE_THRESH = 5
        if np.abs(angle_deg) < ANGLE_THRESH:
            crit_pos = ball_pos
            crit_dist = self.dist_to_player(role, ball_pos)
        return crit_pos, dist

    def dist_to_friendly_goal(self, pos):
        return np.linalg.norm(self.friendly_net - pos)

    def dist_to_opponent_goal(self, pos):
        return np.linalg.norm(self.opponent_net - pos)

    def dist_to_player(self, role, pos):
        return np.linalg.norm(self.team[role].get_position()[0:2] - pos)

    def get_closest_teammate(self, this_role):
        """
        Return the teammate closest to the given one, and the distance between
        them
        """
        closest_teammate = None
        min_dist = float('inf')
        for role, robot in self.team.items():
            if role != this_role:
                dist = self.dist_to_player(this_role, robot.get_position()[0:2])
                if dist < min_dist:
                    min_dist = dist
                    closest_teammate = robot
        return closest_teammate, min_dist

    def get_closest_opponent(self, this_role, opponents):
        """
        Return the oponnent closest to the given player, and the distance
        between them
        """
        closest_opponent = None
        min_dist = float('inf')
        for robot in opponents:
            dist = self.dist_to_player(this_role, robot.get_position()[0:2])
            if dist < min_dist:
                min_dist = dist
                closest_opponent = robot
        return closest_opponent, min_dist

    def get_teammates_in_ascending_order(self, this_role):
        """
        Return list of teammates closest to the given one in ascending order of
        distance
        """
        info = []
        for role, robot in self.team.items():
            if role != this_role:
                dist = self.dist_to_player(this_role, robot.get_position()[0:2])
                info.append((robot, dist))
        info.sort(key=itemgetter(1))
        robot = [e[0] for e in info]
        dists = [e[1] for e in info]
        return robot, dists

    def pass_ball(self, ball, teammate):
        """
        Pass ball to teammate
        """
        teammate_pos = teammate.get_position()[0:2]
        pass_dist = self.dist_to_player(
            self.closest_friendly_to_ball.role, teammate_pos
        )
        magnitude = 100 * min(pass_dist, 1)
        self.kick_ball(ball, teammate_pos, kick_speed_perc=magnitude)

    def kick_ball(self, ball, pos, kick_speed_perc=100):
        """
        Kick ball towards position (full-speed kick)
        """
        delta = pos - ball.get_position()
        dir = delta / np.linalg.norm(delta)
        mag = self.closest_friendly_to_ball.max_kick_speed * kick_speed_perc / 100.0
        self.closest_friendly_to_ball.set_kick_velocity(dir * mag)
        self.closest_friendly_to_ball.status = Robot.Status.KICKING

    def dribble_ball(self, ball, pos):
        """
        Dribble ball towards position (lower-speed kick)
        """
        self.kick_ball(ball, pos, kick_speed_perc=70)

    def is_open(self, opponents, player):
        """Return True if the specified player is open, otherwise False"""
        player_is_open = True
        min_dist = float('inf')
        for robot in opponents:
            dist = self.dist_to_player(player.role, robot.get_position()[0:2])
            if dist < min_dist:
                min_dist = dist
                if min_dist < self.OPEN_THRESH:
                    player_is_open = False
                    break
        return player_is_open

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
            # 6. Determine whether defensive or offensive players have the
            # ball. If defensive players have it, the goal is to get it to the
            # offensive players
            ball_pos = ball.get_position()
            defense_possession = self.closest_friendly_to_ball.role == Robot.Role.GOALIE or \
               self.closest_friendly_to_ball.role == Robot.Role.LEFT_MIDFIELD
            if defense_possession:
                # Defensive strategy update
                # If goalie has the ball, then it holds onto it until:
                #   1. Another friendly player is within the pass threshold of
                #      it, in which case we pass the ball
                #   2. An opponent player is within the pass threshold, in
                #      which case we kick the ball away from the opponent
                if self.closest_friendly_to_ball.role == Robot.Role.GOALIE:
                    # Actions to take if we have possession
                    if self.team_in_pos == self.friendly_team:
                        closest_teammate, teammate_dist = self.get_closest_teammate(Robot.Role.GOALIE)
                        closest_opponent, opponent_dist = self.get_closest_opponent(Robot.Role.GOALIE, opponent)
                        if teammate_dist < self.PASS_THRESH and self.is_open(opponent, closest_teammate):
                            # Check if we should pass
                            self.pass_ball(ball, closest_teammate)
                        elif opponent_dist < self.PASS_THRESH:
                            # Check if we should discard ball away from opponent
                            goalie_pos = self.closest_friendly_to_ball.get_position()[0:2]
                            vec_to_opp = closest_opponent.get_position()[0:2] - goalie_pos
                            perp_vec = np.array([vec_to_opp[Field.Y_COORD], -vec_to_opp[Field.X_COORD]])
                            # Make sure we kick it perpendicular to the opponent,
                            # but in the direction away from the goal!
                            if np.dot(perp_vec, goalie_pos) > 0:
                                perp_vec *= -1 # 180 degree rotation
                            self.kick_ball(ball, perp_vec)
                    else:
                        # Goalie should move to critical defense position
                        crit_pos, dist = self._compute_move_to_crit_line(
                            ball, self.friendly_net, Robot.Role.GOALIE
                        )
                        if self.dist_to_friendly_goal(crit_pos) > self.dist_to_friendly_goal(ball_pos):
                            crit_pos = ball_pos
                        self.move_player_to(Robot.Role.GOALIE, crit_pos)
                    # All other teammates move towards the ball
                    for role, robot in self.team.items():
                        if role != Robot.Role.GOALIE:
                            self.move_player_to(role, ball_pos)
                elif self.closest_friendly_to_ball.role == Robot.Role.LEFT_MIDFIELD:
                    # Actions to take if we have possession
                    if self.team_in_pos == self.friendly_team:
                        # Check if we should pass
                        teammates, dists = self.get_teammates_in_ascending_order(
                            self.closest_friendly_to_ball.role
                        )
                        if teammates[0] == Robot.Role.GOALIE:
                            closest_teammate = teammates[1]
                            closest_dist = dists[1]
                        else:
                            closest_teammate = teammates[0]
                            closest_dist = dists[0]
                        if closest_dist < self.PASS_THRESH:
                            self.pass_ball(ball, closest_teammate)
                        else:
                            # If we're not close enough to pass, then dribble ball towards teammate
                            # TODO: should make it dribble towards teammate BUT away from nearby opponents (if they're close enough)
                            self.dribble_ball(ball, closest_teammate.get_position()[0:2])
                    else:
                        self.move_player_to(Robot.Role.LEFT_MIDFIELD, ball_pos)
                    # Offensive players should move towards ball
                    for role, robot in self.team.items():
                        if role != Robot.Role.GOALIE and role != Robot.Role.LEFT_MIDFIELD:
                            self.move_player_to(role, ball_pos)
                    # Goalie should move to critical defense position
                    crit_pos, dist = self._compute_move_to_crit_line(
                        ball, self.friendly_net, Robot.Role.GOALIE
                    )
                    if self.dist_to_friendly_goal(crit_pos) > self.dist_to_friendly_goal(ball_pos):
                        crit_pos = ball_pos
                    self.move_player_to(Robot.Role.GOALIE, crit_pos)
            else:
                # Offensive strategy update
                has_ball = self.closest_friendly_to_ball
                if has_ball.role == Robot.Role.STRIKER:
                    offense_partner = self.team[Robot.Role.RIGHT_MIDFIELD]
                else:
                    offense_partner = self.team[Robot.Role.STRIKER]
                # Actions to take if we have possession
                if self.team_in_pos == self.friendly_team:
                    dist_to_goal = self.dist_to_opponent_goal(has_ball.get_position()[0:2])
                    if dist_to_goal < self.GOAL_THRESH:
                        # If the one possessing the ball has a shot on net,
                        # it takes it
                        self.kick_ball(ball, self.opponent_net)
                    elif not self.is_open(opponent, has_ball) and \
                        self.is_open(opponent, offense_partner):
                        # Otherwise, if the offense partner is open and the
                        # one with the ball is not, pass to partner
                        self.pass_ball(ball, offense_partner)
                    else:
                        # Otherwise, dribble up the field
                        goal_pos = self.opponent_net
                        if has_ball.role == Robot.Role.STRIKER:
                            edge = Field.LEFT_EDGE
                        else:
                            edge = Field.RIGHT_EDGE
                        tx = 0.2
                        goal_pos[Field.X_COORD] = tx * edge + (1 - tx) * goal_pos[Field.X_COORD]
                        self.dribble_ball(ball, goal_pos)
                else:
                    # Actions to take if we don't have possession (e.g.,
                    # middle of kick)
                    self.move_player_to(has_ball.role, ball.get_position())
                    self.move_player_to(offense_partner.role, self.opponent_net)
                # Position defensive players now
                # Goalie should move to critical defense position
                crit_pos, dist = self._compute_crit_move(
                    ball, self.friendly_net, Robot.Role.GOALIE
                )
                if self.dist_to_friendly_goal(crit_pos) > self.dist_to_friendly_goal(ball_pos):
                    crit_pos = ball_pos
                self.move_player_to(Robot.Role.GOALIE, crit_pos)
                # Same for left midfielder
                crit_pos, dist = self._compute_crit_move(
                    ball, self.friendly_net, Robot.Role.LEFT_MIDFIELD
                )
                self.move_player_to(Robot.Role.LEFT_MIDFIELD, crit_pos)
        elif self.game_state == GameState.OPPONENT_POSSESSION:
            # 7.1 Compute default defense position for each player
            player_moves = {}
            # 7.1.1 Compute distance to critical defense line
            ball_pos = ball.get_position()[0:2]
            crit_moves = {}
            min_dist = float('inf')
            closest_to_crit = None
            for role in Robot.Role:
                crit_pos, dist = self._compute_crit_move(ball, self.friendly_net, role)
                if self.dist_to_friendly_goal(crit_pos) < self.dist_to_friendly_goal(ball_pos):
                    crit_move = crit_pos
                    crit_dist = dist
                else:
                    crit_move = ball_pos
                    crit_dist = self.dist_to_player(role, ball_pos)
                crit_moves[role] = crit_move
                if crit_dist < min_dist and role != Robot.Role.GOALIE:
                    min_dist = crit_dist
                    closest_to_crit = role
            # 7.1.2 Striker
            ty = 0.6
            striker_pos = ty * Field.MIDFIELD + (1 - ty) * self.friendly_net
            tx = 0.6
            striker_pos[Field.X_COORD] = tx * Field.LEFT_EDGE
            player_moves[Robot.Role.STRIKER] = striker_pos
            # 7.1.3 Right midfield
            ty = 0.6
            right_mf_pos = ty * Field.MIDFIELD + (1 - ty) * self.friendly_net
            tx = 0.6
            right_mf_pos[Field.X_COORD] = tx * Field.RIGHT_EDGE
            player_moves[Robot.Role.RIGHT_MIDFIELD] = right_mf_pos
            # 7.1.4 Left midfield
            player_moves[Robot.Role.LEFT_MIDFIELD] = crit_moves[Robot.Role.LEFT_MIDFIELD]
            # 7.1.5 Goalie
            player_moves[Robot.Role.GOALIE] = crit_moves[Robot.Role.GOALIE]
            # 7.1.6 Execute updates. Move all players to their default position
            # except for the one closest to the critical line
            for role, move in player_moves.items():
                if role != closest_to_crit:
                    self.move_player_to(role, move)
                else:
                    self.move_player_to(role, ball_pos)
        else:
            raise ValueError('Game state is invalid!')
