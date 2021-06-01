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

class TeamStrategy(Strategy):

    def __init__(self):
        super().__init__()
        self.game_state = GameState.INIT
        self.POSSESSION_THRESH = 0.2
        self.reset()

    def reset(self):
        self.game_state = GameState.INIT
        self.dist_to_ball = {}
        self.team_in_pos = None
        self.team = {}
        self.closest_robot = None

    def _update_ball_info(self, friendly, opponent, ball):
        """
        Computes distance between each robot and the ball, and determines
            (1) which team has possession of the ball
            (2) which robot has possession of the ball
        where possession is defined as "being closest to the ball"
        """
        ball_pos = ball.get_position()[0:2]
        min_dist = float('inf')
        self.closest_robot = None
        self.team_in_pos = None
        for robot in itertools.chain(friendly, opponent):
            dist = np.linalg.norm(
                ball_pos - robot.get_position()[0:2]
            )
            self.dist_to_ball[robot] = dist
            if dist < min_dist:
                min_dist = dist
                self.closest_robot = robot
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
        robot.set_navigation_position(np.append(pos, 0))
        robot.status = Robot.Status.WALKING

    def update_next_strategy(self, friendly, opponent, ball):
        # 1. Setup
        self.friendly_team = friendly[0].team
        self.friendly_net = np.array(friendly[0].get_net_position())
        self.opponent_net = np.array(friendly[0].get_opponent_net_position())
        self.midfield = np.array([0, 0])
        self.defense_pos = 0.5 * self.midfield + 0.5 * self.friendly_net

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
            # 4.1 Move striker back to intercept opponent's kick
            t = 0.6
            striker_pos = t * self.midfield + (1 - t) * self.friendly_net
            self.move_player_to(Robot.Role.STRIKER, striker_pos)

            # 4.2 Move left midfield into defensive position
            self.move_player_to(Robot.Role.LEFT_MIDFIELD, self.defense_pos)

            # 4.3 Move right midfield into offensive position
            right_mf_pos = self.team[Robot.Role.RIGHT_MIDFIELD].get_position()[0:2]
            right_mf_pos[1] = self.midfield[1]
            self.move_player_to(Robot.Role.RIGHT_MIDFIELD, right_mf_pos)
        elif self.game_state == GameState.FRIENDLY_POSSESSION:
            print('friendly pos')
            pass
        elif self.game_state == GameState.OPPONENT_POSSESSION:
            print('opponent pos')
            pass
        else:
            raise ValueError('Game state is invalid!')
