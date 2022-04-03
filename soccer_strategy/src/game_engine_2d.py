import _thread
import random

try:
    from soccer_msgs.msg import GameState
except:
    from soccer_msgs.fake_msg import GameState

from robot import Robot
from robot_controlled_2d import RobotControlled2D
from ball import Ball
from strategy.strategy_dummy import StrategyDummy
from strategy.strategy_stationary import StrategyStationary
from team import Team
from game_engine_scene import Scene

import math
import numpy as np
import copy


class GameEngine2D:
    PHYSICS_UPDATE_INTERVAL = 0.25  # 4 Times per second
    DISPLAY_UPDATE_INTERVAL = 0.5  # Every 5 seconds

    def __init__(self, display=True, team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy):
        self.display = display

        # Initialize teams
        self.team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([3.5, 0.0, math.pi]),
                ),
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.LEFT_WING,
                    status=Robot.Status.READY,
                    position=np.array([1.5, -1.5, -math.pi]),
                ),
                RobotControlled2D(
                    robot_id=3,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.RIGHT_WING,
                    status=Robot.Status.READY,
                    position=np.array([1.5, 1.5, -math.pi]),
                ),
                RobotControlled2D(
                    robot_id=4,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([0.8, 0.0, -math.pi]),
                ),
            ]
        )
        self.team1.strategy = team_1_strategy()

        self.team1_init = copy.deepcopy(self.team1)

        self.team2 = Team(
            [
                RobotControlled2D(
                    robot_id=5,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([-3.5, 0.0, 0]),
                ),
                RobotControlled2D(
                    robot_id=6,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.LEFT_WING,
                    status=Robot.Status.READY,
                    position=np.array([-1.5, -1.5, 0]),
                ),
                RobotControlled2D(
                    robot_id=7,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.RIGHT_WING,
                    status=Robot.Status.READY,
                    position=np.array([-1.5, 1.5, 0]),
                ),
                RobotControlled2D(
                    robot_id=8,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([-0.8, 0.0, 0]),
                ),
            ]
        )
        self.team2.strategy = team_2_strategy()
        self.team2.flip_positions()
        self.team2_init = copy.deepcopy(self.team2)

        self.ball = Ball()
        self.ball_init = copy.deepcopy(self.ball)

        # Initialize display
        if self.display:
            self.scene = Scene(self.team1.robots + self.team2.robots, self.ball)

        self.gameState = GameState()
        self.gameState.gameState = GameState.GAMESTATE_PLAYING
        self.gameState.secondaryState = GameState.STATE_NORMAL

    # TODO it is slow because it is re-pathing whenever the ball changes its position
    def run(self):
        game_period_seconds = int(2 * 10 * 60 / GameEngine2D.PHYSICS_UPDATE_INTERVAL)  # 2 Periods of 10 minutes each, each step is a second
        friendly_points = 0
        opponent_points = 0

        for step in range(game_period_seconds):
            if step == int(game_period_seconds / 2):
                print("Second Half Started: ")
                self.reset_robots()

            self.update_estimated_physics(self.team1.robots + self.team2.robots, self.ball)

            if step % self.team1.strategy.update_frequency == 0:
                for robot in self.team1.robots:
                    robot.active = True
                    robot.observed_ball = self.ball
                    self.team1.strategy.update_next_strategy(self.team1, self.team2, self.gameState)
                    robot.active = False

            if step % self.team2.strategy.update_frequency == 0:
                for robot in self.team2.robots:
                    robot.active = True
                    robot.observed_ball = self.ball
                    self.team2.strategy.update_next_strategy(self.team2, self.team1, self.gameState)
                    robot.active = False

            # Check victory condition
            if self.ball.position[0] > 4.5:
                print("----------------------------------------------------------------------")
                print("Friendly Scores!")
                friendly_points += 1
                self.reset_robots()
            elif self.ball.position[0] < -4.5:
                print("----------------------------------------------------------------------")
                print("Opponent Scores!")
                opponent_points += 1
                self.reset_robots()

            if self.display and step % GameEngine2D.DISPLAY_UPDATE_INTERVAL == 0:
                self.scene.update(self.team1.robots + self.team2.robots, self.ball)

        print("----------------------------------------------------------------------")
        print(f"Game Finished: Friendly: {friendly_points}, Opponent: {opponent_points}")
        return friendly_points, opponent_points

    def update_estimated_physics(self, robots, ball):
        # Robot do action in a random priority order
        for robot in sorted(robots, key=lambda _: random.random()):
            robot.observe_ball(ball)
            if robot.status == Robot.Status.WALKING:
                robot.path_time = robot.path_time + GameEngine2D.PHYSICS_UPDATE_INTERVAL
                update_position_transformation = robot.path.estimatedPositionAtTime(robot.path_time)
                update_position = robot.transformation_to_position(update_position_transformation)
                if robot.path.isFinished(robot.path_time):
                    robot.status = Robot.Status.READY
                    robot.path_time = 0
                    continue

                robot.position = update_position

            elif robot.status == Robot.Status.KICKING:
                if ball.kick_timeout == 0:
                    ball.velocity = robot.kick_velocity
                    ball.kick_timeout = 5
                robot.status = Robot.Status.READY
                robot.path_time = 0
            else:
                robot.path_time = 0

        # Ball
        if ball.kick_timeout > 0:
            ball.kick_timeout = ball.kick_timeout - 1

        # update ball position
        self.ball.position_is_live_timeout = 10
        self.ball.position = self.ball.position + self.ball.velocity * GameEngine2D.PHYSICS_UPDATE_INTERVAL

        # slow down ball with friction
        if not np.array_equal(self.ball.velocity, np.array([0, 0])):
            ball_unit_velocity = self.ball.velocity / np.linalg.norm(self.ball.velocity)

            ball_delta_speed = Ball.FRICTION * GameEngine2D.PHYSICS_UPDATE_INTERVAL
            ball_speed = np.linalg.norm(self.ball.velocity)
            if ball_speed > ball_delta_speed:
                self.ball.velocity = self.ball.velocity - ball_delta_speed * ball_unit_velocity
            else:
                self.ball.velocity = np.array([0, 0])

    def reset_robots(self):
        self.team1 = copy.deepcopy(self.team1_init)
        self.team2 = copy.deepcopy(self.team2_init)
        self.ball = copy.deepcopy(self.ball_init)
