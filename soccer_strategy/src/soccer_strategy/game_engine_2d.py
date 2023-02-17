import copy
import random
from typing import Optional

import numpy as np
import rosparam
import rospy

from soccer_common import Transformation
from soccer_msgs.msg import GameState
from soccer_strategy.ball import Ball
from soccer_strategy.game_engine_2d_scene import Scene
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled_2d import RobotControlled2D
from soccer_strategy.strategy.strategy import Strategy
from soccer_strategy.strategy.strategy_dummy import StrategyDummy
from soccer_strategy.strategy.strategy_stationary import StrategyStationary
from soccer_strategy.team import Team


class GameEngine2D:
    """
    2D simualtor for the game engine, used for testing strategy quickly without interfacing with webots
    """

    PHYSICS_UPDATE_INTERVAL = 0.25  # 4 Times per second
    DISPLAY_UPDATE_INTERVAL = 0.5  # Every 5 seconds

    def __init__(
        self,
        display=True,
        team_1_strategy: type = StrategyDummy,
        team_2_strategy: type = StrategyDummy,
        team_1: Optional[Team] = None,
        team_2: Optional[Team] = None,
        game_duration: float = 20,
    ):
        """

        :param display: Whether to show the visualizer
        :param team_1_strategy: What strategy the team 1 will use
        :param team_2_strategy: What strategy the team 2 will use
        :param game_duration: How long to run the game (in minutes)
        """
        rospy.set_param("/use_sim_time", True)
        rospy.rostime._set_rostime(rospy.Time(0))

        self.display = display
        self.game_duration = game_duration

        # Initialize teams
        self.team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([-4, 0, 0]),
                ),
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.LEFT_WING,
                    status=Robot.Status.READY,
                    position=np.array([-1, -1, 0]),
                ),
                RobotControlled2D(
                    robot_id=3,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.RIGHT_WING,
                    status=Robot.Status.READY,
                    position=np.array([-1, 1, 0]),
                ),
                RobotControlled2D(
                    robot_id=4,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([-0.5, 0, 0]),
                ),
            ]
        )
        if team_1 is not None:
            self.team1 = team_1

        self.team1.strategy = team_1_strategy()

        self.team1_init = copy.deepcopy(self.team1)

        self.team2 = Team(
            [
                RobotControlled2D(
                    robot_id=5,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([4, 0, -3.14]),
                ),
                RobotControlled2D(
                    robot_id=6,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.LEFT_WING,
                    status=Robot.Status.READY,
                    position=np.array([1, -1, -3.14]),
                ),
                RobotControlled2D(
                    robot_id=7,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.RIGHT_WING,
                    status=Robot.Status.READY,
                    position=np.array([1, 1, -3.14]),
                ),
                RobotControlled2D(
                    robot_id=8,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([0.5, 0, -3.14]),
                ),
            ]
        )
        if team_2 is not None:
            self.team2 = team_2

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

    def run(self):
        """
        Main loop for the 2D strategy executor, runs the strategies for both team against a vispy simulator
        """

        game_period_seconds = int(
            self.game_duration * 60 / GameEngine2D.PHYSICS_UPDATE_INTERVAL
        )  # 2 Periods of 10 minutes each, each step is a second
        friendly_points = 0
        opponent_points = 0

        if self.display:
            for i in range(10):
                self.scene.update(self.team1.robots + self.team2.robots, self.ball)
        for step in range(game_period_seconds):
            if step == int(game_period_seconds / 2):
                print("\033[96m----- Second Half Started -----\033[0m")
                self.reset_robots()
            if step % int(game_period_seconds / 2 / 20) == 0:
                print(f"\033[96mTime Elapsed: {step } / {game_period_seconds}\033[0m")

            self.update_estimated_physics(self.team1.robots + self.team2.robots, self.ball)

            if step % self.team1.strategy.update_frequency == 0:
                for robot in self.team1.robots:
                    robot.active = True
                    self.team1.strategy.step_strategy(self.team1, self.team2, self.gameState)
                    robot.active = False

            if step % self.team2.strategy.update_frequency == 0:
                for robot in self.team2.robots:
                    robot.active = True
                    self.team2.strategy.step_strategy(self.team2, self.team1, self.gameState)
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

            # Step the ros time manually
            rospy.rostime._rostime_current += rospy.Duration(1)

        print("----------------------------------------------------------------------")
        print(f"Game Finished: Friendly: {friendly_points}, Opponent: {opponent_points}")
        rospy.set_param("/use_sim_time", False)
        rospy.rostime._rostime_current = None
        return friendly_points, opponent_points

    def update_estimated_physics(self, robots: [RobotControlled2D], ball: Ball):
        """
        Executes the world physics step, robot's movement, ball movement, and for fairness it runs through the priority
        for kicks in a random fashion.

        :param robots: The list of all robots including enemy robots
        :param ball: The ball in the game
        """

        # Robot do action in a random priority order
        for robot in sorted(robots, key=lambda _: random.random()):
            robot.observe_ball(ball)
            robot.observe_obstacles(robots)
            if robot.status == Robot.Status.WALKING:
                robot.path_time = robot.path_time + GameEngine2D.PHYSICS_UPDATE_INTERVAL
                update_position_transformation: Transformation = robot.path.estimatedPositionAtTime(robot.path_time)
                update_position = update_position_transformation.pos_theta
                if robot.path.isFinished(robot.path_time):
                    robot.status = Robot.Status.READY
                    robot.position = robot.path.end_transform.pos_theta
                    robot.path_time = 0
                    continue

                robot.position = update_position

            elif robot.status == Robot.Status.KICKING:
                if ball.kick_timeout == 0:
                    if robot.can_kick(ball, None, verbose=False):
                        kick_angle_rand = np.random.normal(0, 0.2)
                        kick_force_rand = max(np.random.normal(0.4, 0.3), 0)
                        if kick_force_rand == 0:
                            print("Kick Missed")

                        rotation_rand = np.array(
                            [[np.cos(kick_angle_rand), -np.sin(kick_angle_rand)], [np.sin(kick_angle_rand), np.cos(kick_angle_rand)]]
                        )
                        ball.velocity = kick_force_rand * rotation_rand @ robot.kick_velocity
                        ball.kick_timeout = 5
                robot.status = Robot.Status.GETTING_BACK_UP
                robot.trajectory_timeout = 8
                robot.path_time = 0
            elif robot.status == Robot.Status.GETTING_BACK_UP:
                if robot.trajectory_timeout == 0:
                    robot.status = Robot.Status.READY
                else:
                    robot.trajectory_timeout -= 1
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
        """
        Reset's the robot and the team back to their initial state, so a new game happen
        """
        self.team1 = copy.deepcopy(self.team1_init)
        self.team2 = copy.deepcopy(self.team2_init)
        self.ball = copy.deepcopy(self.ball_init)
