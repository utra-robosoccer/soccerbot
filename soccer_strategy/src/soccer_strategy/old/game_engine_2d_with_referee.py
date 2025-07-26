import copy
import time

import rclpy

from soccer_msgs.msg import GameState
from soccer_strategy.old.game_engine_2d import GameEngine2D
from soccer_strategy.old.game_engine_3d import decide_next_strategy
from soccer_strategy.old.robot import Robot
from soccer_strategy.old.robot_controlled_2d import RobotControlled2D


class GameEngine2DWithReferee(GameEngine2D):
    """
    2D simualtor for the game engine, used for testing strategy quickly without interfacing with webots
    """

    PHYSICS_UPDATE_INTERVAL = 0.1
    DISPLAY_UPDATE_INTERVAL = 0.01

    GAMESTATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("GAMESTATE")}
    SECONDARY_STATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("STATE")}
    SECONDARY_STATE_MODE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("MODE")}

    def __init__(self, *args, **kwargs):

        super().__init__(*args, **kwargs)

        self.game_state_create_subscriptions = {}
        self.individual_robot_gamestates = {}
        self.individual_robot_last_strategy_update_time = {}
        self.last_display_update_time = self.get_clock().now()

        for team in [self.team1, self.team2]:
            for robot in team.robots:
                if team == self.team1:
                    team_id = 16
                else:
                    team_id = 5
                robot_id = robot.robot_id

                self.game_state_create_subscriptions[(team, robot)] = self.create_subscription(
                    f"/team_{team_id}/robot{robot_id}/gamestate", GameState, self.gamestate_callback_robot, robot
                )
                self.individual_robot_gamestates[robot] = copy.deepcopy(self.gameState)
                self.individual_robot_last_strategy_update_time[robot] = self.get_clock().now()

    def run(self):

        if self.display:
            for i in range(10):
                self.scene.update(self.team1.robots + self.team2.robots, self.ball)

        while not self.is_shutdown():
            t_start = time.time()

            self.update_estimated_physics(self.team1.robots + self.team2.robots, self.ball)

            t1 = time.time()

            for teama, teamb in zip([self.team1, self.team2], [self.team2, self.team1]):

                for robot in teama.robots:
                    strategy = self.robot_strategies[(robot.team, robot.robot_id)]
                    gameState = self.individual_robot_gamestates[robot]

                    new_strategy = decide_next_strategy(strategy=strategy, gameState=gameState, this_robot=robot)
                    # if type(strategy) != new_strategy: TODO fix maybe
                    #     self.robot_strategies[(robot.team, robot.robot_id)] = new_strategy()
                    #     self.get_logger().info(f"Team {robot.team} Robot {robot.robot_id} transitioned to strategy {str(new_strategy.__name__)}")

                    if self.get_clock().now() - self.individual_robot_last_strategy_update_time[robot] > self.Duration(strategy.update_frequency):

                        robot.active = True
                        strategy.step_strategy(teama, teamb, gameState)
                        robot.active = False
                        self.individual_robot_last_strategy_update_time[robot] = self.get_clock().now()

            t2 = time.time()

            self.scene.update(self.team1.robots + self.team2.robots, self.ball)
            self.last_display_update_time = self.get_clock().now()

            self.get_logger().error(5, f"Physics Update Time {t1 - t_start} Strategy Update Time {t2 - t1}")

        return

    def gamestate_callback_robot(self, gameState: GameState, robot: RobotControlled2D):
        """
        Callback function for gamestate information from the webot's game controller

        :param gameState: Contains information about the game, whether it is playing or paused etc
        """
        if robot.robot_id == 1 and robot.team == Robot.Team.FRIENDLY:
            if (
                self.gameState.gameState != gameState.gameState
                or self.gameState.secondaryState != gameState.secondaryState
                or self.gameState.secondaryStateMode != gameState.secondaryStateMode
            ):
                self.gameState = gameState
                print(
                    f"\033[92mGame State Changed: {GameEngine2DWithReferee.GAMESTATE_LOOKUP[gameState.gameState]}, Secondary State: {GameEngine2DWithReferee.SECONDARY_STATE_LOOKUP[gameState.secondaryState]}, Secondary State Mode: {GameEngine2DWithReferee.SECONDARY_STATE_MODE_LOOKUP[gameState.secondaryStateMode]}\033[0m"
                )

        if gameState.penalty != GameState.PENALTY_NONE:
            robot.status = Robot.Status.PENALIZED
        elif robot.status in [Robot.Status.DISCONNECTED, Robot.Status.PENALIZED]:
            robot.status = Robot.Status.DETERMINING_SIDE

        self.individual_robot_gamestates[robot] = gameState
