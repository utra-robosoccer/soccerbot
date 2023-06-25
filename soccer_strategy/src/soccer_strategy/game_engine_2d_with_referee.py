import rospy

from soccer_msgs.msg import GameState
from soccer_strategy.game_engine_2d import GameEngine2D
from soccer_strategy.game_engine_3d import decide_next_strategy
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled_2d import RobotControlled2D


class GameEngine2DWithReferee(GameEngine2D):
    """
    2D simualtor for the game engine, used for testing strategy quickly without interfacing with webots
    """

    GAMESTATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("GAMESTATE")}
    SECONDARY_STATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("STATE")}
    SECONDARY_STATE_MODE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("MODE")}

    PHYSICS_UPDATE_INTERVAL = 0.25  # 4 Times per second
    DISPLAY_UPDATE_INTERVAL = 0.5  # Every 5 seconds

    def __init__(self, *args, **kwargs):

        super().__init__(*args, **kwargs)

        self.game_state_subscribers = {}
        self.individual_robot_gamestates = {}
        self.individual_robot_last_strategy_update_time = {}
        self.last_display_update_time = rospy.Time.now()

        for team in [self.team1, self.team2]:
            for robot in team.robots:
                if team == self.team1:
                    team_id = 5
                else:
                    team_id = 16
                robot_id = robot.robot_id + 1

                self.game_state_subscribers[(team, robot)] = rospy.Subscriber(
                    f"/team_{team_id}/robot{robot_id}/gamestate", GameState, self.gamestate_callback_robot, robot
                )
                self.individual_robot_gamestates[robot] = self.gameState
                self.individual_robot_last_strategy_update_time[robot] = rospy.Time.now()

    def run(self):

        if self.display:
            for i in range(10):
                self.scene.update(self.team1.robots + self.team2.robots, self.ball)
        while not rospy.is_shutdown():
            self.update_estimated_physics(self.team1.robots + self.team2.robots, self.ball)

            for robot in self.team1.robots:
                strategy = self.robot_strategies[(robot.team, robot.robot_id)]
                gameState = self.individual_robot_gamestates[robot]

                new_strategy = decide_next_strategy(strategy=strategy, gameState=gameState, this_robot=robot)
                if type(strategy) != new_strategy:
                    self.robot_strategies[(robot.team, robot.robot_id)] = new_strategy()

                if rospy.Time.now() - self.individual_robot_last_strategy_update_time[robot] > rospy.Duration(strategy.update_frequency):
                    robot.active = True
                    strategy.step_strategy(self.team1, self.team2, self.gameState)
                    robot.active = False
                    self.individual_robot_last_strategy_update_time[robot] = rospy.Time.now()

            for robot in self.team2.robots:
                strategy = self.robot_strategies[(robot.team, robot.robot_id)]
                gameState = self.individual_robot_gamestates[robot]

                new_strategy = decide_next_strategy(strategy=strategy, gameState=gameState, this_robot=robot)
                if type(strategy) != new_strategy:
                    self.robot_strategies[(robot.team, robot.robot_id)] = new_strategy()

                if rospy.Time.now() - self.individual_robot_last_strategy_update_time[robot] > rospy.Duration(strategy.update_frequency):
                    robot.active = True
                    strategy.step_strategy(self.team2, self.team1, self.gameState)
                    robot.active = False
                    self.individual_robot_last_strategy_update_time[robot] = rospy.Time.now()

            if self.display and rospy.Time.now() - self.last_display_update_time > rospy.Duration(GameEngine2D.DISPLAY_UPDATE_INTERVAL):
                self.scene.update(self.team1.robots + self.team2.robots, self.ball)
                self.last_display_update_time = rospy.Time.now()

            # Step the ros time manually
            rospy.rostime._rostime_current += rospy.Duration(1)

        rospy.set_param("/use_sim_time", False)
        rospy.rostime._rostime_current = None
        return

    def gamestate_callback_robot(self, gameState: GameState, robot: RobotControlled2D):
        """
        Callback function for gamestate information from the webot's game controller

        :param gameState: Contains information about the game, whether it is playing or paused etc
        """
        if (
            self.gameState.gameState != gameState.gameState
            or self.gameState.secondaryState != gameState.secondaryState
            or self.gameState.secondaryStateMode != gameState.secondaryStateMode
        ):
            print(
                f"\033[92mGame State Changed: {GameEngine2DWithReferee.GAMESTATE_LOOKUP[gameState.gameState]}, Secondary State: {GameEngine2DWithReferee.SECONDARY_STATE_LOOKUP[gameState.secondaryState]}, Secondary State Mode: {GameEngine2DWithReferee.SECONDARY_STATE_MODE_LOOKUP[gameState.secondaryStateMode]}\033[0m"
            )

        self.gameState = gameState
        if gameState.penalty != GameState.PENALTY_NONE:
            robot.status = Robot.Status.PENALIZED
        elif robot.status in [Robot.Status.DISCONNECTED, Robot.Status.PENALIZED]:
            robot.status = Robot.Status.DETERMINING_SIDE

        self.individual_robot_gamestates[robot] = gameState
