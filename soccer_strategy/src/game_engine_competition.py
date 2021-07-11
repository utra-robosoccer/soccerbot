#!/usr/bin/python3

import numpy as np
import math
import tf
import rospy
from std_msgs.msg import Empty
from soccer_msgs.msg import GameState
from soccer_strategy.src.robot_ros import RobotRos
from soccer_strategy.src.robot import Robot
from soccer_strategy.src.ball import Ball
import game_engine
from soccer_strategy.src.strategy.dummyStrategy import DummyStrategy
from soccer_strategy.src.strategy.freekickStrategy import FreekickStrategy
from soccer_strategy.src.strategy.penaltykickStrategy import PenaltykickStrategy
import config
import logging

logger = logging.getLogger('game_controller')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)
robot_name_map = ["robot1", "robot2", "robot3", "robot4"]


class GameEngineCompetition(game_engine.GameEngine):
    STRATEGY_UPDATE_INTERVAL = 1
    NAV_GOAL_UPDATE_INTERVAL = 2
    # position when ready
    blue_initial_position = [[-3.5, 0, 0], [-1, 0, 0], [-1.5, 1.75, 0]]
    red_initial_position = [[3.5, 0, -math.pi], [1, -2.5, -math.pi], [1.5, 1.75, -math.pi]]

    # start positino outisde the field
    blue_start_position = [[-4, -3.6, 1.57], [-1, -3.6, 1.57], [-4, 3.6, -1.57]]
    red_start_position = [[4, -3.6, 1.57], [1, -3.6, 1.57], [4, 3.6, -1.57]]


    GameStateMap = ["GAMESTATE_INITIAL", "GAMESTATE_READY", "GAMESTATE_SET", "GAMESTATE_PLAYING", "GAMESTATE_FINISHED"]
    SecondaryStateModeMap = ["PREPARATION", "PLACING", "END"]
    SecondaryGameStateMap = ["STATE_NORMAL",
                             "STATE_PENALTYSHOOT",
                             "STATE_OVERTIME",
                             "STATE_TIMEOUT",
                             "STATE_DIRECT_FREEKICK",
                             "STATE_INDIRECT_FREEKICK",
                             "STATE_PENALTYKICK",
                             "STATE_CORNER_KICK",
                             "STATE_GOAL_KICK",
                             "STATE_THROW_IN"
                             ]

    def __init__(self):
        print("initializing strategy")
        # self.robot_id = os.getenv("ROBOCUP_ROBOT_ID", 1)
        # self.robot_name = os.getenv("ROBOT_NAME", "robot1")
        # self.is_goal_keeper = os.getenv("GOALIE", "true") == "true"
        # self.robot_name = int(os.getenv("TEAM_ID", "16"))
        self.robot_id = int(rospy.get_param('ROBOCUP_ROBOT_ID'))
        self.robot_name = rospy.get_param('ROBOT_NAME')
        self.is_goal_keeper = bool(rospy.get_param('GOALIE'))
        self.team_id = int(rospy.get_param("TEAM_ID"))
        # game strategy information

        self.robots = [
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY, robot_name="robot1"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY, robot_name="robot2"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY, robot_name="robot3"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY, robot_name="opponent1"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY, robot_name="opponent2"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY, robot_name="opponent3"),
        ]

        self.friendly = self.robots[0:3]

        self.this_robot = None
        for robot in self.robots:
            if robot.robot_name == self.robot_name:
                self.this_robot = robot
        # assert (self.this_robot is not None, "This robot name doesn't exist: " + self.robot_name)

        self.ball = Ball(position=None)
        self.ball.position_timeout = True

        # GameState
        self.gameState = GameState()
        self.gameState.teamColor = GameState.TEAM_COLOR_BLUE
        self.gameState.gameState = GameState.GAMESTATE_FINISHED
        self.gameState.secondaryState = GameState.STATE_NORMAL
        self.gameState.firstHalf = True
        self.gameState.ownScore = 0
        self.gameState.rivalScore = 0
        self.gameState.secondsRemaining = 0
        self.gameState.secondary_seconds_remaining = 0
        self.gameState.hasKickOff = GameState.TEAM_COLOR_BLUE
        self.gameState.penalized = False
        self.gameState.secondsTillUnpenalized = 0

        self.previous_gameState = self.gameState
        self.previous_gameState.gameState = GameState.GAMESTATE_FINISHED

        self.game_state_subscriber = rospy.Subscriber('gamestate', GameState, self.gamestate_callback)
        self.execute_game_interruption_publisher = rospy.Publisher('execute_game_interruption', Empty, queue_size=1)

        self.rostime_previous = 0
        self.listener = tf.TransformListener()
        self.team1_strategy = DummyStrategy()
        self.team2_strategy = DummyStrategy()
        self.freekick_strategy = FreekickStrategy()
        self.penaltykick_strategy = PenaltykickStrategy()

        self.rostime_kickoff = 0
        self.kickoff_started = False

    def gamestate_callback(self, gameState):
        # log on state transition
        if self.gameState.gameState != self.previous_gameState.gameState:
            print(" - Gamestate transition to " + self.GameStateMap[self.gameState.gameState])
        if self.previous_gameState.secondaryState != self.gameState.secondaryState:
            print(" -- SecondaryGamestate transition to " + self.SecondaryGameStateMap[self.gameState.secondaryState])
        if self.previous_gameState.secondaryStateMode != self.gameState.secondaryStateMode:
            print(" --- SecondaryGameMode transition to " + self.SecondaryStateModeMap[self.gameState.secondaryStateMode])

        self.previous_gameState = self.gameState
        self.gameState = gameState


    def update_average_ball_position(self):
        # get estimated ball position with tf information from 4 robots and average them
        # this needs to be team-dependent in the future
        ball_positions = []
        for robot in self.friendly:
            time_diff = rospy.Duration(10)
            try:
                ball_pose = self.listener.lookupTransform('world', robot.robot_name + '/ball', rospy.Time(0))
                header = self.listener.getLatestCommonTime('world', robot.robot_name + '/ball')
                time_diff = rospy.Time.now() - header
                if time_diff < rospy.Duration(0.2):
                    ball_positions.append(np.array([-ball_pose[0][1], ball_pose[0][0]]))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        if ball_positions:
            self.ball.position = np.array(ball_positions).mean(axis=0)
            self.ball.position_is_live_timeout = 10
        else:
            if self.ball.position_is_live_timeout > 0:
                self.ball.position_is_live_timeout = self.ball.position_is_live_timeout - 1
            elif self.ball.position_is_live_timeout == 0:
                rospy.loginfo_throttle(5, "Ball Position Timed Out")

    def stop_all_robot(self):
        for robot in self.friendly:
            robot.stop_requested = True
            print(robot.robot_name + " set to forbid moving")

    def resume_all_robot(self):
        for robot in self.friendly:
            robot.completed_trajectory_publisher.publish(True)
            if robot.stop_requested:
                robot.stop_requested = False
            if robot.status == Robot.Status.STOPPED:
                robot.status = Robot.Status.READY
                print(robot.robot_name + " set to allow moving")

    # run loop
    def run(self):
        while not rospy.is_shutdown():
            rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9

            # normal game
            if self.gameState.secondaryState == GameState.STATE_NORMAL:
                self.run_normal(rostime)

            # free kick
            elif self.gameState.secondaryState == GameState.STATE_DIRECT_FREEKICK:
                self.run_freekick(rostime, self.freekick_strategy)
            elif self.gameState.secondaryState == GameState.STATE_INDIRECT_FREEKICK:
                self.run_freekick(rostime, self.freekick_strategy)
            elif self.gameState.secondaryState == GameState.STATE_CORNER_KICK:
                self.run_freekick(rostime, self.freekick_strategy)
            elif self.gameState.secondaryState == GameState.STATE_GOAL_KICK:
                self.run_freekick(rostime, self.freekick_strategy)
            elif self.gameState.secondaryState == GameState.STATE_THROW_IN:
                self.run_freekick(rostime, self.freekick_strategy)

            # penalty kick
            elif self.gameState.secondaryState == GameState.STATE_PENALTYKICK:
                # reset initial position for goalie
                # if first time pentalty kick state and we are not kicking
                if self.previous_gameState.secondaryState != GameState.STATE_PENALTYKICK and self.gameState.secondaryStateTeam != self.team_id:
                    for robot in self.friendly:
                        if robot.role == Robot.Role.GOALIE:
                            robot.reset_initial_position(
                                config.position_map(config.PENALTYKICK_NON_KICKING_POSITION, self.gameState.teamColor, self.gameState.firstHalf, 1)
                            )
                self.run_freekick(rostime, self.penaltykick_strategy)

            # penalty shootout
            elif self.gameState.secondaryState == GameState.STATE_PENALTYSHOOT:
                # if we have kickoff, run normal game strategy
                if self.gameState.hasKickOff:
                    self.run_normal(rostime)


    def run_normal(self, rostime):
        # INITIAL
        if self.gameState.gameState == GameState.GAMESTATE_INITIAL:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_INITIAL:
                # self.stop_all_robot()
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_INITIAL

                # reset localization initial pose
                for robot in self.friendly:
                    if self.gameState.secondaryState == GameState.STATE_PENALTYSHOOT:
                        robot.reset_initial_position(
                            config.position_map_kickoff(config.PENALTYSHOOT_START_POSITION, self.gameState.hasKickOff)
                        )
                    else:
                        robot.reset_initial_position(
                            config.position_map(config.START_POSITION, self.gameState.teamColor, self.gameState.firstHalf, robot.robot_id)
                        )

        # READY
        if self.gameState.gameState == GameState.GAMESTATE_READY:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_READY:
                # rospy.sleep(1)
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_READY

            if rostime % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL < self.rostime_previous % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL:
                for robot in self.friendly:
                    if robot.status == Robot.Status.READY:
                        robot.set_navigation_position(
                            config.position_map(config.INITIAL_POSITION, self.gameState.teamColor, self.gameState.firstHalf, robot.robot_id)
                        )

        # SET
        if self.gameState.gameState == GameState.GAMESTATE_SET:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_SET:
                self.stop_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_SET


        # PLAYING
        if self.gameState.gameState == GameState.GAMESTATE_PLAYING:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_PLAYING:
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_PLAYING

                self.rostime_kickoff = rostime
                if self.gameState.hasKickOff:
                    self.kickoff_started = True
                    print("we have kickoff")
                else:
                    self.kickoff_started = False
                    print("opponent have kickoff")

            if self.kickoff_started == False:
                if (rostime - self.rostime_kickoff) > 10:
                    self.kickoff_started = True
                    print("kickoff started after 10s")

                # if self.ball.get_position() is not None:
                #     delta = np.linalg.norm(self.ball.get_position())
                #     if delta > 0.1:
                #         self.kickoff_started = True
                #         print("kickoff started after ball moved 10cm")

            if rostime % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL < self.rostime_previous % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL:
                if self.kickoff_started:
                    self.team1_strategy.update_team_strategy(robots=self.robots, ball=self.ball,
                                                             teamcolor=self.gameState.teamColor,
                                                             is_first_half=self.gameState.firstHalf,
                                                             secondaryState=self.gameState.secondaryState)
                    pass
                else:
                    print("kickoff not started, no strategy output")

            pass

        # FINISHED
        if self.gameState.gameState == GameState.GAMESTATE_FINISHED:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_FINISHED:
                self.stop_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_FINISHED
            pass

        self.rostime_previous = rostime
        self.update_average_ball_position()
        for robot in self.friendly:
            robot.update_status()
            robot.get_detected_obstacles()
            robot.obstacles_publisher.publish(robot.obstacles)

    #todo clean up logic
    def run_freekick(self, rostime, strategy):
        # PREPARATION
        if self.gameState.secondaryStateMode == GameState.MODE_PREPARATION:
            if self.gameState.secondaryStateTeam == self.team_id:
                # first time running freekick logic
                if self.previous_gameState.secondaryStateMode == GameState.STATE_NORMAL:
                    self.designate_kicker()

                if rostime % self.NAV_GOAL_UPDATE_INTERVAL < self.rostime_previous % self.NAV_GOAL_UPDATE_INTERVAL:
                    completed = strategy.update_kicking_strategy(self.friendly, self.ball)
                    if completed:
                        self.execute_game_interruption_publisher.publish()

            if self.gameState.secondaryState != self.team_id:
                if rostime % self.NAV_GOAL_UPDATE_INTERVAL < self.rostime_previous % self.NAV_GOAL_UPDATE_INTERVAL:
                    strategy.update_non_kicking_strategy(self.friendly, self.ball, self.gameState.teamColor, self.gameState.firstHalf)

        # PLACING
        if self.gameState.secondaryStateMode == GameState.MODE_PLACING:
            if self.gameState.secondaryStateTeam == self.team_id:
                if rostime % self.NAV_GOAL_UPDATE_INTERVAL < self.rostime_previous % self.NAV_GOAL_UPDATE_INTERVAL:
                    strategy.execute_kicking(self.friendly, self.ball)
            else:
                # todo perform goalie trajectory if penalty kick
                strategy.update_non_kicking_strategy(self.freekick_strategy, self.ball, self.gameState.teamColor, self.gameState.firstHalf)
                pass

        self.rostime_previous = rostime
        self.update_average_ball_position()
        for robot in self.friendly:
            robot.update_status()

    def designate_kicker(self):
        closest_dist = math.inf
        current_closest = None
        for robot in self.friendly:
            dist = np.linalg.norm(self.ball.get_position()[0:2] - robot.get_position()[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        current_closest.designated_kicker = True


