#!/usr/bin/python3
import os

import numpy as np
import math
import tf
import rospy
from std_msgs.msg import Empty
from soccer_msgs.msg import GameState
from robot_ros import RobotRos
from robot import Robot
from ball import Ball
import game_engine
from strategy import DummyStrategy, FreekickStrategy

import logging

logger = logging.getLogger('game_controller')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)
robot_name_map = ["robot1", "robot2", "robot3", "robot4"]


class GameEngineCompetition(game_engine.GameEngine):
    STRATEGY_UPDATE_INTERVAL = 3
    NAV_GOAL_UPDATE_INTERVAL = 2
    blue_initial_position = [[-0.9, -3.9, 0], [-0.9, -0.9, 0], [-1, 0, 0]]
    red_initial_position = [[-0.8, 3.9, 3.14], [-0.8, 0.9, 3.14], [-1, 0, 3.14]]

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
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY, robot_name="robot1"),
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

        # GameState
        self.gameState = GameState()
        self.gameState.teamColor = GameState.TEAM_COLOR_BLUE
        self.gameState.gameState = GameState.GAMESTATE_INITIAL
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
        self.freekick_strategy = FreekickStrategy(False)
        self.penaltykick_strategy = FreekickStrategy(True)

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
                ball_pose = self.listener.lookupTransform('world',
                                                          robot.robot_name + '/ball',
                                                               rospy.Time(0))
                header = self.listener.getLatestCommonTime('world',
                                                           robot.robot_name + '/ball')
                time_diff = rospy.Time.now() - header
                if time_diff < rospy.Duration(0.2):
                    ball_positions.append(np.array([-ball_pose[0][1], ball_pose[0][0]]))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        if ball_positions:
            self.ball.position = np.array(ball_positions).mean(axis=0)

    def stop_all_robot(self):
        for robot in self.friendly:
            robot.stop_requested = True
            print(robot.robot_name + " set to forbid moving")

    def resume_all_robot(self):
        for robot in self.friendly:
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

            self.team1_strategy.update_friendly_strategy(self.robots, self.ball)
            rospy.sleep(0.5)

        # READY
        if self.gameState.gameState == GameState.GAMESTATE_READY:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_READY:
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_READY

            if rostime % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL < self.rostime_previous % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL:
                for robot in self.friendly:
                    if robot.status == Robot.Status.READY:
                        print(self.gameState.teamColor)
                        if self.gameState.teamColor == GameState.TEAM_COLOR_BLUE:
                            robot.set_navigation_position(self.blue_initial_position[robot.robot_id - 1])
                        elif self.gameState.teamColor == GameState.TEAM_COLOR_RED:
                            robot.set_navigation_position(self.red_initial_position[robot.robot_id - 1])

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

            if rostime % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL < \
                    self.rostime_previous % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL:

                self.team1_strategy.update_friendly_strategy(self.robots, self.ball)
                rospy.sleep(0.5)
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

            if self.gameState.secondaryState !=self.team_id:
                if rostime % self.NAV_GOAL_UPDATE_INTERVAL < self.rostime_previous % self.NAV_GOAL_UPDATE_INTERVAL:
                    strategy.update_non_kicking_strategy(self.friendly, self.ball)

        # PLACING
        if self.gameState.secondaryStateMode == GameState.MODE_PLACING:
            if self.gameState.secondaryStateTeam == self.team_id:
                if rostime % self.NAV_GOAL_UPDATE_INTERVAL < self.rostime_previous % self.NAV_GOAL_UPDATE_INTERVAL:
                    strategy.execute_kicking(self.friendly, self.ball)
            else:
                # todo perform goalie trajectory if penalty kick
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


