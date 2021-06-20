#!/usr/bin/python3
import os

import numpy as np
import math
import rospy
from std_msgs.msg import Bool
from soccer_msgs.msg import GameState
from robot_ros import RobotRos
from robot import Robot
from ball import Ball
import game_engine
from strategy import DummyStrategy
import tf
import logging

logger = logging.getLogger('game_controller')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)
robot_name_map = ["robot1", "robot2", "robot3", "robot4"]


class GameEngineCompetition(game_engine.GameEngine):
    STRATEGY_UPDATE_INTERVAL = 5
    blue_initial_position = [[0, -2.6, 0], [1.5, 1.5, 0], [-1.5, 1.5, 0], [0, 1, 0]]
    red_initial_position = [[0, -2.6, 0], [1.5, -1.5, 0], [-1.5, -1.5, 0], [0, -1, 0]]

    def __init__(self):
        print("initializing strategy")
        # self.robot_id = os.getenv("ROBOCUP_ROBOT_ID", 1)
        # self.robot_name = os.getenv("ROBOT_NAME", "robot1")
        # self.is_goal_keeper = os.getenv("GOALIE", "true") == "true"
        self.robot_id = int(rospy.get_param('ROBOCUP_ROBOT_ID'))
        self.robot_name = rospy.get_param('ROBOT_NAME')
        self.is_goal_keeper = bool(rospy.get_param('GOALIE'))
        # game strategy information
        if self.is_goal_keeper:
            # the robot that run strategy will always be the first one in self.robots
            self.friendly = [
                RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.STOPPED,

                         robot_name=self.robot_name)
            ]
        else:
            self.friendly = [
                RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.STRIKER, status=Robot.Status.STOPPED,
                         robot_name=self.robot_name)
            ]

        self.robots = [
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY, robot_name="robot1"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY, robot_name="robot2"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY, robot_name="robot3"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.STRIKER, status=Robot.Status.READY, robot_name="robot4"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY, robot_name="opponent1"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY, robot_name="opponent2"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY, robot_name="opponent3"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.STRIKER, status=Robot.Status.READY, robot_name="opponent4")
        ]

        self.this_robot = None
        for robot in self.robots:
            if robot.robot_name == self.robot_name:
                self.this_robot = robot
        # assert (self.this_robot is not None, "This robot name doesn't exist: " + self.robot_name)

        self.ball = Ball(position=np.array([0, 0]))

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

        self.rostime_previous = 0
        self.team1_strategy = DummyStrategy()
        self.team2_strategy = DummyStrategy()
        # Setup the strategy
        self.opponent = []
        self.listener = tf.TransformListener()


    def gamestate_callback(self, gameState):
        self.previous_gameState = self.gameState
        self.gameState = gameState

    def update_average_ball_position(self):
        # get estimated ball position with tf information from 4 robots and average them
        # this needs to be team-dependent in the future
        ball_positions = []
        for robot in self.friendly:
            if robot.ball_position.all():
                ball_positions.append(robot.ball_position)

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
            if self.gameState.secondaryState == GameState.STATE_NORMAL:
                self.run_normal()
            if self.gameState.secondaryState == GameState.STATE_DIRECT_FREEKICK:
                self.run_freekick()
            if self.gameState.secondaryState == GameState.STATE_PENALTYSHOOT:
                if self.gameState.hasKickOff:
                    self.run_normal()
    temp_bool = True

    def run_normal(self):
        rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9

        if self.gameState.gameState != self.previous_gameState.gameState:
            if self.gameState.gameState == GameState.GAMESTATE_INITIAL:
                new_state = "GAMESTATE_INITIAL"
            if self.gameState.gameState == GameState.GAMESTATE_READY:
                new_state = "GAMESTATE_READY"
            if self.gameState.gameState == GameState.GAMESTATE_SET:
                new_state = "GAMESTATE_SET"
            if self.gameState.gameState == GameState.GAMESTATE_PLAYING:
                new_state = "GAMESTATE_PLAYING"
            if self.gameState.gameState == GameState.GAMESTATE_FINISHED:
                new_state = "GAMESTATE_FINISHED"
            print(" Gamestate transition to "+ new_state)

        # INITIAL
        if self.gameState.gameState == GameState.GAMESTATE_INITIAL:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_INITIAL:
                # self.stop_all_robot()
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_INITIAL

        # READY
        if self.gameState.gameState == GameState.GAMESTATE_READY:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_READY:
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_READY

            if rostime % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL < self.rostime_previous % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL:
                for robot in self.friendly:
                    if robot.status == Robot.Status.READY:
                        robot.status = Robot.Status.WALKING
                        if self.gameState.teamColor == GameState.TEAM_COLOR_BLUE:
                            robot.set_navigation_position(self.blue_initial_position[robot.robot_id - 1])
                        else:
                            print("Here", self.gameState.teamColor)
                            robot.set_navigation_position(self.red_initial_position[robot.robot_id - 1])
            self.rostime_previous = rostime

        # SET
        if self.gameState.gameState == GameState.GAMESTATE_SET:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_SET:
                self.stop_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_SET

        # PLAYING
        if self.gameState.gameState == GameState.GAMESTATE_PLAYING:

            if self.previous_gameState.gameState != GameState.GAMESTATE_READY:
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_READY

            if rostime % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL < \
                    self.rostime_previous % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL:
                try:

                    self.ball_pose = self.listener.lookupTransform(robot_name_map[self.robot_id - 1] + '/ball',
                                                                   robot_name_map[self.robot_id - 1] + '/torso',
                                                                   rospy.Time(0))
                    # print(self.ball_pose)
                    header = self.listener.getLatestCommonTime(robot_name_map[self.robot_id - 1] + '/ball',
                                                               robot_name_map[self.robot_id - 1] + '/torso')
                    self.last_pose = rospy.Time.now() - header

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
                if self.last_pose < rospy.Duration(0.2):
                    self.strategy.update_next_strategy(self.friendly, self.opponent, self.ball)
                    self.team1_strategy.update_friendly_strategy(self.robots, self.ball)
            self.rostime_previous = rostime


        # PLAYING
        if self.gameState.gameState == GameState.GAMESTATE_PLAYING:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_PLAYING:
                self.resume_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_PLAYING

            if rostime % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL < \
                    self.rostime_previous % GameEngineCompetition.STRATEGY_UPDATE_INTERVAL:
                self.team1_strategy.update_friendly_strategy(self.robots, self.ball)
            self.rostime_previous = rostime
            pass

        # FINISHED
        if self.gameState.gameState == GameState.GAMESTATE_FINISHED:
            # on state transition
            if self.previous_gameState.gameState != GameState.GAMESTATE_FINISHED:
                self.stop_all_robot()
                self.previous_gameState.gameState = GameState.GAMESTATE_FINISHED
            pass

        self.update_average_ball_position()
        for robot in self.friendly:
            robot.update_status()

    def run_freekick(self):

        pass
