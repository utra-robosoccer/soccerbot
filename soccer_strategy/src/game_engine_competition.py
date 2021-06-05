#!/usr/bin/python3

import numpy as np

import rospy
from soccer_msgs.msg import GameState as GameStateMsg
import enum
from robot_ros import RobotRos
from robot import Robot
from ball import Ball

import logging

logger = logging.getLogger('game_controller')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)

class gameState(enum.IntEnum):
    GAMESTATE_INITIAL = 0
    GAMESTATE_READY = 1
    GAMESTATE_SET = 2
    GAMESTATE_PLAYING = 3
    GAMESTATE_FINISHED = 4


class secondaryState(enum.IntEnum):
    STATE_NORMAL = 0
    STATE_PENALTYSHOOT = 1
    STATE_OVERTIME = 2
    STATE_TIMEOUT = 3
    STATE_DIRECT_FREEKICK = 4
    STATE_INDIRECT_FREEKICK = 5
    STATE_PENALTYKICK = 6
    STATE_CORNER_KICK = 7
    STATE_GOAL_KICK = 8
    STATE_THROW_IN = 9


class Status(enum.IntEnum):
    READY = 1
    LOCALIZING = 2
    WALKING = 3
    KICKING = 4
    FALLEN_FRONT = 5
    FALLEN_BACK = 6
    PENALTY = 7
    OUT_OF_BOUNDS = 8
    TRAJECTORY_IN_PROGRESS = 9


class secondaryStateMode(enum.IntEnum):
    MODE_PREPARATION = 0
    MODE_PLACING = 1
    MODE_END = 2
    # The secondary state contains a sub mode in which phase of execution the secondary state is


class teamColor(enum.IntEnum):
    BLUE = 0
    RED = 1


blue_initial_position = [[3, 0, 0], [1.5, 1.5, 0], [1.5, -1.5, 0], [1, 0, 0]]
red_initial_position = [[-3, 0, 0], [-1.5, 1.5, 0], [-1.5, -1.5, 0], [-1, 0, 0]]


class GameStatus:
    def __init__(self):
        self.robot = RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.TERMINATE,
                              robot_name="robot1")
        self.ball = Ball(position=np.array([0, 0]))
        self.terminate = False #terminate all action

        self.teamColor = teamColor.BLUE
        self.robot_id = rospy.get_param("robot_id")
        self.gameState = gameState.GAMESTATE_INITIAL
        self.state_transition = False
        self.secondaryState = secondaryState.STATE_NORMAL
        self.firstHalf = True
        self.ownScore = 0
        self.rivalScore = 0
        self.secondsRemaining = 0
        self.secondary_seconds_remaining = 0
        self.hasKickOff = self.teamColor
        self.penalized = False
        self.secondsTillUnpenalized = 0
        self.allowedToMove = False

        self.gamecontroller_subscriber = rospy.Subscriber('gamestate', GameStateMsg, self.gamecontroller_callback)


    def gamecontroller_callback(self, data):
        new_gameState = gameState(data.gameState)
        if not new_gameState == self.gameState:
            self.state_transition = True
            print("--------state transition to " + str(new_gameState))
        self.gameState = new_gameState

        self.secondaryState = secondaryState(data.secondaryState)
        self.teamColor = teamColor(data.teamColor)
        self.firstHalf = data.firstHalf
        self.ownScore = data.ownScore
        self.rivalScore = data.rivalScore
        self.secondsRemaining = data.secondsRemaining
        self.secondary_seconds_remaining = data.secondary_seconds_remaining
        self.hasKickOff = data.hasKickOff
        self.penalized = data.penalized
        self.secondsTillUnpenalized = data.secondsTillUnpenalized
        self.allowedToMove = data.allowedToMove


    def update_average_ball_position(self):
        self.ball = self.robot.ball_position

    def run(self):
        while not rospy.is_shutdown():

            if self.state_transition:
                print(self.gameState)
                if self.allowedToMove:
                    self.terminate = False
                    self.robot.status = Robot.Status.READY
                    print("allow moving")
                else:
                    self.terminate = True
                    print("not allow moving")

                if self.gameState == gameState.GAMESTATE_READY:
                    if self.teamColor == teamColor.BLUE:
                        self.robot.set_navigation_position(blue_initial_position[self.robot_id-1])
                    elif self.teamColor == teamColor.RED:
                        self.robot.set_navigation_position(red_initial_position[self.robot_id-1])
                    else:
                        print("failed to get team color")

                if self.gameState == gameState.GAMESTATE_SET:
                    pass

                if self.gameState == gameState.GAMESTATE_PLAYING:
                    pass

                if self.gameState == gameState.GAMESTATE_FINISHED:
                    pass
                    #todo terminate

                self.state_transition = False

            self.basicRobotAI()


    def basicRobotAI(self):
        self.update_average_ball_position()

        if self.robot.status == Robot.Status.WALKING:
            if self.terminate:
                self.robot.terminate_walking_publisher.publish()
                self.robot.status = Robot.Status.TERMINATE

        elif self.robot.status == Robot.Status.KICKING:
            self.robot.trajectory_publisher.publish("rightkick")
            self.robot.trajectory_complete = False
            self.robot.status = Robot.Status.TRAJECTORY_IN_PROGRESS
            print("kicking")

        elif self.robot.status == Robot.Status.FALLEN_BACK:
            self.robot.terminate_walking_publisher.publish()
            self.robot.trajectory_publisher.publish("getupback")
            self.robot.trajectory_complete = False
            self.robot.status = Robot.Status.TRAJECTORY_IN_PROGRESS
            print("getupback")

        elif self.robot.status == Robot.Status.FALLEN_FRONT:
            self.robot.terminate_walking_publisher.publish()
            self.robot.trajectory_publisher.publish("getupfront")
            self.robot.trajectory_complete = False
            self.robot.status = Robot.Status.TRAJECTORY_IN_PROGRESS
            print("getupback")

        elif self.robot.status == Robot.Status.READY:
            if np.linalg.norm(self.robot.position[0:2] - self.robot.ball_position) < 0.2:
                self.robot.status = Robot.Status.KICKING
            else:
                self.robot.status = Robot.Status.WALKING
                self.robot.set_navigation_position(np.append(self.robot.ball_position, 0))

        elif self.robot.status == Robot.Status.TRAJECTORY_IN_PROGRESS:
            if self.robot.trajectory_complete:
                if not self.terminate:
                    self.robot.status = Robot.Status.READY
                else:
                    self.robot.status = Robot.Status.TERMINATE
            else:
                pass

        if self.robot.status != self.robot.previous_status:
            print(self.robot.robot_name + " status changes to " + str(self.robot.status))
            self.robot.previous_status = self.robot.status

