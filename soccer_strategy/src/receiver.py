#!/usr/bin/python3

#-*- coding:utf-8 -*-

from __future__ import unicode_literals, print_function

"""
This module shows how the GameController Communication protocol can be used
in python and also allows to be changed such that every team using python to
interface with the GC can utilize the new protocol.

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>
.. moduleauthor:: Robert Kessler <8kessler@informatik.uni-hamburg.de>

"""


import socket
import time
import logging
import rospy
from std_msgs.msg import Bool, Int8
import os

# Requires construct==2.5.3
from soccer_msgs.msg import GameState as GameStateMsg
from construct import Container, ConstError
from gamestate import GameState, ReturnData, GAME_CONTROLLER_RESPONSE_VERSION

logger = logging.getLogger('game_controller')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)

#DEFAULT_LISTENING_HOST = '127.0.1.1'
DEFAULT_LISTENING_HOST = os.environ.get('ROBOCUP_GAMECONTROLLER_IP')
GAME_CONTROLLER_LISTEN_PORT = 3838
GAME_CONTROLLER_ANSWER_PORT = 3939

class GameStateReceiver(object):
    """ This class puts up a simple UDP Server which receives the
    *addr* parameter to listen to the packages from the game_controller.

    If it receives a package it will be interpreted with the construct data
    structure and the :func:`on_new_gamestate` will be called with the content.

    After this we send a package back to the GC """

    def __init__(self, team, player, is_goalkeeper, addr=(DEFAULT_LISTENING_HOST, GAME_CONTROLLER_LISTEN_PORT), answer_port=GAME_CONTROLLER_ANSWER_PORT):

        # Information that is used when sending the answer to the game controller
        self.team = team
        self.player = player
        self.is_goalkeeper = is_goalkeeper

        self.man_penalize = True
        self.game_controller_lost_time = 20
        self.game_controller_connected_publisher = rospy.Publisher('game_controller_connected', Bool, queue_size=1)

        rospy.loginfo('We are playing as player {} in team {}'.format(self.player, self.team))
        self.state_publisher = rospy.Publisher('gamestate', GameStateMsg, queue_size=1)

        # The address listening on and the port for sending back the robots meta data
        self.addr = addr
        self.answer_port = answer_port

        # The state and time we received last form the GC
        self.state = None
        self.time = None

        # The socket and whether it is still running
        self.socket = None
        self.running = True

        self._open_socket()

    def _open_socket(self):
        """ Erzeugt das Socket """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.addr)
        self.socket.settimeout(0.5)
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def receive_forever(self):
        """ Waits in a loop that is terminated by setting self.running = False """
        while self.running and not rospy.is_shutdown():
            try:
                self.receive_once()
            except IOError as e:
                logger.debug("Fehler beim Senden des KeepAlive: " + str(e))

    def receive_once(self):
        """ Receives a package and interprets it.
            Calls :func:`on_new_gamestate`
            Sends an answer to the GC """
        try:
            data, peer = self.socket.recvfrom(GameState.sizeof())

            #print(len(data))
            # Throws a ConstError if it doesn't work
            parsed_state = GameState.parse(data)

            # Assign the new package after it parsed successful to the state
            self.state = parsed_state
            self.time = time.time()

            # Call the handler for the package
            self.on_new_gamestate(self.state)

            # Answer the GameController
            self.answer_to_gamecontroller(peer)

        except AssertionError as ae:
            logger.error(ae.message)
        except socket.timeout:
            logger.warning("Socket timeout")
        except ConstError:
            logger.warning("Parse Error: Probably using an old protocol!")
        except Exception as e:
            if self.get_time_since_last_package() > self.game_controller_lost_time:
                self.time += 5  # Resend message every five seconds
                rospy.logwarn_throttle(5.0, 'No game controller messages received, allowing robot to move')
                msg = GameStateMsg()
                msg.allowedToMove = True
                msg.gameState = 3  # PLAYING
                self.state_publisher.publish(msg)
                msg2 = Bool()
                msg2.data = False
                self.game_controller_connected_publisher.publish(msg2)

    def answer_to_gamecontroller(self, peer):
        """ Sends a life sign to the game controller """
        return_message = 0 if self.man_penalize else 2
        if self.is_goalkeeper:
            return_message = 3

        data = Container(
            header=b"RGrt",
            version=GAME_CONTROLLER_RESPONSE_VERSION,
            team=self.team,
            player=self.player,
            message=return_message)
        try:
            destination = peer[0], GAME_CONTROLLER_ANSWER_PORT
            self.socket.sendto(ReturnData.build(data), destination)
        except Exception as e:
            logger.log("Network Error: %s" % str(e))

    def on_new_gamestate(self, state):
        #print(state.game_state)
        """ Is called with the new game state after receiving a package.
            The information is processed and published as a standard message to a ROS topic.
            :param state: Game State
        """
        if state.teams[0].team_number == self.team:
            own_team = state.teams[0]
            rival_team = state.teams[1]
        elif state.teams[1].team_number == self.team:
            own_team = state.teams[1]
            rival_team = state.teams[0]
        else:
            rospy.logerr('Team {} not playing, only {} and {}'.format(self.team,
                                                                      state.teams[0].team_number,
                                                                      state.teams[1].team_number))
            return

        try:
            me = own_team.players[self.player - 1]
        except IndexError:
            rospy.logerr('Robot {} not playing'.format(self.player))
            return

        msg = GameStateMsg()
        msg.header.stamp = rospy.Time.now()
        msg.gameState = state.game_state.intvalue
        msg.secondaryState = state.secondary_state.intvalue
        msg.firstHalf = state.first_half
        msg.ownScore = own_team.score
        msg.rivalScore = rival_team.score
        msg.secondsRemaining = state.seconds_remaining
        msg.secondary_seconds_remaining = state.secondary_seconds_remaining
        msg.hasKickOff = state.kick_of_team == self.team
        msg.penalized = me.penalty != 0
        msg.secondsTillUnpenalized = me.secs_till_unpenalized

        if me.penalty != 0:
            msg.allowedToMove = False
        elif state.game_state in ('STATE_INITIAL', 'STATE_SET'):
            msg.allowedToMove = False
        elif state.game_state == 'STATE_READY':
            msg.allowedToMove = True
        elif state.game_state == 'STATE_PLAYING':
            if state.kick_of_team >= 128:
                # Drop ball
                msg.allowedToMove = True
            elif state.secondary_state in (
                    'STATE_DIRECT_FREEKICK',
                    'STATE_INDIRECT_FREEKICK',
                    'STATE_PENALTYKICK',
                    'STATE_CORNERKICK',
                    'STATE_GOALKICK',
                    'STATE_THROWIN'):
                if state.secondary_state_info[1] in (0, 2):
                    msg.allowedToMove = False
                else:
                    msg.allowedToMove = True
                msg.secondaryStateTeam = state.secondary_state_info[0]
            elif state.secondary_state == 'STATE_PENALTYSHOOT':
                # we have penalty kick
                if state.kick_of_team == self.team:
                    msg.allowedToMove = True
                else:
                    msg.allowedToMove = False
            elif state.kick_of_team == self.team:
                msg.allowedToMove = True
            else:
                # Other team has kickoff
                if msg.secondary_seconds_remaining != 0:
                    msg.allowedToMove = False
                else:
                    # We have waited the kickoff time
                    msg.allowedToMove = True

        msg.teamColor = own_team.team_color.intvalue
        msg.dropInTeam = state.drop_in_team
        msg.dropInTime = state.drop_in_time
        msg.penaltyShot = own_team.penalty_shot
        msg.singleShots = own_team.single_shots
        msg.coach_message = own_team.coach_message
        self.state_publisher.publish(msg)
        pub = rospy.Publisher("state", Int8, queue_size=1)  # black magic publisher
        pub.publish(state.game_state.intvalue)
        #print("publish state: "+ str(state.game_state) )

    def get_last_state(self):
        return self.state, self.time

    def get_time_since_last_package(self):
        return time.time() - self.time

    def stop(self):
        self.running = False

    def set_manual_penalty(self, flag):
        self.man_penalize = flag

if __name__ == '__main__':
    rospy.init_node('game_controller')

    team_id = rospy.get_param("team_id")
    robot_id = rospy.get_param("robot_id")

    is_goal_keeper = rospy.get_param("is_goal_keeper")
    print(team_id)
    print(robot_id)
    print(is_goal_keeper)
    rec = GameStateReceiver(team=team_id, player=robot_id, is_goalkeeper=is_goal_keeper)
    rec.receive_forever()