#!/usr/bin/env python
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
import argparse
import sys

# Requires construct==2.5.3
from construct import Container, ConstError
from gamestate import GameState, ReturnData, GAME_CONTROLLER_RESPONSE_VERSION

logger = logging.getLogger('game_controller')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)

DEFAULT_LISTENING_HOST = '0.0.0.0'
GAME_CONTROLLER_LISTEN_PORT = 3838
GAME_CONTROLLER_ANSWER_PORT = 3939

parser = argparse.ArgumentParser()
parser.add_argument('--team', type=int, default=1, help="team ID, default is 1")
parser.add_argument('--player', type=int, default=1, help="player ID, default is 1")
parser.add_argument('--goalkeeper', action="store_true", help="if this flag is present, the player takes the role of the goalkeeper")


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
        self.man_penalize = True
        self.is_goalkeeper = is_goalkeeper

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
        while self.running:
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

            print(len(data))
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
            logger.exception(e)
            pass

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
        """ Is called with the new game state after receiving a package
            Needs to be implemented or set
            :param state: Game State
        """
        raise NotImplementedError()

    def get_last_state(self):
        return self.state, self.time

    def get_time_since_last_package(self):
        return time.time() - self.time

    def stop(self):
        self.running = False

    def set_manual_penalty(self, flag):
        self.man_penalize = flag


class SampleGameStateReceiver(GameStateReceiver):

    def on_new_gamestate(self, state):
       print(state)
       print(state.secondary_state_info)


if __name__ == '__main__':
    args = parser.parse_args(sys.argv[1:])
    rec = SampleGameStateReceiver(team=args.team, player=args.player, is_goalkeeper=args.goalkeeper)
    rec.receive_once()
    rec.answer_to_gamecontroller()