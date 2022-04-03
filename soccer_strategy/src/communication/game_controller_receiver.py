#!/usr/bin/python3

import os
import socket

import rospy
from construct import ConstError, Container
from gamestate import GAME_CONTROLLER_RESPONSE_VERSION, GameState, ReturnData
from std_msgs.msg import Empty

from soccer_msgs.msg import GameState as GameStateMsg


class GameStateReceiver(object):
    team_id = int(os.getenv("ROBOCUP_TEAM_ID", 16))
    robot_id = int(os.getenv("ROBOCUP_ROBOT_ID", 2))

    DEFAULT_LISTENING_HOST = "0.0.0.0"
    GAME_CONTROLLER_LISTEN_PORT = 3838
    GAME_CONTROLLER_ANSWER_PORT = 3939

    def __init__(self):
        rospy.loginfo("Listening to " + str(self.GAME_CONTROLLER_LISTEN_PORT) + " " + str(self.GAME_CONTROLLER_ANSWER_PORT))
        rospy.loginfo("We are playing as player {} in team {}".format(self.robot_id, self.team_id))

        self.state_publisher = rospy.Publisher("gamestate", GameStateMsg, queue_size=1)
        self.execute_game_interruption_subscriber = rospy.Subscriber("execute_game_interruption", Empty, self.game_interruption_callback)
        self.execute_game_interruption = False

        self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.receiver_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
        self.receiver_socket.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton("237.252.249.227") + socket.inet_aton("0.0.0.0"),
        )
        self.receiver_socket.settimeout(2)
        self.receiver_socket.bind((self.DEFAULT_LISTENING_HOST, self.GAME_CONTROLLER_LISTEN_PORT))
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

    def receive_forever(self):
        while not rospy.is_shutdown():
            try:
                data, peer = self.receiver_socket.recvfrom(GameState.sizeof())
                self.on_new_gamestate(GameState.parse(data))
                self.answer_to_gamecontroller(peer)
                rospy.loginfo_once("\033[96mConnected to Game Controller\033[0m")

            except AssertionError as ae:
                rospy.logerr_throttle(10, ae)
            except socket.timeout as s:
                rospy.logerr_throttle(10, "Socket Timeout, rebinding socket: " + str(s))
                self.receiver_socket.close()
                self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                self.receiver_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
                self.receiver_socket.setsockopt(
                    socket.IPPROTO_IP,
                    socket.IP_ADD_MEMBERSHIP,
                    socket.inet_aton("237.252.249.227") + socket.inet_aton("0.0.0.0"),
                )
                self.receiver_socket.settimeout(2)
                self.receiver_socket.bind((self.DEFAULT_LISTENING_HOST, self.GAME_CONTROLLER_LISTEN_PORT))
            except ConstError as c:
                rospy.logwarn_throttle(10, c)
            except Exception as e:
                rospy.logerr_throttle(10, "Error")
                rospy.logerr(e)
        self.receiver_socket.close()
        self.send_socket.close()

    def answer_to_gamecontroller(self, peer):
        return_message = 2

        if self.execute_game_interruption:
            return_message = 4

        data = Container(
            header=b"RGrt",
            version=GAME_CONTROLLER_RESPONSE_VERSION,
            team=self.team_id,
            player=self.robot_id,
            message=return_message,
        )

        destination = peer[0], self.GAME_CONTROLLER_ANSWER_PORT
        self.send_socket.sendto(ReturnData.build(data), destination)

    def on_new_gamestate(self, state):
        if state.teams[0].team_number == self.team_id:
            own_team = state.teams[0]
            rival_team = state.teams[1]
        elif state.teams[1].team_number == self.team_id:
            own_team = state.teams[1]
            rival_team = state.teams[0]
        else:
            rospy.logerr("Team {} not playing, only {} and {}".format(self.team_id, state.teams[0].team_number, state.teams[1].team_number))
            return

        try:
            me = own_team.players[self.robot_id - 1]
        except IndexError:
            rospy.logerr("Robot {} not playing".format(self.robot_id))
            return

        # reset execute game interruption if gamestate back to normal
        if state.secondary_state == GameState.secondary_state.STATE_NORMAL and self.execute_game_interruption:
            self.execute_game_interruption = False

        msg = GameStateMsg()
        msg.header.stamp = rospy.Time.now()
        msg.gameState = state.game_state.intvalue
        msg.secondaryState = state.secondary_state.intvalue
        msg.firstHalf = state.first_half
        msg.ownScore = own_team.score
        msg.rivalScore = rival_team.score
        msg.secondsRemaining = state.seconds_remaining
        msg.secondary_seconds_remaining = state.secondary_seconds_remaining
        msg.hasKickOff = state.kick_of_team == self.team_id
        msg.penalty = me.penalty
        msg.secondsTillUnpenalized = me.secs_till_unpenalized

        msg.secondaryStateTeam = state.secondary_state_info[0]
        msg.secondaryStateMode = state.secondary_state_info[1]
        msg.teamColor = own_team.team_color.intvalue
        msg.dropInTeam = state.drop_in_team
        msg.dropInTime = state.drop_in_time
        msg.penaltyShot = own_team.penalty_shot
        msg.singleShots = own_team.single_shots
        msg.coach_message = own_team.coach_message
        self.state_publisher.publish(msg)

    def game_interruption_callback(self, data):
        self.execute_game_interruption = True


if __name__ == "__main__":
    rospy.init_node("game_controller")
    rec = GameStateReceiver()
    rec.receive_forever()
