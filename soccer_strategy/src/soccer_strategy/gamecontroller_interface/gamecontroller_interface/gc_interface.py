import os

import rclpy
from gc_client import GameControllerClient
from rclpy.node import Node

from soccer_msgs.msg import GameState as GameStateMsg


class GameControllerInterface(Node):
    def __init__(self):
        super().__init__("gamecontroller_interface")

        self.gc = GameControllerClient()
        self.team_id = int(os.getenv("ROBOCUP_TEAM_ID", 10))
        self.robot_id = self.declare_parameter("robot_id", 1).get_parameter_value().integer_value
        self.pub = self.create_publisher(GameStateMsg, "gamestate", 10)

        self.create_timer(0.5, self.receive_and_publish)

    def receive_and_publish(self):
        try:
            state = self.gc.receive_once()
            teams = state.teams
            own_team = None
            rival_team = None

            if teams[0].team_number == self.team_id:
                own_team, rival_team = teams[0], teams[1]
            elif teams[1].team_number == self.team_id:
                own_team, rival_team = teams[1], teams[0]
            else:
                self.get_logger().error(f"Team {self.team_id} not playing")
                return

            try:
                me = own_team.players[self.robot_id - 1]
            except IndexError:
                self.get_logger().error(f"Robot {self.robot_id} not listed in GameController packet")
                return

            msg = GameStateMsg()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.gamestate = state.game_state.intvalue
            msg.secondary_state = state.secondary_state.intvalue
            msg.first_half = state.first_half
            msg.own_score = own_team.score
            msg.rival_score = rival_team.score
            msg.seconds_remaining = state.seconds_remaining
            msg.secondary_seconds_remaining = state.secondary_seconds_remaining
            msg.has_kick_off = state.kick_of_team == self.team_id
            msg.penalty = me.penalty
            msg.seconds_till_unpenalized = me.secs_till_unpenalized
            msg.secondary_state_team = state.secondary_state_info[0]
            msg.secondary_state_mode = state.secondary_state_info[1]
            msg.team_color = own_team.team_color.intvalue
            msg.drop_in_team = state.drop_in_team
            msg.drop_in_time = state.drop_in_time
            msg.penalty_shot = own_team.penalty_shot
            msg.single_shots = own_team.single_shots
            msg.coach_message = own_team.coach_message

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"GC receive error: {e}", throttle_duration_sec=5.0)
