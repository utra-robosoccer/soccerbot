#!/usr/bin/env python3

import os
import socket
import struct
import time

import rclpy

from soccer_msgs.msg import RobotState
from soccer_strategy.communication import robot_state_pb2

"""
Sends information between robots using a mirror port
"""

if __name__ == "__main__":
    self.init_node("team_communication")
    self.get_logger().info("Initializing team_communication...", logger_name="team_comm")

    player_id = self.get_param("robot_id", 1)
    team_id = int(os.getenv("ROBOCUP_TEAM_ID", 16))
    mirror_server_ip = os.getenv("ROBOCUP_MIRROR_SERVER_IP", "127.0.0.1")

    s = None

    def robot_state_callback(robot_state: RobotState):
        m = robot_state_pb2.Message()
        m.timestamp.seconds = robot_state.header.stamp.secs
        m.timestamp.nanos = robot_state.header.stamp.nsecs
        m.player_id = robot_state.player_id
        m.status = robot_state.status
        m.role = robot_state.role
        m.pose.position.x = robot_state.pose.position.x
        m.pose.position.y = robot_state.pose.position.y
        m.pose.position.z = robot_state.pose.position.z
        m.pose.orientation.x = robot_state.pose.orientation.x
        m.pose.orientation.y = robot_state.pose.orientation.y
        m.pose.orientation.z = robot_state.pose.orientation.z
        m.pose.orientation.w = robot_state.pose.orientation.w
        m.localized = robot_state.localized
        m.ball_pose.x = robot_state.ball_pose.x
        m.ball_pose.y = robot_state.ball_pose.y
        m.ball_pose.theta = robot_state.ball_pose.theta
        m.ball_detected = robot_state.ball_detected
        m_str = m.SerializeToString()
        id_address = socket.gethostbyname(mirror_server_ip)
        self.get_logger().info(f"\033[96mSending to 3737 on {id_address}\033[0m")
        s.sendto(m_str, (id_address, 3737))

    while not self.is_shutdown() and s is None:
        try:
            self.get_logger().info("Binding to port 3737")
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("0.0.0.0", 3737))
        except self.exceptions.ROSInterruptException:
            exit(0)
        except OSError as ex:
            print(ex)
            time.sleep(3)
    self.get_logger().info("\033[96mBinded to Team Communication Port\033[0m")

    state_create_subscription = self.create_subscription("state", RobotState, robot_state_callback)
    state_create_publishers = {}
    for i in range(1, 5):
        if i is not player_id:
            state_create_publishers[i] = self.create_publisher("/robot" + str(i) + "/state", RobotState, queue_size=1)

    rate = self.Rate(10)
    while not self.is_shutdown():
        try:
            msg = s.recv(1024)
            if not msg:
                rate.sleep()
                continue
        except (struct.error, socket.timeout):
            rate.sleep()
            print("Nothing received")
            continue

        self.get_logger().info("\033[96mReceiving Data Started\033[0m")
        m = robot_state_pb2.Message()
        m.ParseFromString(msg)
        if m.player_id == player_id:
            rate.sleep()
            continue

        r = RobotState()
        r.header.stamp.secs = m.timestamp.seconds
        r.header.stamp.nsecs = m.timestamp.nanos
        r.player_id = m.player_id
        r.status = m.status
        r.role = m.role
        r.localized = m.localized
        r.pose.position.x = m.pose.position.x
        r.pose.position.y = m.pose.position.y
        r.pose.position.z = m.pose.position.z
        r.pose.orientation.x = m.pose.orientation.x
        r.pose.orientation.y = m.pose.orientation.y
        r.pose.orientation.z = m.pose.orientation.z
        r.pose.orientation.w = m.pose.orientation.w
        r.ball_detected = m.ball_detected
        r.ball_pose.x = m.ball_pose.x
        r.ball_pose.y = m.ball_pose.y
        r.ball_pose.theta = m.ball_pose.theta
        state_create_publishers[r.player_id].publish(r)

        rate.sleep()
