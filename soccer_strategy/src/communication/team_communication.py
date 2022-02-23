#!/usr/bin/env python3

import socket

import rospy
import struct
import os

from soccer_msgs.msg import RobotState

import robot_state_pb2


if __name__ == '__main__':
    rospy.init_node("team_communication")
    rospy.loginfo("Initializing team_communication...", logger_name="team_comm")

    player_id = int(os.getenv('ROBOCUP_ROBOT_ID', 1))
    team_id = int(os.getenv('ROBOCUP_TEAM_ID', 16))

    send_ports = []
    receive_port = 0
    for i in range(4):
        port = 4000 + player_id + 10 * team_id
        if i == player_id:
            receive_port = port
        else:
            send_ports.append(port)

    s = None

    def robot_state_callback(robot_state: RobotState):
        m = robot_state_pb2.Message()
        m.timestamp.seconds = robot_state.header.stamp.secs
        m.timestamp.nanos= robot_state.header.stamp.nsecs
        m.player_id = robot_state.player_id
        m.status = robot_state.status
        m.pose.position.x = robot_state.pose.position.x
        m.pose.position.y = robot_state.pose.position.y
        m.pose.position.z = robot_state.pose.position.z
        m.pose.orientation.x = robot_state.pose.orientation.x
        m.pose.orientation.y = robot_state.pose.orientation.y
        m.pose.orientation.z = robot_state.pose.orientation.z
        m.pose.orientation.w = robot_state.pose.orientation.w
        m.ball_pose.x = robot_state.ball_pose.x
        m.ball_pose.y = robot_state.ball_pose.y
        m.ball_pose.theta = robot_state.ball_pose.theta
        m_str = m.SerializeToString()
        for port in send_ports:
            rospy.loginfo(f'Sending to {port} on 127.0.0.1', logger_name="team_comm")
            s.sendto(m_str, ('127.0.0.1', port))

    while not rospy.is_shutdown() and s is None:
        rospy.loginfo(f"Binding to port 3737", logger_name="team_comm")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.bind(('0.0.0.0', 3737))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            rospy.sleep(1)
        except rospy.exceptions.ROSInterruptException:
            exit(0)

    state_subscriber = rospy.Subscriber("state", RobotState, robot_state_callback)
    state_publishers = {}
    for i in range(1, 4):
        if i is not player_id:
            state_publishers[i] = rospy.Publisher("/robot" + str(i) + "/state", RobotState)

    while not rospy.is_shutdown():
        try:
            msg = s.recv(1024)
        except (struct.error, socket.timeout):
            continue

        m = robot_state_pb2.Message()
        m.ParseFromString(msg)

        r = RobotState()
        r.header.stamp.secs = m.timestamp.seconds
        r.header.stamp.nsecs = m.timestamp.nanos
        r.player_id = m.player_id
        r.status = m.status
        r.pose.position.x = m.pose.position.x
        r.pose.position.y = m.pose.position.y
        r.pose.position.z = m.pose.position.z
        r.pose.orientation.x = m.pose.orientation.x
        r.pose.orientation.y = m.pose.orientation.y
        r.pose.orientation.z = m.pose.orientation.z
        r.pose.orientation.w = m.pose.orientation.w
        r.ball_pose.x = m.ball_pose.x
        r.ball_pose.y = m.ball_pose.y
        r.ball_pose.theta = m.ball_pose.theta
        state_publishers[player_id].publish(r)