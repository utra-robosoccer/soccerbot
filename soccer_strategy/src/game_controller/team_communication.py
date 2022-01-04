#!/usr/bin/env python3

import socket

import rospy
import struct
import os
import numpy as np
import tf
import transforms3d

from std_msgs.msg import Header
from soccer_msgs.msg import TeamData


import robocup_extension_pb2


class HumanoidLeagueTeamCommunication:
    def __init__(self):
        self.socket = None

        rospy.init_node("team_communication")
        rospy.loginfo("Initializing team_communication...", logger_name="team_comm")

        self.player_id = int(os.getenv('ROBOCUP_ROBOT_ID', 1))
        self.team_id = int(os.getenv('ROBOCUP_TEAM_ID', 16))

        self.config = rospy.get_param("~")

        self.target_host = self.config['target_host']
        if self.target_host == '127.0.0.1':
            # local mode, bind to port depending on bot id and team id
            self.target_ports = [port + 10 * self.team_id for port in self.config['local_target_ports']]
            self.receive_port = self.target_ports[self.player_id - 1]
        else:
            self.target_ports = [self.config['target_port']]
            self.receive_port = self.config['receive_port']

        self.listener = tf.TransformListener()

        self.create_publishers()

        # we will try multiple times till we manage to get a connection
        while not rospy.is_shutdown() and self.socket is None:
            self.socket = self.get_connection()
            rospy.sleep(1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        rospy.Timer(rospy.Duration.from_sec(1 / self.config['rate']), self.send_message)
        self.receive_forever()

    def create_publishers(self):
        self.pub_team_data = rospy.Publisher(self.config['team_data_topic'], TeamData, queue_size=1)

    def get_connection(self):
        rospy.loginfo(f"Binding to port {self.receive_port}", logger_name="team_comm")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.bind(('0.0.0.0', self.receive_port))
        return sock

    def close_connection(self):
        if self.socket:
            self.socket.close()
            rospy.loginfo("Connection closed.", logger_name="team_comm")

    def receive_msg(self):
        return self.socket.recv(1024)

    def __del__(self):
        self.close_connection()

    def receive_forever(self):
        while not rospy.is_shutdown():
            try:
                msg = self.receive_msg()
            except (struct.error, socket.timeout):
                continue

            if msg:  # Not handle empty messages or None
                self.handle_message(msg)

    def handle_message(self, msg):
        def covariance_proto_to_ros(fmat3, ros_covariance):
            # ROS covariance is row-major 36 x float, protobuf covariance is column-major 9 x float [x, y, θ]
            ros_covariance[0] = fmat3.x.x
            ros_covariance[1] = fmat3.y.x
            ros_covariance[5] = fmat3.z.x
            ros_covariance[6] = fmat3.x.y
            ros_covariance[7] = fmat3.y.y
            ros_covariance[11] = fmat3.z.y
            ros_covariance[30] = fmat3.x.z
            ros_covariance[31] = fmat3.y.z
            ros_covariance[35] = fmat3.z.z

        def pose_proto_to_ros(robot, pose):
            pose.pose.position.x = robot.position.x
            pose.pose.position.y = robot.position.y

            quat = transforms3d.euler.euler2quat(0.0, 0.0, robot.position.z)  # wxyz -> ros: xyzw
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            pose.pose.orientation.w = quat[0]

            if pose.covariance:
                covariance_proto_to_ros(robot.covariance, pose.covariance)

        message = robocup_extension_pb2.Message()
        message.ParseFromString(msg)

        player_id = message.current_pose.player_id
        team_id = message.current_pose.team

        if team_id != self.team_id:
            # Skip information from ourselves or from the other team
            return

        team_data = TeamData()

        header = Header()
        # The robots' times can differ, therefore use our own time here
        header.stamp = rospy.Time.now()

        # Handle timestamp
        ##################
        team_data.header = header

        # Handle robot ID
        #################
        team_data.robot_id = player_id

        # Handle pose of current player
        ###############################
        pose_proto_to_ros(message.current_pose, team_data.robot_position)

        # Handle ball
        #############
        team_data.ball_absolute.pose.position.x = message.ball.position.x
        team_data.ball_absolute.pose.position.y = message.ball.position.y
        team_data.ball_absolute.pose.position.z = message.ball.position.z

        if message.ball.covariance:
            covariance_proto_to_ros(message.ball.covariance, team_data.ball_absolute.covariance)

        self.pub_team_data.publish(team_data)

    def send_message(self, event):
        message = robocup_extension_pb2.Message()
        now = rospy.Time.now()
        message.timestamp.seconds = now.secs
        message.timestamp.nanos = now.nsecs

        message.current_pose.player_id = self.player_id
        message.current_pose.team = self.team_id

        # get ball pose from tf
        ball_position = None
        try:
            ball_pose = self.listener.lookupTransform('world', "robot" + str(self.player_id) + '/ball', rospy.Time(0))
            header = self.listener.getLatestCommonTime('world', "robot" + str(self.player_id) + '/ball')
            time_diff = rospy.Time.now() - header
            if time_diff < rospy.Duration(self.config['lifetime']):
                ball_position = np.array([ball_pose[0][0], ball_pose[0][1], ball_pose[0][2]])

                message.ball.position.x = ball_position[0]
                message.ball.position.y = ball_position[1]
                message.ball.position.z = ball_position[2]
                message.ball.covariance.x.x = 1
                message.ball.covariance.y.y = 1
                message.ball.covariance.z.z = 1
            else:
                rospy.logdebug("ball position timeout")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logdebug("cannot get ball position from tf tree")

        # get robot pose from tf
        robot_position = None
        try:
            (trans, rot) = self.listener.lookupTransform('world', "robot" + str(self.player_id) + '/base_footprint', rospy.Time(0))
            header = self.listener.getLatestCommonTime('world', "robot" + str(self.player_id) + '/base_footprint')
            time_diff = rospy.Time.now() - header
            if time_diff < rospy.Duration(self.config['lifetime']):

                eul = tf.transformations.euler_from_quaternion(rot)

                message.current_pose.position.x = trans[0]
                message.current_pose.position.y = trans[1]
                message.current_pose.position.z = eul[2]
                message.current_pose.covariance.x.x = 1
                message.current_pose.covariance.y.y = 1
                message.current_pose.covariance.z.z = 1
            else:
                rospy.logdebug("robot position timeout")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logdebug("cannot get robot position from tf tree")

        msg = message.SerializeToString()
        for port in self.target_ports:
            rospy.logdebug(f'Sending to {port} on {self.target_host}', logger_name="team_comm")
            self.socket.sendto(msg, (self.target_host, port))


if __name__ == '__main__':
    HumanoidLeagueTeamCommunication()