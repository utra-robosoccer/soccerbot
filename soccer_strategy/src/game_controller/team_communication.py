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

if __name__ == '__main__':
    rospy.init_node("team_communication")
    rospy.loginfo("Initializing team_communication...", logger_name="team_comm")

    player_id = int(os.getenv('ROBOCUP_ROBOT_ID', 1))
    team_id = int(os.getenv('ROBOCUP_TEAM_ID', 16))

    target_ports = [port + 10 * team_id for port in [4001, 4002, 4003, 4004]]
    receive_port = target_ports[player_id - 1]

    listener = tf.TransformListener()

    position_publishers = []
    for i in range(1, 4):
        position_publishers.append(rospy.Publisher())
        pass

    # Get the socket
    s = None
    while not rospy.is_shutdown() and s is None:
        rospy.loginfo(f"Binding to port 3737", logger_name="team_comm")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.bind(('0.0.0.0', 3737))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            rospy.sleep(1)
        except rospy.exceptions.ROSInterruptException:
            exit(0)

    # Loop through
    while not rospy.is_shutdown():
        # Send information
        message = robocup_extension_pb2.Message()
        now = rospy.Time.now()
        message.timestamp.seconds = now.secs
        message.timestamp.nanos = now.nsecs

        message.current_pose.player_id = player_id
        message.current_pose.team = team_id

        # get ball pose from tf
        ball_position = None
        try:
            ball_pose = listener.lookupTransform('world', "robot" + str(player_id) + '/ball', rospy.Time(0))
            header = listener.getLatestCommonTime('world', "robot" + str(player_id) + '/ball')
            time_diff = rospy.Time.now() - header
            if time_diff < rospy.Duration(1):
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
            (trans, rot) = listener.lookupTransform('world', "robot" + str(player_id) + '/base_footprint',
                                                         rospy.Time(0))
            header = listener.getLatestCommonTime('world', "robot" + str(player_id) + '/base_footprint')
            time_diff = rospy.Time.now() - header
            if time_diff < rospy.Duration(1):

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
        for port in target_ports:
            rospy.logdebug(f'Sending to {port} on 127.0.0.1', logger_name="team_comm")
            s.sendto(msg, ('127.0.0.1', port))

        # Receive information
        try:
            msg = s.recv(1024)
        except (struct.error, socket.timeout):
            continue

        if msg:
            message = robocup_extension_pb2.Message()
            message.ParseFromString(msg)

            player_id = message.current_pose.player_id

            if message.current_pose.team != team_id:
                continue

            team_data = TeamData()

            header = Header()
            header.stamp = rospy.Time.now()
            team_data.header = header
            team_data.robot_id = player_id
            pose_proto_to_ros(message.current_pose, team_data.robot_position)
            team_data.ball_absolute.pose.position.x = message.ball.position.x
            team_data.ball_absolute.pose.position.y = message.ball.position.y
            team_data.ball_absolute.pose.position.z = message.ball.position.z

            if message.ball.covariance:
                covariance_proto_to_ros(message.ball.covariance, team_data.ball_absolute.covariance)
