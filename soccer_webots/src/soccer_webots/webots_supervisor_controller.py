#!/usr/bin/env python3
from controller import Robot, Node, Supervisor
import os
import rospy
from geometry_msgs.msg import Quaternion, PointStamped, Pose, Point, Twist
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

G = 9.81
import transforms3d
import numpy as np

import nav_msgs.msg


class SupervisorController:
    def __init__(self):
        """
        The SupervisorController, a Webots controller that can control the world.
        Set the environment variable WEBOTS_ROBOT_NAME to "supervisor_robot" if used with 1_bot.wbt or 4_bots.wbt.

        :param ros_active: Whether to publish ROS messages
        :param mode: Webots mode, one of 'normal', 'paused', or 'fast'
        :param do_ros_init: Whether rospy.init_node should be called
        :param base_ns: The namespace of this node, can normally be left empty
        """
        # requires WEBOTS_ROBOT_NAME to be set to "supervisor_robot"

        self.time = 0
        self.clock_msg = Clock()
        self.supervisor = Supervisor()

        self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

        self.motors = []
        self.sensors = []
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # resolve the node for corresponding name
        self.robot_names = ["red_player_1", "red_player_2", "red_player_3", "red_player_4"]
        self.robot_nodes = {}
        self.translation_fields = {}
        self.rotation_fields = {}

        # check if None
        for name in self.robot_names:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                self.robot_nodes[name] = node
                self.translation_fields[name] = node.getField("translation")
                self.rotation_fields[name] = node.getField("rotation")

        clock_topic = "/clock"

        self.clock_publisher = rospy.Publisher(clock_topic, Clock, queue_size=1)

        self.reset_service = rospy.Subscriber("/reset", String, self.reset)
        self.initial_poses_service = rospy.Service("/initial_pose", Empty, self.set_initial_poses)

        self.reset_ball_service = rospy.Service("/reset_ball", Empty, self.reset_ball)
        self.world_info = self.supervisor.getFromDef("world_info")
        self.ball = self.supervisor.getFromDef("ball")

    def step_sim(self):
        self.time += self.timestep / 1000
        self.supervisor.step(self.timestep)

    def step(self):
        self.step_sim()
        self.publish_clock()
        for name in self.robot_names:
            temp_node = self.supervisor.getFromDef(name)
            if temp_node is not None:
                self.publish_odom(name)

    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)

    def publish_odom(self, name="red_player_1"):
        if name == "red_player_1":
            base = "robot1"
        elif name == "red_player_2":
            base = "robot2"
        elif name == "red_player_3":
            base = "robot3"
        elif name == "red_player_4":
            base = "robot4"
        odom = rospy.Publisher('/' + base + '/base_pose_ground_truth', nav_msgs.msg.Odometry, queue_size=1)
        odometry = nav_msgs.msg.Odometry()
        odometry.header.frame_id = base + '/odom'
        odometry.child_frame_id = base + '/base_footprint'
        odometry.header.stamp = rospy.Time.from_seconds(self.time)
        odometry.pose.pose.position.x = self.get_robot_position(name)[0]
        odometry.pose.pose.position.y = self.get_robot_position(name)[1]
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = self.get_robot_orientation_quat(name)[0]
        odometry.pose.pose.orientation.y = self.get_robot_orientation_quat(name)[1]
        odometry.pose.pose.orientation.z = self.get_robot_orientation_quat(name)[2]
        odometry.pose.pose.orientation.w = self.get_robot_orientation_quat(name)[3]
        odometry.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        odom.publish(odometry)

    def reset_ball(self, req=None):
        self.ball.getField("translation").setSFVec3f([0, 0, 0.0772])
        self.ball.getField("rotation").setSFRotation([0, 0, 1, 0])
        self.ball.resetPhysics()

    def set_initial_poses(self, req=None):
        self.reset_robot_pose_rpy([-1, 3, 0.42], [0, 0, 0], name="red_player_1")
        # self.reset_robot_pose_rpy([-1, -3, 0.42], [0, 0.24, 1.57], name="red_player_2")
        # self.reset_robot_pose_rpy([-3, 3, 0.42], [0, 0.24, -1.57], name="red_player_3")
        # self.reset_robot_pose_rpy([-3, -3, 0.42], [0, 0.24, 1.57], name="red_player_4")

    def reset(self, msg):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()
        for name in self.robot_names:
            if name == "red_player_1":
                base = "robot1"
            elif name == "red_player_2":
                base = "robot2"
            elif name == "red_player_3":
                base = "robot3"
            elif name == "red_player_4":
                base = "robot4"
            temp_node = self.supervisor.getFromDef(name)
            if temp_node is not None:
                # rospy.set_param(base + "/send_odom", "true")
                pass

    def get_robot_position(self, name="red_player_1"):
        if name in self.translation_fields:
            return self.translation_fields[name].getSFVec3f()

    def get_robot_orientation_axangles(self, name="red_player_1"):
        if name in self.rotation_fields:
            return self.rotation_fields[name].getSFRotation()

    def get_robot_orientation_quat(self, name="red_player_1"):
        ax_angle = self.get_robot_orientation_axangles(name)
        # transforms 3d uses scalar (i.e. the w part in the quaternion) first notation of quaternions, ros uses scalar last
        quat_scalar_first = transforms3d.quaternions.axangle2quat(ax_angle[:3], ax_angle[3])
        quat_scalar_last = np.append(quat_scalar_first[1:], quat_scalar_first[0])
        return list(quat_scalar_last)

    def get_robot_pose_rpy(self, name="red_player_1"):
        return self.get_robot_position(name), self.get_robot_orientation_rpy(name)

    def get_robot_pose_quat(self, name="red_player_1"):
        return self.get_robot_position(name), self.get_robot_orientation_quat(name)

    def reset_robot_pose(self, pos, quat, name="red_player_1"):
        self.set_robot_pose_quat(pos, quat, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def reset_robot_pose_rpy(self, pos, rpy, name="red_player_1"):
        self.set_robot_pose_rpy(pos, rpy, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def set_robot_rpy(self, rpy, name="red_player_1"):
        axis, angle = transforms3d.euler.euler2axangle(rpy[0], rpy[1], rpy[2], axes='sxyz')
        self.set_robot_axis_angle(axis, angle, name)

    def set_robot_quat(self, quat, name="red_player_1"):
        axis, angle = transforms3d.quaternions.quat2axangle([quat[3], quat[0], quat[1], quat[2]])
        self.set_robot_axis_angle(axis, angle, name)

    def set_robot_position(self, pos, name="red_player_1"):
        if name in self.translation_fields:
            self.translation_fields[name].setSFVec3f(list(pos))

    def set_robot_pose_rpy(self, pos, rpy, name="red_player_1"):
        self.set_robot_position(pos, name)
        self.set_robot_rpy(rpy, name)

    def set_robot_pose_quat(self, pos, quat, name="red_player_1"):
        self.set_robot_position(pos, name)
        self.set_robot_quat(quat, name)

    def set_robot_axis_angle(self, axis, angle, name="red_player_1"):
        if name in self.rotation_fields:
            self.rotation_fields[name].setSFRotation(list(np.append(axis, angle)))
