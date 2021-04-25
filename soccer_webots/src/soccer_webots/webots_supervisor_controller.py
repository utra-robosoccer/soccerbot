#!/usr/bin/env python3
from controller import Robot, Node, Supervisor
import os
import rospy
from geometry_msgs.msg import Quaternion, PointStamped, Pose, Point, Twist
from gazebo_msgs.msg import ModelStates

from rosgraph_msgs.msg import Clock



G = 9.81


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
        base_ns = "/robot1"

        self.time = 0
        self.clock_msg = Clock()
        self.supervisor = Supervisor()

        self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

        self.motors = []
        self.sensors = []
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # resolve the node for corresponding name
        self.robot_names = ["robot1"]
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

        self.world_info = self.supervisor.getFromDef("world_info")
        self.ball = self.supervisor.getFromDef("ball")

    def step_sim(self):
        self.time += self.timestep / 1000
        self.supervisor.step(self.timestep)

    def step(self):
        self.step_sim()
        self.publish_clock()

    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)



