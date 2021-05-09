#!/usr/bin/env python3

import os
import subprocess
import sys

import rospkg
import rospy

import argparse

from soccer_webots.webots_supervisor_controller import SupervisorController

rospack = rospkg.RosPack()
path = rospack.get_path("soccer_webots")
parser = argparse.ArgumentParser()
parser.add_argument('--single', help="which robot should be started")


args, unknown = parser.parse_known_args()

mode = "normal"
batch = ""
no_rendering = ""

if args.single == "true":
    world_name = "kid.wbt"
else:
    world_name = "kid_4.wbt"
print(world_name)
print(args.single)
arguments = [os.path.join(os.environ['WEBOTS_HOME'], 'webots'),
             batch,
             no_rendering,
             path + "/worlds/" + world_name]
sim_proc = subprocess.Popen(arguments)

rospy.init_node("webots_ros_supervisor", argv=['clock:=/clock'])
rospy.set_param("/webots_pid" , str(sim_proc.pid))

os.environ["WEBOTS_PID"] = str(sim_proc.pid)
os.environ["WEBOTS_ROBOT_NAME"] = "supervisor_robot"

supervisor_controller = SupervisorController()

while not rospy.is_shutdown():
    supervisor_controller.step()
