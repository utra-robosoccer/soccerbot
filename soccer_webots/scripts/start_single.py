#!/usr/bin/env python3
import argparse
import os
import rospy
import time
from soccer_webots.webots_robot_controller import RobotController
from geometry_msgs.msg import PoseWithCovarianceStamped


parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', help="which robot should be started")
temp_bool = True
args, unknown = parser.parse_known_args()

rospy.init_node("webots_ros_interface", argv=['clock:=/clock'])
pid_param_name = "/webots_pid"

while not rospy.has_param(pid_param_name):
    print("Waiting for parameter " + pid_param_name + " to be set..")
    time.sleep(2.0)

webots_pid = rospy.get_param(pid_param_name)

rospy.set_param("name", args.robot_name)
rospy.set_param("competition", "False")
os.environ["WEBOTS_PID"] = webots_pid
os.environ["WEBOTS_ROBOT_NAME"] = args.robot_name

rospy.loginfo("Starting ros interface for " + args.robot_name)

r = RobotController(base_ns=args.robot_name)
init_pub = rospy.Publisher("/" + args.robot_name + "/initialpose", PoseWithCovarianceStamped, queue_size=50)

current_time = rospy.Time.from_seconds(r.time)
last_time = rospy.Time.from_seconds(r.time)

while not rospy.is_shutdown():
    r.step()