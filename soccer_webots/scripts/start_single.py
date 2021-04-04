#!/usr/bin/env python3
import argparse
import os
import rospy
import time
from soccer_webots.webots_robot_controller import RobotController
from std_msgs.msg import Float64


def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', help="which robot should be started")

args, unknown = parser.parse_known_args()

rospy.init_node("webots_ros_interface", argv=['clock:=/clock'])
pid_param_name = "/webots_pid"

velocity = 0
message = ''
while not rospy.has_param(pid_param_name):
    print("Waiting for parameter " + pid_param_name + " to be set..")
    time.sleep(2.0)

webots_pid = rospy.get_param(pid_param_name)
os.environ["WEBOTS_PID"] = webots_pid
os.environ["WEBOTS_ROBOT_NAME"] = args.robot_name

rospy.logdebug("Starting ros interface for " + args.robot_name)
r = RobotController()
rospy.Subscriber('/' + args.robot_name + '/left_arm_motor_1/command', Float64, callback)
while not rospy.is_shutdown():
    if message:
        print(message)
        message = ''
    motor_index = r.external_motor_names.index("left_arm_motor_1")
    r.motors[motor_index].setPosition(velocity)
    r.step()
