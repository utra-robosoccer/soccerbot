#!/usr/bin/env python3
import argparse
import os
import rospy
import time
from soccer_webots.webots_robot_controller import RobotController
from std_msgs.msg import Float64
import math
from math import sin, cos, pi
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped


def callback(data):
    global temp_bool
    temp_bool = False


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
os.environ["WEBOTS_PID"] = webots_pid
os.environ["WEBOTS_ROBOT_NAME"] = args.robot_name

rospy.logdebug("Starting ros interface for " + args.robot_name)

r = RobotController()
odom_pub = rospy.Publisher('/' + args.robot_name + "/odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
rospy.Subscriber('/' + args.robot_name + '/amcl_pose', PoseWithCovarianceStamped, callback)

current_time = rospy.Time.from_seconds(r.time)
last_time = rospy.Time.from_seconds(r.time)
x = 0.0
y = 0.0
th = 0.0


while not rospy.is_shutdown():
    r.step()
    if temp_bool:
        current_time = rospy.Time.from_seconds(r.time)

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (0, 0, 0),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
        odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(x, y, 0), Vector3(0, 0, th))
        odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        # publish the message
        odom_pub.publish(odom)
        last_time = current_time
        # temp_bool = False
