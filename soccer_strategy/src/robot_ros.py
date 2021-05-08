import enum
from robot import Robot
import rospy
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
import math
import tf.transformations
from sensor_msgs.msg import Imu


class RobotRos(Robot):
    def __init__(self, team, role, status, robot_name):
        self.robot_pose_sub = rospy.Subscriber('/' + robot_name + "/amcl_pose",
                                               geometry_msgs.msg.PoseWithCovarianceStamped,
                                               self.robot_pose_callback)
        self.ball_pose_sub = rospy.Subscriber('/' + robot_name + "/ball_pose",
                                              geometry_msgs.msg.PoseWithCovarianceStamped,
                                              self.ball_pose_callback)
        self.imu_sub = rospy.Subscriber('/' + robot_name + "/imu_data", Imu, self.imu_callback)
        self.pub_goal = rospy.Publisher('/' + robot_name + "/imu_raw", geometry_msgs.msg.Pose2D, queue_size=1)
        self.pub_trajectory = rospy.Publisher('/' + robot_name + "/command", std_msgs.msg.String, queue_size=1)

        self.team = team
        self.role = role
        self.status = status
        self.goal_position = self.position
        self.robot_name = robot_name

        # for static trajectories
        self.last_kick = 0
        self.last_getupfront = 0
        self.last_getupback = 0
        self.publishing_static_trajectory = False

    def robot_pose_callback(self, data):
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        x_vector = np.cos(euler[2]) * np.cos(euler[0])
        y_vector = np.sin(euler[2]) * np.cos(euler[0])
        angle = math.atan2(y_vector, x_vector)
        self.position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, angle])
        pass

    def ball_pose_callback(self, data):
        self.ball_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        pass

    def imu_callback(self, msg):
        angle_threshold = 1  # in radian
        q = msg.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # We want to publish once on state transition
        if pitch > angle_threshold and self.status != self.Status.FALLEN_FRONT:
            self.status = self.Status.FALLEN_FRONT

        if pitch < -angle_threshold and self.status != self.Status.FALLEN_BACK:
            self.status = self.Status.FALL_BACK
