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
        self.ball_pose_sub = rospy.Subscriber('/' + robot_name + "/ball",
                                              geometry_msgs.msg.PoseWithCovarianceStamped,
                                              self.ball_pose_callback)
        self.imu_sub = rospy.Subscriber('/' + robot_name + "/imu_filtered", Imu, self.imu_callback)
        self.pub_goal = rospy.Publisher('/' + robot_name + "/goal", geometry_msgs.msg.Pose2D, queue_size=1)
        self.pub_trajectory = rospy.Publisher('/' + robot_name + "/command", std_msgs.msg.String, queue_size=1)

        self.team = team
        self.role = role
        self.status = status
        self.position = np.array([0.0, 0.0, 0])
        self.goal_position = np.array([0.0, 0.0, 0])
        self.ball_position = np.array([0.0, 0.0])
        self.robot_name = robot_name
        self.max_kick_speed = 2

        # for static trajectories
        self.last_kick = 0
        self.last_getupfront = 0
        self.last_getupback = 0
        self.publishing_static_trajectory = False

    def robot_pose_callback(self, data):
        quaternion = (
            data.pose.pose.orientation.w,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = np.array([data.pose.pose.position.y, -data.pose.pose.position.x, -euler[0] + math.pi/2])
        pass

    def ball_pose_callback(self, data):
        self.ball_position = np.array([data.pose.pose.position.y, -data.pose.pose.position.x])
        pass

    def imu_callback(self, msg):
        angle_threshold = 1  # in radian
        q = msg.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])

        # We want to publish once on state transition
        if pitch > angle_threshold and self.status != self.Status.FALLEN_BACK:
            self.status = self.Status.FALLEN_BACK

        if pitch < -angle_threshold and self.status != self.Status.FALLEN_FRONT:
            self.status = self.Status.FALLEN_FRONT


