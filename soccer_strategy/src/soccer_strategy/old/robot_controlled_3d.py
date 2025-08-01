import os

import numpy as np
import rclpy
import tf2_py
import tf.transformations
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty

from soccer_common import Transformation
from soccer_msgs.msg import FixedTrajectoryCommand, RobotState
from soccer_strategy.old.ball import Ball
from soccer_strategy.old.obstacle import Obstacle
from soccer_strategy.old.robot import Robot
from soccer_strategy.old.robot_controlled import RobotControlled


class RobotControlled3D(RobotControlled):
    def __init__(self, team, role, status):
        self.position_default = np.array([0, 0, 0])

        super().__init__(team=team, role=role, status=status, position=self.position_default)

        # create_subscriptions
        self.amcl_pose_create_subscription = self.create_subscription("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.amcl_pose = None
        self.imu_create_subscription = self.create_subscription("imu_filtered", Imu, self.imu_callback)
        self.action_completed_create_subscription = self.create_subscription("action_complete", Empty, self.action_completed_callback, queue_size=1)
        self.head_centered_on_ball_create_subscription = self.create_subscription(
            "head_centered_on_ball", Empty, self.head_centered_on_ball_callback, queue_size=1
        )
        self.reset_robot_create_subscription = self.create_subscription("reset_robot", PoseStamped, self.reset_robot_callback, queue_size=1)

        # create_publishers
        self.robot_initial_pose_create_publisher = self.create_publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.goal_create_publisher = self.create_publisher("goal", PoseStamped, queue_size=1, latch=True)
        self.trajectory_create_publisher = self.create_publisher("command", FixedTrajectoryCommand, queue_size=1, latch=True)
        self.kicking_range_create_publisher = self.create_publisher("kicking_angle", Range, queue_size=1, latch=True)

        self.tf_listener = tf.TransformListener()

        self.robot_id = self.get_param("robot_id")
        self.robot_name = "robot " + str(self.robot_id)

        self.time_since_action_completed = self.Time(0)

        self.obstacles = PoseArray()

        self.update_robot_state_timer = self.Timer(self.Duration(0.2), self.update_robot_state, reset=True)
        self.robot_state_create_publisher = self.create_publisher("state", RobotState, queue_size=1)

        self.active = True
        self.delocalized_threshold = self.get_param("delocalized_threshold", 0.15)
        self.relocalized_threshold = self.get_param("relocalized_threshold", 0.05)
        self.node_init_time = self.get_clock().now()

    def set_navigation_position(self, goal_position):
        goal_position = self.shorten_navigation_position(goal_position)

        if not super().set_navigation_position(goal_position):
            return False

        p = PoseStamped()
        p.header.stamp = self.get_clock().now()
        p.header.frame_id = "world"
        p.pose.position.x = goal_position[0]
        p.pose.position.y = goal_position[1]
        p.pose.position.z = 0
        angle_fixed = goal_position[2]
        q = tf.transformations.quaternion_about_axis(angle_fixed, (0, 0, 1))
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        self.get_logger().info("Sending New Goal: " + str(goal_position))
        self.goal_position = goal_position
        self.goal_create_publisher.publish(p)
        return True

    def reset_robot_callback(self, pose: PoseStamped):
        q = tf.transformations.euler_from_quaternion(
            [pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z]
        )
        self.position = np.array([pose.pose.position.x, pose.pose.position.y, q[2]])
        self.get_logger().info(f"Robot Reset Called to {pose.pose.position.x} {pose.pose.position.y} {q[2]} (self.position = {self.position}")
        if self.role == Robot.Role.UNASSIGNED:
            self.role = Robot.Role.STRIKER
        self.localized = True
        self.status = Robot.Status.READY
        self.reset_initial_position()

    def update_robot_state(self, _):
        # Get Ball Position from TF
        ground_truth = not bool(os.getenv("COMPETITION", True))
        if ground_truth:
            self.get_logger().info("Using Ground Truth")
        else:
            self.get_logger().info("Using Actual Measurements")

        try:
            self.observed_ball = Ball()
            if ground_truth:
                self.observed_ball.last_observed_time_stamp = self.tf_listener.getLatestCommonTime("world", "robot" + str(self.robot_id) + "/ball_gt")
                ball_pose = self.tf_listener.lookupTransform(
                    "world", "robot" + str(self.robot_id) + "/ball_gt", self.observed_ball.last_observed_time_stamp
                )
            else:
                self.observed_ball.last_observed_time_stamp = self.tf_listener.getLatestCommonTime("world", "robot" + str(self.robot_id) + "/ball")
                ball_pose = self.tf_listener.lookupTransform(
                    "world", "robot" + str(self.robot_id) + "/ball", self.observed_ball.last_observed_time_stamp
                )
            self.observed_ball.position = np.array([ball_pose[0][0], ball_pose[0][1]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_py.TransformException):
            self.get_logger().error(30, "Still looking for ball in TF Tree")
            self.observed_ball = None

        # Get Obstacles from TF
        self.observed_obstacles.clear()
        more_obstacles = True
        obstacle_num = 0
        while more_obstacles:
            try:
                obstacle_pose = self.tf_listener.lookupTransform(
                    "world", "robot" + str(self.robot_id) + "/obstacle_" + str(obstacle_num), self.Time(0)
                )
                o = Obstacle()
                o.position = np.array([obstacle_pose[0][0], obstacle_pose[0][1]])
                self.observed_obstacles.append(o)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_py.TransformException):
                more_obstacles = False
            obstacle_num += 1

        # Get Robot Position from TF
        trans = [self.position[0], self.position[1], 0]
        rot = tf.transformations.quaternion_from_euler(0, 0, self.position[2])

        if self.status not in [Robot.Status.FALLEN_BACK, Robot.Status.FALLEN_SIDE, Robot.Status.FALLEN_FRONT, Robot.Status.GETTING_BACK_UP]:
            try:
                if ground_truth:
                    (trans, rot) = self.tf_listener.lookupTransform("world", "robot" + str(self.robot_id) + "/base_footprint_gt", self.Time(0))
                else:
                    (trans, rot) = self.tf_listener.lookupTransform("world", "robot" + str(self.robot_id) + "/base_footprint", self.Time(0))
                eul = tf.transformations.euler_from_quaternion(rot)
                self.position = np.array([trans[0], trans[1], eul[2]])

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self.get_clock().now() - self.node_init_time > self.Duration(5):
                    self.logwarn_throttle(5, "Unable to locate robot in TF tree")

        # Publish Robot state info
        r = RobotState()
        r.header.stamp = self.get_clock().now()
        r.player_id = self.robot_id
        r.status = self.status
        r.role = self.role
        r.localized = self.localized
        r.pose.position.x = trans[0]
        r.pose.position.y = trans[1]
        r.pose.position.z = trans[2]
        r.pose.orientation.x = rot[0]
        r.pose.orientation.y = rot[1]
        r.pose.orientation.z = rot[2]
        r.pose.orientation.w = rot[3]
        if self.observed_ball is not None:
            r.ball_detected = True
            r.ball_pose.x = self.observed_ball.position[0]
            r.ball_pose.y = self.observed_ball.position[1]
            r.ball_pose.theta = 0
        else:
            r.ball_detected = False
            r.ball_pose.x = 0
            r.ball_pose.y = 0
            r.ball_pose.theta = 0
        self.robot_state_create_publisher.publish(r)
        pass

    def amcl_pose_callback(self, amcl_pose: PoseWithCovarianceStamped):
        self.amcl_pose = amcl_pose
        if self.status == Robot.Status.LOCALIZING:
            covariance_trace = np.sqrt(amcl_pose.pose.covariance[0] ** 2 + amcl_pose.pose.covariance[7] ** 2)
            self.logwarn_throttle(1, "Relocalizing, current cov trace: " + str(covariance_trace))
            if covariance_trace < self.relocalized_threshold:
                self.get_logger().info("Relocalized")
                if self.role != Robot.Role.UNASSIGNED and self.localized:
                    self.status = Robot.Status.READY
                else:
                    self.status = Robot.Status.DETERMINING_SIDE

    def action_completed_callback(self, data):
        if self.status == Robot.Status.GETTING_BACK_UP:
            self.reset_initial_position(variance=0.3)
            self.status = Robot.Status.LOCALIZING
        elif self.status in [
            Robot.Status.WALKING,
            Robot.Status.TERMINATING_WALK,
            Robot.Status.KICKING,
        ]:
            self.goal_position = None
            if self.amcl_pose is not None:
                covariance_trace = np.sqrt(self.amcl_pose.pose.covariance[0] ** 2 + self.amcl_pose.pose.covariance[7] ** 2)
            else:
                covariance_trace = 0
            if covariance_trace >= self.delocalized_threshold:
                self.logwarn("Robot Delocalized, Sending Robot back to localizing, current cov trace: " + str(covariance_trace))
                self.status = Robot.Status.LOCALIZING
            else:
                if self.role == Robot.Role.UNASSIGNED or not self.localized:
                    self.status = Robot.Status.DETERMINING_SIDE
                else:
                    self.status = Robot.Status.READY

            self.time_since_action_completed = self.get_clock().now()
        elif self.status == Robot.Status.PENALIZED:
            self.goal_position = None
            self.time_since_action_completed = self.get_clock().now()
        else:
            self.get_logger().error("Invalid Action Completed " + str(self.status))

    def head_centered_on_ball_callback(self, data):
        self.robot_focused_on_ball_time = self.get_clock().now()

    def imu_callback(self, msg):
        angle_threshold = np.pi / 4  # in radian
        t = Transformation([0, 0, 0], [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        yaw, pitch, roll = t.orientation_euler
        if self.status in [
            Robot.Status.DETERMINING_SIDE,
            Robot.Status.READY,
            Robot.Status.WALKING,
            Robot.Status.TERMINATING_WALK,
            Robot.Status.KICKING,
            Robot.Status.LOCALIZING,
        ]:
            if pitch < np.pi / 6:  # -angle_threshold:
                if roll < 0:
                    # RHR
                    self.logwarn_throttle(1, f"Fallen Back Left: (R: {roll}, P: {pitch}, Y: {yaw}), {t.quaternion}")
                    self.status = Robot.Status.FALLEN_BACK_LEFT
                else:
                    self.logwarn_throttle(1, f"Fallen Back Right: (R: {roll}, P: {pitch}, Y: {yaw}), {t.quaternion}")
                    self.status = Robot.Status.FALLEN_BACK_RIGHT

            elif pitch > angle_threshold:
                self.logwarn_throttle(1, f"Fallen Front: (R: {roll}, P: {pitch}, Y: {yaw}), {t.quaternion}")
                self.status = Robot.Status.FALLEN_FRONT

            elif roll < -angle_threshold or roll > angle_threshold:
                self.logwarn_throttle(1, f"Fallen Side: (R: {roll}, P: {pitch}, Y: {yaw}), {t.quaternion}")
                self.status = Robot.Status.FALLEN_SIDE

    def reset_initial_position(self, variance=0.02):

        position = self.position

        p = PoseWithCovarianceStamped()
        p.header.frame_id = "world"
        p.header.stamp = self.get_rostime()
        p.pose.pose.position.x = position[0]
        p.pose.pose.position.y = position[1]
        p.pose.pose.position.z = 0
        angle_fixed = position[2]
        q = tf.transformations.quaternion_about_axis(angle_fixed, (0, 0, 1))
        p.pose.pose.orientation.x = q[0]
        p.pose.pose.orientation.y = q[1]
        p.pose.pose.orientation.z = q[2]
        p.pose.pose.orientation.w = q[3]
        self.get_logger().error_identical(10, "Setting " + self.robot_name + " localization position " + str(position) + " orientation " + str(q))
        # fmt: off
        p.pose.covariance = [variance ** 2, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, variance ** 2, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, (variance * 4) ** 2]
        # fmt: on
        self.robot_initial_pose_create_publisher.publish(p)
        self.sleep(1)

    def kick(self, kick_velocity):
        return self.run_fixed_trajectory("rightkick")

    def run_fixed_trajectory(self, trajectory_name="rightkick"):
        f = FixedTrajectoryCommand()
        f.trajectory_name = trajectory_name
        if not self.kick_with_right_foot:
            f.mirror = True
        self.trajectory_create_publisher.publish(f)
        if "kick" in trajectory_name:
            self.status = Robot.Status.KICKING
        else:
            self.status = Robot.Status.GETTING_BACK_UP
        self.get_logger().info(self.robot_name + " " + f.trajectory_name)

    def get_detected_obstacles(self):
        # TODO know if they are friendly or enemy robot
        obstacles = []
        for i in range(1, 10):
            try:
                (trans, rot) = self.tf_listener.lookupTransform("world", self.robot_name + "/detected_robot_" + str(i), self.Time(0))
                header = self.tf_listener.getLatestCommonTime("world", self.robot_name + "/detected_robot_" + str(i))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:

                continue

            if abs(self.get_clock().now() - header) > self.Duration(0.5):
                continue
            obstacle_pose = Pose()
            obstacle_pose.position = trans
            obstacle_pose.orientation = rot
            self.obstacles.poses.append(obstacle_pose)

            eul = tf.transformations.euler_from_quaternion(rot)
            obstacle_position = [trans[0], trans[1], eul[2]]
            obstacles.append(obstacle_position)
            pass
        return obstacles

    def can_kick(self, *args, **kwargs):
        # Initialize and create a Range visualizer for kicking angle
        r = Range()
        r.header.stamp = self.get_clock().now()
        r.header.frame_id = f"robot{self.robot_id}/base_footprint"
        r.field_of_view = self.min_kick_angle * 2
        r.min_range = 0
        r.max_range = self.min_kick_angle
        r.range = self.min_kick_distance
        r.radiation_type = 1
        self.kicking_range_create_publisher.publish(r)

        return super().can_kick(*args, **kwargs)
