import os
from robot import Robot
from robot_controlled import RobotControlled
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import Empty, Bool
from soccer_msgs.msg import FixedTrajectoryCommand, RobotState
from soccer_common import Transformation
import numpy as np
import tf.transformations
import time

from sensor_msgs.msg import Imu


class RobotControlled3D(RobotControlled):
    def __init__(self, team, role, status):
        x_pos_default = float(os.getenv('X_POS', 4))
        y_pos_default = float(os.getenv('Y_POS', -3.15))
        yaw_default = float(os.getenv('YAW', 1.57))
        self.position_default = [x_pos_default, y_pos_default, yaw_default]

        super().__init__(team=team, role=role, status=status, position=self.position_default)

        # Subscibers
        self.amcl_pose_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.amcl_pose = None
        self.imu_subsciber = rospy.Subscriber("imu_filtered", Imu, self.imu_callback)
        self.action_completed_subscriber = rospy.Subscriber("action_complete", Empty,
                                                            self.action_completed_callback, queue_size=1)
        self.head_centered_on_ball_subscriber = rospy.Subscriber("head_centered_on_ball", Empty,
                                                                 self.head_centered_on_ball_callback, queue_size=1)
        # Publishers
        self.robot_initial_pose_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.goal_publisher = rospy.Publisher("goal", PoseStamped, queue_size=1, latch=True)
        self.trajectory_publisher = rospy.Publisher("command", FixedTrajectoryCommand, queue_size=1, latch=True)

        self.tf_listener = tf.TransformListener()

        self.robot_id = int(os.getenv('ROBOCUP_ROBOT_ID', 1))
        self.robot_name = "robot " + str(self.robot_id)

        self.time_since_action_completed = rospy.Time(0)

        self.obstacles = PoseArray()

        self.update_robot_state_timer = rospy.Timer(rospy.Duration(0.2), self.update_robot_state, reset=True)
        self.robot_state_publisher = rospy.Publisher("state", RobotState, queue_size=1)

        self.active = True

    def set_kick_velocity(self, kick_velocity):
        pass

    def set_navigation_position(self, goal_position):
        if not super().set_navigation_position(goal_position):
            return False

        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
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
        print("Sending New Goal: " + str(goal_position))
        self.goal_position = goal_position
        self.goal_publisher.publish(p)
        return True

    def update_robot_state(self, _):
        # Get Ball Position from TF
        try:
            self.observed_ball.last_observed_time_stamp = self.tf_listener.getLatestCommonTime('world', "robot" + str(self.robot_id) + '/ball')
            ball_pose = self.tf_listener.lookupTransform('world', "robot" + str(self.robot_id) + '/ball', self.observed_ball.last_observed_time_stamp)
            self.observed_ball.position = np.array([ball_pose[0][0], ball_pose[0][1], ball_pose[0][2]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(30, "Unable to locate ball in TF tree")
            self.observed_ball.position = None

        # Get Robot Position from TF
        trans = [self.position[0], self.position[1], 0]
        rot = tf.transformations.quaternion_from_euler(0, 0, self.position[2])
        try:
            (trans, rot) = self.tf_listener.lookupTransform('world', "robot" + str(self.robot_id) + '/base_footprint',
                                                            rospy.Time(0))
            eul = tf.transformations.euler_from_quaternion(rot)
            self.position = np.array([trans[0], trans[1], eul[2]])
            if self.status == Robot.Status.DISCONNECTED:
                self.status = Robot.Status.READY

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(30, "Unable to locate robot in TF tree")

        # Publish Robot state info
        r = RobotState()
        r.header.stamp = rospy.Time.now()
        r.player_id = self.robot_id
        r.status = self.status
        r.role = self.role
        r.pose.position.x = trans[0]
        r.pose.position.y = trans[1]
        r.pose.position.z = trans[2]
        r.pose.orientation.x = rot[0]
        r.pose.orientation.y = rot[1]
        r.pose.orientation.z = rot[2]
        r.pose.orientation.w = rot[3]
        if self.observed_ball.position is not None:
            r.ball_pose.x = self.observed_ball.position[0]
            r.ball_pose.y = self.observed_ball.position[1]
            r.ball_pose.theta = 0
        else:
            r.ball_pose.x = 0
            r.ball_pose.y = 0
            r.ball_pose.theta = 0
        self.robot_state_publisher.publish(r)
        pass

    def amcl_pose_callback(self, amcl_pose: PoseWithCovarianceStamped):
        self.amcl_pose = amcl_pose
        if self.status == Robot.Status.LOCALIZING:
            covariance_trace = np.sqrt(amcl_pose.pose.covariance[0] ** 2 + amcl_pose.pose.covariance[7] ** 2)
            rospy.logwarn_throttle(1, "Relocalizing, current cov trace: " + str(covariance_trace))
            if covariance_trace < 0.06:
                rospy.loginfo("Relocalized")
                self.status = Robot.Status.READY
            elif rospy.Time.now() - self.time_since_action_completed > rospy.Duration(
                    10):  # Timeout localization after 10 seconds
                rospy.logwarn("Relocalization timeout hit")
                self.status = Robot.Status.READY

    def action_completed_callback(self, data):
        if self.status in [Robot.Status.WALKING, Robot.Status.TERMINATING_WALK, Robot.Status.KICKING,
                           Robot.Status.TRAJECTORY_IN_PROGRESS]:
            self.goal_position = None
            covariance_trace = np.sqrt(self.amcl_pose.pose.covariance[0] ** 2 + self.amcl_pose.pose.covariance[7] ** 2)
            if covariance_trace > 0.2:
                rospy.logwarn(1, "Robot Delocalized, Sending Robot back to localizing, current cov trace: " + str(covariance_trace))
                self.status = Robot.Status.LOCALIZING
            else:
                self.status = Robot.Status.READY

            self.time_since_action_completed = rospy.Time.now()
        elif self.status == Robot.Status.PENALIZED:
            self.goal_position = None
            self.time_since_action_completed = rospy.Time.now()
        else:
            rospy.logerr("Invalid Action Completed " + str(self.status))

    def head_centered_on_ball_callback(self, data):
        self.navigation_goal_localized_time = time.time()

    def imu_callback(self, msg):
        angle_threshold = 1.1  # in radian
        t = Transformation([0, 0, 0], [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                       msg.orientation.w])
        roll, pitch, yaw = t.get_orientation_euler()
        if self.status in [Robot.Status.DETERMINING_SIDE, Robot.Status.READY, Robot.Status.WALKING,
                           Robot.Status.TERMINATING_WALK, Robot.Status.KICKING, Robot.Status.LOCALIZING]:
            if pitch < -angle_threshold:
                rospy.logwarn_throttle(1, "Fallen Back")
                self.status = Robot.Status.FALLEN_BACK

            elif pitch > angle_threshold:
                rospy.logwarn_throttle(1, "Fallen Front")
                self.status = Robot.Status.FALLEN_FRONT

            elif yaw < -angle_threshold or yaw > angle_threshold:
                rospy.logwarn_throttle(1, "Fallen Side")
                self.status = Robot.Status.FALLEN_SIDE

    def reset_initial_position(self, position):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'world'
        p.header.stamp = rospy.get_rostime()
        p.pose.pose.position.x = position[0]
        p.pose.pose.position.y = position[1]
        p.pose.pose.position.z = 0
        angle_fixed = position[2]
        q = tf.transformations.quaternion_about_axis(angle_fixed, (0, 0, 1))
        p.pose.pose.orientation.x = q[0]
        p.pose.pose.orientation.y = q[1]
        p.pose.pose.orientation.z = q[2]
        p.pose.pose.orientation.w = q[3]
        print("Setting initial Robot " + self.robot_name + " position " + str(position) + " orientation " + str(q))
        p.pose.covariance = [0.0025, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0025, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        self.robot_initial_pose_publisher.publish(p)

    def kick(self):
        return self.run_fixed_trajectory("rightkick")

    def run_fixed_trajectory(self, trajectory_name="rightkick"):
        f = FixedTrajectoryCommand()
        f.trajectory_name = trajectory_name
        if not self.kick_with_right_foot:
            f.mirror = True
        self.trajectory_publisher.publish(f)
        self.status = Robot.Status.TRAJECTORY_IN_PROGRESS
        rospy.loginfo(self.robot_name + " " + f.trajectory_name)

    def get_detected_obstacles(self):
        # TODO know if they are friendly or enemy robot
        obstacles = []
        for i in range(1, 10):
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', self.robot_name + '/detected_robot_' + str(i),
                                                                rospy.Time(0))
                header = self.tf_listener.getLatestCommonTime('world', self.robot_name + '/detected_robot_' + str(i))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:

                continue

            if abs(rospy.Time.now() - header) > rospy.Duration(0.5):
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
