import os
from robot import Robot
from robot_controlled import RobotControlled

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import Empty, Bool
from soccer_msgs.msg import FixedTrajectoryCommand, RobotState
import numpy as np
import tf.transformations

from sensor_msgs.msg import Imu


class RobotControlled3D(RobotControlled):
    def __init__(self, team, role, status):
        x_pos_default = float(os.getenv('X_POS', 4))
        y_pos_default = float(os.getenv('Y_POS', -3.15))
        yaw_default = float(os.getenv('YAW', 1.57))
        self.position_default = [x_pos_default, y_pos_default, yaw_default]

        super().__init__(team=team, role=role, status=status, position=self.position_default)

        # Subscibers
        self.imu_subsciber = rospy.Subscriber("imu_filtered", Imu, self.imu_callback)
        self.action_completed_subscriber = rospy.Subscriber("action_complete", Empty,
                                                             self.action_completed_callback)

        # Publishers
        self.robot_initial_pose_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.goal_publisher = rospy.Publisher("goal", PoseStamped, queue_size=1, latch=True)
        self.trajectory_publisher = rospy.Publisher("command", FixedTrajectoryCommand, queue_size=1, latch=True)

        self.tf_listener = tf.TransformListener()

        self.robot_id = int(os.getenv('ROBOCUP_ROBOT_ID', 1))
        self.robot_name = "robot " + str(self.robot_id)

        # Configuration
        self.kick_with_right_foot = True

        # terminate all action
        self.relocalization_timeout = 0

        self.obstacles = PoseArray()


        self.update_robot_state_timer = rospy.Timer(rospy.Duration(0.2), self.update_robot_state, reset=True)
        self.robot_state_publisher = rospy.Publisher("state", RobotState, queue_size=1)

    def set_navigation_position(self, goal_position):
        if not super().set_navigation_position(goal_position):
            return False

        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = self.goal_position[0]
        p.pose.position.y = self.goal_position[1]
        p.pose.position.z = 0
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        print("Sending New Goal: " + str(goal_position))
        self.goal_publisher.publish(p)
        return True

    def update_robot_state(self, _):
        # Get Ball Position from TF
        try:
            ball_pose = self.tf_listener.lookupTransform('world', "robot" + str(self.robot_id) + '/ball', rospy.Time(0))
            header = self.tf_listener.getLatestCommonTime('world', "robot" + str(self.robot_id) + '/ball')
            time_diff = rospy.Time.now() - header
            if time_diff < rospy.Duration(1):
                self.observed_ball.position = np.array([ball_pose[0][0], ball_pose[0][1], ball_pose[0][2]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(30, "Unable to locate ball in TF tree")

        # Get Robot Position from TF
        trans = [self.position[0], self.position[1], 0]
        rot = tf.transformations.quaternion_from_euler(0, 0, self.position[2])
        try:
            (trans, rot) = self.tf_listener.lookupTransform('world', "robot" + str(self.robot_id) + '/base_footprint', rospy.Time(0))
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
        r.ball_pose.x = self.observed_ball.position[0]
        r.ball_pose.y = self.observed_ball.position[1]
        r.ball_pose.theta = 0
        self.robot_state_publisher.publish(r)
        pass


    def terminate_walk(self):
        self.terminate_walking_publisher.publish()

    def kick(self):
        f = FixedTrajectoryCommand()
        f.trajectory_name = "rightkick"
        if not self.kick_with_right_foot:
            f.mirror = True
        self.trajectory_publisher.publish(f)
        rospy.loginfo(self.robot_name + " kicking")

    def get_back_up(self, type: str="getupback"):
        self.terminate_walking_publisher.publish()
        f = FixedTrajectoryCommand()
        f.trajectory_name = type
        self.trajectory_publisher.publish(f)
        self.relocalization_timeout = 5
        rospy.loginfo(self.robot_name + type)

    def ball_pose_callback(self, data):
        self.ball_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        pass

    def action_completed_callback(self, data):
        if self.status == Robot.Status.TERMINATING_WALK:
            self.status = Robot.Status.READY
        elif self.status == Robot.Status.KICKING:
            self.status = Robot.Status.READY
        elif self.status == Robot.Status.TRAJECTORY_IN_PROGRESS:
            self.status = Robot.Status.READY
        else:
            rospy.logerr("Invalid Action Completed " + str(self.status))

    def imu_callback(self, msg):
        angle_threshold = 1.25  # in radian
        q = msg.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
        if self.status in [Robot.Status.DETERMINING_SIDE, Robot.Status.READY, Robot.Status.WALKING, Robot.Status.TERMINATING_WALK, Robot.Status.KICKING, Robot.Status.LOCALIZING]:
            if pitch > angle_threshold:
                rospy.logwarn("Fallen Back")
                self.status = Robot.Status.FALLEN_BACK

            elif pitch < -angle_threshold:
                rospy.logwarn("Fallen Front")
                self.status = Robot.Status.FALLEN_FRONT

            elif yaw < -angle_threshold or yaw > angle_threshold:
                rospy.logwarn("Fallen Side")
                self.status = Robot.Status.FALLEN_SIDE

    def reset_initial_position(self, position):
        rospy.loginfo("Setting initial Robot " + self.robot_name + " position " + str(position))
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
        p.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        self.robot_initial_pose_publisher.publish(p)

    def update_status(self):
        if self.status != self.previous_status:
            rospy.loginfo(self.robot_name + " status changes to " + str(self.status))
            self.previous_status = self.status

        if self.status == Robot.Status.DISCONNECTED:
            pass

        elif self.status == Robot.Status.READY:
            if self.stop_requested:
                self.status = Robot.Status.STOPPED

        elif self.status == Robot.Status.WALKING:
            if self.stop_requested:
                self.status = Robot.Status.TERMINATING_WALK

        elif self.status == Robot.Status.KICKING:
            f = FixedTrajectoryCommand()
            f.trajectory_name = "rightkick"
            if not self.kick_with_right_foot:
                f.mirror = True
            self.trajectory_publisher.publish(f)
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS
            rospy.loginfo(self.robot_name + " kicking")

        elif self.status == Robot.Status.FALLEN_BACK:
            self.terminate_walking_publisher.publish()
            f = FixedTrajectoryCommand()
            f.trajectory_name = "getupback"
            self.trajectory_publisher.publish(f)
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS
            self.relocalization_timeout = 5
            rospy.loginfo(self.robot_name + "getupback")

        elif self.status == Robot.Status.FALLEN_FRONT:
            self.terminate_walking_publisher.publish()
            f = FixedTrajectoryCommand()
            f.trajectory_name = "getupfront"
            self.trajectory_publisher.publish(f)
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS
            self.relocalization_timeout = 5
            rospy.loginfo(self.robot_name + "getupfront")

        elif self.status == Robot.Status.FALLEN_SIDE:
            self.terminate_walking_publisher.publish()
            f = FixedTrajectoryCommand()
            f.trajectory_name = "getupside"
            self.trajectory_publisher.publish(f)
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS
            rospy.loginfo(self.robot_name + "getupside")

        elif self.status == Robot.Status.TRAJECTORY_IN_PROGRESS:
            rospy.loginfo_throttle(20, self.robot_name + " trajectory in progress")

        elif self.status == Robot.Status.STOPPED or self.status == Robot.Status.READY:
            pass

        else:
            rospy.logerr_throttle(20, self.robot_name + " is in invalid status " + str(self.status))

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
