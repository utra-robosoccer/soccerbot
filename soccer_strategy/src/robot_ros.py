from robot import Robot
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import Empty, Bool
from soccer_msgs.msg import FixedTrajectoryCommand
import numpy as np
import tf.transformations

from sensor_msgs.msg import Imu

robot_id_map = {"robot1": 1, "robot2": 2, "robot3": 3, "robot4": 4, "opponent1": 1, "opponent2": 2, "opponent3": 3,
                "opponent4": 4}


class RobotRos(Robot):
    def __init__(self, team, role, status, robot_name):
        self.robot_pose_sub = rospy.Subscriber('/' + robot_name + "/amcl_pose", PoseWithCovarianceStamped,
                                               self.robot_pose_callback)
        self.robot_initial_pose_pub = rospy.Publisher('/' + robot_name + "/initialpose", PoseWithCovarianceStamped,
                                                      queue_size=1)

        self.imu_sub = rospy.Subscriber('/' + robot_name + "/imu_filtered", Imu, self.imu_callback)
        self.goal_publisher = rospy.Publisher('/' + robot_name + "/goal", PoseStamped, queue_size=1, latch=True)
        self.trajectory_publisher = rospy.Publisher('/' + robot_name + "/command", FixedTrajectoryCommand, queue_size=1)
        self.terminate_walking_publisher = rospy.Publisher('/' + robot_name + "/terminate_walking", Empty, queue_size=1)
        self.completed_walking_subscriber = rospy.Subscriber('/' + robot_name + "/completed_walking", Empty,
                                                             self.completed_walking_callback)
        self.completed_trajectory_subscriber = rospy.Subscriber('/' + robot_name + "/trajectory_complete", Bool,
                                                                self.completed_trajectory_subscriber)
        self.completed_trajectory_publisher = rospy.Publisher('/' + robot_name + "/trajectory_complete", Bool,
                                                              queue_size=1)
        self.move_head_sub = rospy.Subscriber('/' + robot_name + "/move_head", Bool, self.move_head_callback)
        self.obstacles_publisher = rospy.Publisher('/' + robot_name + "/obstacles", PoseArray,
                                                              queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.localization_reset_publisher = rospy.Publisher('/' + robot_name + "/localization_mode", Bool, queue_size=1)
        self.team = team
        self.role = role
        self.status = status
        self.position = np.array([-3, -3, 0])  # 1.57
        self.goal_position = np.array([0.0, 0.0, 0])
        self.ball_position = np.array([0.0, 0.0])
        self.robot_name = robot_name
        self.robot_id = robot_id_map[self.robot_name]
        self.max_kick_speed = 2
        self.previous_status = Robot.Status.READY
        self.kick_with_right_foot = True

        self.send_nav = False
        # terminate all action
        self.stop_requested = False
        self.designated_kicker = False
        self.relocalization_timeout = 0

        self.obstacles = PoseArray()

    def robot_pose_callback(self, data):
        if self.status == Robot.Status.DISCONNECTED:
            self.status = Robot.Status.READY

    def ball_pose_callback(self, data):
        self.ball_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        pass

    def move_head_callback(self, data):
        self.send_nav = data.data
        pass

    def completed_walking_callback(self, data):
        rospy.loginfo(f"{self.robot_name} Completed Walking")
        if self.status == Robot.Status.WALKING:
            self.status = Robot.Status.READY
            temp = Bool()
            temp.data = True
            self.localization_reset_publisher.publish(temp)
            rospy.sleep(1.25)
            temp.data = False
            self.localization_reset_publisher.publish(temp)

    def completed_trajectory_subscriber(self, data):
        rospy.loginfo(f"{self.robot_name} Completed Trajectory")
        # assert self.status == Robot.Status.TRAJECTORY_IN_PROGRESS, self.status
        if data.data and self.status == Robot.Status.TRAJECTORY_IN_PROGRESS:
            if self.stop_requested:
                self.status = Robot.Status.STOPPED
            else:
                self.status = Robot.Status.READY
                temp = Bool()
                temp.data = True
                self.localization_reset_publisher.publish(temp)
                rospy.sleep(1.25)
                temp.data = False
                self.localization_reset_publisher.publish(temp)

    def get_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('world', self.robot_name + '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return self.position

        eul = tf.transformations.euler_from_quaternion(rot)
        self.position = [trans[0], trans[1], eul[2]]
        return self.position

    def set_navigation_position(self, position):
        # assert (self.status == Robot.Status.WALKING)
        if self.status == Robot.Status.READY or self.status == Robot.Status.WALKING:
            super(RobotRos, self).set_navigation_position(position)
            rospy.loginfo("Sending Robot " + self.robot_name + " to position" + str(position))
            p = PoseStamped()
            p.header.stamp = rospy.get_rostime()
            p.header.frame_id = "world"
            p.pose.position.x = position[0]
            p.pose.position.y = position[1]
            p.pose.position.z = 0
            angle_fixed = position[2]
            # print(angle_fixed)
            q = tf.transformations.quaternion_about_axis(angle_fixed, (0, 0, 1))
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            self.goal_publisher.publish(p)
            self.status = Robot.Status.WALKING
        else:
            print("Failed to send nav position, robot currently in " + str(self.status))

    def imu_callback(self, msg):
        angle_threshold = 1  # in radian
        q = msg.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
        if self.status == Robot.Status.WALKING or self.status == Robot.Status.READY:
            # We want to publish once on state transition
            if pitch > angle_threshold:
                print("fall back triggered")
                self.status = Robot.Status.FALLEN_BACK

            elif pitch < -angle_threshold:
                print("fall front triggered")
                self.status = Robot.Status.FALLEN_FRONT

            elif yaw < -angle_threshold or yaw > angle_threshold:
                print("fall side triggered")
                self.status = Robot.Status.FALLEN_SIDE
        pass

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
        self.robot_initial_pose_pub.publish(p)
        # rospy.sleep(1)

    def update_status(self):
        if self.status != self.previous_status:
            rospy.loginfo(self.robot_name + " status changes to " + str(self.status))
            self.previous_status = self.status

        if self.status == Robot.Status.READY:
            if self.stop_requested:
                self.status = Robot.Status.STOPPED

        if self.status == Robot.Status.WALKING:
            if self.stop_requested:
                self.terminate_walking_publisher.publish()
                self.status = Robot.Status.STOPPED

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
