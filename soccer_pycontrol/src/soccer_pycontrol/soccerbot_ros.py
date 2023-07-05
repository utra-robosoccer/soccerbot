import os
from typing import Optional

import tf
import tf2_py
import torch
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float64

from soccer_msgs.msg import RobotState
from soccer_pycontrol.soccerbot import *


class SoccerbotRos(Soccerbot):
    """
    The main class for the robot, which receives and sends information to ROS
    """

    def __init__(self, pose: Transformation, useFixedBase=False, useCalibration=True):

        super().__init__(pose, useFixedBase, useCalibration)

        self.cleats_offset = rospy.get_param(
            "cleats_offset", -0.01634
        )  #: Additional height added by cleats and grass, consists of 1cm grass and 0.5cm cleats
        self.motor_publishers = {}
        self.pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=1)
        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)
        self.path_publisher = rospy.Publisher("path", Path, queue_size=1, latch=True)
        self.path_odom_publisher = rospy.Publisher("path_odom", Path, queue_size=1, latch=True)
        self.br = tf.TransformBroadcaster()
        self.imu_subscriber = rospy.Subscriber("imu_filtered", Imu, self.imu_callback, queue_size=1)
        self.imu_ready = False
        self.listener = tf.TransformListener()
        self.last_ball_found_timestamp = None
        self.last_ball_pose = None
        self.last_ball_tracking_walking_timestamp: Optional[rospy.Time] = None
        self.ball_pixel_subscriber = rospy.Subscriber("ball_pixel", Pose2D, self.ball_pixel_callback, queue_size=1)
        self.ball_pixel: Optional[Pose2D] = None
        self.last_ball_pixel: Optional[Pose2D] = None
        self.last_ball_pixel_update = rospy.Time.now()
        self.head_centered_on_ball_publisher = rospy.Publisher("head_centered_on_ball", Empty, queue_size=1)

        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.state_callback)
        self.robot_state = RobotState()
        self.robot_state.status = RobotState.STATUS_DISCONNECTED

        #: Frequency for the head's yaw while searching and relocalizing (left and right movement)
        self.head_yaw_freq = rospy.get_param("head_yaw_freq", 0.02 if torch.cuda.is_available() else 0.003)
        self.head_yaw_freq_relocalizing = rospy.get_param("head_yaw_freq_relocalizing", 0.005 if torch.cuda.is_available() else 0.003)

        #: Frequency for the head's while searching and relocalizing yaw (up and down movement)
        self.head_pitch_freq = rospy.get_param("head_pitch_freq", 0.02 if torch.cuda.is_available() else 0.003)
        self.head_pitch_freq_relocalizing = rospy.get_param("head_pitch_freq_relocalizing", 0.005 if torch.cuda.is_available() else 0.003)

    def state_callback(self, robot_state: RobotState):
        """
        Callback function for information about the robot's state

        :param robot_state: A class which tells you information about the robot, disconnected etc
        """
        self.robot_state = robot_state

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU information

        :param msg: IMU Message
        """
        self.imu_msg = msg
        self.imu_ready = True

    def ball_pixel_callback(self, msg: Pose2D):
        """
        Callback function for ball pixel, used for head rotation calculations

        :param msg: The location of the pixel of the center of the ball in the Camera
        """

        self.ball_pixel = msg

    def updateRobotConfiguration(self) -> None:
        """
        Reads the joint_states message and resets all the positions of all the joints
        """

        self.configuration_offset = [0] * len(Joints)
        try:
            joint_state = rospy.wait_for_message("joint_states", JointState, timeout=3)
            joint_pos = { j: p for j, p in zip(joint_state.name, joint_state.position) }

            self.configuration[0:18] = [joint_pos[name] if name in joint_pos else self.configuration[i] for i, name in enumerate(self._motor_map.keys())] # coerce to existing configuration value if there is no response from that joint
        except (ROSException, KeyError, AttributeError) as ex:
            rospy.logerr(ex)
        except ValueError as ex:
            print(ex)
            rospy.logerr("Not all joint states are reported, cable disconnect?")
            rospy.logerr("Joint States")
            rospy.logerr(joint_state)
            rospy.logerr("Motor Names")
            print(self.motor_names)
            self.configuration[0:18] = [0] * len(Joints)

    def publishAngles(self):
        """
        Send the robot angles based on self.configuration + self.configuration_offset to ROS
        """
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.now()
        js.position = []
        js.effort = []
        angles = self.get_angles()
        for i, n in enumerate(self.motor_names):
            js.name.append(n)
            js.position.append(angles[i])
        try:
            rospy.loginfo_once("Started Publishing Motors")
            self.pub_all_motor.publish(js)
        except rospy.exceptions.ROSException as ex:
            print(ex)
            exit(0)

    def stepPath(self, t):
        super(SoccerbotRos, self).stepPath(t)

        # Get odom from odom_path
        self.odom_pose = (
            self.odom_pose_start_path
            @ self.robot_path.start_transformed_inv
            @ self.robot_path.torsoPosition(t, invert_calibration=True)
            @ self.torso_offset
        )

    def publishPath(self, robot_path=None):
        """
        Publishes the robot path to rviz for debugging and visualization

        :param robot_path: The path to publish, leave empty to publish the robot's current path
        """

        if robot_path is None:
            robot_path = self.robot_path

        def createPath(robot_path, invert_calibration=False) -> Path:
            p = Path()
            p.header.frame_id = "world"
            p.header.stamp = rospy.Time.now()
            for i in range(0, robot_path.torsoStepCount(), 1):
                step = robot_path.getTorsoStepPose(i)
                if invert_calibration:
                    step = adjust_navigation_transform(robot_path.start_transform, step)

                position = step.position
                orientation = step.quaternion
                pose = PoseStamped()
                pose.header.seq = i
                pose.header.frame_id = "world"
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.position.z = position[2]

                pose.pose.orientation.x = orientation[0]
                pose.pose.orientation.y = orientation[1]
                pose.pose.orientation.z = orientation[2]
                pose.pose.orientation.w = orientation[3]
                p.poses.append(pose)
            return p

        self.path_publisher.publish(createPath(robot_path))
        self.path_odom_publisher.publish(createPath(robot_path, invert_calibration=True))

    def publishOdometry(self, time: rospy.Time):
        """
        Send the odometry of the robot to be used in localization by ROS
        """

        o = Odometry()
        o.header.stamp = time
        o.header.frame_id = os.environ["ROS_NAMESPACE"][1:] + "/odom"
        o.child_frame_id = os.environ["ROS_NAMESPACE"][1:] + "/base_link"
        o.pose.pose = self.odom_pose.pose
        o.pose.pose.position.z = 0

        # fmt: off
        o.pose.covariance = [1e-4, 0, 0, 0, 0, 0,
                             0, 1e-4, 0, 0, 0, 0,
                             0, 0, 1e-18, 0, 0, 0,
                             0, 0, 0, 1e-18, 0, 0,
                             0, 0, 0, 0, 1e-18, 0,
                             0, 0, 0, 0, 0, 1e-2]
        # fmt: on
        self.odom_publisher.publish(o)

        # Publish the height of the center of the torso of the robot (used for camera vision calculations and odometry)
        height = self.pose.position[2] + self.foot_box[2] / 2 + self.cleats_offset
        self.br.sendTransform(
            (0, 0, height),
            (0, 0, 0, 1),
            time,
            os.environ["ROS_NAMESPACE"] + "/torso",
            os.environ["ROS_NAMESPACE"] + "/base_footprint",
        )
        pass

    def get_imu(self):
        """
        Gets the IMU at the IMU link location.

        :return: calculated orientation of the center of the torso of the robot
        """

        assert self.imu_ready
        return Transformation(
            [0, 0, 0],
            [
                self.imu_msg.orientation.x,
                self.imu_msg.orientation.y,
                self.imu_msg.orientation.z,
                self.imu_msg.orientation.w,
            ],
        )

    def get_foot_pressure_sensors(self, floor):
        # TODO subscribe to foot pressure sensors
        pass

    def apply_head_rotation(self):
        if self.robot_state.status == RobotState.STATUS_PENALIZED:
            self.configuration[Joints.HEAD_1] = 0
            self.configuration[Joints.HEAD_2] = 0
            self.publishAngles()
        elif self.robot_state.status == RobotState.STATUS_DETERMINING_SIDE:
            self.configuration[Joints.HEAD_1] = math.cos(self.head_step * self.head_pitch_freq_relocalizing) * (math.pi / 4)
            self.configuration[Joints.HEAD_2] = 0
            self.head_step += 1
        elif self.robot_state.status == RobotState.STATUS_LOCALIZING:
            self.configuration[Joints.HEAD_1] = math.cos(self.head_step * self.head_yaw_freq_relocalizing) * (math.pi / 4)
            self.configuration[Joints.HEAD_2] = math.pi * rospy.get_param("head_rotation_yaw_center", 0.175) - math.sin(
                self.head_step * self.head_pitch_freq_relocalizing
            ) * math.pi * rospy.get_param("head_rotation_yaw_range", 0.15)
            self.head_step += 1
        elif self.robot_state.status == self.robot_state.STATUS_READY:
            try:
                ball_found_timestamp = self.listener.getLatestCommonTime(
                    os.environ["ROS_NAMESPACE"] + "/camera", os.environ["ROS_NAMESPACE"] + "/ball"
                )

                if self.last_ball_found_timestamp is None or ball_found_timestamp - self.last_ball_found_timestamp > rospy.Duration(2):
                    self.last_ball_found_timestamp = ball_found_timestamp
                    rospy.loginfo_throttle(5, f"Ball found at timestamp {self.last_ball_found_timestamp}")

                    self.listener.waitForTransform("world", os.environ["ROS_NAMESPACE"] + "/ball", rospy.Time(0), rospy.Duration(nsecs=500000000))
                    last_ball_position, last_ball_orientation = self.listener.lookupTransform(
                        "world", os.environ["ROS_NAMESPACE"] + "/ball", rospy.Time(0)
                    )
                    self.last_ball_pose = Transformation(last_ball_position)
                    rospy.loginfo_once("Last ball pose set")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_py.TransformException):
                rospy.loginfo_throttle(5, "Ball no longer in field of view, searching for the ball")
                pass

            # So that the arms don't block the vision
            self.configuration[Joints.LEFT_ARM_1] = (-0.3 - self.configuration[Joints.LEFT_ARM_1]) * 0.05 + self.configuration[Joints.LEFT_ARM_1]
            self.configuration[Joints.RIGHT_ARM_1] = (-0.3 - self.configuration[Joints.RIGHT_ARM_1]) * 0.05 + self.configuration[Joints.RIGHT_ARM_1]
            self.configuration[Joints.LEFT_ARM_2] = (0.8 - self.configuration[Joints.LEFT_ARM_2]) * 0.05 + self.configuration[Joints.LEFT_ARM_2]
            self.configuration[Joints.RIGHT_ARM_2] = (0.8 - self.configuration[Joints.RIGHT_ARM_2]) * 0.05 + self.configuration[Joints.RIGHT_ARM_2]

            # If the last time it saw the ball was 2 seconds ago
            if self.last_ball_found_timestamp is not None and (rospy.Time.now() - self.last_ball_found_timestamp) < rospy.Duration(3):

                assert self.ball_pixel is not None

                camera_movement_speed = 0.0008
                pixel_threshold = 15
                if self.ball_pixel != self.last_ball_pixel or (rospy.Time.now() - self.last_ball_pixel_update) > rospy.Duration(0.2):
                    xpixeldiff = self.ball_pixel.x - 640 / 2
                    if abs(xpixeldiff) > pixel_threshold:
                        self.configuration[Joints.HEAD_1] -= max(
                            min(camera_movement_speed * xpixeldiff, camera_movement_speed * 100), -camera_movement_speed * 100
                        )
                    ypixeldiff = self.ball_pixel.y - 480 / 2
                    if abs(ypixeldiff) > pixel_threshold:
                        self.configuration[Joints.HEAD_2] += max(
                            min(camera_movement_speed * ypixeldiff, camera_movement_speed * 100), -camera_movement_speed * 100
                        )

                    self.last_ball_pixel = self.ball_pixel
                    self.last_ball_pixel_update = rospy.Time.now()

                    if abs(xpixeldiff) <= pixel_threshold and abs(ypixeldiff) <= pixel_threshold:
                        self.head_centered_on_ball_publisher.publish()
                        rospy.loginfo_throttle(
                            1, f"\033[92mCentered Camera on Ball (x,y) ({self.ball_pixel.x}, {self.ball_pixel.y}) -> (320, 240)\033[0m"
                        )

                    else:
                        rospy.loginfo_throttle(1, f"Centering Camera on Ball (x,y) ({self.ball_pixel.x}, {self.ball_pixel.y}) -> (320, 240)")
            elif (
                self.last_ball_tracking_walking_timestamp is not None
                and rospy.Time.now() - self.last_ball_tracking_walking_timestamp < rospy.Duration(1)
            ):
                rospy.loginfo_throttle(1, "Slowly raising head to locate ball")
                self.configuration[Joints.HEAD_2] = max(self.configuration[Joints.HEAD_2] - 0.025, 0)
                self.head_step += 1
            else:
                rospy.loginfo_throttle(5, "Searching for ball again")
                self.last_ball_pose = None
                self.configuration[Joints.HEAD_1] = math.sin(self.head_step * self.head_yaw_freq) * (math.pi / 4)
                self.configuration[Joints.HEAD_2] = math.pi * rospy.get_param("head_rotation_yaw_center", 0.185) - math.cos(
                    self.head_step * self.head_pitch_freq
                ) * math.pi * rospy.get_param("head_rotation_yaw_range", 0.15)
                self.head_step += 1
        elif self.robot_state.status == self.robot_state.STATUS_WALKING:
            # If it is walking moving, turn it's head to the last location where it saw the ball

            if self.last_ball_pose is not None:

                rospy.loginfo_throttle(1, f"Searching for ball last location {self.last_ball_pose.position}")

                try:
                    camera_position, camera_orientation = self.listener.lookupTransform(
                        "world", os.environ["ROS_NAMESPACE"] + "/camera", rospy.Time(0)
                    )
                    euler_yaw_only = Transformation.get_euler_from_quaternion(camera_orientation)
                    euler_yaw_only[1] = 0
                    euler_yaw_only[2] = 0
                    camera_pose = Transformation(camera_position, euler=euler_yaw_only)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr_throttle(5, "Unable to get robot to camera pose")
                    return

                camera_to_ball = scipy.linalg.inv(camera_pose) @ self.last_ball_pose
                camera_to_ball_position = camera_to_ball.position
                yaw = math.atan2(camera_to_ball_position[1], camera_to_ball_position[0])
                pitch = math.atan2(camera_to_ball_position[2], camera_to_ball_position[0])

                # If the ball last location is too difficult for the head to turn to
                if abs(yaw) > np.pi * 0.5 or abs(pitch) > np.pi * 0.7:
                    return

                if self.configuration[Joints.HEAD_1] != yaw:
                    self.configuration[Joints.HEAD_1] += yaw * 0.0005
                self.configuration[Joints.HEAD_2] = -pitch

                rospy.loginfo_throttle(
                    1,
                    f"Following head to last seen ball location {self.last_ball_pose.position}. Camera Location {camera_pose.position}, Camera To Ball {camera_to_ball_position}, Calculated Yaw {yaw}, Pitch {pitch}",
                )
                self.last_ball_tracking_walking_timestamp = rospy.Time.now()
            else:
                self.configuration[Joints.HEAD_1] = 0
                self.configuration[Joints.HEAD_2] = 0
                self.head_step = 0
        elif self.robot_state.status == self.robot_state.STATUS_KICKING:
            self.last_ball_tracking_walking_timestamp = rospy.Time.now()
