import math
import os
from typing import Optional

import numpy as np
import rospy
import scipy
import tf
import tf2_py
import torch
from geometry_msgs.msg import Pose2D, PoseStamped
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.inverse_kinematics.ik_actions import IKActions
from soccer_pycontrol.model.model_ros.kinematic_data_ros import KinematicDataROS
from soccer_pycontrol.model.model_ros.motor_control_ros import MotorControlROS
from soccer_pycontrol.model.model_ros.sensors_ros import SensorsROS
from soccer_pycontrol.old.joints import Joints
from std_msgs.msg import Empty

from soccer_common import Transformation
from soccer_msgs.msg import RobotState


class BezROS(Bez):
    def __init__(self):

        self.data = KinematicDataROS()

        self.motor_control = MotorControlROS(self.data.motor_names)
        self.sensors = SensorsROS()

        self.ik_actions = IKActions(self.data)

        self.pose = Transformation()  # TODO need a better way
        self.robot_pose: PoseStamped = None
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

        self.head_rotation_yaw_center = rospy.get_param("head_rotation_yaw_center", 0.175)
        self.head_rotation_yaw_range = rospy.get_param("head_rotation_yaw_range", 0.15)
        self.head_step = 0

    def state_callback(self, robot_state: RobotState):
        """
        Callback function for information about the robot's state

        :param robot_state: A class which tells you information about the robot, disconnected etc
        """
        self.robot_state = robot_state

    def ball_pixel_callback(self, msg: Pose2D):
        """
        Callback function for ball pixel, used for head rotation calculations

        :param msg: The location of the pixel of the center of the ball in the Camera
        """

        self.ball_pixel = msg

    # TODO dont like this placement
    def set_walking_torso_height(self, pose: Transformation) -> Transformation:
        """
        Takes a 2D pose and sets the height of the pose to the height of the torso
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        """
        # if pose.position[2] < self.walking_torso_height:
        # pose.position = (pose.position[0], pose.position[1], self.data.walking_torso_height)
        p = pose
        position = p.position
        position[2] = self.data.walking_torso_height
        p.position = position
        return p

    def update_robot_pose(self, footprint_name="/base_footprint") -> bool:
        """
        Function to update the location of the robot based on odometry. Called before movement to make sure the starting
        position is correct

        :param footprint_name:
        :return: True if the position is updated, otherwise False
        """
        try:
            (trans, rot) = self.sensors.tf_listener.lookupTransform("world", os.environ["ROS_NAMESPACE"] + footprint_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            return False

        self.robot_pose = Transformation(position=trans, quaternion=rot).pose_stamped
        return True

    def setPose(self, pose: Transformation = Transformation()) -> None:
        """
        Teleports the robot to the desired pose

        :param pose: 3D position in pybullet
        """
        self.pose = self.set_walking_torso_height(pose)

        [y, _, _] = pose.orientation_euler
        self.pose.orientation_euler = [y, 0, 0]
        self.robot_pose = self.pose  # TODO this is really weird

    # def ready(self) -> None:
    #     super(BezROS, self).ready()

    def apply_head_rotation(self):
        # TODO this feels overly complicated and should be in strategy
        # TODO maybe make actions for the different types of traj
        if self.robot_state.status == RobotState.STATUS_PENALIZED:
            self.motor_control.set_head_target_angles(np.array([0, 0]))
            self.motor_control.set_motor()  # TODO should this be here ?
            self.head_step = 0
        elif self.robot_state.status == RobotState.STATUS_DETERMINING_SIDE:
            angle1 = math.cos(self.head_step * self.head_pitch_freq_relocalizing) * (math.pi / 4)
            self.motor_control.set_head_target_angles(np.array([angle1, 0]))
            self.head_step += 1
        elif self.robot_state.status == RobotState.STATUS_LOCALIZING:
            angle1 = math.cos(self.head_step * self.head_yaw_freq_relocalizing) * (math.pi / 4)
            angle2 = (
                math.pi * self.head_rotation_yaw_center
                - math.sin(self.head_step * self.head_pitch_freq_relocalizing) * math.pi * self.head_rotation_yaw_range
            )
            self.motor_control.set_head_target_angles(np.array([angle1, angle2]))
            self.head_step += 1

        elif self.robot_state.status == self.robot_state.STATUS_READY:
            try:
                # todo this only is correct if it finds a new ball
                ball_found_timestamp = self.listener.getLatestCommonTime(
                    os.environ["ROS_NAMESPACE"] + "/camera", os.environ["ROS_NAMESPACE"] + "/ball"
                )
                # TODO this doesnt make sense and is opposite of what it should be
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

            # So that the arms don't block the vision TODO this is really weird
            # anglel1 = (-0.3 - self.motor_control.configuration[Joints.LEFT_ARM_1]) * 0.05 + self.motor_control.configuration[Joints.LEFT_ARM_1]
            # anglel2 = (
            #     (0.8 - self.motor_control.configuration[Joints.LEFT_ARM_2]) * 0.05
            #     + self.motor_control.configuration[Joints.LEFT_ARM_2](0.8 - self.motor_control.configuration[Joints.LEFT_ARM_2]) * 0.05
            #     + self.motor_control.configuration[Joints.LEFT_ARM_2]
            # )
            #
            # angler1 = (-0.3 - self.motor_control.configuration[Joints.RIGHT_ARM_1]) * 0.05 + self.motor_control.configuration[Joints.RIGHT_ARM_1]
            # angler2 = (
            #     (0.8 - self.motor_control.configuration[Joints.RIGHT_ARM_2]) * 0.05
            #     + self.motor_control.configuration[Joints.RIGHT_ARM_2](0.8 - self.motor_control.configuration[Joints.RIGHT_ARM_2]) * 0.05
            #     + self.motor_control.configuration[Joints.RIGHT_ARM_2]
            # )

            # self.motor_control.set_left_arm_target_angles(np.array([anglel1, anglel2]))
            # self.motor_control.set_right_arm_target_angles(np.array([angler1, angler2]))

            # If the last time it saw the ball was 2 seconds ago
            if self.last_ball_found_timestamp is not None and (rospy.Time.now() - self.last_ball_found_timestamp) < rospy.Duration(3):

                assert self.ball_pixel is not None

                camera_movement_speed = 0.0008
                pixel_threshold = 15
                if self.ball_pixel != self.last_ball_pixel or (rospy.Time.now() - self.last_ball_pixel_update) > rospy.Duration(0.2):
                    xpixeldiff = self.ball_pixel.x - 640 / 2
                    if abs(xpixeldiff) > pixel_threshold:
                        self.motor_control.configuration[Joints.HEAD_1] -= max(
                            min(camera_movement_speed * xpixeldiff, camera_movement_speed * 100), -camera_movement_speed * 100
                        )
                    ypixeldiff = self.ball_pixel.y - 480 / 2
                    if abs(ypixeldiff) > pixel_threshold:
                        self.motor_control.configuration[Joints.HEAD_2] += max(
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
                self.motor_control.configuration[Joints.HEAD_2] = max(
                    self.motor_control.configuration[Joints.HEAD_2] - 0.025, 0
                )  # TODO should change
                self.head_step += 1
            else:
                rospy.loginfo_throttle(5, "Searching for ball again")
                self.last_ball_pose = None
                angle1 = math.sin(self.head_step * self.head_yaw_freq) * (math.pi / 4)
                angle2 = (
                    math.pi * self.head_rotation_yaw_center - math.cos(self.head_step * self.head_pitch_freq) * math.pi * self.head_rotation_yaw_range
                )
                self.motor_control.set_head_target_angles(np.array([angle1, angle2]))
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

                if self.motor_control.configuration[Joints.HEAD_1] != yaw:
                    self.motor_control.configuration[Joints.HEAD_1] += yaw * 0.0005
                self.motor_control.configuration[Joints.HEAD_2] = -pitch

                rospy.loginfo_throttle(
                    1,
                    f"Following head to last seen ball location {self.last_ball_pose.position}. Camera Location {camera_pose.position}, Camera To Ball {camera_to_ball_position}, Calculated Yaw {yaw}, Pitch {pitch}",
                )
                self.last_ball_tracking_walking_timestamp = rospy.Time.now()
            else:
                self.motor_control.configuration[Joints.HEAD_1] = 0
                self.motor_control.configuration[Joints.HEAD_2] = 0
                self.head_step = 0
        elif self.robot_state.status == self.robot_state.STATUS_KICKING:
            self.last_ball_tracking_walking_timestamp = rospy.Time.now()
