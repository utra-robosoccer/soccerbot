import math
import os
import time

import rospy
import tf
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64

from soccer_msgs.msg import RobotState
from soccer_pycontrol.soccerbot import *


class SoccerbotRos(Soccerbot):
    def __init__(self, position, useFixedBase=False):

        super().__init__(position, useFixedBase)

        self.motor_publishers = {}
        self.pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=1)
        self.motor_names = [pb.getJointInfo(self.body, i)[1].decode("utf-8") for i in range(18)]
        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)
        self.odom_pose = tr()
        self.torso_height_publisher = rospy.Publisher("torso_height", Float64, queue_size=1, latch=True)
        self.path_publisher = rospy.Publisher("path", Path, queue_size=1, latch=True)
        self.path_odom_publisher = rospy.Publisher("path_odom", Path, queue_size=1, latch=True)
        self.imu_subscriber = rospy.Subscriber("imu_filtered", Imu, self.imu_callback, queue_size=1)
        self.imu_ready = False
        self.listener = tf.TransformListener()
        self.last_ball_found_timestamp = None
        self.ball_pixel_subscriber = rospy.Subscriber("ball_pixel", Pose2D, self.ball_pixel_callback, queue_size=1)
        self.ball_pixel: Pose2D = None
        self.last_ball_pixel: Pose2D = None
        self.last_ball_pixel_update = rospy.Time.now()
        self.head_centered_on_ball_publisher = rospy.Publisher("head_centered_on_ball", Empty, queue_size=1)

        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.state_callback)
        self.robot_state = RobotState()
        self.robot_state.status = RobotState.STATUS_DISCONNECTED

    def state_callback(self, robot_state: RobotState):
        self.robot_state = robot_state

    def imu_callback(self, msg: Imu):
        self.imu_msg = msg
        self.imu_ready = True

    def ball_pixel_callback(self, msg: Pose2D):
        self.ball_pixel = msg

    def publishAngles(self):
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.now()
        js.position = []
        js.effort = []
        for i, n in enumerate(self.motor_names):
            js.name.append(n)
            js.position.append(self.get_angles()[i])
        try:
            rospy.loginfo_once("Started Publishing Motors")
            self.pub_all_motor.publish(js)
        except rospy.exceptions.ROSException as ex:
            print(ex)
            exit(0)

    def stepPath(self, t, verbose=False):
        super(SoccerbotRos, self).stepPath(t, verbose=verbose)

        # Get odom from odom_path
        t_adjusted = t * self.robot_odom_path.duration() / self.robot_path.duration()
        crotch_position = self.robot_odom_path.crotchPosition(t_adjusted) @ self.torso_offset

        base_pose = crotch_position.get_position()
        base_orientation = crotch_position.get_orientation()
        self.odom_pose = tr(base_pose, base_orientation)

    def publishPath(self, robot_path=None):
        if robot_path is None:
            robot_path = self.robot_path

        def createPath(robot_path) -> Path:
            p = Path()
            p.header.frame_id = "world"
            p.header.stamp = rospy.Time.now()
            for i in range(0, robot_path.bodyStepCount() + 1, 1):
                step = robot_path.getBodyStepPose(i)
                position = step.get_position()
                orientation = step.get_orientation()
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
        if self.robot_odom_path is not None:
            self.path_odom_publisher.publish(createPath(self.robot_odom_path))

    def publishOdometry(self):
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = os.environ["ROS_NAMESPACE"][1:] + "/odom"
        o.child_frame_id = os.environ["ROS_NAMESPACE"][1:] + "/base_link"
        pose = self.odom_pose.get_position()
        o.pose.pose.position.x = pose[0]
        o.pose.pose.position.y = pose[1]
        o.pose.pose.position.z = 0
        orientation = self.odom_pose.get_orientation()
        o.pose.pose.orientation.x = orientation[0]
        o.pose.pose.orientation.y = orientation[1]
        o.pose.pose.orientation.z = orientation[2]
        o.pose.pose.orientation.w = orientation[3]

        # fmt: off
        o.pose.covariance = [1E-2, 0, 0, 0, 0, 0,
                             0, 1E-2, 0, 0, 0, 0,
                             0, 0, 1E-6, 0, 0, 0,
                             0, 0, 0, 1E-6, 0, 0,
                             0, 0, 0, 0, 1E-6, 0,
                             0, 0, 0, 0, 0, 1E-2]
        # fmt: on
        self.odom_publisher.publish(o)
        self.publishHeight()
        pass

    def publishHeight(self):
        f = Float64()
        f.data = self.pose.get_position()[2]
        self.torso_height_publisher.publish(f)
        pass

    def get_imu(self):
        assert self.imu_ready
        return tr(
            [0, 0, 0],
            [
                self.imu_msg.orientation.x,
                self.imu_msg.orientation.y,
                self.imu_msg.orientation.z,
                self.imu_msg.orientation.w,
            ],
        )

    def is_fallen(self) -> bool:
        pose = self.get_imu()
        [roll, pitch, yaw] = pose.get_orientation_euler()
        return not np.pi / 6 > pitch > -np.pi / 6

    def get_foot_pressure_sensors(self, floor):
        # TODO subscribe to foot pressure sensors
        pass

    HEAD_YAW_FREQ = 0.005
    HEAD_PITCH_FREQ = 0.005

    def apply_head_rotation(self):
        if self.robot_state.status in [self.robot_state.STATUS_DETERMINING_SIDE, self.robot_state.STATUS_PENALIZED]:
            self.configuration[Joints.HEAD_1] = math.sin(-self.head_step * SoccerbotRos.HEAD_YAW_FREQ * 3) * (math.pi * 0.05)
            self.head_step += 1
        elif self.robot_state.status == self.robot_state.STATUS_READY:
            try:
                ball_found_timestamp = self.listener.getLatestCommonTime(
                    os.environ["ROS_NAMESPACE"] + "/camera", os.environ["ROS_NAMESPACE"] + "/ball"
                )
                if self.last_ball_found_timestamp is None or ball_found_timestamp - self.last_ball_found_timestamp > rospy.Duration(2):
                    self.last_ball_found_timestamp = ball_found_timestamp

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo_throttle(5, "Ball no longer in field of view, searching for the ball")
                pass

            # So that the arms don't block the vision
            self.configuration[Joints.LEFT_ARM_2] = (0.3 - self.configuration[Joints.LEFT_ARM_2]) * 0.05 + self.configuration[Joints.LEFT_ARM_2]
            self.configuration[Joints.RIGHT_ARM_2] = (0.3 - self.configuration[Joints.RIGHT_ARM_2]) * 0.05 + self.configuration[Joints.RIGHT_ARM_2]

            # If the last time it saw the ball was 5 seconds ago
            if self.last_ball_found_timestamp is not None and (rospy.Time.now() - self.last_ball_found_timestamp) < rospy.Duration(10):

                assert self.ball_pixel is not None

                camera_movement_speed = 0.0005
                pixel_threshold = 15
                if self.ball_pixel != self.last_ball_pixel or (rospy.Time.now() - self.last_ball_pixel_update) > rospy.Duration(0.5):
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
                        rospy.loginfo_throttle(1, f"Centered Camera on Ball (x,y) ({self.ball_pixel.x}, {self.ball_pixel.y}) -> (320, 240)")

                    else:
                        rospy.loginfo_throttle(1, f"Centering Camera on Ball (x,y) ({self.ball_pixel.x}, {self.ball_pixel.y}) -> (320, 240)")

            else:
                self.configuration[Joints.HEAD_1] = math.sin(self.head_step * SoccerbotRos.HEAD_YAW_FREQ) * (math.pi / 4)
                self.configuration[Joints.HEAD_2] = math.pi * 0.185 - math.cos(self.head_step * SoccerbotRos.HEAD_PITCH_FREQ) * math.pi * 0.15
                self.head_step += 1

        elif self.robot_state.status == RobotState.STATUS_LOCALIZING:
            self.configuration[Joints.HEAD_1] = math.cos(self.head_step * SoccerbotRos.HEAD_YAW_FREQ) * (math.pi / 4)
            self.configuration[Joints.HEAD_2] = math.pi * 0.175 - math.sin(self.head_step * SoccerbotRos.HEAD_PITCH_FREQ) * math.pi * 0.15
            self.head_step += 1
        else:
            self.configuration[Joints.HEAD_1] = 0
            self.configuration[Joints.HEAD_2] = 0
            self.head_step = 0
