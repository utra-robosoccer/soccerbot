import math
import time

from sensor_msgs.msg import JointState, Imu
from soccer_msgs.msg import RobotState
from std_msgs.msg import Float64, Empty
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PointStamped
from soccer_pycontrol.soccerbot import *
import rospy
import os
import tf


class SoccerbotRos(Soccerbot):

    def __init__(self, position, useFixedBase=False):

        super().__init__(position, useFixedBase)

        self.motor_publishers = {}
        self.pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=10)
        self.motor_names = [
            "left_arm_motor_0",
            "left_arm_motor_1",
            "right_arm_motor_0",
            "right_arm_motor_1",
            "left_leg_motor_0",
            "left_leg_motor_1",
            "left_leg_motor_2",
            "left_leg_motor_3",
            "left_leg_motor_4",
            "left_leg_motor_5",
            "right_leg_motor_0",
            "right_leg_motor_1",
            "right_leg_motor_2",
            "right_leg_motor_3",
            "right_leg_motor_4",
            "right_leg_motor_5",
            "head_motor_0",
            "head_motor_1"
        ]
        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)
        self.torso_height_publisher = rospy.Publisher("torso_height", Float64, queue_size=1, latch=True)
        self.path_publisher = rospy.Publisher("path", Path, queue_size=1, latch=True)
        self.imu_subscriber = rospy.Subscriber("imu_filtered", Imu, self.imu_callback, queue_size=1)
        self.imu_ready = False
        self.listener = tf.TransformListener()
        self.last_ball_found_timestamp = None
        self.head_centered_on_ball_publisher = rospy.Publisher("head_centered_on_ball", Empty, queue_size=1)

        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.state_callback)
        self.robot_state = RobotState()
        self.robot_state.status = RobotState.STATUS_DISCONNECTED

    def state_callback(self, robot_state: RobotState):
        self.robot_state = robot_state

    def imu_callback(self, msg: Imu):
        self.imu_msg = msg
        self.imu_ready = True

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
            self.pub_all_motor.publish(js)
        except rospy.exceptions.ROSException as ex:
            print(ex)
            exit(0)

    def stepPath(self, t, verbose=False):
        super(SoccerbotRos, self).stepPath(t, verbose=verbose)

        # base_pose, base_orientation = pb.getBasePositionAndOrientation(self.body) # Get odometry from the simulator itself >:)
        # Get odometry from the simulator itself >:)

        base_pose = self.pose.get_position()
        base_orientation = self.pose.get_orientation()
        self.odom_pose = tr(base_pose, base_orientation)

    def publishPath(self, robot_path=None):
        if robot_path is None:
            robot_path = self.robot_path

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
        self.path_publisher.publish(p)

    def publishOdometry(self):
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = os.environ["ROS_NAMESPACE"] + "/odom"
        o.child_frame_id = os.environ["ROS_NAMESPACE"] + "/torso"
        pose = self.odom_pose.get_position()
        o.pose.pose.position.x = pose[0]
        o.pose.pose.position.y = pose[1]
        o.pose.pose.position.z = 0
        orientation = self.odom_pose.get_orientation()
        o.pose.pose.orientation.x = orientation[0]
        o.pose.pose.orientation.y = orientation[1]
        o.pose.pose.orientation.z = orientation[2]
        o.pose.pose.orientation.w = orientation[3]

        o.pose.covariance = [1E-2, 0, 0, 0, 0, 0,
                             0, 1E-2, 0, 0, 0, 0,
                             0, 0, 1E-6, 0, 0, 0,
                             0, 0, 0, 1E-6, 0, 0,
                             0, 0, 0, 0, 1E-6, 0,
                             0, 0, 0, 0, 0, 1E-2]
        self.odom_publisher.publish(o)
        self.publishHeight()
        pass

    def ready(self):
        super(SoccerbotRos, self).ready()

    def publishHeight(self):
        f = Float64()
        f.data = self.pose.get_position()[2]
        self.torso_height_publisher.publish(f)
        pass

    def get_imu(self):
        assert(self.imu_ready)
        return tr([0, 0, 0], [self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z,
                              self.imu_msg.orientation.w])

    def is_fallen(self) -> bool:
        pose = self.get_imu()
        [roll, pitch, yaw] = pose.get_orientation_euler()
        return not np.pi / 6 > pitch > -np.pi / 6

    def get_foot_pressure_sensors(self, floor):
        # TODO subscribe to foot pressure sensors
        pass

    HEAD_YAW_FREQ = 0.003
    HEAD_PITCH_FREQ = 0.003

    def apply_head_rotation(self):
        if self.robot_state.status == self.robot_state.STATUS_DETERMINING_SIDE:
            self.configuration[Joints.HEAD_1] = math.sin(self.head_step * SoccerbotRos.HEAD_YAW_FREQ) * (math.pi / 16)
            self.head_step += 1
        elif self.robot_state.status == self.robot_state.STATUS_READY:
            recenterCameraOnBall = False
            try:
                (trans, rot) = self.listener.lookupTransform(os.environ["ROS_NAMESPACE"] + '/camera',
                                                             os.environ["ROS_NAMESPACE"] + '/ball',
                                                             rospy.Time(0))
                ball_found_timestamp = self.listener.getLatestCommonTime(os.environ["ROS_NAMESPACE"] + '/camera',
                                                                         os.environ["ROS_NAMESPACE"] + '/ball')
                if rospy.Time.now() - ball_found_timestamp > rospy.Duration(1):
                    self.last_ball_found_timestamp = None
                    rospy.loginfo_throttle(5, "Ball no longer in field of view, searching for the ball")
                else:
                    self.last_ball_found_timestamp = ball_found_timestamp
                    recenterCameraOnBall = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.last_ball_found_timestamp = None
                rospy.loginfo_throttle(5, "Ball no longer in field of view, searching for the ball")
                pass

            # Search for the ball if can't find the ball
            if self.last_ball_found_timestamp is None:
                self.configuration[Joints.HEAD_1] = math.sin(self.head_step * SoccerbotRos.HEAD_YAW_FREQ) * (math.pi / 4)
                self.configuration[Joints.HEAD_2] = math.pi * 0.175 - math.cos(self.head_step * SoccerbotRos.HEAD_PITCH_FREQ) * math.pi * 0.15
                self.head_step += 1

            # Recenter the head onto the ball
            elif recenterCameraOnBall:
                anglelr = math.atan2(trans[1], trans[0])
                angleud = math.atan2(trans[2], trans[0])

                rospy.loginfo_throttle(2, f"Centering Camera on Ball ({ anglelr }, { angleud })")

                if abs(anglelr) < 0.05 and abs(angleud) < 0.05:
                    rospy.loginfo_throttle(10, "\033[1mCamera Centered on ball\033[0m")
                    rospy.sleep(0.5)
                    self.head_centered_on_ball_publisher.publish()

                self.configuration[Joints.HEAD_1] = self.configuration[Joints.HEAD_1] + anglelr * 0.0025
                self.configuration[Joints.HEAD_2] = self.configuration[Joints.HEAD_2] - angleud * 0.0016

        elif self.robot_state.status == RobotState.STATUS_LOCALIZING:
            self.configuration[Joints.HEAD_1] = math.sin(self.head_step * SoccerbotRos.HEAD_YAW_FREQ) * (math.pi / 4)
            self.configuration[Joints.HEAD_2] = math.pi * 0.175 - math.cos(self.head_step * SoccerbotRos.HEAD_PITCH_FREQ) * math.pi * 0.15
            self.head_step += 1
        else:
            self.configuration[Joints.HEAD_1] = 0
            self.configuration[Joints.HEAD_2] = 0

