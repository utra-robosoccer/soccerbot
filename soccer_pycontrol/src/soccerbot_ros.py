from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from soccerbot import *
import rospy
import os


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
        self.path_publisher = rospy.Publisher("path", Path, queue_size=1)
        self.imu_subscriber = rospy.Subscriber("imu_filtered", Imu, self.imu_callback, queue_size=1)

        self.imu_ready = False
        self.odom_pose = None
        self.odom_pose_previous = None
        self.odom_ros_time_previous = None
        self.odom_seq = 0

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
        self.pub_all_motor.publish(js)

    def stepPath(self, t, verbose=False):
        super(SoccerbotRos, self).stepPath(t, verbose=verbose)

        base_pose = self.pose.get_position()
        base_orientation = self.pose.get_orientation()
        # base_pose, base_orientation = pb.getBasePositionAndOrientation(self.body) # Get odometry from the simulator itself >:)
        self.odom_pose = tr(base_pose, base_orientation)

    def publishPath(self):
        p = Path()
        p.header.frame_id = "world"
        p.header.stamp = rospy.Time.now()
        for i in range(0, self.robot_path.bodyStepCount() + 1, 1):
            step = self.robot_path.getBodyStep(i)
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
        o.header.seq = self.odom_seq
        o.header.stamp = rospy.Time.now()
        if self.odom_ros_time_previous == o.header.stamp:
            return
        o.header.frame_id = os.environ["ROS_NAMESPACE"][1:] + "/odom"
        o.child_frame_id = os.environ["ROS_NAMESPACE"][1:] + "/base_footprint"
        o.pose.pose.orientation.w = 1
        o.pose.covariance = [1E-2, 0, 0, 0, 0, 0,
                             0, 1E-2, 0, 0, 0, 0,
                             0, 0, 1E-6, 0, 0, 0,
                             0, 0, 0, 1E-6, 0, 0,
                             0, 0, 0, 0, 1E-6, 0,
                             0, 0, 0, 0, 0, 1E-2]
        o.twist.covariance = [1E-3, 0, 0, 0, 0, 0,
                             0, 1E-3, 0, 0, 0, 0,
                             0, 0, 1E-6, 0, 0, 0,
                             0, 0, 0, 1E-6, 0, 0,
                             0, 0, 0, 0, 1E-6, 0,
                             0, 0, 0, 0, 0, 1E-3]

        if self.odom_pose is not None:
            pose = self.odom_pose.get_position()
            o.pose.pose.position.x = pose[0]
            o.pose.pose.position.y = pose[1]
            o.pose.pose.position.z = 0
            orientation = self.odom_pose.get_orientation()
            o.pose.pose.orientation.x = orientation[0]
            o.pose.pose.orientation.y = orientation[1]
            o.pose.pose.orientation.z = orientation[2]
            o.pose.pose.orientation.w = orientation[3]


            # Velocity calcululations
            if self.odom_pose_previous is not None:
                dt_ros = (o.header.stamp - self.odom_ros_time_previous)
                dt = dt_ros.to_sec() + dt_ros.to_nsec() * 10E-9
                o.twist.twist.linear.x = (self.odom_pose.get_position()[0] - self.odom_pose_previous.get_position()[0]) / dt
                o.twist.twist.linear.y = (self.odom_pose.get_position()[1] - self.odom_pose_previous.get_position()[1]) / dt
                o.twist.twist.angular.z = (self.odom_pose.get_orientation_euler()[2] - self.odom_pose_previous.get_orientation_euler()[2]) / dt

            self.odom_pose_previous = self.odom_pose
        else:
            orientation = self.pose.get_orientation()
            o.pose.pose.orientation.x = orientation[0]
            o.pose.pose.orientation.y = orientation[1]
            o.pose.pose.orientation.z = orientation[2]
            o.pose.pose.orientation.w = orientation[3]

        self.odom_ros_time_previous = o.header.stamp
        self.odom_publisher.publish(o)
        self.odom_seq += 1
        pass

    def get_imu(self):
        return tr([0, 0, 0], [self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z,
                              self.imu_msg.orientation.w])

    def is_fallen(self) -> bool:
        pose = self.get_imu()
        [roll, pitch, yaw] = pose.get_orientation_euler()
        return not np.pi/6 > pitch > -np.pi/6

    def get_foot_pressure_sensors(self, floor):
        # TODO subscribe to foot pressure sensors
        pass
