from sensor_msgs.msg import JointState
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
        # self.motor_publishers[Joints.LEFT_ARM_1] = rospy.Publisher("left_arm_motor_0/command", Float64, queue_size=1)
        # self.motor_publishers[Joints.LEFT_ARM_2] = rospy.Publisher("left_arm_motor_1/command", Float64, queue_size=1)
        # self.motor_publishers[Joints.RIGHT_ARM_1] = rospy.Publisher("right_arm_motor_0/command", Float64, queue_size=1)
        # self.motor_publishers[Joints.RIGHT_ARM_2] = rospy.Publisher("right_arm_motor_1/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_1 - 4] = rospy.Publisher("left_leg_motor_0/command", Float64,
                                                                       queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_2 - 4] = rospy.Publisher("left_leg_motor_1/command", Float64,
                                                                       queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_3 - 4] = rospy.Publisher("left_leg_motor_2/command", Float64,
                                                                       queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_4 - 4] = rospy.Publisher("left_leg_motor_3/command", Float64,
                                                                       queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_5 - 4] = rospy.Publisher("left_leg_motor_4/command", Float64,
                                                                       queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_6 - 4] = rospy.Publisher("left_leg_motor_5/command", Float64,
                                                                       queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_1 - 4] = rospy.Publisher("right_leg_motor_0/command", Float64,
                                                                        queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_2 - 4] = rospy.Publisher("right_leg_motor_1/command", Float64,
                                                                        queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_3 - 4] = rospy.Publisher("right_leg_motor_2/command", Float64,
                                                                        queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_4 - 4] = rospy.Publisher("right_leg_motor_3/command", Float64,
                                                                        queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_5 - 4] = rospy.Publisher("right_leg_motor_4/command", Float64,
                                                                        queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_6 - 4] = rospy.Publisher("right_leg_motor_5/command", Float64,
                                                                        queue_size=1)

        self.pub_all_motor = rospy.Publisher("all_motor", JointState, queue_size=10)
        self.motor_names = ["left_leg_motor_0", "left_leg_motor_1",
                            "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                            "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3",
                            "right_leg_motor_4", "right_leg_motor_5"
                            ]
        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)
        r = rospy.Rate(10)
        while not rospy.has_param("competition"):
            r.sleep()
        self.competition = rospy.get_param("competition")
        self.path_publisher = rospy.Publisher("path", Path, queue_size=1)

    def publishAngles(self):
        ban_list = ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1"]
        if self.competition == "True":
            for m in self.motor_publishers:
                self.motor_publishers[m].publish(self.configuration[m + 4])  # Skips arms
        elif self.competition == "False":
            js = JointState()
            js.name = []
            js.header.stamp = rospy.Time.now()  # rospy.Time.from_seconds(self.time)
            js.position = []
            js.effort = []
            for i, n in enumerate(self.motor_names):
                js.name.append(n)
                js.position.append(self.configuration[i + 4])
            self.pub_all_motor.publish(js)

    def stepPath(self, t, verbose=False):
        super(SoccerbotRos, self).stepPath(t, verbose=verbose)

        # Get odometry from the simulator itself >:)
        base_pose, base_orientation = pb.getBasePositionAndOrientation(self.body)
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
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = os.environ["ROS_NAMESPACE"] + "/odom"
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
        pass

    def get_imu(self, verbose=False):
        # TODO get ROS IMU
        pass

    def get_feet(self, floor):
        # TODO subscribe to foot pressure sensors
        pass
