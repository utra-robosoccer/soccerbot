from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from soccerbot import *
import rospy

class SoccerbotRos(Soccerbot):

    def __init__(self, position, useFixedBase=False):

        super().__init__(position, useFixedBase)

        self.motor_publishers = {}
        self.motor_publishers[Joints.LEFT_ARM_1] = rospy.Publisher("left_arm_motor_0/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_ARM_2] = rospy.Publisher("left_arm_motor_1/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_ARM_1] = rospy.Publisher("right_arm_motor_0/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_ARM_2] = rospy.Publisher("right_arm_motor_1/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_1] = rospy.Publisher("left_leg_motor_0/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_2] = rospy.Publisher("left_leg_motor_1/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_3] = rospy.Publisher("left_leg_motor_2/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_4] = rospy.Publisher("left_leg_motor_3/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_5] = rospy.Publisher("left_leg_motor_4/command", Float64, queue_size=1)
        self.motor_publishers[Joints.LEFT_LEG_6] = rospy.Publisher("left_leg_motor_5/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_1] = rospy.Publisher("right_leg_motor_0/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_2] = rospy.Publisher("right_leg_motor_1/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_3] = rospy.Publisher("right_leg_motor_2/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_4] = rospy.Publisher("right_leg_motor_3/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_5] = rospy.Publisher("right_leg_motor_4/command", Float64, queue_size=1)
        self.motor_publishers[Joints.RIGHT_LEG_6] = rospy.Publisher("right_leg_motor_5/command", Float64, queue_size=1)

        self.pub_all_motor = rospy.Publisher("all_motor", JointState, queue_size=10)
        self.motor_names = ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1",
                            "left_leg_motor_0", "left_leg_motor_1",
                            "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                            "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3",
                            "right_leg_motor_4", "right_leg_motor_5"
                            ]

    def publishAngles(self):
        # for m in self.motor_publishers:
        # self.motor_publishers[m].publish(self.configuration[m])
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.now()  # rospy.Time.from_seconds(self.time)
        js.position = []
        js.effort = []
        for i, n in enumerate(self.motor_names):
            js.name.append(n)
            js.position.append(self.configuration[i])
        self.pub_all_motor.publish(js)


    def get_imu(self, verbose=False):
        # TODO get ROS IMU
        pass

    def get_feet(self, floor):
        # TODO subscribe to foot pressure sensors
        pass
