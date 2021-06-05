#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Empty
from tf.transformations import euler_from_quaternion
from enum import Enum


class BalanceClass:
    def __init__(self):
        self.Kp = 5
        self.Kd = 250
        self.Ki = 0.001

        self.desired = 0.0
        self.integral = 0
        self.derivative = 0
        self.lasterror = 0
        self.last_F = 0
        self.start = False

        self.competition = rospy.get_param("competition")

        self.imu = rospy.Subscriber('imu_filtered', Imu, self.imu_callback)
        self.start_balance = rospy.Subscriber('start_walking', Empty, self.start_walk)
        self.stop_rushed = rospy.Subscriber('terminate_walking', Empty, self.stop_walk_rushed)
        self.stop_completed = rospy.Subscriber('completed_walking', Empty, self.stop_walk_complete)

    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # print(pitch)
        competition = rospy.get_param("competition")
        # Simulation
        pub_all_motor = rospy.Publisher("all_motor", JointState, queue_size=10)

        # Competition/ Real
        left_motor_publishers = rospy.Publisher("left_arm_motor_0/command", Float64, queue_size=1)
        right_motor_publishers = rospy.Publisher("right_arm_motor_0/command", Float64, queue_size=1)

        # PID
        if self.start:
            error = self.desired - pitch
            # if not (0.02 > error > -0.02):
            #     self.integral += error
            self.derivative = error - self.lasterror

            F = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * self.derivative)
            if F > 1.57:
                F = 1.57
            elif F < -1.57:
                F = -1.57

            self.last_F = F
            self.lasterror = error
            # print(" Error: ", round(error, 4), " F: ", round(F, 4), " self.integral: ", round(self.integral, 4),
            #       " self.derivative: ", round(self.derivative, 4))

            if self.competition == "False":
                js = JointState()
                js.name = []
                js.header.stamp = rospy.Time.now()  # rospy.Time.from_seconds(self.time)
                js.position = []
                js.effort = []
                js.name.append("left_arm_motor_0")
                js.position.append(F)
                js.name.append("right_arm_motor_0")
                js.position.append(F)

                pub_all_motor.publish(js)
            elif self.competition == "True":
                right_angle = Float64()
                right_angle.data = F

                left_angle = Float64()
                left_angle.data = F

                left_motor_publishers.publish(left_angle)
                right_motor_publishers.publish(right_angle)
        # We want to publish once on state transition

    def start_walk(self, data):
        print("balance on")
        self.integral = 0
        self.derivative = 0
        self.lasterror = 0
        self.last_F = 0
        self.start = True

    def stop_walk_complete(self, data):
        # rospy.sleep(0.2)
        print("balance off")
        self.integral = 0
        self.derivative = 0
        self.lasterror = 0
        self.last_F = 0
        self.start = False

    def stop_walk_rushed(self, data):
        print("balance off")
        self.integral = 0
        self.derivative = 0
        self.lasterror = 0
        self.last_F = 0
        self.start = False

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('balance')

        r = rospy.Rate(10)
        while not rospy.has_param("competition"):
            r.sleep()

        balance_object = BalanceClass()
        balance_object.run()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
