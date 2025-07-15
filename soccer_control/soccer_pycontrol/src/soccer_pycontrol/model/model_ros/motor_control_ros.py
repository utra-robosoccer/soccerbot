from typing import List

import rclpy
from rospy import ROSException
from sensor_msgs.msg import JointState
from soccer_pycontrol.model.motor_control import MotorControl, MotorData


class MotorControlROS(MotorControl):
    def __init__(
        self,
        motor_names,
        ns: str = "",
    ):
        self.motor_names = motor_names
        self.numb_of_motors = len(self.motor_names)

        # Todo make it numpy and add getter and setters
        self.configuration = MotorData(self.motor_names)
        self.configuration_offset = MotorData(self.motor_names)

        # TODO should separate config to current and target

        # TODo fix namespace
        self.pub_all_motor = self.create_publisher(ns + "joint_command", JointState, queue_size=1)

    def set_motor(self) -> None:
        """
        Send the robot angles based on self.configuration + self.configuration_offset to ROS
        """
        js = JointState()
        js.name = []
        js.header.stamp = self.get_clock().now()
        js.position = []
        js.effort = []
        angles = self.get_angles()
        for joint in self.motor_names:
            js.name.append(joint)
            js.position.append(angles[self.motor_names[joint][0]])
        try:
            self.get_logger().info_once("Started Publishing Motors")
            self.pub_all_motor.publish(js)
        except self.exceptions.ROSException as ex:
            print(ex)
            exit(0)
