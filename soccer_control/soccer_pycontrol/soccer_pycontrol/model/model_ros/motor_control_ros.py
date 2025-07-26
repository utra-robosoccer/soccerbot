from typing import List

import rclpy
from sensor_msgs.msg import JointState
from soccer_pycontrol.model.motor_control import MotorControl, MotorData


class MotorControlROS(MotorControl):
    def __init__(
        self,
        node,
        motor_names,
        ns: str = "",
    ):
        self.node = node
        self.motor_names = motor_names
        self.numb_of_motors = len(self.motor_names)

        # Todo make it numpy and add getter and setters
        self.configuration = MotorData(self.motor_names)
        self.configuration_offset = MotorData(self.motor_names)

        # TODO should separate config to current and target

        # TODo fix namespace
        self.pub_all_motor = self.node.create_publisher(JointState, ns + "joint_command", qos_profile=10)

    def set_motor(self) -> None:
        """
        Send the robot angles based on self.configuration + self.configuration_offset to ROS
        """
        js = JointState()
        js.name = []
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.position = []
        js.effort = []
        angles = self.get_angles()
        for joint in self.motor_names:
            js.name.append(joint)
            js.position.append(angles[self.motor_names[joint][0]])
        try:
            self.node.get_logger().info("Started Publishing Motors", throttle_duration_sec=1)
            self.pub_all_motor.publish(js)
        except Exception as ex:
            print(ex)
            self.node.destroy_node()
            rclpy.shutdown()
            exit(0)
