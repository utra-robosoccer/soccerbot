from typing import List

import numpy as np
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
        self._joint_state_sub = self.node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 1
        )
        self.joint_state = JointState()
        self.joint_vel = [0] * 12
        self.last_time = self.node.get_clock().now()
        self.joint_angles_leg = None
        self.last_joint_angles_leg = None

        self.joint_state_ready = False

    def _joint_state_callback(self, msg:JointState):

        self.joint_state = msg
        names = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "left_ankle_roll",
                 "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch",
                 "right_ankle_roll"]
        self.joint_angles_leg = np.array([
            self.joint_state.position[self.joint_state.name.index(name)]
            for name in names
        ], dtype=np.float32)
        # if self.joint_angles_leg is None:
        self.timestep = msg.header.stamp - self.last_time

        if self.last_joint_angles_leg is not None:
            self.joint_vel = (self.joint_angles_leg - self.last_joint_angles_leg) / self.timestep
            self.joint_state_ready = True # TODO
        self.last_joint_angles_leg = self.joint_angles_leg


    def get_q_legs(self):
        assert self.joint_angles_leg is not None
        return self.joint_angles_leg


    def get_qvel_legs(self):
        return  self.joint_vel

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
