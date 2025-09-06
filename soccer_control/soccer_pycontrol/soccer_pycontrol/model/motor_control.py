from typing import List

import numpy as np

from soccer_pycontrol.mujoco.simulator import Simulator  # Import Simulator for MuJoCo motor control


def wrapToPi(num: float) -> float:
    """
    Wraps an angle to [-pi, pi].
    :param num: Angle in radians
    """
    rem = (num + np.pi) % (2 * np.pi) - np.pi
    return rem


class MotorData:
    def __init__(self, motor_names: dict):
        self.motor_names = motor_names
        self.data = [0.0] * len(motor_names)

    def __getitem__(self, index):
        if isinstance(index, slice):
            if type(index.start) is str:
                return self.data[self.motor_names[index.start][1] : self.motor_names[index.stop][1] + 1]
            else:
                return self.data[slice(index.start, index.stop)]
        if type(index) is str:
            return self.data[self.motor_names[index][1]]

        return self.data[index]

    def __setitem__(self, index, value):
        if isinstance(index, slice):
            if type(index.start) is str:
                self.data[self.motor_names[index.start][1] : self.motor_names[index.stop][1] + 1] = value
            else:
                self.data[slice(index.start, index.stop)] = value
        else:
            if type(index) is str:
                self.data[self.motor_names[index][1]] = value
            else:
                self.data[index] = value

    def reset(self):
        self.data = [0.0] * len(self.motor_names)


class MotorControl:
    """
    Controller for managing motor/actuator commands in MuJoCo simulations.

    This class handles setting joint angles, applying offsets, and sending control
    signals to the MuJoCo actuators. It replaces PyBullet's motor control with
    MuJoCo's actuator-based system.

    Attributes:
        simulator (Simulator): The MuJoCo simulator instance.
        motor_names (dict): Mapping of motor names to indices.
        numb_of_motors (int): Total number of motors.
        configuration (MotorData): Current motor angle targets.
        configuration_offset (MotorData): Offset values for motor angles.
        max_forces (list): Maximum forces for each motor.
    """

    def __init__(self, simulator: Simulator):
        """
        Initialize motor control with a MuJoCo Simulator.

        Args:
            simulator (Simulator): The MuJoCo simulator instance.
        """
        self.simulator = simulator
        self.motor_names = self.find_motor_names()  # Map actuator names to indices
        self.numb_of_motors = len(self.motor_names)
        self.configuration = MotorData(self.motor_names)
        self.configuration_offset = MotorData(self.motor_names)
        self.max_forces = [100.0] * self.numb_of_motors  # Default max forces
        self.set_motor()  # Apply initial motor settings

    def find_motor_names(self) -> dict:
        """
        Discover and map actuator names from the MuJoCo model.

        Returns:
            dict: Mapping of actuator names to [index, array_index] pairs.
        """
        names = {}
        actuator_names = self.simulator.dof_names()  # Get names from simulator
        for idx, name in enumerate(actuator_names):
            names[name] = [idx, len(names)]
        return names

    def get_motor_indexes(self) -> List[int]:
        """
        Get the actuator indices for control.

        Returns:
            List[int]: List of actuator indices.
        """
        return [i[0] for i in list(self.motor_names.values())]

    def get_angles(self):
        """
        Compute the final motor angles by combining configuration and offsets.

        Returns:
            list: List of wrapped angles for all motors.
        """
        angles = [wrapToPi(a + b) for a, b in zip(self.configuration.data, self.configuration_offset.data)]
        return angles

    def set_motor(self) -> None:
        """
        Send motor control commands to MuJoCo actuators.
        """
        for name in self.motor_names:
            target_angle = self.configuration[name] + self.configuration_offset[name]
            self.simulator.set_control(name, wrapToPi(target_angle))  # Apply wrapped angle

    def set_single_motor(self, name: str, target_angle: float) -> None:
        """
        Set the target angle for a single motor.

        Args:
            name (str): Motor name.
            target_angle (float): Desired angle.
        """
        self.configuration[name] = target_angle
        self.set_motor()  # Update immediately

    def set_head_target_angles(self, target_angles: np.ndarray) -> None:
        """
        Set angles for head motors.

        Args:
            target_angles (np.ndarray): Angles for yaw and pitch.
        """
        self.configuration["head_yaw":"head_pitch"] = target_angles
        self.set_motor()

    def set_right_arm_target_angles(self, target_angles: np.ndarray) -> None:
        """
        Set angles for right arm motors.

        Args:
            target_angles (np.ndarray): Angles for shoulder and elbow.
        """
        self.configuration["right_shoulder_pitch":"right_elbow"] = target_angles
        self.set_motor()

    def set_left_arm_target_angles(self, target_angles: np.ndarray) -> None:
        """
        Set angles for left arm motors.

        Args:
            target_angles (np.ndarray): Angles for shoulder and elbow.
        """
        self.configuration["left_shoulder_pitch":"left_elbow"] = target_angles
        self.set_motor()

    def set_right_leg_target_angles(self, target_angles: np.ndarray) -> None:
        """
        Set angles for right leg motors.

        Args:
            target_angles (np.ndarray): Angles for leg joints.
        """
        self.configuration["right_hip_yaw":"right_ankle_roll"] = target_angles
        self.set_motor()

    def set_left_leg_target_angles(self, target_angles: np.ndarray) -> None:
        """
        Set angles for left leg motors.

        Args:
            target_angles (np.ndarray): Angles for leg joints.
        """
        self.configuration["left_hip_yaw":"left_ankle_roll"] = target_angles
        self.set_motor()

    def set_leg_joint_2_target_angle(self, target: float) -> None:
        """
        Apply hip roll offset for balance.

        Args:
            target (float): Offset value.
        """
        self.configuration_offset["left_hip_roll"] = -target
        self.configuration_offset["right_hip_roll"] = +target
        self.set_motor()

    def set_leg_joint_3_target_angle(self, target: float) -> None:
        """
        Apply hip pitch offset for balance.

        Args:
            target (float): Offset value.
        """
        self.configuration_offset["left_hip_pitch"] = target
        self.configuration_offset["right_hip_pitch"] = target
        self.set_motor()

    def set_leg_joint_5_target_angle(self, target: float) -> None:
        """
        Apply ankle pitch offset.

        Args:
            target (float): Offset value.
        """
        self.configuration_offset["left_ankle_pitch"] = target
        self.configuration_offset["right_ankle_pitch"] = target
        self.set_motor()

    def set_leg_joint_6_target_angle(self, target: float) -> None:
        """
        Apply ankle roll offset.

        Args:
            target (float): Offset value.
        """
        self.configuration_offset["left_ankle_roll"] -= target
        self.configuration_offset["right_ankle_roll"] += target
        self.set_motor()

    def set_angles_from_placo(self, planner) -> None:
        """
        Set motor angles based on Placo planner output.

        Args:
            planner: Placo planner object with get_joint method.
        """
        for joint in self.motor_names:
            self.configuration[joint] = planner.get_joint(joint)
        self.set_motor()
