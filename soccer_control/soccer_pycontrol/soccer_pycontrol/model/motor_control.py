from typing import List

import mujoco
import numpy as np


def wrapToPi(num: float) -> float: # TODO put in common
    """
    Wraps a angle to pi, etc -3pi -> pi
    :param num: Angle in radians
    """
    rem = (num + np.pi) % (2 * np.pi) - np.pi
    return rem

# class MotorData:
#     def __init__(self, motor_names: dict):
#         self.motor_names = motor_names
#         self.data = [0.0] * len(motor_names)
#
#     def __getitem__(self, index):
#         if isinstance(index, slice):
#             if type(index.start) is str:
#                 return self.data[self.motor_names[index.start][1] : self.motor_names[index.stop][1] + 1]
#             else:
#                 return self.data[slice(index.start, index.stop)]
#         if type(index) is str:
#             return self.data[self.motor_names[index][1]]
#
#         return self.data[index]
#
#     def __setitem__(self, index, value):
#         if isinstance(index, slice):
#             if type(index.start) is str:
#                 self.data[self.motor_names[index.start][1] : self.motor_names[index.stop][1] + 1] = value
#             else:
#                 self.data[slice(index.start, index.stop)] = value
#         else:
#             if type(index) is str:
#                 self.data[self.motor_names[index][1]] = value
#             else:
#                 self.data[index] = value
#
#     def reset(self):
#         self.data = [0.0] * len(self.motor_names)
class MotorControl:
    """
    Class controls access to motor information and sets motor angles in MuJoCo
    """

    # TODO update with the modified for pycontrol
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        # Retrieve the degrees of freedom id/name pairs
        joints = len(self.model.jnt_pos) - 1  # -1 for ball
        #  Used for reading from joint data not for controlling
        self.dofs = [[k, self.model.jnt(k).name] for k in range(1, joints)]
        self.dofs_to_index = {dof: k for k, dof in self.dofs}

        self.dof_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(self.model.nu)]
        self.ctrl_dofs_to_index = {}
        for name in self.dof_names:
            self.ctrl_dofs_to_index[name] = self.get_actuator_index(name)

        self.configuration = np.zeros(self.dof)
        self.configuration_offset = np.zeros(self.dof)

        self.set_motor()

    @property
    def dof(self):  # TODO might rename
        return len(self.dof_names)

    def get_range(self, name: str) -> np.ndarray:
        # Range of the joint
        return self.model.actuator_ctrlrange[self.get_actuator_index(name)]

    def get_q(self, name: str):
        """
        Gets the position of a given joint.

        Args:
            name (str): joint name

        Returns:
            float: joint position
        """
        # is the actual position
        addr = self.model.jnt_qposadr[self.dofs_to_index[name]]
        return self.data.qpos[addr]

    def get_qdot(self, name: str):
        """
        Gets the velocity of a given joint.

        Args:
            name (str): joint name

        Returns:
            float: joint velocity
        """
        addr = self.model.jnt_dofadr[self.dofs_to_index[name]]
        return self.data.qvel[addr]

    def set_q(self, name: str, value: float):
        """
        Sets a value of a given joint.

        Args:
            name (str): joint name
            value (float): target value
        """
        addr = self.model.jnt_qposadr[self.dofs_to_index[name]]
        self.data.qpos[addr] = value

    def get_control(self, name: str):
        """
        Gets the control for a given actuator

        Args:
            name (str): actuator name
        """
        # gets the target joint value
        actuator_idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        return self.data.ctrl[actuator_idx]

    def get_actuator_index(self, name: str):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def set_single_motor(self, name: str, value: float, reset: bool = False) -> None:
        """
        Sets the control for a given actuator.
        If the actuator is a position actuator, the value is the desired position.
        If the actuator is a motor actuator, the value is the desired torque.

        Args:
            name (str): actuator name
            value (float): target value
        """

        # Sets the target position
        actuator_idx = self.get_actuator_index(name)
        # self.data.ctrl[actuator_idx] = value
        self.configuration[actuator_idx] = value

        # Sets the position
        if reset:
            self.set_q(name, value)

    def reset_motors(self, q: np.ndarray):
        for idx, name in enumerate(self.ctrl_dofs_to_index.keys()):
            self.set_q(name, q[idx])

    def clear_motor_target(self):
        self.configuration.fill(0)
        self.configuration_offset.fill(0)

    def get_angles(self):
        """
        Function for getting all the angles, combines the configuration with the configuration offset

        :return: All angles of the robot in an array formation
        """
        # TODO is this needed
        angles = [wrapToPi(a + b) for a, b in zip(self.configuration, self.configuration_offset)]
        return angles

    def set_motor(self) -> None:
        indexes = list(self.ctrl_dofs_to_index.values())
        self.data.ctrl[indexes] = self.get_angles()

    def get_dof_indexes(self, names: List[str]) -> list:
        index = []
        for name in names:
            index.append(self.ctrl_dofs_to_index[name])
        return index

    def set_head_target_angles(self, target_angles: np.ndarray) -> None:
        dof_idx = self.get_dof_indexes(["head_yaw", "head_pitch"])
        self.configuration[dof_idx] = target_angles

    def set_right_arm_target_angles(self, target_angles: np.ndarray) -> None:
        dof_idx = self.get_dof_indexes(["right_shoulder_pitch", "right_shoulder_roll", "right_elbow"])
        self.configuration[dof_idx] = target_angles

    def set_left_arm_target_angles(self, target_angles: np.ndarray) -> None:
        dof_idx = self.get_dof_indexes(["left_shoulder_pitch", "left_shoulder_roll", "left_elbow"])
        self.configuration[dof_idx] = target_angles

    def set_right_leg_target_angles(self, target_angles: np.ndarray) -> None:
        dof_idx = self.get_dof_indexes(["right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch", "right_ankle_roll"])
        self.configuration[dof_idx] = target_angles

    def set_left_leg_target_angles(self, target_angles: np.ndarray) -> None:
        dof_idx = self.get_dof_indexes(["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "left_ankle_roll"])
        self.configuration[dof_idx] = target_angles

    def set_leg_hip_roll_target_angle(self, target: float) -> None:
        self.configuration_offset[self.ctrl_dofs_to_index["left_hip_roll"]] = -target # todo maybe can optimize thhis
        self.configuration_offset[self.ctrl_dofs_to_index["right_hip_roll"]] = +target

    def set_leg_hip_pitch_target_angle(self, target: float) -> None:
        self.configuration_offset[self.ctrl_dofs_to_index["left_hip_pitch"]] = target
        self.configuration_offset[self.ctrl_dofs_to_index["right_hip_pitch"]] = target

    def set_leg_ankle_pitch_target_angle(self, target: float) -> None:
        self.configuration_offset[self.ctrl_dofs_to_index["left_ankle_pitch"]] = target
        self.configuration_offset[self.ctrl_dofs_to_index["right_ankle_pitch"]] = target

    def set_leg_ankle_roll_target_angle(self, target: float) -> None:
        self.configuration_offset[self.ctrl_dofs_to_index["left_ankle_roll"]] -= target
        self.configuration_offset[self.ctrl_dofs_to_index["right_ankle_roll"]] += target

    def set_angles_from_placo(self, planner) -> None:
        for joint in self.ctrl_dofs_to_index.keys():
            self.configuration[self.ctrl_dofs_to_index[joint]] = planner.get_joint(joint)
