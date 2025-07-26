from typing import List

import numpy as np

# import pybullet as pb


def wrapToPi(num: float) -> float:
    """
    Wraps a angle to pi, etc -3pi -> pi
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


# x = {"1": [0,0], "2":[1,1], "3":[2,2] } # TODO do analysis if this hurts performance
# m = MotorData(x)
# print(m["1":"2"])
# print(m["1"])
# print(m["2"])
# m["1"] = 4
# m["1":"2"] = [3,4]
# print(m[:])
# m[:] = 0
# print("end", m[:])
class MotorControl:
    """
    Class controls access to motor information and sets motor angles in pybullet
    """

    # TODO update with the modified for pycontrol
    def __init__(self, body):
        self.body = body

        self.motor_names = self.find_motor_names()  # motorname : [pybullet index, array index]
        self.numb_of_motors = len(self.motor_names)

        # Todo make it numpy and add getter and setters
        self.configuration = MotorData(self.motor_names)
        self.configuration_offset = MotorData(self.motor_names)

        self.max_forces = []

        for i in range(0, self.numb_of_motors):  # TODO change
            self.max_forces.append(pb.getJointInfo(self.body, i)[10] or 6)  # TODO why is this acting so weird

        self.set_motor()

    def find_motor_names(self) -> dict:
        names = {}
        for i in range(pb.getNumJoints(self.body)):
            joint_info = pb.getJointInfo(self.body, i)
            if joint_info[2] == pb.JOINT_REVOLUTE:
                names[joint_info[1].decode("utf-8")] = [i, len(names)]

        return names

    def get_motor_pybullet_indexes(self) -> List[float]:
        return [i[0] for i in list(self.motor_names.values())]

    def get_motor_indexes(self) -> List[float]:
        return [i[1] for i in list(self.motor_names.values())]

    def get_angles(self):
        """
        Function for getting all the angles, combines the configuration with the configuration offset

        :return: All 18 angles of the robot in an array formation
        """
        # TODO is this needed
        angles = [wrapToPi(a + b) for a, b in zip(self.configuration.data, self.configuration_offset.data)]
        return angles

    def set_motor(self) -> None:
        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=self.get_motor_pybullet_indexes(),  # list(range(0, self.numb_of_motors, 1)),
            targetPositions=self.get_angles(),
            forces=self.max_forces,
        )

    def set_single_motor(self, name, target_angle: float) -> None:
        self.configuration[name] = target_angle

    def set_head_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration["head_yaw":"head_pitch"] = target_angles

    def set_right_arm_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration["right_shoulder_pitch":"right_elbow"] = target_angles

    def set_left_arm_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration["left_shoulder_pitch":"left_elbow"] = target_angles

    def set_right_leg_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration["right_hip_yaw":"right_ankle_roll"] = target_angles

    def set_left_leg_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration["left_hip_yaw":"left_ankle_roll"] = target_angles

    def set_leg_joint_2_target_angle(self, target: float) -> None:
        self.configuration_offset["left_hip_roll"] = -target
        self.configuration_offset["right_hip_roll"] = +target

    def set_leg_joint_3_target_angle(self, target: float) -> None:
        self.configuration_offset["left_hip_pitch"] = target
        self.configuration_offset["right_hip_pitch"] = target

    def set_leg_joint_5_target_angle(self, target: float) -> None:
        self.configuration_offset["left_ankle_pitch"] = target
        self.configuration_offset["right_ankle_pitch"] = target

    def set_leg_joint_6_target_angle(self, target: float) -> None:
        self.configuration_offset["left_ankle_roll"] -= target
        self.configuration_offset["right_ankle_roll"] += target

    def set_angles_from_placo(self, planner) -> None:
        for joint in self.motor_names:
            self.configuration[joint] = planner.get_joint(joint)
