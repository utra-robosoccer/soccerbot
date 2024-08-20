import numpy as np
import pybullet as pb

from soccer_common.utils import wrapToPi


class MotorControl:
    """
    Class controls access to motor information and sets motor angles in pybullet
    """

    # TODO update with the modified for pycontrol
    def __init__(self, robot_model: str, body: pb.loadURDF):
        self.body = body

        mot = []
        for i in range(pb.getNumJoints(self.body)):
            joint_info = pb.getJointInfo(self.body, i)
            if joint_info[2] == pb.JOINT_REVOLUTE:
                mot.append(joint_info[1].decode("utf-8"))
        self.motor_names = mot

        if robot_model == "sigmaban":
            self.motor_names = [pb.getJointInfo(self.body, i)[1].decode("utf-8") for i in range(pb.getNumJoints(self.body))]

        self.numb_of_motors = len(self.motor_names)
        # Todo make it numpy and add getter and setters
        self.configuration = [0.0] * self.numb_of_motors
        self.configuration_offset = [0.0] * self.numb_of_motors

        self.max_forces = []

        for i in range(0, self.numb_of_motors):
            self.max_forces.append(pb.getJointInfo(self.body, i)[10] or 6)  # TODO why is this acting so weird

        self.set_motor()

    def get_angles(self):
        """
        Function for getting all the angles, combines the configuration with the configuration offset

        :return: All 18 angles of the robot in an array formation
        """
        angles = [wrapToPi(a + b) for a, b in zip(self.configuration, self.configuration_offset)]
        return angles

    def set_motor(self) -> None:
        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, self.numb_of_motors, 1)),
            targetPositions=self.get_angles(),
            forces=self.max_forces,
        )

    def set_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[0 : self.numb_of_motors] = target_angles

    def set_head_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[self.motor_names.index("head_yaw") : self.motor_names.index("head_pitch") + 1] = (
            target_angles  # TODO find new links number
        )

    def set_right_arm_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[self.motor_names.index("right_shoulder_pitch") : self.motor_names.index("right_elbow") + 1] = (
            target_angles  # TODO find new links number
        )

    def set_left_arm_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[self.motor_names.index("left_shoulder_pitch") : self.motor_names.index("left_elbow") + 1] = (
            target_angles  # TODO find new links number
        )

    def set_right_leg_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[self.motor_names.index("right_hip_yaw") : self.motor_names.index("right_ankle_roll") + 1] = (
            target_angles  # TODO find new links number
        )

    def set_left_leg_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[self.motor_names.index("left_hip_yaw") : self.motor_names.index("left_ankle_roll") + 1] = (
            target_angles  # TODO find new links number
        )

    def set_leg_joint_2_target_angle(self, target: float) -> None:
        self.configuration_offset[self.motor_names.index("left_hip_roll")] -= target
        self.configuration_offset[self.motor_names.index("right_hip_roll")] += target

    def set_leg_joint_3_target_angle(self, target: float) -> None:
        self.configuration_offset[self.motor_names.index("left_hip_pitch")] = +target
        self.configuration_offset[self.motor_names.index("right_hip_pitch")] = +target

    def set_leg_joint_5_target_angle(self, target: float) -> None:
        self.configuration_offset[self.motor_names.index("left_ankle_pitch")] = target
        self.configuration_offset[self.motor_names.index("right_ankle_pitch")] = target

    def set_leg_joint_6_target_angle(self, target: float) -> None:
        self.configuration_offset[self.motor_names.index("left_ankle_roll")] -= target
        self.configuration_offset[self.motor_names.index("right_ankle_roll")] += target
