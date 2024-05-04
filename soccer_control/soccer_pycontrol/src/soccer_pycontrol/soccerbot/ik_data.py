import numpy as np

from soccer_common import Transformation


class IKData:
    def __init__(
        self,
        thigh_length: float = 0.089,
        tibia_length: float = 0.0827,
        torso_to_right_hip: Transformation = Transformation(position=[0.0135, -0.035, -0.156]),
        right_hip_to_left_hip: Transformation = Transformation(position=[0, -0.07, 0]),
        right_foot_init_position: Transformation = Transformation(position=[0.0135, -0.035, -0.3277]),
        left_foot_init_position: Transformation = Transformation(position=[0.0135, 0.035, -0.3277]),
        arm_0_center: float = -0.45,  # rospy.get_param("arm_0_center", -0.45)
        arm_1_center: float = np.pi * 0.8,  # rospy.get_param("arm_0_center", np.pi * 0.8)
        foot_center_to_floor: float = 0.0221,
        pitch_correction: Transformation = Transformation([0, 0, 0], euler=[0, 0, 0]),
        walking_torso_height: float = 0.315,
    ):
        # TODO should be a position vector
        self.thigh_length = thigh_length
        self.tibia_length = tibia_length
        self.torso_to_right_hip = torso_to_right_hip
        self.right_hip_to_left_hip = right_hip_to_left_hip

        self.arm_0_center = arm_0_center
        self.arm_1_center = arm_1_center

        self.right_foot_init_position = right_foot_init_position
        self.left_foot_init_position = left_foot_init_position

        # TODO what does this do also dont like that it needs to happen as first step, clean up interface
        self.right_foot_init_position[2, 3] = -walking_torso_height + foot_center_to_floor
        self.right_foot_init_position[0, 3] -= 0.0  # rospy.get_param("torso_offset_x_ready", 0.0)
        # self.right_foot_init_position = pitch_correction @ self.right_foot_init_position

        self.left_foot_init_position[2, 3] = -walking_torso_height + foot_center_to_floor

        self.left_foot_init_position[0, 3] -= 0.0  # rospy.get_param("torso_offset_x_ready", 0.0)
        # self.left_foot_init_position = pitch_correction @ self.left_foot_init_position
