from os.path import expanduser

import numpy as np
import pinocchio

from soccer_common import Transformation


class KinematicData:
    def __init__(
        self,
        robot_model: str = "bez1",
        arm_0_center: float = -0.45,  # rospy.get_param("arm_0_center", -0.45)
        arm_1_center: float = np.pi * 0.8,  # rospy.get_param("arm_0_center", np.pi * 0.8)
        walking_torso_height: float = 0.315,  # rospy.get_param("walking_torso_height", 0.315)
        ready_pitch_correction: Transformation = Transformation([0, 0, 0], euler=[0, 0, 0]),
        torso_offset_x_ready: float = 0.0,  # rospy.get_param("torso_offset_x_ready", 0.0)
    ):
        self.urdf_model_path = expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}" f"_description/urdf/{robot_model}.urdf"

        motor_offsets = self.load_urdf()

        self.arm_0_center = arm_0_center
        self.arm_1_center = arm_1_center

        self.thigh_length = (motor_offsets["right_leg_motor_2"] - motor_offsets["right_leg_motor_3"])[2]
        self.tibia_length = (motor_offsets["right_leg_motor_3"] - motor_offsets["right_leg_motor_4"])[2]
        self.torso_to_right_hip = Transformation(position=motor_offsets["right_leg_motor_0"])
        self.right_hip_to_left_hip = Transformation(position=(motor_offsets["right_leg_motor_0"] - motor_offsets["left_leg_motor_0"]))

        self.right_foot_init_position = Transformation(position=motor_offsets["right_leg_motor_5"])
        self.left_foot_init_position = Transformation(position=motor_offsets["left_leg_motor_5"])

        #: Height of the robot's torso (center between two arms) while walking
        self.walking_torso_height = walking_torso_height

        #: Dimensions of the foot collision box #TODO get it from URDF also what do they mean
        self.foot_box = [0.09, 0.07, 0.01474]
        #  rospy.get_param("foot_box", [0.09, 0.07, 0.01474])

        # : Transformations from the right foots joint position to the center of the collision box of the foot
        # https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        self.right_foot_joint_center_to_collision_box_center = [0.00385, 0.00401, -0.00737]
        # rospy.get_param("right_foot_joint_center_to_collision_box_center", [0.00385, 0.00401, -0.00737])

        self.foot_center_to_floor = -self.right_foot_joint_center_to_collision_box_center[2] + self.foot_box[2]

        # TODO what does this do also dont like that it needs to happen as first step, clean up interface
        self.right_foot_init_position[2, 3] = -self.walking_torso_height + self.foot_center_to_floor
        self.right_foot_init_position[0, 3] -= torso_offset_x_ready
        self.right_foot_init_position = ready_pitch_correction @ self.right_foot_init_position

        self.left_foot_init_position[2, 3] = -self.walking_torso_height + self.foot_center_to_floor

        self.left_foot_init_position[0, 3] -= torso_offset_x_ready
        self.left_foot_init_position = ready_pitch_correction @ self.left_foot_init_position

    def load_urdf(self):
        # TODO should make it modular in the future if we use pinnochio more

        model = pinocchio.buildModelFromUrdf(self.urdf_model_path)

        data = model.createData()

        q = np.zeros_like(pinocchio.randomConfiguration(model))
        v = pinocchio.utils.zero(model.nv)

        pinocchio.ccrba(model, data, q, v)
        # print(time.time() - s)
        # for name, oMi in zip(model.names, data.oMi):
        #     print(("{:<24} : {: .2f} {: .2f} {: .2f}"
        #            .format(name, *oMi.translation.T.flat)))
        # TODO should make a unit test to make sure the data is correct and maybe use pybullet toverify
        return {model.names[i]: data.oMi[i].translation.T for i in range(len(model.names))}


if __name__ == "__main__":

    k = KinematicData(robot_model="bez1")
