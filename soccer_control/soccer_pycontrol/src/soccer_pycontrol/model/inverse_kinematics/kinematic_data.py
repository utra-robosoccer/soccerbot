from os.path import expanduser

import numpy as np
import pinocchio
import yaml

from soccer_common import Transformation


class KinematicData:
    """
    Class to contain all relevant information about the kinematics of model. Uses pinnoccio to load from urdf
    """

    def __init__(
        self,
        sim: str = "_sim",
        robot_model: str = "bez1",
    ):

        with open(
            expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_control/soccer_pycontrol/config/{robot_model}/{robot_model}{sim}.yaml", "r"
        ) as file:
            parameters = yaml.safe_load(file)
            file.close()

        self.urdf_model_path = expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}" f"_description/urdf/{robot_model}.urdf"

        motor_offsets = self.load_urdf()

        self.motor_names = list(motor_offsets.keys())[1:]

        self.arm_0_center = parameters["arm_0_center"]
        self.arm_1_center = parameters["arm_1_center"]

        self.thigh_length = np.linalg.norm(motor_offsets["right_leg_motor_2"] - motor_offsets["right_leg_motor_3"])
        self.tibia_length = np.linalg.norm(motor_offsets["right_leg_motor_3"] - motor_offsets["right_leg_motor_4"])
        self.torso_to_right_hip = Transformation(position=(motor_offsets["right_leg_motor_0"] + [0, 0, 0]), euler=[0, 0, 0])
        self.right_hip_to_left_hip = Transformation(position=(motor_offsets["right_leg_motor_0"] - motor_offsets["left_leg_motor_0"]))

        self.right_foot_init_position = Transformation(position=(motor_offsets["right_leg_motor_5"] - [parameters["weird_x_offset"], 0, 0]))
        self.left_foot_init_position = Transformation(position=(motor_offsets["left_leg_motor_5"] - [parameters["weird_x_offset"], 0, 0]))

        #: Height of the robot's torso (center between two arms) while walking
        self.walking_torso_height = parameters["walking_torso_height"]

        #: Dimensions of the foot collision box #TODO get it from URDF also what do they mean
        self.foot_box = parameters["foot_box"]

        # : Transformations from the right foots joint position to the center of the collision box of the foot
        # https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        self.right_foot_joint_center_to_collision_box_center = parameters["right_foot_joint_center_to_collision_box_center"]

        self.foot_center_to_floor = -self.right_foot_joint_center_to_collision_box_center[2] + self.foot_box[2]
        self.cleats_offset = parameters["cleats_offset"]

        ready_pitch_correction = Transformation([0, 0, 0], euler=[0, parameters["torso_offset_pitch_ready"], 0])

        # TODO what does this do also dont like that it needs to happen as first step, clean up interface
        self.right_foot_init_position[2, 3] = -self.walking_torso_height + self.foot_center_to_floor
        self.right_foot_init_position[0, 3] -= parameters["torso_offset_x_ready"]
        self.right_foot_init_position = ready_pitch_correction @ self.right_foot_init_position

        self.left_foot_init_position[2, 3] = -self.walking_torso_height + self.foot_center_to_floor

        self.left_foot_init_position[0, 3] -= parameters["torso_offset_x_ready"]
        self.left_foot_init_position = ready_pitch_correction @ self.left_foot_init_position

        self.torso_offset_pitch = parameters["torso_offset_pitch"]
        self.torso_offset_x = parameters["torso_offset_x"]

    def load_urdf(self):
        # TODO should make it modular in the future if we use pinnochio more

        model = pinocchio.buildModelFromUrdf(self.urdf_model_path)

        data = model.createData()

        q = np.zeros_like(pinocchio.randomConfiguration(model))
        v = pinocchio.utils.zero(model.nv)

        pinocchio.ccrba(model, data, q, v)
        # for name, oMi in zip(model.names, data.oMi):
        #     print(("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)))
        # TODO should make a unit test to make sure the data is correct and maybe use pybullet toverify
        return {model.names[i]: data.oMi[i].translation.T for i in range(len(model.names))}


if __name__ == "__main__":
    k = KinematicData(robot_model="bez2")
    # k = KinematicData(robot_model="bez2")
