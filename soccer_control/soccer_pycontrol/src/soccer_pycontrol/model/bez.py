from soccer_pycontrol.model.inverse_kinematics.ik_actions import IKActions
from soccer_pycontrol.model.inverse_kinematics.kinematic_data import KinematicData
from soccer_pycontrol.model.motor_control import MotorControl
from soccer_pycontrol.model.sensors import Sensors
from soccer_pycontrol.pybullet_usage.pybullet_load_model import LoadModel

from soccer_common import Transformation


class Bez:
    """
    High level abstraction to represent the model.
    """

    # TODO should create more interfaces so that its easier to access objects, streamline interface

    def __init__(
        self,
        robot_model: str = "bez1",
        pose: Transformation = Transformation(),
        fixed_base: bool = False,
    ):
        self.robot_model = robot_model
        self.data = KinematicData(robot_model=robot_model)

        self.model = LoadModel(self.data.urdf_model_path, self.data.walking_torso_height, pose, fixed_base)

        self.motor_control = MotorControl(self.model.body, self.data.motor_names)
        self.sensors = Sensors(self.model.body)

        self.ik_actions = IKActions(self.data)

    def ready(self) -> None:
        """
        Puts the robot into a ready pose to begin walking
        """
        self.motor_control.set_target_angles(self.ik_actions.ready())
        # TODO maybe change name to action or make it more clear

    def find_joint_angles(self, torso_to_right_foot: Transformation, torso_to_left_foot: Transformation):
        r_theta = self.ik_actions.get_right_leg_angles(torso_to_right_foot)
        l_theta = self.ik_actions.get_left_leg_angles(torso_to_left_foot)
        self.motor_control.set_right_leg_target_angles(r_theta[0:6])
        self.motor_control.set_left_leg_target_angles(l_theta[0:6])

    @staticmethod
    def fallen(pitch: float) -> bool:
        angle_threshold = 1.25  # in radian
        if pitch > angle_threshold:
            print("Fallen Back")
            return True

        elif pitch < -angle_threshold:
            print("Fallen Front")
            return True
