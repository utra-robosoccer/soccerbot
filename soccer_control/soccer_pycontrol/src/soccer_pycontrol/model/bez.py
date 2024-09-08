from os.path import expanduser

import yaml
from soccer_pycontrol.model.motor_control import MotorControl
from soccer_pycontrol.model.sensors import Sensors
from soccer_pycontrol.pybullet_usage.pybullet_load_model import LoadModel
import pybullet

from soccer_common import Transformation

import warnings


class Bez:
    """
    High level abstraction to represent the model.
    """

    # TODO should create more interfaces so that its easier to access objects, streamline interface

    def __init__(self, robot_model: str = "bez1", pose: Transformation = Transformation(), fixed_base: bool = False):
        self.robot_model = robot_model
        self.parameters = self.get_parameters()

        try:
            self.model = LoadModel(self.robot_model, self.parameters["walking_torso_height"], pose, fixed_base)

            self.motor_control = MotorControl(self.model.body)

            self.sensors = Sensors(self.model.body)
        except pybullet.error as e:
            warnings.warn(str(e))


    def get_parameters(self) -> dict:
        with open(
            expanduser("~")
            + f"/catkin_ws/src/soccerbot/soccer_control/soccer_pycontrol/config/{self.robot_model}/{self.robot_model}_sim_pybullet.yaml",
            "r",
        ) as file:
            parameters = yaml.safe_load(file)
            file.close()
        return parameters

    @staticmethod
    def fallen(pitch: float) -> bool:
        angle_threshold = 1.25  # in radian
        if pitch > angle_threshold:
            print("Fallen Front")
            return True

        elif pitch < -angle_threshold:
            print("Fallen Back")
            return True
        return False
