from os.path import expanduser

import numpy as np
import pinocchio
import rospy
import yaml
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.model_ros.motor_control_ros import MotorControlROS
from soccer_pycontrol.model.model_ros.sensors_ros import SensorsROS


class BezROS(Bez):
    def __init__(self, ns: str = ""):
        self.ns = ns
        self.robot_model = "assembly"#rospy.get_param("robot_model", "assembly")

        if rospy.get_param("/use_sim_time", False):
            sim = "_sim"
        else:
            sim = ""

        self.parameters = self.get_parameters(sim)

        motor_offsets = self.get_motor_names()

        motor_names = list(motor_offsets.keys())[1:]
        del motor_offsets["universe"]
        self.motor_control = MotorControlROS(motor_offsets, ns)

        self.sensors = SensorsROS(ns)

    # TODO fix dupe
    def get_motor_names(self):
        urdf_model_path = (
            expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_description/{self.robot_model}" f"_description/urdf/{self.robot_model}.urdf"
        )

        model = pinocchio.buildModelFromUrdf(urdf_model_path)

        data = model.createData()

        q = np.zeros_like(pinocchio.randomConfiguration(model))
        v = pinocchio.utils.zero(model.nv)

        pinocchio.ccrba(model, data, q, v)
        # for name, oMi in zip(model.names, data.oMi):
        #     print(("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)))
        # TODO should make a unit test to make sure the data is correct and maybe use pybullet toverify
        return {
            model.names[i]: [i - 1, i - 1] for i in range(len(model.names))
        }  # {model.names[i]: data.oMi[i].translation.T for i in range(len(model.names))}

    def get_parameters(self, sim: str) -> dict:
        with open(
            expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_control/soccer_pycontrol/config/{self.robot_model}/{self.robot_model}{sim}.yaml", "r"
        ) as file:
            parameters = yaml.safe_load(file)
            file.close()
        return parameters
