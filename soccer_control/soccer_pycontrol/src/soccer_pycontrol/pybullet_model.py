import time
from os.path import expanduser

import pybullet as pb
import pybullet_data
from soccer_pycontrol.pybullet_dynamics import PybulletDynamics

from soccer_common import Transformation


class PybulletModel:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(self, pose: Transformation = Transformation(), robot_model: str = "bez1"):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """

        home = expanduser("~")
        self.body = pb.loadURDF(
            home + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/{robot_model}.urdf",
            useFixedBase=False,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | (pb.URDF_MERGE_FIXED_LINKS if False else 0),
            basePosition=pose.position,
            baseOrientation=pose.quaternion,
        )

        self.motor_names = [pb.getJointInfo(self.body, i)[1].decode("utf-8") for i in range(18)]

    def motor_control(self, target: list) -> None:
        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 18, 1)),
            targetPositions=target,
        )


if __name__ == "__main__":
    p = PybulletModel(robot_model="bez2")
    # p.wait(1000)
