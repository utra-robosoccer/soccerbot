import time
from os.path import expanduser
from typing import List

import pybullet as pb
import pybullet_data

from soccer_common.transformation import Transformation
from soccer_pycontrol.links import Links
from soccer_pycontrol.ramp import Ramp


class PybulletSetup:
    """
    The 2D Navigator class, has a running loop that reads commands by the user and outputs actions to the soccerbot
    class.
    Doesn't require ROS and used for unit tests. All functions called here should be related to pybullet simulation
    """

    PYBULLET_STEP = 0.01

    def __init__(self, real_time=False, display=True, useCalibration=True, robot_model: str = "bez1"):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """
        self.display = display
        self.real_time = real_time
        assert pb.isConnected() == 0
        if display:
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=90, cameraPitch=0,
                                      cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

        home = expanduser("~")
        self.body = pb.loadURDF(
            home
            + f"/catkin_ws/src/soccerbot/{robot_model}_description/urdf/{robot_model}.urdf",
            useFixedBase=False,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | (pb.URDF_MERGE_FIXED_LINKS if False else 0),
            basePosition=[0, 0, 0.315],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
        )

        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9,
                         rollingFriction=0.0)

        self.motor_names = [pb.getJointInfo(self.body, i)[1].decode("utf-8") for i in range(18)]
        self.t = 0

    def close(self):
        if pb.isConnected(self.client_id):
            pb.disconnect(self.client_id)
        assert pb.isConnected() == 0

    def wait(self, steps) -> None:
        """
        Make the robot wait for a few steps

        :param steps: Defined by Navigator.PYBULLET_STEP, which is usually 0.01
        """
        for i in range(steps):
            if self.real_time:
                time.sleep(self.PYBULLET_STEP)
            pb.stepSimulation()

    @staticmethod
    def step() -> None:
        pb.stepSimulation()
        time.sleep(1 / 100)

    def motor_control(self, target: list) -> None:
        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 18, 1)),
            targetPositions=target,
        )


if __name__ == "__main__":
    p = PybulletSetup(robot_model="bez2")
    t = 0
    for i in range(1000):
        pb.stepSimulation()
        time.sleep(1 / 100)
