import time
from os.path import expanduser

import pybullet as pb
import pybullet_data
from soccer_pycontrol.pybullet_dynamics import PybulletDynamics

from soccer_common import Transformation


class PybulletWorld:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(
        self,
        real_time: bool = False,
        rate: int = 100,
        display: bool = True,
    ):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """
        self.rate = rate
        self.display = display
        self.real_time = real_time

        assert pb.isConnected() == 0
        if display:
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)

        self.setup_world()

        self.world = PybulletDynamics("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

    @staticmethod
    def setup_world():
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=90, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    def close(self):
        if pb.isConnected(self.client_id):
            pb.disconnect(self.client_id)
        assert pb.isConnected() == 0

    def wait(self, steps) -> None:
        for i in range(steps):
            self.step()

    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)
        pb.stepSimulation()


if __name__ == "__main__":
    p = PybulletWorld(real_time=True)
    p.wait(1000)
