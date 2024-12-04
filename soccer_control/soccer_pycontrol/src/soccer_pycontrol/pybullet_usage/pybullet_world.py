import os
import time

import numpy as np
import pybullet as pb
import pybullet_data


class PybulletWorld:
    """
    Class for interacting and managing with pybullet.
    """

    def __init__(
        self,
        path: str = "plane.urdf",
        position: tuple = (0, 0, 0),
        orientation: tuple = (0, 0, 0),
        lateral_friction: float = 0.9,
        spinning_friction: float = 0.9,
        rolling_friction: float = 0.0,
        display: bool = "DISPLAY" in os.environ,
        camera_yaw: float = 90,
        cameraTargetPosition: list = (0, 0, 0.45),
        real_time: bool = False,
        rate: int = 100,  # TODO should convert some of this to yaml
    ):
        """
        Initializes the ramp


        """
        self.rate = rate
        self.real_time = real_time

        assert pb.isConnected() == 0
        if display:
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)

        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=camera_yaw, cameraPitch=0, cameraTargetPosition=cameraTargetPosition)
        pb.setGravity(0, 0, -9.81)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

        if path != "":
            self.plane = pb.loadURDF(path, basePosition=position, baseOrientation=pb.getQuaternionFromEuler(orientation))
            # TODO does this need to be a global var?
            pb.changeDynamics(
                self.plane,
                linkIndex=-1,
                lateralFriction=lateral_friction,
                spinningFriction=spinning_friction,
                rollingFriction=rolling_friction,
            )
        ball = pb.loadURDF("soccerball.urdf", [1, 0, 0.1], globalScaling=0.14)
        pb.changeDynamics(ball, -1, linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001)
        pb.changeVisualShape(ball, -1, rgbaColor=[0.8, 0.8, 0.8, 1])

    def close(self):
        if pb.isConnected(self.client_id):
            pb.disconnect(self.client_id)
        assert pb.isConnected() == 0

    def wait(self, steps: int) -> None:
        for i in range(steps):
            self.step()

    def wait_motor(self) -> None:
        # TODO this if for interpolation
        for _ in np.arange(0, 1.00, 0.040):
            self.step()

    # TODO maybe put into separate part
    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)
        pb.stepSimulation()
