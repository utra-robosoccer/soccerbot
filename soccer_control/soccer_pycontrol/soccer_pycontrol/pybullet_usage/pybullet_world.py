import os
import time

import numpy as np

# import pybullet as pb
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
        optionstring = "--width={} --height={} ".format(640, 480)
        assert pb.isConnected() == 0
        if display:
            self.client_id = pb.connect(pb.GUI, optionstring)
        else:
            self.client_id = pb.connect(pb.DIRECT)
        pb.setTimeStep(1.0 / rate)
        # pb.setRealTimeSimulation(0)

        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=camera_yaw, cameraPitch=0, cameraTargetPosition=cameraTargetPosition)
        pb.setGravity(0, 0, -9.81)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

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
