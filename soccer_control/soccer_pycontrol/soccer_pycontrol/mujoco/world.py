import os
import time

import numpy as np

# import pybullet as pb
import pybullet_data


class SimWorld:
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

    def wait_motor(self) -> None:
        # TODO this if for interpolation
        for _ in np.arange(0, 1.00, 0.040):
            self.step()
