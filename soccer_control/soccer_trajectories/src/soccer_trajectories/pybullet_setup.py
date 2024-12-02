import time
from os.path import expanduser
from typing import List

import pybullet as pb
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld

from soccer_common import Transformation


class PybulletSetup:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(
        self, pose: Transformation = Transformation(), robot_model: str = "bez1", real_time=False, rate: int = 75, display=True, camera_yaw=90
    ):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """
        self.rate = rate
        self.display = display
        self.real_time = real_time

        self.ramp = PybulletWorld(
            "plane.urdf", (0, 0, 0), (0, 0, 0), lateral_friction=0.9, spinning_friction=0.9, rolling_friction=0.0, camera_yaw=camera_yaw
        )
        home = expanduser("~")
        self.body = pb.loadURDF(
            home + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/{robot_model}.urdf",
            useFixedBase=False,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | (pb.URDF_MERGE_FIXED_LINKS if False else 0),
            basePosition=pose.position,
            baseOrientation=pose.quaternion,
        )

        self.motor_names = self.find_motor_names()

        self.numb_of_motors = len(self.motor_names)
        self.max_forces = []

        for i in range(0, self.numb_of_motors):
            self.max_forces.append(pb.getJointInfo(self.body, i)[10] or 6)  # TODO why is this acting so weird

    def find_motor_names(self) -> List[str]:

        names = []
        for i in range(pb.getNumJoints(self.body)):
            joint_info = pb.getJointInfo(self.body, i)
            if joint_info[2] == pb.JOINT_REVOLUTE:
                names.append(joint_info[1].decode("utf-8"))

        return names

    def wait(self, steps) -> None:
        """
        Make the robot wait for a few steps

        :param steps: Defined by Navigator.PYBULLET_STEP, which is usually 0.01
        """
        for i in range(steps):
            self.step()

    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)
        pb.stepSimulation()

    def motor_control(self, target: list) -> None:
        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, self.numb_of_motors, 1)),
            targetPositions=target,
            forces=self.max_forces,
        )


if __name__ == "__main__":
    p = PybulletSetup(robot_model="bez2")
    p.wait(1000)
    p.ramp.close()
