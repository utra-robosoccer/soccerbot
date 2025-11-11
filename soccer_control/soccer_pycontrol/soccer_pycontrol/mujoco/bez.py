import time
from os.path import expanduser

import numpy as np
import yaml
from soccer_pycontrol.mujoco.motor_control import MotorControl
from soccer_pycontrol.mujoco.sensors import Sensors
from soccer_pycontrol.mujoco.simulator import SimWorld
from soccer_pycontrol.mujoco.status_enum import BezStatusEnum

from soccer_common import Transformation


class Bez:
    """
    High level abstraction to represent the model.
    """

    # TODO should create more interfaces so that its easier to access objects, streamline interface

    def __init__(
        self,
        world: SimWorld,
        robot_model: str = "assembly",
        pose: Transformation = Transformation(),
        fixed_base: bool = False,
        status: BezStatusEnum = BezStatusEnum.BALANCE,
    ):
        self.world = world
        self.robot_model = robot_model
        self.parameters = self.get_parameters()
        self._status = status

        self.motor_control = MotorControl(self.world.model, self.world.data)

        self.sensors = Sensors(self.world)

    @property
    def status(self) -> BezStatusEnum:
        return self._status

    @status.setter
    def status(self, status: BezStatusEnum) -> None:
        self._status = status

    def get_parameters(self) -> dict:
        with open(
            expanduser("~")
            + f"/ros2_ws/src/soccerbot/soccer_control/soccer_pycontrol/config/{self.robot_model}/{self.robot_model}_sim_mujoco.yaml",
            "r",
        ) as file:
            parameters = yaml.safe_load(file)
            file.close()
        return parameters

    @staticmethod
    def fallen(pitch: float) -> bool:  # TODO add fallen side
        angle_threshold = 1.25  # in radian
        if pitch > angle_threshold:
            print("Fallen Front")
            return True

        elif pitch < -angle_threshold:
            print("Fallen Back")
            return True
        return False

    @property
    def is_balance(self) -> bool:
        return self.status == BezStatusEnum.BALANCE

    @property
    def is_find_ball(self) -> bool:
        return self.status == BezStatusEnum.FIND_BALL

    @property
    def is_walk(self) -> bool:
        return self.status == BezStatusEnum.WALK

    @property
    def is_fallen(self) -> bool:
        return self.status == BezStatusEnum.FALLEN


if __name__ == "__main__":

    # sim = Simulator(scene_name="scene_bez1.xml")
    sim = SimWorld(scene_name="scene_bez2.xml")
    bez = Bez(sim)

    # sim = Simulator(scene_name="scene_bez3.xml")
    # euler="-1.57  0 0.2 "  left_hip_pitch
    sim.step()
    sim.set_T_world_site("left_foot", np.eye(4))

    sim.step()
    start = time.time()

    while True:
        sim.render(True)

        sim.step()

        elapsed = time.time() - start
        frames = sim.frame
        # motor.set_head_target_angles([1, 1])
        # motor.set_motor()
        print(bez.sensors.get_pose().orientation_euler)
        # print(sim.dofs)
        # print(sim.dofs_to_index)
        # print(sim.left_actuators_indexes)
        # print(sim.get_control("left_knee"))
        # print(sim.get_q("left_knee"))
        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")
