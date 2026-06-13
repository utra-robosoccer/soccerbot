import time
from os.path import expanduser

import numpy as np
import yaml
from soccer_pycontrol.model.motor_control import MotorControl
from soccer_pycontrol.model.sensors import Sensors
from soccer_pycontrol.model.sim_world import SimWorld
from soccer_pycontrol.model.status_enum import BezStatusEnum


class Bez:
    """
    High level abstraction to represent the model.
    """

    # TODO should create more interfaces so that its easier to access objects, streamline interface

    def __init__(
        self,
        world: SimWorld,
        robot_model: str = "bez2",
        status: BezStatusEnum = BezStatusEnum.BALANCE,
    ):
        self.world = world
        self.robot_model = robot_model
        self.parameters = self.get_parameters()
        self._status = status

        self.motor_control = MotorControl(self.world.model, self.world.data)

        self.sensors = Sensors(self.world)

        self.default_angles = np.array(
            [
                -0.00038,
                0.00058,
                -0.44675,
                0.00074,
                2.50858,
                -0.02901,
                0.06566,
                0.87410,
                -1.54022,
                0.67634,
                -0.07002,
                -0.44962,
                -0.00074,
                2.50967,
                -0.01830,
                0.04949,
                0.85711,
                -1.54784,
                0.69839,
                -0.05164,
            ]
        )
        self.default_leg_angles = np.array(
            [-0.02901, 0.06566, 0.87410, -1.54022, 0.67634, -0.07002, -0.01830, 0.04949, 0.85711, -1.54784, 0.69839, -0.05164]
        )

    @property
    def status(self) -> BezStatusEnum:
        return self._status

    @status.setter
    def status(self, status: BezStatusEnum) -> None:
        self._status = status

    def ready(self):  # TODO fix default angles it leans
        # self.motor_control.set_all_angles(self.default_angles)
        names = [
            "left_hip_yaw",
            "left_hip_roll",
            "left_hip_pitch",
            "left_knee",
            "left_ankle_pitch",
            "left_ankle_roll",
            "right_hip_yaw",
            "right_hip_roll",
            "right_hip_pitch",
            "right_knee",
            "right_ankle_pitch",
            "right_ankle_roll",
        ]

        self.motor_control.set_motor(names, self.default_leg_angles)

    def get_parameters(self) -> dict:
        with open(
            expanduser("~") + f"/ros2_ws/src/soccerbot/soccer_control/soccer_pycontrol/config/{self.robot_model}/{self.robot_model}_sim_mujoco.yaml",
            "r",
        ) as file:
            parameters = yaml.safe_load(file)
            file.close()
        return parameters

    @staticmethod
    def fallen(pitch: float, roll: float) -> str | None:  # TODO add fallen side
        angle_threshold = 1.25  # in radian
        if pitch > angle_threshold:
            print("Fallen Front")
            return "getupfront"
        elif pitch < -angle_threshold:
            print("Fallen Back")
            return "getupback"
        elif roll > angle_threshold:
            print("Fallen Side Right")
            return "getupsideright"
        elif roll < -angle_threshold:
            print("Fallen Side Left")
            return "getupsideleft"
        return None

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

    sim = SimWorld(robot_model="bez2")
    # sim = SimWorld(scene_name="scene_bez2.xml")
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
    # sim.close_viewer()
