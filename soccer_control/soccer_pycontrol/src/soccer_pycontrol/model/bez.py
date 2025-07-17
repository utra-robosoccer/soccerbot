import enum
from os.path import expanduser

import yaml
from soccer_pycontrol.model.motor_control import MotorControl
from soccer_pycontrol.model.sensors import Sensors
from soccer_pycontrol.pybullet_usage.pybullet_load_model import LoadModel

from soccer_common import Transformation

import numpy as np
import math

class BezStatusEnum(enum.IntEnum):
    BALANCE = 0
    FIND_BALL = 1
    WALK = 2
    FALLEN = 3
    ROTATING = 4
    LOCALIZE = 5

class HeadScanner:
    def __init__(self, max_yaw=np.pi / 2, min_yaw=-np.pi / 2, step_size=0.01):
        self.max_yaw = max_yaw
        self.min_yaw = min_yaw
        self.step_size = step_size
        self.yaw = 0.0  # current angle
        self.state = "LOOKING_RIGHT"
        self.goalposts_found = 0
        self.max_yaw_reached = False
        self.min_yaw_reached = False
        self.goalpost_distances = []


    def update(self, sees_goal: bool) -> float:
        """
        Update head angle based on goalpost detection.
        :param sees_goal: whether goalpost is visible in current frame
        :return: new head yaw angle
        """

        if sees_goal:
            self.goalposts_found += 1
            if self.goalposts_found == 1:
                self.state = "LOOKING_LEFT"
            elif self.goalposts_found == 2:
                self.state = "DONE"

        if self.state == "LOOKING_RIGHT":
            self.yaw = min(self.yaw + self.step_size, self.max_yaw)
            self.max_yaw_reached = (self.yaw + self.step_size >= self.max_yaw)
        elif self.state == "LOOKING_LEFT":
            self.yaw = max(self.yaw - self.step_size, self.min_yaw)
            self.min_yaw_reached = (self.yaw - self.step_size <= self.min_yaw)
        elif self.state == "DONE":
            pass  # Stop moving

        return self.yaw

    def reset(self):
        self.yaw = 0.0
        self.state = "LOOKING_RIGHT"
        self.goalposts_found = 0


class Bez:
    """
    High level abstraction to represent the model.
    """

    # TODO should create more interfaces so that its easier to access objects, streamline interface

    def __init__(
        self,
        robot_model: str = "bez1",
        pose: Transformation = Transformation(),
        fixed_base: bool = False,
        status: BezStatusEnum = BezStatusEnum.BALANCE,
        found_ball: bool = False,
    ):
        self.robot_model = robot_model
        self.parameters = self.get_parameters()
        self._status = status
        self._body_status = status
        self._head_status = status
        self._found_ball = found_ball
        self._found_home_side = False
        self._home_side_yaw_positive = False
        self._Location = []


        self.model = LoadModel(self.robot_model, self.parameters["walking_torso_height"], pose, fixed_base)

        self.motor_control = MotorControl(self.model.body)

        self.sensors = Sensors(self.model.body, self.model.ball)

        self._sway_amplitude = 2
        self.head_scanner = HeadScanner(max_yaw=self._sway_amplitude, min_yaw=-self._sway_amplitude, step_size=0.4)

    @property
    def status(self) -> BezStatusEnum:
        return self._status

    @status.setter
    def status(self, status: BezStatusEnum) -> None:
        self._status = status

    @property
    def head_status(self) -> BezStatusEnum:
        return self._head_status

    @head_status.setter
    def head_status(self, status: BezStatusEnum) -> None:
        self._head_status = status

    @property
    def body_status(self) -> BezStatusEnum:
        return self._body_status

    @body_status.setter
    def body_status(self, status: BezStatusEnum) -> None:
        self._body_status = status

    @property
    def found_ball(self) -> bool:
        return self._found_ball

    @found_ball.setter
    def found_ball(self, status: bool) -> None:
        self._found_ball = status

    @property
    def found_home_side(self) -> bool:
        return self._found_home_side

    @found_home_side.setter
    def found_home_side(self, status: bool) -> None:
        self._found_home_side = status

    @property
    def home_side_yaw_positive(self) -> bool:
        return self._home_side_yaw_positive

    @home_side_yaw_positive.setter
    def home_side_yaw_positive(self, status: bool) -> None:
        self._home_side_yaw_positive = status

    @property
    def location(self) -> list:
        return self._Location

    @location.setter
    def location(self, location: list) -> None:
        self._Location = location

    def get_parameters(self) -> dict:
        with open(
            expanduser("~")
            + f"/ros2_ws/src/soccerbot/soccer_control/soccer_pycontrol/config/{self.robot_model}/{self.robot_model}_sim_pybullet.yaml",
            "r",
        ) as file:
            parameters = yaml.safe_load(file)
            file.close()
        return parameters

    @staticmethod
    def fallen(pitch: float) -> bool:
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
