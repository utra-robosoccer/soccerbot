from abc import ABC, abstractmethod

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.walk_engine.stabilize_controller import StabilizeController


class WalkInterface(ABC):
    def __init__(self, bez: Bez, imu_feedback_enabled: bool = False):
        self.bez = bez
        self.imu_feedback_enabled = imu_feedback_enabled
        self.stabilize_controller = StabilizeController(self.bez.parameters)

        self.t = None
        self.enable_walking = None
        self.reset_walk()

    def reset_walk(self):
        self.t = -1
        self.enable_walking = True

    def stabilize_walk(self, pitch: float, roll: float) -> None:
        error_pitch, error_roll = self.stabilize_controller.update(pitch, roll)
        self.bez.motor_control.set_leg_hip_pitch_target_angle(error_pitch)  # TODO retune
        self.bez.motor_control.set_leg_hip_roll_target_angle(error_roll)

    @abstractmethod
    def setup(self) -> None:
        if self.imu_feedback_enabled:
            self.stabilize_controller.reset()

    @abstractmethod
    def walking(self,dx, dy, dtheta) -> None:
        if self.imu_feedback_enabled and self.bez.sensors.imu_ready:
            [_, pitch, roll] = self.bez.sensors.get_imu()
            # print(pitch,"  ", roll)
            self.stabilize_walk(pitch, roll)

    def stop(self) -> None:
        # self.bez.ready()
        self.enable_walking = False



