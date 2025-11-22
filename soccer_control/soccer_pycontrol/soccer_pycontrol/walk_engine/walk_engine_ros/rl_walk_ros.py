from os.path import expanduser

import numpy as np
import onnxruntime as rt
import rclpy

from soccer_pycontrol.model.bez import Bez
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.rl_walk import RLWalk


class RLWalkRos(Node, RLWalk):
    """ONNX controller for the Booster T1 humanoid."""
    def __init__(
        self,
        policy_name: str,
        ctrl_dt: float,
        action_scale: float = 0.5,
        vel_scale_x: float = 1.0,
        vel_scale_y: float = 1.0,
        vel_scale_rot: float = 1.0,

    ):
        policy_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_control/soccer_pycontrol/models/" + policy_name


        self._output_names = ["continuous_actions"]
        self._policy = rt.InferenceSession(
            policy_path, providers=["CPUExecutionProvider"]
        )
        self.bez =  BezROS(self) # TODO fix this later
        self._action_scale = action_scale

        self._default_angles = self.bez.default_leg_angles
        self._last_action = np.zeros_like(self._default_angles, dtype=np.float32)

        self._phase = np.array([0.0, np.pi])
        self._gait_freq = 1.5
        self._phase_dt = 2 * np.pi * self._gait_freq * ctrl_dt

        # self._joystick = KeyboardGamepad(
        #     vel_scale_x=vel_scale_x,
        #     vel_scale_y=vel_scale_y,
        #     vel_scale_rot=vel_scale_rot,
        # )
        self._cmd = [0, 0 ,0]

        self._timer = self.create_timer(ctrl_dt, self._timer_callback)

    @property
    def cmd(self):
        return self._cmd

    @cmd.setter
    def cmd(self, command: list) -> None:
        self._cmd = command

    def get_obs(self) -> np.ndarray:
        joint_angles = self.bez.motor_control.get_q_legs() - self._default_angles # TODO will need to investigate how this scales
        phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
        command =  self.cmd #self._joystick.get_command()
        obs = np.hstack([
            self.bez.sensors.get_gyro(),
            self.bez.sensors.get_gravity_vec(),
            command,
            joint_angles,
            self.bez.motor_control.get_qvel_legs(),
            self._last_action,
            phase,
        ])
        return obs.astype(np.float32)

    def get_control(self) :
      obs = self.get_obs()
      onnx_input = {"obs": obs.reshape(1, -1)}
      onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0]
      self._last_action = onnx_pred.copy()
      phase_tp1 = self._phase + self._phase_dt
      self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi
      action = onnx_pred * self._action_scale + self._default_angles
      # TODO idk if i like this whole design
      self.bez.motor_control.set_left_leg_target_angles(action[0:6])
      self.bez.motor_control.set_right_leg_target_angles(action[6:])
      self.bez.motor_control.set_motor()

    def _timer_callback(self):
        if not self.bez.sensors.imu_ready or not self.bez.motor_control.joint_state_ready:
            self.get_logger().warning(
                "Waiting for all sensors to be available", throttle_duration_sec=1.0
            )
            return

        self.get_control()

def main():
    rclpy.init()
    node = RLWalkRos(policy_name="bez222_policy.onnx",
            ctrl_dt=0.02)
    try:
        rclpy.spin(node)

    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()