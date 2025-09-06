import enum
from os.path import expanduser

import yaml

from soccer_pycontrol.model.motor_control import MotorControl
from soccer_pycontrol.model.sensors import Sensors
from soccer_pycontrol.mujoco.simulator import Simulator  # Use MuJoCo Simulator exclusively

from soccer_common import Transformation


class BezStatusEnum(enum.IntEnum):
    BALANCE = 0
    FIND_BALL = 1
    WALK = 2
    FALLEN = 3


class Bez:
    """
    High-level abstraction for the Bez humanoid robot in MuJoCo simulations.

    This class represents the robot model, integrating sensors, motor control,
    and status management. It uses MuJoCo exclusively for simulation.

    Attributes:
        robot_model (str): Name of the robot model (e.g., "bez1").
        parameters (dict): Configuration parameters loaded from YAML.
        status (BezStatusEnum): Current robot status.
        simulator (Simulator): MuJoCo simulator instance.
        motor_control (MotorControl): Motor controller.
        sensors (Sensors): Sensor interface.
    """

    def __init__(
        self,
        robot_model: str = "bez1",
        pose: Transformation = Transformation(),
        status: BezStatusEnum = BezStatusEnum.BALANCE,
        simulator: Simulator = None,
    ):
        """
        Initialize the Bez robot with MuJoCo simulator.

        Args:
            robot_model (str): Robot model name.
            pose (Transformation): Initial pose (currently unused in MuJoCo setup).
            status (BezStatusEnum): Initial status.
            simulator (Simulator): MuJoCo simulator instance (required).

        Raises:
            ValueError: If simulator is not provided.
        """
        self.robot_model = robot_model
        self.parameters = self.get_parameters()
        self._status = status

        if simulator is None:
            raise ValueError("Simulator must be provided for MuJoCo integration")
        self.simulator = simulator

        self.motor_control = MotorControl(self.simulator)  # Initialize motor control
        self.sensors = Sensors(self.simulator)  # Initialize sensors

    @property
    def status(self) -> BezStatusEnum:
        """
        Get the current robot status.

        Returns:
            BezStatusEnum: Current status.
        """
        return self._status

    @status.setter
    def status(self, status: BezStatusEnum) -> None:
        """
        Set the robot status.

        Args:
            status (BezStatusEnum): New status.
        """
        self._status = status

    def get_parameters(self) -> dict:
        """
        Load robot parameters from YAML configuration file.

        Returns:
            dict: Parameter dictionary.
        """
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
        """
        Check if the robot has fallen based on pitch angle.

        Args:
            pitch (float): Pitch angle in radians.

        Returns:
            bool: True if fallen.
        """
        angle_threshold = 1.25  # Threshold for fall detection
        if pitch > angle_threshold:
            print("Fallen Front")
            return True
        elif pitch < -angle_threshold:
            print("Fallen Back")
            return True
        return False

    @property
    def is_balance(self) -> bool:
        """
        Check if robot is in balance mode.

        Returns:
            bool: True if balancing.
        """
        return self.status == BezStatusEnum.BALANCE

    @property
    def is_find_ball(self) -> bool:
        """
        Check if robot is in ball-finding mode.

        Returns:
            bool: True if finding ball.
        """
        return self.status == BezStatusEnum.FIND_BALL

    @property
    def is_walk(self) -> bool:
        """
        Check if robot is walking.

        Returns:
            bool: True if walking.
        """
        return self.status == BezStatusEnum.WALK

    @property
    def is_fallen(self) -> bool:
        """
        Check if robot has fallen.

        Returns:
            bool: True if fallen.
        """
        return self.status == BezStatusEnum.FALLEN
