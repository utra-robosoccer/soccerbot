import numpy as np

from soccer_common.transformation import Transformation
from soccer_pycontrol.mujoco.simulator import Simulator  # Import Simulator for MuJoCo integration


class Sensors:
    """
    Interface for retrieving sensor data from the MuJoCo simulation.

    This class provides methods to access pose, IMU, ball position, and pressure sensor
    data using the MuJoCo Simulator. It replaces PyBullet-based sensor access with
    MuJoCo's native sensor and body data retrieval.

    Attributes:
        simulator (Simulator): The MuJoCo simulator instance.
        ball_body_id: Identifier for the ball body in the simulation (if applicable).
        imu_ready (bool): Flag indicating if IMU data is available.
    """

    def __init__(self, simulator: Simulator, ball_body_id=None):
        """
        Initialize the Sensors interface with a MuJoCo Simulator.

        Args:
            simulator (Simulator): The MuJoCo simulator instance.
            ball_body_id: Optional identifier for the ball body.
        """
        self.simulator = simulator
        self.ball_body_id = ball_body_id
        self.imu_ready = False
        self.get_imu()  # Initialize IMU to set imu_ready flag

    def get_pose(self, link_name: str = "trunk") -> Transformation:
        """
        Retrieve the 3D pose of a specified link/body in the simulation.

        Args:
            link_name (str): Name of the link/body (e.g., "trunk").

        Returns:
            Transformation: Pose including position and orientation.
        """
        T = self.simulator.get_T_world_body(link_name)  # Get transformation matrix
        position = T[:3, 3]  # Extract position vector
        quaternion = self._matrix_to_quaternion(T[:3, :3])  # Convert rotation to quaternion
        return Transformation(position=position, quaternion=quaternion)

    def get_imu(self) -> np.ndarray:
        """
        Retrieve IMU data (gyroscope readings) from the simulation.

        Returns:
            np.ndarray: Gyroscope data (roll, pitch, yaw).
        """
        gyro_data = self.simulator.get_gyro()  # Fetch gyro from MuJoCo sensors
        self.imu_ready = True
        return gyro_data  # Return as Euler angles

    def get_ball(self):
        """
        Get the ball's position relative to the robot's trunk.

        Returns:
            Transformation: Relative pose of the ball.
        """
        robot_pos = self.get_pose("trunk").position
        ball_pos = self.simulator.get_T_world_body("ball")[:3, 3]  # Assume "ball" body exists
        relative_pos = np.array(ball_pos) - np.array(robot_pos)
        return Transformation(position=relative_pos.tolist())

    def get_ball_global(self):
        """
        Get the ball's global position in the simulation.

        Returns:
            Transformation: Global pose of the ball.
        """
        ball_pos = self.simulator.get_T_world_body("ball")[:3, 3]
        return Transformation(position=ball_pos.tolist())

    def get_foot_pressure_sensors(self) -> dict:
        """
        Retrieve foot pressure sensor data.

        Returns:
            dict: Pressure data for left and right feet.
        """
        return self.simulator.get_pressure_sensors()  # Use MuJoCo's pressure sensors

    def get_camera_image(self):
        """
        Placeholder for camera image capture (not implemented for MuJoCo yet).
        """
        raise NotImplementedError("Camera image capture not yet implemented for MuJoCo")

    def get_height(self):
        """
        Get the height of the robot's trunk.

        Returns:
            float: Z-position of the trunk.
        """
        return self.get_pose("trunk").position[2]

    def _matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """
        Convert a 3x3 rotation matrix to a quaternion.

        Args:
            R (np.ndarray): 3x3 rotation matrix.

        Returns:
            np.ndarray: Quaternion [w, x, y, z].
        """
        # Standard matrix-to-quaternion conversion algorithm
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        return np.array([w, x, y, z])
