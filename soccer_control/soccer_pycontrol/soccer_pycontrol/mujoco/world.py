import os
import time
import mujoco
import mujoco.viewer
import numpy as np

class MujocoWorld:
    """
    High-level wrapper around a MuJoCo simulation model that handles model/data
    creation, basic simulation stepping, optional real-time pacing, and an optional
    GUI viewer.

    This class centralizes common setup and runtime tasks when using mujoco
    (mujoco-py or mujoco>=2 Python bindings depending on your environment):
    - Load an MJCF model file into a mujoco.MjModel and create corresponding
        mujoco.MjData.
    - Configure global simulation parameters (gravity and timestep).
    - Optionally launch a passive viewer if a display is available.
    - Provide convenience methods to step the simulator, wait for multiple steps,
        and perform a short "motor wait" interpolation loop useful for actuator
        interpolation or settling.

    Parameters (constructor):
    - path (str): Path to the MJCF (XML) model file to load. Default: "bez2.xml".
    - position (tuple): Intended scene or object position (reserved for future use;
        not used directly by current implementation). Default: (0, 0, 0).
    - orientation (tuple): Intended scene or object orientation (reserved for
        future use; not used directly by current implementation). Default: (0, 0, 0).
    - display (bool): If True, launch a passive mujoco viewer. By default this is
        inferred from the environment (True when "DISPLAY" is present in os.environ).
    - real_time (bool): If True, the step() method will sleep to approximate
        real-time execution at the configured rate. Default: False.
    - rate (int): Simulation rate used to compute the model timestep and, when
        real_time=True, the wall-clock sleep duration between steps. Default: 100 Hz.

    Attributes (set during initialization):
    - rate (int): configured simulation frequency in Hz.
    - real_time (bool): whether to pace execution to real time.
    - model (mujoco.MjModel): loaded MuJoCo model object.
    - data (mujoco.MjData): simulation data container associated with the model.
    - viewer: viewer object returned by mujoco.viewer.launch_passive when display
        is True, otherwise None.

    Methods:
    - close(): Close the GUI viewer (if launched). Safe to call if viewer is None.
    - wait(steps: int): Advance the simulation by the specified number of steps.
        Each iteration calls step(), so real-time pacing and viewer synchronization
        are honored.
    - wait_motor(): A short, fixed-duration stepping loop intended to give motors
        / actuators time to interpolate to new targets. Current implementation iterates
        over np.arange(0, 1.0, 0.04) (about 25 iterations); adjust the range and
        increment as needed to match the desired settling behavior.
    - step(): Advance the simulation a single timestep by calling mujoco.mj_step.
        If real_time is enabled, sleep for 1 / rate seconds to approximate real-time
        execution. If a viewer exists, call viewer.sync() to update the GUI.

    Notes and usage tips:
    - The constructor currently sets model.opt.gravity and model.opt.timestep
        directly; if your MJCF already specifies gravity or timestep, you may need to
        adjust or remove these overrides.
    - The position and orientation constructor parameters are placeholders for
        scene placement and are not applied automatically; extend the implementation
        if you need to programmatically position models after loading.
    - The wait_motor loop uses a fixed increment for interpolation; for precise
        actuator control consider using explicit interpolation of target commands
        with a timestep-based loop that accounts for model.opt.timestep.
    - Viewer behavior depends on your mujoco Python package and display/server
        configuration; in headless environments set display=False.

    Example:
            # Create a world with a GUI and run for 100 steps (approx 1 second at 100 Hz)
            world = MujocoWorld(path="robot.xml", display=True, real_time=True, rate=100)
            world.wait(100)
            world.close()
    """

    def __init__(
        self,
        path: str = "plane.xml",  # MJCF file instead of URDF
        position: tuple = (0, 0, 0),
        orientation: tuple = (0, 0, 0),
        display: bool = "DISPLAY" in os.environ,
        real_time: bool = False,
        rate: int = 100,
    ):
        """
        Initializes the MuJoCo world.
        """
        self.rate      = rate
        self.real_time = real_time
        self.model     = mujoco.MjModel.from_xml_path(path)
        self.data      = mujoco.MjData(self.model)

        # Set gravity (similar to PyBullet)
        self.model.opt.gravity = [0, 0, -9.81]

        # Configure timestep
        self.model.opt.timestep = 1.0 / rate

        # Viewer for GUI (if display is True)
        if display:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

    def close(self):
        if self.viewer:
            self.viewer.close()

    def wait(self, steps: int) -> None:
        for i in range(steps):
            self.step()

    def wait_motor(self) -> None:
        # Adapted for interpolation (adjust as needed)
        for _ in np.arange(0, 1.00, 0.040):
            self.step()

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
        if self.real_time:
            time.sleep(1 / self.rate)
        if self.viewer:
            self.viewer.sync()
