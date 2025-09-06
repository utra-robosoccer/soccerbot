import os
import time
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation


class Simulator:
    """
    A high-level wrapper for MuJoCo simulations, providing easy access to model loading,
    stepping, control, and sensor data for robotics applications.

    This class encapsulates MuJoCo's MjModel and MjData, offering methods to interact
    with the simulation environment, set controls, retrieve sensor data, and render
    visualizations. It's designed for humanoid robot control, with support for joint
    control, pose manipulation, and real-time simulation.

    Attributes:
        model_dir (str): Directory containing MJCF model files.
        model (mujoco.MjModel): Loaded MuJoCo model.
        data (mujoco.MjData): Simulation data associated with the model.
        dofs (list): List of [index, name] pairs for degrees of freedom.
        dofs_to_index (dict): Mapping from DoF names to indices.
        viewer: MuJoCo viewer instance for rendering (if enabled).
        t (float): Current simulation time.
        dt (float): Simulation timestep.
        frame (int): Current frame number.
    """

    def __init__(self, model_dir: Optional[str] = None, scene_name: str = "scene_sig.xml"):
        """
        Initialize the MuJoCo Simulator with a model file.

        Args:
            model_dir (Optional[str]): Path to the directory containing MJCF files.
                Defaults to the 'model/' subdirectory relative to this file.
            scene_name (str): Name of the MJCF scene file to load (e.g., 'scene_bez2.xml').
        """
        # Set model directory, defaulting to local 'model/' folder
        if model_dir is None:
            model_dir = os.path.join(os.path.dirname(__file__) + "/model/")
        self.model_dir = model_dir

        # Load the MuJoCo model from the specified MJCF file
        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(f"{model_dir}/{scene_name}")
        self.data: mujoco.MjData = mujoco.MjData(self.model)

        # Extract degrees of freedom (DoFs) for joint access
        joints = len(self.model.jnt_pos)
        self.dofs = [[k, self.model.jnt(k).name] for k in range(1, joints)]
        self.dofs_to_index = {dof: k for k, dof in self.dofs}

        # Initialize simulation state variables
        self.viewer = None
        self.t: float = 0.0
        self.dt: float = self.model.opt.timestep
        self.frame: int = 0
        self.data.ctrl[:] = 0  # Reset control inputs to zero

    def set_floor_friction(self, friction: float) -> None:
        """
        Adjust the friction coefficient of the floor geometry in the simulation.

        Args:
            friction (float): New friction value for the floor.
        """
        self.model.geom("floor").friction[0] = friction
        self.model.geom("floor").priority = 1  # Ensure floor has high priority in contacts

    def self_collisions(self) -> float:
        """
        Calculate the total force from self-collisions in the simulation.

        Returns:
            float: Magnitude of the total contact force from self-collisions.
        """
        forcetorque = np.zeros(6)
        contacts = self.data.contact
        selector = (contacts.geom[:, 0] != 0) * (contacts.geom[:, 1] != 0)  # Filter valid contacts
        forces = 0.0
        for id in np.argwhere(selector):
            mujoco.mj_contactForce(self.model, self.data, id, forcetorque)
            forces += np.linalg.norm(forcetorque[:3])  # Sum force magnitudes
        return forces

    def centroidal_force(self):
        """
        Compute the magnitude of the centroidal force from constraints.

        Returns:
            float: Norm of the centroidal constraint forces.
        """
        return np.linalg.norm(self.data.qfrc_constraint[3:])  # Extract and norm forces

    def dof_names(self) -> list:
        """
        Get the names of all degrees of freedom (DoFs) in the model.

        Returns:
            list: List of DoF names.
        """
        return [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(self.model.nu)]

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)

    def reset_render(self) -> None:
        self.t = 0
        self.frame = 0
        self.viewer_start = time.time()

    def get_range(self, name: str) -> np.ndarray:
        return self.model.joint(name).range

    def get_q(self, name: str):
        """
        Gets the position of a given joint.

        Args:
            name (str): joint name

        Returns:
            float: joint position
        """
        addr = self.model.jnt_qposadr[self.dofs_to_index[name]]
        return self.data.qpos[addr]

    def get_qdot(self, name: str):
        """
        Gets the velocity of a given joint.

        Args:
            name (str): joint name

        Returns:
            float: joint velocity
        """
        addr = self.model.jnt_dofadr[self.dofs_to_index[name]]
        return self.data.qvel[addr]

    def set_q(self, name: str, value: float):
        """
        Sets a value of a given joint.

        Args:
            name (str): joint name
            value (float): target value
        """
        addr = self.model.jnt_qposadr[self.dofs_to_index[name]]
        self.data.qpos[addr] = value

    def get_control(self, name: str):
        """
        Gets the control for a given actuator

        Args:
            name (str): actuator name
        """
        actuator_idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        return self.data.ctrl[actuator_idx]

    def get_actuator_index(self, name: str):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def set_control(self, name: str, value: float, reset: bool = False) -> None:
        """
        Sets the control for a given actuator.
        If the actuator is a position actuator, the value is the desired position.
        If the actuator is a motor actuator, the value is the desired torque.

        Args:
            name (str): actuator name
            value (float): target value
        """
        actuator_idx = self.get_actuator_index(name)
        self.data.ctrl[actuator_idx] = value

        if reset:
            self.set_q(name, value)

    def get_gyro(self) -> np.ndarray:
        """
        Gets the gyroscope data.

        Returns:
            np.ndarray: gyroscope data
        """
        return self.data.sensor("gyro").data

    def get_accel(self) -> np.ndarray:
        """
        Gets the gyroscope data.

        Returns:
            np.ndarray: gyroscope data
        """
        return self.data.sensor("accelerometer").data

    def get_T_world_body(self, body_name: str) -> np.ndarray:
        """
        Gets the transformation from world to body frame.

        Args:
            body_name (str): body name
        """
        T = np.eye(4)
        body = self.data.body(body_name)
        T[:3, :3] = body.xmat.reshape(3, 3)
        T[:3, 3] = body.xpos
        return T

    def get_T_world_site(self, site_name: str) -> np.ndarray:
        """
        Gets the transformation from world to site frame.

        Args:
            site_name (str): site name
        """
        T = np.eye(4)
        site = self.data.site(site_name)
        T[:3, :3] = site.xmat.reshape(3, 3)
        T[:3, 3] = site.xpos
        return T

    def get_T_world_fbase(self) -> np.ndarray:
        """
        Gets the transformation from world to floating base frame.
        """
        data = self.data.joint("root").qpos
        quat = data[3:]
        pos = data[:3]

        T = Rotation.from_quat(quat).as_matrix()
        T = np.eye(4)[:3, :3] @ T
        T[:3, 3] = pos
        return T

    def set_T_world_fbase(self, T: np.ndarray) -> None:
        """
        Updates the floating base so that a body transformation match the target one

        Args:
            T (np.ndarray): target transformation
        """
        joint = self.data.joint("root")

        quat = Rotation.from_matrix(T[:3, :3]).as_quat()
        pos = T[:3, 3]

        joint.qpos[:] = [*pos, *quat]
        self.reset_velocity()

    def reset_velocity(self) -> None:
        """
        Resets the velocity of all the joints
        """
        self.data.qvel[:] = 0

    def set_T_world_body(self, body_name: str, T_world_bodyTarget: np.ndarray) -> None:
        """
        Updates the floating base so that a body transformation match the target one

        Args:
            body_name (str): body name
        """
        T_world_fbase = self.get_T_world_fbase()
        T_world_body = self.get_T_world_body(body_name)
        T_body_fbase = np.linalg.inv(T_world_body) @ T_world_fbase

        self.set_T_world_fbase(T_world_bodyTarget @ T_body_fbase)

    def set_T_world_site(self, site_name: str, T_world_siteTarget: np.ndarray) -> None:
        """
        Updates the floating base so that a site transformation match the target one

        Args:
            site_name (str): site name
        """
        T_world_fbase = self.get_T_world_fbase()
        T_world_site = self.get_T_world_site(site_name)
        T_site_fbase = np.linalg.inv(T_world_site) @ T_world_fbase

        self.set_T_world_fbase(T_world_siteTarget @ T_site_fbase)

    def get_pressure_sensors(self) -> dict:
        left_pressures = [
            -self.data.sensor("left_foot_cleat_front_right").data[2],
            -self.data.sensor("left_foot_cleat_front_left").data[2],
            -self.data.sensor("left_foot_cleat_back_right").data[2],
            -self.data.sensor("left_foot_cleat_back_left").data[2],
        ]
        right_pressures = [
            -self.data.sensor("right_foot_cleat_front_right").data[2],
            -self.data.sensor("right_foot_cleat_front_left").data[2],
            -self.data.sensor("right_foot_cleat_back_right").data[2],
            -self.data.sensor("right_foot_cleat_back_left").data[2],
        ]

        return {"left": left_pressures, "right": right_pressures}

    def step(self) -> None:
        self.t = self.frame * self.dt
        mujoco.mj_step(self.model, self.data)
        self.frame += 1

    def set_gravity(self, gravity: np.ndarray) -> None:
        """
        Sets the gravity vector.

        Args:
            gravity (np.ndarray): gravity vector
        """
        self.model.opt.gravity[:] = gravity

    def render(self, realtime: bool = True):
        """
        Renders the visualization of the simulation.

        Args:
            realtime (bool, optional): if True, render will sleep to ensure real time viewing. Defaults to True.
        """
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.reset_render()
        if not hasattr(self, "viewer_start"):
            self.reset_render()
        # This snippet shows how you might reload the model
        # after a certain time. In a switching mechanism, you would reinstantiate
        # a new Simulator instead.
        if realtime:
            current_ts = self.viewer_start + self.frame * self.dt
            to_sleep = current_ts - time.time()
            if to_sleep > 0:
                time.sleep(to_sleep)

        self.viewer.sync()

    def get_head_height(self):
        left_foot = self.get_T_world_site("left_foot")[0:3][:, 3]
        right_foot = self.get_T_world_site("right_foot")[0:3][:, 3]
        foot = (left_foot + right_foot) / 2
        head_height = self.get_T_world_site("camera")[2][3] - foot[2]
        if self.get_T_world_site("camera")[2][3] < foot[2]:
            head_height = -10
        return head_height

    def get_rpy(self, site: str = "trunk") -> np.ndarray:
        R = self.data.site(site).xmat
        # pitch = np.arctan2(-R[6], np.sqrt(R[0] ** 2 + R[3] ** 2)) #
        pitch = -np.arctan2(R[6], R[8])
        roll = np.arctan2(R[7], R[8])  # atan2(R[2,1], R[2,2])
        yaw = np.arctan2(R[3], R[0])
        # fused, tilt = self.fused_tilt_from_xmat(R)
        # psi, theta, phi, h = fused

        # return np.array([phi, psi, theta])
        return np.array([roll, pitch, yaw])

    def close_viewer(self) -> None:
        """
        Closes the current viewer if one is open.
        """
        if self.viewer is not None:
            try:
                # Try to call a close or finish method if available.
                if hasattr(self.viewer, "close"):
                    self.viewer.close()
                elif hasattr(self.viewer, "finish"):
                    self.viewer.finish()
            except Exception as e:
                print(f"Error closing viewer: {e}")
            finally:
                self.viewer = None


if __name__ == "__main__":

    # sim = Simulator(scene_name="scene_bez1.xml")
    # sim = Simulator(scene_name="scene_op3_rot.xml")
    sim = Simulator(scene_name="scene_bez2.xml")
    # sim = Simulator(scene_name="scene_bez3.xml")
    # sim = Simulator(scene_name="scene_sigmaban.xml")
    # sim = Simulator(scene_name="scene_wolfgang.xml")
    # sim = Simulator(scene_name="scene_nugus.xml") #euler="-1.57  0 0.2 "  left_hip_pitch
    sim.step()
    sim.set_T_world_site("left_foot", np.eye(4))

    sim.step()
    start = time.time()
    model_dir = os.path.join(os.path.dirname(__file__) + "/model/")
    once = True
    count = 0
    ti = 0
    while True:
        sim.render(True)

        sim.step()

        elapsed = time.time() - start
        frames = sim.frame
        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")
