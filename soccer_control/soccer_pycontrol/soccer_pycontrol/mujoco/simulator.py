import os
import time
from typing import Optional

import meshcat.transformations as tf
import mujoco
import mujoco.viewer
import numpy as np


class SimWorld:
    """
    Class for interacting and management with MuJoCo simulator.
    """

    def __init__(self, model_dir: Optional[str] = None, scene_name: str = "scene_bez2.xml"):
        # If model_dir is not provided, use the current directory
        if model_dir is None:
            model_dir = os.path.join(os.path.dirname(__file__) + "/model/")
        self.model_dir = model_dir

        # Load the model and data
        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(f"{model_dir}/{scene_name}")
        self.data: mujoco.MjData = mujoco.MjData(self.model)

        self.viewer = None
        self.t: float = 0.0
        self.dt: float = self.model.opt.timestep
        self.frame: int = 0
        self.data.ctrl[:] = 0

    def set_floor_friction(self, friction: float) -> None:
        self.model.geom("floor").friction[0] = friction
        self.model.geom("floor").priority = 1

    def self_collisions(self) -> float:
        forcetorque = np.zeros(6)
        contacts = self.data.contact
        selector = (contacts.geom[:, 0] != 0) * (contacts.geom[:, 1] != 0)
        forces = 0.0
        for id in np.argwhere(selector):
            mujoco.mj_contactForce(self.model, self.data, id, forcetorque)
            forces += np.linalg.norm(forcetorque[:3])

        return forces

    def set_T_world_fbase(self, T: np.ndarray) -> None:
        """
        Updates the floating base so that a body transformation match the target one

        Args:
            T (np.ndarray): target transformation
        """
        joint = self.data.joint("root")

        quat = tf.quaternion_from_matrix(T)
        pos = T[:3, 3]

        joint.qpos[:] = [*pos, *quat]
        self.reset_velocity()

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

        T = tf.quaternion_matrix(quat)
        T[:3, 3] = pos
        return T

    def reset_velocity(self) -> None:
        """
        Resets the velocity of all the joints
        """
        self.data.qvel[:] = 0

    def set_gravity(self, gravity: np.ndarray) -> None:
        """
        Sets the gravity vector.

        Args:
            gravity (np.ndarray): gravity vector
        """
        self.model.opt.gravity[:] = gravity

    def step(self) -> None:
        self.t = self.frame * self.dt
        mujoco.mj_step(self.model, self.data)
        self.frame += 1

    def wait(self, steps: int, render: bool = True) -> None:
        for i in range(steps):
            if render:
                self.render(True)
            self.step()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)

    def reset_render(self) -> None:
        self.t = 0
        self.frame = 0
        self.viewer_start = time.time()

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
    sim = SimWorld(scene_name="scene_bez2.xml")
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
        # print(sim.dofs)
        # print(sim.dofs_to_index)
        # print(sim.left_actuators_indexes)
        # print(sim.get_control("left_knee"))
        # print(sim.get_q("left_knee"))
        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")
