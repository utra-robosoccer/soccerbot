"""Example of using the MuJoCo Python viewer UI for a simple control task"""

import mujoco
import mujoco.viewer as viewer
import numpy as np
from scipy.linalg import solve_continuous_are


# Find a K matrix for a linearized double pendulum using LQR
def double_pendulum_lqr_K(m, d):
    # Calculate system linearization
    A = np.zeros((2 * m.nv + m.na, 2 * m.nv + m.na))
    B = np.zeros((2 * m.nv + m.na, m.nu))
    mujoco.mjd_transitionFD(m, d, 1e-7, 1, A=A, B=B, C=None, D=None)

    # Convert to continuous time
    A = A - np.eye(A.shape[0])
    A = A / m.opt.timestep
    B = B / m.opt.timestep

    Q = 1 * np.diag((10, 10, 100, 100))
    R = np.eye(B.shape[1])

    S = solve_continuous_are(A, B, Q, R)

    K = np.linalg.inv(R) @ (B.T @ S)
    return K


def double_pendulum_control(m, d, K):
    x = np.concatenate((d.joint("shoulder").qpos, d.joint("elbow").qpos, d.joint("shoulder").qvel, d.joint("elbow").qvel))

    u = -K @ x
    d.actuator("shoulder").ctrl[0] = u


def load_callback(m=None, d=None):
    # Clear the control callback before loading a new model
    # or a Python exception is raised
    mujoco.set_mjcb_control(None)

    # m = mujoco.MjModel.from_xml_path('./double_pendulum.xml')
    m = mujoco.MjModel.from_xml_path("../../../../../soccer_description/bez2_description/urdf/bez2.urdf")
    d = mujoco.MjData(m)

    # if m is not None:
    #   # Calculate K matrix
    #   K = double_pendulum_lqr_K(m, d)
    #
    #   # Set some initial conditions
    d.joint("left_arm_motor_1").qpos = 1
    d.joint("head_motor_0").qpos = 1
    #   # Set the control callback
    #   mujoco.set_mjcb_control(lambda m, d: double_pendulum_control(m, d, K))

    return m, d


if __name__ == "__main__":
    viewer.launch(loader=load_callback)
