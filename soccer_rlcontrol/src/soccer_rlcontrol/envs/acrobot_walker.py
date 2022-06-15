"""A special kind of acrobot which designed to walk"""
import os
import time
from os.path import expanduser
from typing import Optional

import dill
import numpy as np
import pybullet as pb
import pybullet_data
import sympy as sym
from gym import core, spaces
from gym.error import DependencyNotInstalled
from numpy import cos, pi, sin
from sympy.utilities.lambdify import implemented_function, lambdify

from soccer_common import Transformation as tr

__copyright__ = "Copyright 2022, UTRA Robosoccer"
__credits__ = [
    "Jiashen Wang",
    "Jonathan Spragett",
    "Shahryar Rajabzadeh",
]
__license__ = "BSD 3-Clause"
__author__ = "Jiashen Wang <jiashen.wang@mail.utoronto.ca.de>"

# SOURCE:
# TODO


class AcrobotWalkerEnv(core.Env):
    """
    ### Description
    The system consists of two links connected linearly to form a chain, with one end of
    the chain fixed temporarily. The joint between the two links is actuated. The goal is to walk the most
    distance by the acrobot without falling
    ### Action Space
    The action is continuous, deterministic, and represents the torque applied on the actuated
    joint between the two links.
    | Num | Action                                | Unit         |
    |-----|---------------------------------------|--------------|
    | 0   | apply -1 torque to the actuated joint | torque (N m) |
    | 1   | apply 0 torque to the actuated joint  | torque (N m) |
    | 2   | apply 1 torque to the actuated joint  | torque (N m) |
    ### Observation Space
    The observation is a `ndarray` with shape `(6,)` that provides information about the
    two rotational joint angles as well as their angular velocities:
    | Num | Observation                  | Min                 | Max               |
    |-----|------------------------------|---------------------|-------------------|
    | 0   | theta1                       | -pi                 | pi                |
    | 1   | theta2                       | -pi                 | pi                |
    | 2   | Angular velocity of theta1   | -20 * pi            | 20 * pi           |
    | 3   | Angular velocity of theta2   | -20 * pi            | 20 * pi           |
    | 4   | Which Foot on the ground     | 0                   | 1                 |
    | 5   | Displacement of ground foot  | -inf                | inf               |
    where
    - `theta1` is the angle of the first joint, where an angle of 0 indicates the first link is pointing directly
    downwards.
    - `theta2` is ***relative to the angle of the first link.***
        An angle of 0 corresponds to having the same angle between the two links.
    The angular velocities of `theta1` and `theta2` are bounded at ±4π, and ±9π rad/s respectively.
    A state of `[pi/2, 0, 0, 0, 1]` indicates that the acrobot is a straight line up
    ### Rewards
    The goal is to make two steps of the acrobot and return to the safe state the most distance from the two legged
    acrobot, for the safe of simplicity, assume there is no bounce and sliding of the acrobot and such all steps that
    do not reach the goal incur a reward of -1.
    Achieving the target height results in termination with a reward of 0. The reward threshold is -100.
    ### Starting State
    Each parameter in the underlying state (`theta1`, `theta2`, and the two angular velocities) is initialized
    uniformly between -0.1 and 0.1. This means both links are pointing downwards with some initial stochasticity.
    ### Episode Termination
    The episode terminates if one of the following occurs:
    1. The robot returns to the initial state
    2. Episode length is greater than 500
    ### Arguments
    No additional arguments are currently supported.
    ```
    env = gym.make('Acrobot-Walker-v1')
    ```
    """

    metadata = {
        "render_modes": ["pybullet", "numerical"],
        "render_fps": 15,
    }

    dt = 0.01

    SCREEN_DIM = 500

    def __init__(self, render_mode: Optional[str] = None):
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        if render_mode == "pybullet":
            self.client_id = pb.connect(pb.GUI)
            pb.setAdditionalSearchPath(pybullet_data.getDataPath())
            pb.setGravity(0, 0, -9.81)
            ramp = pb.loadURDF("plane.urdf", basePosition=(0, 0, 0), baseOrientation=pb.getQuaternionFromEuler((0, 0, 0)))
            pb.changeDynamics(
                ramp,
                linkIndex=-1,
                lateralFriction=0.9,
                spinningFriction=0.9,
                rollingFriction=0.0,
            )
        else:
            self.client_id = pb.connect(pb.DIRECT)

        home = expanduser("~")

        self.body = pb.loadURDF(
            home + "/catkin_ws/src/soccerbot/acrobot_description/models/acrobot.urdf",
            useFixedBase=False,
            flags=pb.URDF_USE_INERTIA_FROM_FILE,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
        )

        if render_mode == "pybullet":
            pass
        else:
            pass

        ls0 = pb.getLinkState(self.body, 0)
        ls1 = pb.getLinkState(self.body, 1)
        self.leg_length = pb.getLinkState(self.body, 0)[4][2] + 0.018  # Foot radius (4.081e-3)
        self.m0 = pb.getDynamicsInfo(self.body, 0)[0]
        self.m1 = pb.getDynamicsInfo(self.body, 1)[0]
        self.com0 = pb.getDynamicsInfo(self.body, 0)[3][0]
        self.com1 = -pb.getDynamicsInfo(self.body, 1)[3][0]
        self.moi0 = pb.getDynamicsInfo(self.body, 0)[2][1]
        self.moi1 = pb.getDynamicsInfo(self.body, 1)[2][1]

        self.motor_friction = 0.01
        self.g = 9.81

        # Using parallel axes theorem for MOI translation along the axis (https://en.wikipedia.org/wiki/Parallel_axis_theorem)
        self.moi0 += self.m0 * self.com0**2
        self.moi1 += self.m1 * self.com1**2

        if render_mode != "pybullet":
            pb.disconnect(self.client_id)

        # Solve Robotics Equations
        # self.solveRoboticsEquations()
        self.loadRoboticsEquations()

        # Input constraints

        # Joint constraints
        joint_velocity_limit = 1.8
        motor_torque_limit = 0.22
        high = np.array([pi, pi, joint_velocity_limit, joint_velocity_limit, 1, np.inf], dtype=np.float32)
        low = np.array([-pi, -pi, -joint_velocity_limit, -joint_velocity_limit, 0, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-motor_torque_limit]), high=np.array([motor_torque_limit]), dtype=np.float32)
        self.state = None

    def reset(self, *, seed: Optional[int] = None, return_info: bool = False, options: Optional[dict] = None):
        self.state = np.array([np.pi / 2, 0, 0, 0, 0, 0])

        if not return_info:
            return self._get_ob()
        else:
            return self._get_ob(), {}

    def solveRoboticsEquations(self):

        q1, q2, qd1, qd2, qdd1, qdd2, l1, l2, lc1, lc2, i1, i2, m1, m2, g = sym.symbols("q1 q2 qd1 qd2 qdd1 qdd2 l1 l2 lc1 lc2 i1 i2 m1 m2 g")

        q = sym.Matrix([q1, q2])
        qd = sym.Matrix([qd1, qd2])
        qdd = sym.Matrix([qdd1, qdd2])

        # Positions
        rc1 = (l1 - lc1) * sym.Matrix([sym.cos(q1), sym.sin(q1)])
        rH = l1 * sym.Matrix([sym.cos(q1), sym.sin(q1)])
        rc2 = rH + lc2 * sym.Matrix([sym.cos(q1 + q2), sym.sin(q1 + q2)])
        rend = rH + l2 * sym.Matrix([sym.cos(q1 + q2), sym.sin(q1 + q2)])

        # Velocities
        rc1dot = rc1.jacobian(q) * qd
        rc2dot = rc2.jacobian(q) * qd
        renddot = rend.jacobian(q)

        w01 = qd1
        w02 = qd1 + qd2

        # Kinetic and Potential Energy
        T1 = (0.5 * m1 * (rc1dot.T * rc1dot))[0] + 0.5 * i1 * w01**2
        T2 = (0.5 * m2 * (rc2dot.T * rc2dot))[0] + 0.5 * i2 * w02**2
        T = T1 + T2
        U1 = m1 * g * rc1[1]
        U2 = m2 * g * rc2[1]
        U = U1 + U2
        L = T - U
        EE = T + U

        dLdq = sym.Matrix([L]).jacobian(q).T
        dLdqdot = sym.Matrix([L]).jacobian(qd)
        ddtdLdqdot = dLdqdot.jacobian(q) * qd + dLdqdot.jacobian(qd) * qdd
        tau = ddtdLdqdot - dLdq

        # Find the C, D, P, B Matrix
        D, b = sym.linear_eq_to_matrix(tau, list(qdd))
        P = b.subs([(qd1, 0), (qd2, 0)])
        C = sym.zeros(2, 2)

        for k in range(2):
            for j in range(2):
                for i in range(2):
                    Qijk = 0.5 * (sym.diff(D[k, j], q[i]) + sym.diff(D[k, i], q[j]) - sym.diff(D[i, j], q[k]))
                    C[k, j] = C[k, j] + Qijk * qd[i]
        B = np.multiply(sym.Matrix([[0], [self.motor_friction]]), qd)

        # Impact Map
        # Solving for EOM and impact map following
        # "Feedback Control of Dynamic Bipedal Robot Locomotion" by Grizzle, p. 55
        # Based on the 3 link model on p. 67 and the paper "Asymptotically Stable
        # Walking for Biped Robots: Analysis via Systems with Impulse Effects"

        q3, q4, qd3, qd4, qdd3, qdd4 = sym.symbols("q3 q4 q3dot q4dot q3ddot q4ddot")

        qe = sym.Matrix([q1, q2, q3, q4])
        qde = sym.Matrix([qd1, qd2, qd3, qd4])
        qdde = sym.Matrix([qdd1, qdd2, qdd3, qdd4])

        e = sym.Matrix([q3, q4])
        rc1e = rc1 + e
        rHe = rH + e
        rc2e = rc2 + e
        rende = rend + e

        rc1edot = rc1e.jacobian(qe) * qde
        rc2edot = rc2e.jacobian(qe) * qde

        # Kinetic and potential energy
        Te1 = 0.5 * m1 * (rc1edot.T * rc1edot)[0] + 0.5 * i1 * w01**2
        Te2 = 0.5 * m2 * (rc2edot.T * rc2edot)[0] + 0.5 * i2 * w02**2
        Te = Te1 + Te2
        Ue1 = m1 * g * rc1e[1]
        Ue2 = m2 * g * rc2e[1]
        Ue = Ue1 + Ue2
        Le = Te - Ue

        # Finding the EOM
        dLedq = sym.Matrix([Le]).jacobian(qe).T
        dLedqdot = sym.Matrix([Le]).jacobian(qde).T
        ddtdLedqdot = dLedqdot.jacobian(qe) * qde + dLedqdot.jacobian(qde) * qdde
        Taue = ddtdLedqdot - dLedq

        # Solving for D Matrix
        De = sym.linear_eq_to_matrix(Taue, list(qdde))

        # Upsilons
        E = rende.jacobian(qe)
        dUde = rHe.jacobian(q)

        # Inverse kinematics
        x, y, r, q1d, q2d, qd = sym.symbols("x y r q1d q2d qd")
        q2d = sym.acos((l1**2 + l2**2 - (x**2 + y**2)) / l1 / l2 / 2) - pi
        q1d = sym.atan2(y, x) + (pi - (pi + q2d)) / 2
        qd = sym.Matrix([q1d, q2d])

        dill.settings["recurse"] = True

        symbols = [renddot, D, C, P, B, De, E, dUde, EE, qd, rend]
        symbol_names = ["renddot", "D", "C", "P", "B", "De", "E", "dUde", "EE", "qd", "rend"]

        for symbol, symbol_name in zip(symbols, symbol_names):
            s = sym.simplify(symbol)
            s = sym.lambdify(list(s.free_symbols), s)
            dir_path = os.path.dirname(os.path.realpath(__file__))
            with open(dir_path + f"/gen/{symbol_name}.pkl", "wb+") as file:
                dill.dump(s, file)

    def loadRoboticsEquations(self):
        symbol_names = ["renddot", "D", "C", "P", "B", "De", "E", "dUde", "EE", "qd", "rend"]

        dir_path = os.path.dirname(os.path.realpath(__file__))
        for symbol_name in symbol_names:
            with open(dir_path + f"/gen/{symbol_name}.pkl", "rb+") as file:
                symbol_lambda = dill.load(file)
                setattr(self, "calc_" + symbol_name, symbol_lambda)
        pass

    def step(self, a):
        s = self.state
        assert s is not None, "Call reset before using AcrobotEnv object."

        if not self.action_space.contains([a]):
            raise Exception(f"Torque { a } provided beyond provided bounds [{self.action_space.high[0]} {self.action_space.low[0]}]")
        tau = a

        # Add noise to the force action (TODO)

        # Now, augment the state with our force action so it can be passed to
        # _dsdt
        s_augmented = np.append(s, tau)

        ns = rk4(self._dsdt, s_augmented, [0, self.dt])

        ns[0] = wrapToPi(ns[0])
        ns[1] = wrapToPi(ns[1])
        ns[2] = bound(ns[2], -self.MAX_VEL_1, self.MAX_VEL_1)
        ns[3] = bound(ns[3], -self.MAX_VEL_2, self.MAX_VEL_2)
        self.state = ns
        terminal = self._terminal()
        reward = -1.0 if not terminal else 0.0

        self.renderer.render_step()
        return self._get_ob(), reward, terminal, {}

    def _get_ob(self):
        s = self.state
        assert s is not None, "Call reset before using AcrobotEnv object."
        return s

    def _terminal(self):
        s = self.state
        assert s is not None, "Call reset before using AcrobotEnv object."
        return bool(-cos(s[0]) - cos(s[1] + s[0]) > 1.0)

    def _switch_leg(self, num):
        if self.state[4] % 2 == 1:
            if num == 0:
                return 1
            else:
                return 0
        else:
            return num

    def _linertia(self, num):
        num = self._switch_leg(num)
        if num == 0:
            return self.moi0
        else:
            return self.moi1

    def _lcom(self, num):
        num = self._switch_leg(num)
        if num == 0:
            return self.com0
        else:
            return self.com1

    def _lmass(self, num):
        num = self._switch_leg(num)
        if num == 0:
            return self.m0
        else:
            return self.m1

    def _dist_to_floor(self):
        q1 = self.state[0]
        q2 = self.state[1]
        rH = self.leg_length * np.array([cos(q1), sin(q1)])
        rc2 = rH + self.leg_length * np.array([cos(q1 + q2), sin(q1 + q2)])
        dist = rc2[1]
        return dist

    def _impact_foot(self, s_augmented):
        q1 = s_augmented[0]
        q2 = s_augmented[1]
        q1_dot = s_augmented[2]
        q2_dot = s_augmented[3]

        q = s_augmented[0:2]
        qdot = s_augmented[2:4]

        De = self.calc_De(self._linertia(0), self._linertia(1), self.leg_length, self._lcom(0), self._lcom(1), self._lmass(0), self._lmass(1), q1, q2)
        E = self.calc_E(self.leg_length, self.leg_length, q1, q2)
        dUde = self.calc_dUde(self.leg_length, q1)
        last_term = np.concatenate([np.zeros(2), dUde])

        delta_F = -np.solve(E @ np.inv(De) @ E.T, E) * last_term
        delta_qedot = np.linalg.solve(De, E).T * delta_F + last_term
        T = np.array([[1, 1], [0, -1]])

        qp = wrapTo2Pi(T @ q + np.array([[-pi], [0]]))
        qp_dot = np.concatenate([T, np.zeros(2, 2)]) * (delta_qedot * qdot) * 1  # obj.lcurve.energy_loss

        # Collision with floor?
        rend_dot = self.calc_J(self.leg_length, self.leg_length, qp[0], qp[1]) * qp_dot
        if rend_dot < 0:
            x_next = np.concatenate([qp, -qp_dot])
        else:
            x_next = np.concatenate([qp, qp_dot])

        rH = self.leg_length * np.array([cos(q1), sin(q1)])
        step_diff = rH + self.leg_length * np.array([cos(q1 + q2), sin(q1 + q2)])

        return x_next, step_diff

    def _dsdt(self, s_augmented):
        tau = s_augmented[-1]
        q = s_augmented[0:2]
        qdot = s_augmented[2:4]

        # First calculate foot impact
        dist = self._dist_to_floor()

        # TODO fix math i2=linertia(1)
        D = self.calc_D(
            i1=self._linertia(0),
            i2=self._linertia(0),
            l1=self.leg_length,
            lc1=self._lcom(0),
            lc2=self._lcom(1),
            m1=self._lmass(0),
            m2=self._lmass(1),
            q2=q[1],
        )
        C = self.calc_C(l1=self.leg_length, lc2=self._lcom(1), m2=self._lmass(1), q2=q[1], qd1=qdot[0], qd2=qdot[1])
        P = self.calc_P(g=self.g, l1=self.leg_length, lc1=self._lcom(0), lc2=self._lcom(1), m1=self._lmass(0), m2=self._lmass(1), q1=q[0], q2=q[1])
        B = self.calc_B(qd2=qdot[1])

        # Torque from tau
        self.tau_q = np.linalg.lstsq(D, np.array([0, tau]))[0]

        # Torque from gravity
        self.tau_g = np.linalg.lstsq(D, (-C @ np.array([qdot]).T - P - B))[0].T.flatten()

        qddot_new = self.tau_g + self.tau_q
        return np.concatenate([qdot, qddot_new])

    def render(self, mode="human"):
        if self.render_mode is not None:
            return self.renderer.get_renders()
        # else:
        #     return self._render(mode)

    # def _render(self, mode="human"):
    #     assert mode in self.metadata["render_modes"]
    #     try:
    #         import pygame
    #         from pygame import gfxdraw
    #     except ImportError:
    #         raise DependencyNotInstalled(
    #             "pygame is not installed, run `pip install gym[classic_control]`"
    #         )
    #
    #     if self.screen is None:
    #         pygame.init()
    #         if mode == "human":
    #             pygame.display.init()
    #             self.screen = pygame.display.set_mode(
    #                 (self.SCREEN_DIM, self.SCREEN_DIM)
    #             )
    #         else:  # mode in {"rgb_array", "single_rgb_array"}
    #             self.screen = pygame.Surface((self.SCREEN_DIM, self.SCREEN_DIM))
    #     if self.clock is None:
    #         self.clock = pygame.time.Clock()
    #
    #     surf = pygame.Surface((self.SCREEN_DIM, self.SCREEN_DIM))
    #     surf.fill((255, 255, 255))
    #     s = self.state
    #
    #     bound = self.LINK_LENGTH_1 + self.LINK_LENGTH_2 + 0.2  # 2.2 for default
    #     scale = self.SCREEN_DIM / (bound * 2)
    #     offset = self.SCREEN_DIM / 2
    #
    #     if s is None:
    #         return None
    #
    #     p1 = [
    #         -self.LINK_LENGTH_1 * cos(s[0]) * scale,
    #         self.LINK_LENGTH_1 * sin(s[0]) * scale,
    #     ]
    #
    #     p2 = [
    #         p1[0] - self.LINK_LENGTH_2 * cos(s[0] + s[1]) * scale,
    #         p1[1] + self.LINK_LENGTH_2 * sin(s[0] + s[1]) * scale,
    #     ]
    #
    #     xys = np.array([[0, 0], p1, p2])[:, ::-1]
    #     thetas = [s[0] - pi / 2, s[0] + s[1] - pi / 2]
    #     link_lengths = [self.LINK_LENGTH_1 * scale, self.LINK_LENGTH_2 * scale]
    #
    #     pygame.draw.line(
    #         surf,
    #         start_pos=(-2.2 * scale + offset, 1 * scale + offset),
    #         end_pos=(2.2 * scale + offset, 1 * scale + offset),
    #         color=(0, 0, 0),
    #     )
    #
    #     for ((x, y), th, llen) in zip(xys, thetas, link_lengths):
    #         x = x + offset
    #         y = y + offset
    #         l, r, t, b = 0, llen, 0.1 * scale, -0.1 * scale
    #         coords = [(l, b), (l, t), (r, t), (r, b)]
    #         transformed_coords = []
    #         for coord in coords:
    #             coord = pygame.math.Vector2(coord).rotate_rad(th)
    #             coord = (coord[0] + x, coord[1] + y)
    #             transformed_coords.append(coord)
    #         gfxdraw.aapolygon(surf, transformed_coords, (0, 204, 204))
    #         gfxdraw.filled_polygon(surf, transformed_coords, (0, 204, 204))
    #
    #         gfxdraw.aacircle(surf, int(x), int(y), int(0.1 * scale), (204, 204, 0))
    #         gfxdraw.filled_circle(surf, int(x), int(y), int(0.1 * scale), (204, 204, 0))
    #
    #     surf = pygame.transform.flip(surf, False, True)
    #     self.screen.blit(surf, (0, 0))
    #
    #     if mode == "human":
    #         pygame.event.pump()
    #         self.clock.tick(self.metadata["render_fps"])
    #         pygame.display.flip()
    #
    #     elif mode in {"rgb_array", "single_rgb_array"}:
    #         return np.transpose(
    #             np.array(pygame.surfarray.pixels3d(self.screen)), axes=(1, 0, 2)
    #         )


def close(self):
    if self.screen is not None:
        import pygame

        pygame.display.quit()
        pygame.quit()
        self.isopen = False


def wrapTo2Pi(num: float) -> float:
    rem = num % (2 * np.pi)
    return rem


def wrapToPi(num: float) -> float:
    rem = num % (np.pi)
    return rem


def bound(x, m, M=None):
    """Either have m as scalar, so bound(x,m,M) which returns m <= x <= M *OR*
    have m as length 2 vector, bound(x,m, <IGNORED>) returns m[0] <= x <= m[1].
    Args:
        x: scalar
        m: The lower bound
        M: The upper bound
    Returns:
        x: scalar, bound between min (m) and Max (M)
    """
    if M is None:
        M = m[1]
        m = m[0]
    # bound x between min (m) and Max (M)
    return min(max(x, m), M)


def rk4(derivs, y0, t):
    """
    Integrate 1-D or N-D system of ODEs using 4-th order Runge-Kutta.
    Args:
        derivs: the derivative of the system and has the signature ``dy = derivs(yi)``
        y0: initial state vector
        t: sample times
    Returns:
        yout: Runge-Kutta approximation of the ODE
    """

    try:
        Ny = len(y0)
    except TypeError:
        yout = np.zeros((len(t),), np.float_)
    else:
        yout = np.zeros((len(t), Ny), np.float_)

    yout[0] = y0

    for i in np.arange(len(t) - 1):

        this = t[i]
        dt = t[i + 1] - this
        dt2 = dt / 2.0
        y0 = yout[i]

        k1 = np.asarray(derivs(y0))
        k2 = np.asarray(derivs(y0 + dt2 * k1))
        k3 = np.asarray(derivs(y0 + dt2 * k2))
        k4 = np.asarray(derivs(y0 + dt * k3))
        yout[i + 1] = y0 + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    # We only care about the final timestep and we cleave off action value which will be zero
    return yout[-1][:4]
