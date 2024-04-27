import time
from os.path import expanduser

import numpy as np
import pybullet as pb
import pybullet_data
from matplotlib import pyplot as plt
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.plot import BlitManager
from soccer_pycontrol.pybullet_model import PybulletModel
from soccer_pycontrol.pybullet_world import PybulletWorld
from soccer_pycontrol.soccerbot.inverse_kinematics_pybullet import (
    InverseKinematicsPybullet,
)

from soccer_common import Transformation


class PybulletEnv:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(self, model: PybulletModel, world: PybulletWorld, real_time: bool = False, rate: int = 100):
        """
        Initialize the Navigator

        """
        self.rate = rate
        self.real_time = real_time

        self.world = world
        self.model = model

        # TODO cleanup
        # self.steps = []
        # self.imu_data = []
        #
        # x = np.linspace(0, 10, 100)
        # y = np.linspace(-1.57, 1.57, 100)
        # make a new figure
        # fig, self.ax = plt.subplots()
        # # add a line
        # (self.ln,) = self.ax.plot(x, y, animated=True)
        # # add a frame number
        # self.fr_number = self.ax.annotate(
        #     "0",
        #     (0, 1),
        #     xycoords="axes fraction",
        #     xytext=(10, -10),
        #     textcoords="offset points",
        #     ha="left",
        #     va="top",
        #     animated=True,
        # )
        # self.bm = BlitManager(fig.canvas, [self.ln, self.fr_number])
        # plt.show(block=False)
        # plt.pause(0.1)

    def wait(self, steps) -> None:
        # self.t_start = time.time()
        self.model.ready()
        h = 0
        x, _, _ = self.model.ik.ik.x_sweep(h)
        for i in range(steps):
            previous_configuration = self.model.motor_control.configuration
            configuration = [0.0] * len(Joints)
            configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            # TODO clean up math
            for r in np.arange(0, 1.00, 0.040):
                self.model.motor_control.configuration[0:18] = (
                    np.array(np.array(configuration[0:18]) - np.array(previous_configuration[0:18])) * r + np.array(previous_configuration[0:18])
                ).tolist()
                self.model.motor_control.set_motor()
                self.step()
            # pb.stepSimulation()
        # for i in range(steps):
        #     self.model.motor_control.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
        #     self.model.motor_control.set_motor()
        #     # print("Here: ", self.model.ik.get_link_transformation(Links.IMU, Links.RIGHT_LEG_6).position)
        #     # print("Here: ", pb.getLinkState(self.model.body, linkIndex=Links.RIGHT_LEG_6)[4])  # - self.model.ik.right_foot_init_position.position)
        #     # import time
        #     #
        #     # s = time.time()
        #     # print(self.model.ik.inverseKinematicsRightFoot(np.copy(self.model.ik.right_foot_init_position)))
        #     # print((time.time() - s) * 1000)
        #     self.step()

        for i in range(steps):
            # self.model.motor_control.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            # self.model.motor_control.set_motor()
            # print("Here: ", self.model.ik.get_link_transformation(Links.IMU, Links.RIGHT_LEG_6).position)
            # print("Here: ", pb.getLinkState(self.model.body, linkIndex=Links.RIGHT_LEG_6)[4])  # - self.model.ik.right_foot_init_position.position)
            # import time
            #
            # s = time.time()
            # print(self.model.ik.inverseKinematicsRightFoot(np.copy(self.model.ik.right_foot_init_position)))
            # print((time.time() - s) * 1000)
            self.step()

        x, _, _ = self.model.ik.ik.y_sweep(h)
        for i in range(steps):
            previous_configuration = self.model.motor_control.configuration
            configuration = [0.0] * len(Joints)
            configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            # TODO clean up math
            for r in np.arange(0, 1.00, 0.040):
                self.model.motor_control.configuration[0:18] = (
                    np.array(np.array(configuration[0:18]) - np.array(previous_configuration[0:18])) * r + np.array(previous_configuration[0:18])
                ).tolist()
                self.model.motor_control.set_motor()
                self.step()

        for i in range(steps):
            # self.model.motor_control.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            # self.model.motor_control.set_motor()
            # print("Here: ", self.model.ik.get_link_transformation(Links.IMU, Links.RIGHT_LEG_6).position)
            # print("Here: ", pb.getLinkState(self.model.body, linkIndex=Links.RIGHT_LEG_6)[4])  # - self.model.ik.right_foot_init_position.position)
            # import time
            #
            # s = time.time()
            # print(self.model.ik.inverseKinematicsRightFoot(np.copy(self.model.ik.right_foot_init_position)))
            # print((time.time() - s) * 1000)
            self.step()

        x, _, _ = self.model.ik.ik.z_sweep()
        for i in range(steps):
            previous_configuration = self.model.motor_control.configuration
            configuration = [0.0] * len(Joints)
            configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            # TODO clean up math
            for r in np.arange(0, 1.00, 0.040):
                self.model.motor_control.configuration[0:18] = (
                    np.array(np.array(configuration[0:18]) - np.array(previous_configuration[0:18])) * r + np.array(previous_configuration[0:18])
                ).tolist()
                self.model.motor_control.set_motor()
                self.step()

        for i in range(steps):
            # self.model.motor_control.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            # self.model.motor_control.set_motor()
            # print("Here: ", self.model.ik.get_link_transformation(Links.IMU, Links.RIGHT_LEG_6).position)
            # print("Here: ", pb.getLinkState(self.model.body, linkIndex=Links.RIGHT_LEG_6)[4])  # - self.model.ik.right_foot_init_position.position)
            # import time
            #
            # s = time.time()
            # print(self.model.ik.inverseKinematicsRightFoot(np.copy(self.model.ik.right_foot_init_position)))
            # print((time.time() - s) * 1000)
            self.step()

    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)

        # self.steps.append(len(self.steps) / self.rate)
        # self.imu_data.append(self.model.sensors.get_imu().orientation_euler[1])

        # TODO add to plot library
        # self.ln.set_data(self.steps, self.imu_data)
        # tx = "Mean Frame Rate:\n {fps:.3f}FPS".format(fps=((len(self.steps)) / (time.time() - self.t_start)))
        # self.fr_number.set_text(f"frame: {tx}")
        # self.bm.update()
        # self.ax.set_xlim(0, self.steps[-1], auto=True)
        # self.ax.set_ylim(min(self.imu_data), max(self.imu_data), auto=True)

        pb.stepSimulation()


if __name__ == "__main__":
    world = PybulletWorld(path="")
    model = PybulletModel(fixed_base=True)  # TODO dont know if i like this configuration
    p = PybulletEnv(model, world, real_time=True, rate=250)
    p.wait(50)
    p.world.close()
