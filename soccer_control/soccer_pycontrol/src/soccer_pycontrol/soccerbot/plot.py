import math

import numpy as np
from matplotlib import pyplot as plt
from soccer_pycontrol.joints import Joints


class Plotter:
    def __init__(self):
        pass

    def plot_angles(self):
        """
        Creates a plot of all the angles
        """

        angles = []
        iterator = np.linspace(
            0,
            self.robot_path.duration(),
            num=math.ceil(self.robot_path.duration() / self.robot_path.step_precision) + 1,
        )
        plot_angles = np.zeros((len(iterator), 18))
        i = 0
        for t in iterator:
            self.stepPath(t)
            angles.append((t, self.get_angles().copy()))
            plot_angles[i] = np.array(self.get_angles().copy())
            i = i + 1
        fig = plt.figure(3, tight_layout=True)

        # Left Leg
        plt.subplot(311)
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_1], label="LEFT_LEG_1")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_2], label="LEFT_LEG_2")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_3], label="LEFT_LEG_3")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_4], label="LEFT_LEG_4")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_5], label="LEFT_LEG_5")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_6], label="LEFT_LEG_6")
        plt.title("Left Foot")
        plt.xlabel("time (t)")
        plt.ylabel("Angles")
        plt.legend()
        plt.grid(b=True, which="both", axis="both")

        # Right Leg
        plt.subplot(312)
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_1], label="RIGHT_LEG_1")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_2], label="RIGHT_LEG_2")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_3], label="RIGHT_LEG_3")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_4], label="RIGHT_LEG_4")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_5], label="RIGHT_LEG_5")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_6], label="RIGHT_LEG_6")
        plt.title("Right Foot")
        plt.xlabel("time (t)")
        plt.ylabel("Angles")
        plt.legend()
        plt.grid(b=True, which="both", axis="both")

        # Head & Arms
        plt.subplot(313)
        plt.plot(iterator, plot_angles[:, Joints.HEAD_1], label="HEAD_1")
        plt.plot(iterator, plot_angles[:, Joints.HEAD_2], label="HEAD_2")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_ARM_1], label="RIGHT_ARM_1")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_ARM_2], label="RIGHT_ARM_2")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_ARM_1], label="LEFT_ARM_1")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_ARM_2], label="LEFT_ARM_2")
        plt.title("Head & Arms")
        plt.xlabel("time (t)")
        plt.ylabel("Angles")
        plt.legend()
        plt.grid(b=True, which="both", axis="both")

        fig.canvas.draw()
        plt.show()
