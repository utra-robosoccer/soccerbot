from collections import namedtuple
from functools import cached_property
from typing import List

import matplotlib.pyplot as plt
import numpy as np

from soccer_common import Transformation

Point = namedtuple("Point", "x y")
Transform2D = namedtuple("Transform2D", "dx dy dtheta")
Line = namedtuple("Line", "p1 p2")
Circle = namedtuple("Circle", "center radius")


class Map:
    def __init__(self):
        # Dimensions given here https://cdn.robocup.org/hl/wp/2021/06/V-HL21_Rules_v4.pdf
        self.lw = 0.05
        self.A = 9
        self.B = 6
        self.C = 0.6
        self.D = 2.6
        self.E = 1
        self.F = 3
        self.G = 1.5
        self.H = 1.5
        self.I = 1
        self.J = 2
        self.K = 5

    @cached_property
    def lines(self) -> List[Line]:
        lines: List[Line] = []
        return lines

    def draw(self):
        lw = self.lw
        A = self.A
        B = self.B
        C = self.C
        D = self.D
        E = self.E
        F = self.F
        G = self.G
        H = self.H
        I = self.I
        J = self.J
        K = self.K

        # Draw field
        plt.axes().set_facecolor("limegreen")

        # Circle
        plt.gca().add_patch(plt.Circle((0, 0), H / 2 + lw / 2, color="white"))
        plt.gca().add_patch(plt.Circle((0, 0), H / 2 - lw / 2, color="limegreen"))

        # Outer rectangle
        plt.fill_between(x=[-A / 2 - lw / 2, A / 2 + lw / 2], y1=B / 2 - lw / 2, y2=B / 2 + lw / 2, color="white")
        plt.fill_between(x=[-A / 2 - lw / 2, A / 2 + lw / 2], y1=-B / 2 - lw / 2, y2=-B / 2 + lw / 2, color="white")
        plt.fill_betweenx(y=[-B / 2 - lw / 2, B / 2 + lw / 2], x1=-A / 2 - lw / 2, x2=-A / 2 + lw / 2, color="white")
        plt.fill_betweenx(y=[-B / 2 - lw / 2, B / 2 + lw / 2], x1=A / 2 - lw / 2, x2=A / 2 + lw / 2, color="white")
        plt.fill_betweenx(y=[-B / 2 - lw / 2, B / 2 + lw / 2], x1=-lw / 2, x2=lw / 2, color="white")

        # Penalty Area (large box)
        def draw_double_boxes(h, v):
            plt.fill_between(x=[-A / 2, -A / 2 + h], y1=v / 2 - lw / 2, y2=v / 2 + lw / 2, color="white")
            plt.fill_between(x=[-A / 2, -A / 2 + h], y1=-v / 2 - lw / 2, y2=-v / 2 + lw / 2, color="white")
            plt.fill_between(x=[A / 2 - h, A / 2], y1=v / 2 - lw / 2, y2=v / 2 + lw / 2, color="white")
            plt.fill_between(x=[A / 2 - h, A / 2], y1=-v / 2 - lw / 2, y2=-v / 2 + lw / 2, color="white")
            plt.fill_betweenx(y=[-v / 2 - lw / 2, v / 2 + lw / 2], x1=-A / 2 + h - lw / 2, x2=-A / 2 + h + lw / 2, color="white")
            plt.fill_betweenx(y=[-v / 2 - lw / 2, v / 2 + lw / 2], x1=A / 2 - h - lw / 2, x2=A / 2 - h + lw / 2, color="white")

        draw_double_boxes(J, K)
        draw_double_boxes(E, F)
        draw_double_boxes(-C, D)

        # Crosses
        def draw_cross(pos_x):
            plt.fill_between(x=[pos_x - 0.1, pos_x + 0.1], y1=-lw / 2, y2=lw / 2, color="white")
            plt.fill_betweenx(y=[-0.1, 0.1], x1=pos_x - lw / 2, x2=pos_x + lw / 2, color="white")

        draw_cross(A / 2 - G)
        draw_cross(-A / 2 + G)
        draw_cross(0)

        plt.axis("equal")
        plt.xlabel("Y (m)")
        plt.ylabel("X (m)")
        plt.title("UKF Robot localization")

    def closestPointToLine(self) -> [Transform2D]:
        pass

    def matchPointsWithMap(self, points: np.array) -> Transformation:
        pass
