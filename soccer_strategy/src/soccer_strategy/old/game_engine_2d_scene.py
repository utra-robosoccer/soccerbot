import math
from typing import List

import numpy as np
import pygame
from pygame.color import THECOLORS

from soccer_strategy.old.ball import Ball
from soccer_strategy.old.robot import Robot
from soccer_strategy.old.robot_controlled_2d import RobotControlled2D


class Scene:
    """
    Scene used by the 2d simulator, contains drawing functions

    Rules and Dimensions https://cdn.robocup.org/hl/wp/2021/04/V-HL21_Rules_changesMarked.pdf
    """

    def __init__(self, robots, ball):
        pygame.init()

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
        self.LINE_WIDTH = 0.05

        self.pygame_size = (1100, 800)
        self.meter_to_pixel_x = self.pygame_size[0] / (self.A + self.I * 2)
        self.meter_to_pixel_y = self.pygame_size[1] / (self.B + self.I * 2)
        self.screen = pygame.display.set_mode(self.pygame_size, pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.SRCALPHA)

        self.screen_overlay_0 = pygame.Surface(self.pygame_size, pygame.SRCALPHA)
        self.screen_overlay_0.fill(THECOLORS["green"])
        width = self.pygame_size[0]
        height = self.pygame_size[1]

        # Draw the ellipse
        pos_x = (self.I + self.A / 2 - self.H / 2) * self.meter_to_pixel_x
        pos_y = (self.I + self.B / 2 - self.H / 2) * self.meter_to_pixel_y
        width = self.H * self.meter_to_pixel_x
        height = self.H * self.meter_to_pixel_y

        pygame.draw.ellipse(
            self.screen_overlay_0, THECOLORS["white"], rect=(pos_x, pos_y, width, height), width=int(self.LINE_WIDTH * self.meter_to_pixel_x)
        )

        # Draw the lines
        vline1 = [[0, -self.B / 2], [0, self.B / 2]]
        vline2 = [[-self.A / 2, -self.B / 2], [-self.A / 2, self.B / 2]]
        vline3 = [[self.A / 2, -self.B / 2], [self.A / 2, self.B / 2]]

        hline1 = [[-self.A / 2, -self.B / 2], [self.A / 2, -self.B / 2]]
        hline2 = [[-self.A / 2, self.B / 2], [self.A / 2, self.B / 2]]

        goal_line_1 = [[-self.A / 2 - self.C, -self.F / 2], [-self.A / 2 - self.C, self.F / 2]]
        goal_line_2 = [[self.A / 2 + self.C, -self.F / 2], [self.A / 2 + self.C, self.F / 2]]

        lines = [vline1, vline2, vline3, hline1, hline2, goal_line_1, goal_line_2]

        # TODO draw the rest of the lines

        for line in lines:
            l_convert = self.convert_list_of_points_to_pixel_coordinates(line)
            pygame.draw.line(
                self.screen_overlay_0,
                THECOLORS["white"],
                start_pos=l_convert[0],
                end_pos=l_convert[1],
                width=int(self.LINE_WIDTH * self.meter_to_pixel_x),
            )

        self.font = pygame.font.SysFont("arial", 9)

    def convert_point_to_pixel_coordinates(self, point):
        return (point[0] + self.I + self.A / 2) * self.meter_to_pixel_x, (point[1] + self.I + self.B / 2) * self.meter_to_pixel_y

    def convert_list_of_points_to_pixel_coordinates(self, line):
        points = []
        for l in line:
            points.append(self.convert_point_to_pixel_coordinates(l))
        return points

    def update(self, robots: List[RobotControlled2D], ball: Ball):
        self.screen.fill((0, 0, 0))
        self.screen.blit(self.screen_overlay_0, (0, 0))

        for robot in robots:
            x = robot.position[0]
            y = robot.position[1]
            position = (x, y)

            polygon_points = robot.get_robot_polygon()
            polygon_points_pixel = self.convert_list_of_points_to_pixel_coordinates(polygon_points)
            pygame.draw.polygon(self.screen, THECOLORS["white"], polygon_points_pixel)
            if robot.team == Robot.Team.OPPONENT:
                pygame.draw.circle(
                    self.screen, THECOLORS["blue"], center=self.convert_point_to_pixel_coordinates(position), radius=self.meter_to_pixel_x * 0.05
                )
            else:
                pygame.draw.circle(
                    self.screen, THECOLORS["red"], center=self.convert_point_to_pixel_coordinates(position), radius=self.meter_to_pixel_x * 0.05
                )

            text = self.font.render(f"{robot.robot_id}", True, THECOLORS["white"])
            textRect = text.get_rect()
            textRect.center = self.convert_point_to_pixel_coordinates(position)
            self.screen.blit(text, textRect)

            # Draw robot arrow
            theta = robot.position[2]
            arrow_len = 0.3
            arrow_end_x = math.cos(theta) * arrow_len
            arrow_end_y = math.sin(theta) * arrow_len
            direction = np.array([arrow_end_x, arrow_end_y])
            direction = direction / np.linalg.norm(direction)

            arrow_end = [position[0] + arrow_end_x, position[1] + arrow_end_y]

            pygame.draw.line(
                self.screen,
                THECOLORS["white"],
                start_pos=self.convert_point_to_pixel_coordinates(position),
                end_pos=self.convert_point_to_pixel_coordinates(arrow_end),
                width=2,
            )

            # Draw vision cone
            robot_pos = np.array([x, y])
            theta = -RobotControlled2D.ObservationConstants.FOV / 2
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            vision_cone_right = np.matmul(R, direction) * RobotControlled2D.ObservationConstants.VISION_RANGE
            theta = RobotControlled2D.ObservationConstants.FOV / 2
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            vision_cone_left = np.matmul(R, direction) * RobotControlled2D.ObservationConstants.VISION_RANGE

            vision_cone_right_end = [position[0] + vision_cone_right[0], position[1] + vision_cone_right[1]]
            vision_cone_left_end = [position[0] + vision_cone_left[0], position[1] + vision_cone_left[1]]

            pygame.draw.line(
                self.screen,
                THECOLORS["white"],
                start_pos=self.convert_point_to_pixel_coordinates(position),
                end_pos=self.convert_point_to_pixel_coordinates(vision_cone_right_end),
                width=2,
            )
            pygame.draw.line(
                self.screen,
                THECOLORS["white"],
                start_pos=self.convert_point_to_pixel_coordinates(position),
                end_pos=self.convert_point_to_pixel_coordinates(vision_cone_left_end),
                width=2,
            )

            for i, obstacle in enumerate(robot.observed_obstacles):
                if robot.team == Robot.Team.OPPONENT:
                    color = (255, 0, 0, obstacle.probability * 0.5)
                else:
                    color = (0, 0, 255, obstacle.probability * 0.5)
                pygame.draw.circle(
                    self.screen, color, center=self.convert_point_to_pixel_coordinates(obstacle.position), radius=self.meter_to_pixel_x * 0.1, width=1
                )

            # Draw robot path
            if robot.path is not None:
                verts = []
                for j in range(0, 11):
                    path_vert = robot.path.poseAtRatio(j / 10).position
                    verts.append([path_vert[0], path_vert[1]])

                pygame.draw.lines(
                    self.screen, THECOLORS["orange"], closed=False, points=self.convert_list_of_points_to_pixel_coordinates(verts), width=1
                )

        if ball.position is not None:
            x = ball.position[0]
            y = ball.position[1]

            pygame.draw.circle(
                self.screen, THECOLORS["yellow"], center=self.convert_point_to_pixel_coordinates((x, y)), radius=self.meter_to_pixel_x * 0.07
            )

        pygame.display.flip()
        pygame.event.get()
