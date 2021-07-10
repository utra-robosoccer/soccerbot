import math
import numpy as np
import rospy

from soccer_strategy.src.strategy.FreekickStrategy import FreekickStrategy
from soccer_strategy.src.robot.robot import Robot


class PenaltykickStrategy(FreekickStrategy):
    # preparation if we are not the kicking team
    def update_non_kicking_strategy(self, friendly, ball, teamcolor, is_first_half):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            if is_first_half == 1:
                if robot.role == Robot.Role.LEFT_MIDFIELD:
                    if teamcolor == 0:
                        nav_pose = np.array([-1, -1, 1.57])
                    else:
                        nav_pose = np.array([-1, 1, -1.57])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.RIGHT_MIDFIELD:
                    if teamcolor == 0:
                        nav_pose = np.array([1, -1, 1.57])
                    else:
                        nav_pose = np.array([1, 1, -1.57])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.GOALIE:
                    if teamcolor == 0:
                        nav_pose = np.array([0, -4.5, 1.57])
                    else:
                        nav_pose = np.array([0, 4.5, -1.57])
                    robot.set_navigation_position(nav_pose)
            # second half
            else:
                if robot.role == Robot.Role.LEFT_MIDFIELD:
                    if teamcolor == 0:
                        nav_pose = np.array([-1, 1, -1.57])
                    else:
                        nav_pose = np.array([-1, -1, 1.57])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.RIGHT_MIDFIELD:
                    if teamcolor == 0:
                        nav_pose = np.array([1, 1, -1.57])
                    else:
                        nav_pose = np.array([1, -1, 1.57])
                    robot.set_navigation_position(nav_pose)
                if robot.role == Robot.Role.GOALIE:
                    if teamcolor == 0:
                        nav_pose = np.array([0, 4.5, -1.57])
                    else:
                        nav_pose = np.array([0, -4.5, 1.57])
                    robot.set_navigation_position(nav_pose)