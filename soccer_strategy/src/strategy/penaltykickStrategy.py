import numpy as np

import soccer_strategy.src.config as config
from soccer_strategy.src.strategy.freekickStrategy import FreekickStrategy
from soccer_strategy.src.robot import Robot


class PenaltykickStrategy(FreekickStrategy):
    # preparation if we are not the kicking team
    def update_non_kicking_strategy(self, friendly, ball, teamcolor, is_first_half):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            robot.set_navigation_position(
                config.position_map(config.PENALTYKICK_NON_KICKING_POSITION, teamcolor, is_first_half, robot.robot_id)
            )
