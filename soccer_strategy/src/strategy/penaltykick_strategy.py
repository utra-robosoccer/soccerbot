import numpy as np

import config as config
from strategy.freekick_strategy import FreekickStrategy


class PenaltykickStrategy(FreekickStrategy):
    # preparation if we are not the kicking team
    def update_non_kicking_strategy(self, friendly, ball, game_properties):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            robot.set_navigation_position(
                config.position_map(
                    config.PENALTYKICK_NON_KICKING_POSITION,
                    game_properties.team_color,
                    game_properties.is_first_half,
                    robot.robot_id
                )
            )
