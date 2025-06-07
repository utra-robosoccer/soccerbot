from typing import List

from soccer_common import Transformation
from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior
import numpy as np


class Rotate(Behavior):
    def __init__(self, clockwise: bool = True):
        super().__init__()
        self._clockwise = clockwise

    def action(self) -> None:
        self.bez.status = BezStatusEnum.BALANCE

    def run_algorithim(self) -> None:
        # rotate +- 90 degrees

        if self._clockwise:
            # yaw=-90°, pitch=0, roll=0
            target_goal = Transformation(position=[0, 0, 0], euler=[-np.pi / 2, 0, 0])
        else:
            target_goal = Transformation(position=[0, 0, 0], euler=[np.pi / 2, 0, 0])

        self.nav.walk(target_goal)

        if not self.nav.enable_walking:
            self.nav.wait(200)
            # self.context.transition_to(Kick())  # should we transition to walk from balance?

    def ready_to_switch_to(self) -> bool:
        return True
