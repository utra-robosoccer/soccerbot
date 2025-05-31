from typing import List

from soccer_common import Transformation
from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior


class Balance(Behavior):
    def __init__(self, target_goal: List=[0.0, 0.0, 0.0, 10, 10]):
        super().__init__()
        self._target_goal = target_goal

    def action(self) -> None:
        self.bez.status = BezStatusEnum.BALANCE

    def run_algorithim(self) -> None:
        # just walking in place
        self.nav.walk(self._target_goal)
        if not self.nav.enable_walking:
            self.nav.wait(200)
            # self.context.transition_to(Kick())  # should we transition to walk from balance?

    def ready_to_switch_to(self) -> bool:
        return True
