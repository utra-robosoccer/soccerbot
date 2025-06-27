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
        self.nav.reset_walk()

    def run_algorithim(self) -> None:
        pose = self.bez.sensors.get_pose()
        pos = pose.position

        # rotate +- 90 degrees from current pos
        if self._clockwise:
            # yaw=-90°, pitch=0, roll=0
            target_goal = Transformation(pos, euler=[-np.pi / 2, 0, 0])
        else:
            target_goal = Transformation(pos, euler=[np.pi / 2, 0, 0])


        self.nav.walk(target_goal)

        # simulating finding the ball (if ball still not found, rotate again)
        if self.nav.t > 5 and self.bez.found_ball == False:
            self.context.transition_to(Rotate(clockwise=True))

        elif self.bez.found_ball:
            self.bez.head_status = BezStatusEnum.WALK
            # TODO: where to walk after finding the ball

        # check if fallen
        from soccer_strategy.behavior.state.get_up import GetUp
        y, p, r = self.bez.sensors.get_imu()

        if p > 1.25:
            self.context.transition_to(GetUp("getupfront"))
        elif p < -1.25:
            print("getupback: ")
            self.context.transition_to(GetUp("getupback"))
        elif r < -1.54:
            self.context.transition_to(GetUp("getupsideleft"))
        elif r > 1.54:
            self.context.transition_to(GetUp("getupsideright"))

    def ready_to_switch_to(self) -> bool:
        return True
