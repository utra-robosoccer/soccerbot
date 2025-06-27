from typing import List

from soccer_common import Transformation
from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior
from soccer_strategy.behavior.state.get_up import GetUp


class Balance(Behavior):
    def __init__(self, target_goal: List=[0.0, 0.0, 0.0, 10, 10]):
        super().__init__()
        self._target_goal = target_goal

    def action(self) -> None:
        self.bez.status = BezStatusEnum.BALANCE

    def run_algorithim(self) -> None:
        # just walking in place
        self.nav.walk(self._target_goal)

        # simulating looking for the ball (if the head doesnt find the ball in 5 seconds then rotate the body)
        if self.nav.t > 5 and self.bez.found_ball == False:
            self.bez.body_status = BezStatusEnum.ROTATING
            from soccer_strategy.behavior.state.rotate import Rotate
            # should add some logic for which direction to rotate
            self.context.transition_to(Rotate(clockwise=True))

        elif self.bez.found_ball == True:
            from soccer_strategy.behavior.state.walk import Walk
            self.bez.head_status = BezStatusEnum.WALK
            # TODO: where to walk after finding the ball


        # check if fallen
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

        # switch to walking after some time (testing state transitions)
        if self.bez.head_status == BezStatusEnum.WALK:
            from soccer_strategy.behavior.state.walk import Walk
            self.nav.wait(200)
            self.bez.body_status = BezStatusEnum.WALK
            target_goal = Transformation(position=[0.8, 0, 0], euler=[0, 0, 0])
            self.context.transition_to(Walk(target_goal))

    def ready_to_switch_to(self) -> bool:
        return True
