from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_common.transformation import Transformation
from soccer_strategy.behavior import Behavior
from soccer_strategy.behavior.state.balance import Balance
from soccer_strategy.behavior.state.kick import Kick
from soccer_strategy.behavior.state.rotate import Rotate



class Walk(Behavior):
    def __init__(self, target_goal: Transformation):
        super().__init__()
        self._target_goal = target_goal

    def action(self) -> None:
        self.bez.status = BezStatusEnum.WALK

    def run_algorithim(self) -> None:
        # enable walking, essentially adding the unit test from test_placo into here, which we can then call in test_placto
        # behavior_context may have issues with the objects defined in the beginning
        # read: https://refactoring.guru/design-patterns/state
        self.nav.walk(self._target_goal)

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

        # after walking to target, transition to rotate, balance, or kick the ball
        if not self.nav.enable_walking:
            if self.bez.head_status == BezStatusEnum.ROTATING:
                self.context.transition_to(Rotate(True))

            elif self.bez.head_status == BezStatusEnum.BALANCE:
                self.context.transition_to(Balance())

            else:
                self.nav.wait(200)
                self.context.transition_to(Kick())

    def ready_to_switch_to(self) -> bool:
        return True
