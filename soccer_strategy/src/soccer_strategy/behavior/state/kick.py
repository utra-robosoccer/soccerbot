from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_common.transformation import Transformation
from soccer_strategy.behavior import Behavior

from soccer_strategy.behavior.state.get_up import GetUp




class Kick(Behavior):
    def action(self) -> None:
        self.nav.kick_ready()

    def run_algorithim(self) -> None:
        # enable walking, essentially adding the unit test from test_placo into here, which we can then call in test_placto
        # behavior_context may have issues with the objects defined in the beginning
        # read: https://refactoring.guru/design-patterns/state
        self.nav.kick()

        if self.nav.kick_finished():
            self.nav.wait(200)
            self.ready_to_switch_to()
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
            else:
                from soccer_strategy.behavior.state.balance import Balance
                self.context.transition_to(Balance())


    def ready_to_switch_to(self) -> bool:
        return True
