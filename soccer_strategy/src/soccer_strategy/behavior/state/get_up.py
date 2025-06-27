from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior



class GetUp(Behavior):
    def __init__(self, get_up_traj: str):
        super().__init__()
        self._traj = get_up_traj

    def action(self) -> None:
        self.bez.status = BezStatusEnum.FALLEN

    def run_algorithim(self) -> None:
        self.context.tm.send_trajectory(self._traj)
        y, p, r = self.bez.sensors.get_imu()

        if -0.1 < p < 0.1 and -0.1 < r < 0.1:
            from soccer_strategy.behavior.state.balance import Balance
            self.context.transition_to(Balance())
        else:
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
