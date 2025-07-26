from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior


class Balance(Behavior):
    def action(self) -> None:
        self.bez.status = BezStatusEnum.BALANCE

    def run_algorithim(self) -> None:
        pass

    def ready_to_switch_to(self) -> bool:
        return True
