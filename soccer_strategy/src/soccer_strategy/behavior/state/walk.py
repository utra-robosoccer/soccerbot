from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior


class Walk(Behavior):
    def action(self) -> None:
        self.bez.status = BezStatusEnum.WALK

    def run_algorithim(self) -> None:
        # enable walking, essentially adding the unit test from test_placo into here, which we can then call in test_placto
        # behavior_context may have issues with the objects defined in the beginning
        # read: https://refactoring.guru/design-patterns/state
        pass

    def ready_to_switch_to(self) -> bool:
        return True
