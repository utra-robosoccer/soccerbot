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

    def ready_to_switch_to(self) -> bool:
        return True
