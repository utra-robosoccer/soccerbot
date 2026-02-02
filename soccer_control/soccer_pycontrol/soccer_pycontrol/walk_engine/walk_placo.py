import time

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.walk_interface import WalkInterface


class WalkPlaco(WalkInterface):
    def __init__(self,bez: Bez, imu_feedback_enabled: bool = False, ball: bool = False, sim: bool = True ):
        super().__init__(bez, imu_feedback_enabled)
        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, time.time, ball=ball, sim=sim)
        self._n_substeps = int(self.foot_step_planner.DT/self.bez.world.dt)
        self._counter = 0

    def setup(self, dx: float = 0.0, dy: float = 0.0,dtheta: float = 0.0, nb_steps: int = 10 ) -> None:
        super().setup()

        self.foot_step_planner.setup_walk(dx, dy, dtheta, nb_steps)
        self.t = 0

    def walking(self,dx:float= 0, dy:float= 0, dtheta:float = 0) -> None:
        self._counter += 1
        if self._counter % self._n_substeps == 0:
            self.foot_step_planner.configure_planner(dx, dy, dtheta)

            self.foot_step_planner.plan_steps(self.t)

            self.bez.motor_control.set_angles_from_placo(self.foot_step_planner.robot)
            super().walking(dx, dy, dtheta)

            self.bez.motor_control.set_motor()
            self.t = self.foot_step_planner.step(self.t)