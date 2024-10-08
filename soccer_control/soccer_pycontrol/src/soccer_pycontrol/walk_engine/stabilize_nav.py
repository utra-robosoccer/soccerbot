from typing import List, Tuple

from soccer_common import PID


class StabilizeNav:
    """
    Manages PID loops for pitch, roll while standing and walking.
    """

    def __init__(self, max_vel: float):
        self.max_vel = max_vel
        self.nav_x_pid = PID(
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )
        self.nav_y_pid = PID(  # TODO properly tune later
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-0.05, 0.05),
        )  # TODO could also mod if balance is decreasing

        self.nav_yaw_pid = PID(
            Kp=0.05,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-0.3, 0.3),
        )

    # TODO i feel like this can be better organized
    def reset_imus(self) -> None:
        """
        Reset the walking and standing PID values
        """

        self.nav_x_pid.reset()
        self.nav_y_pid.reset()
        self.nav_yaw_pid.reset()

    def set_target(self, targets: List[float]) -> None:
        self.nav_x_pid.setpoint = targets[0]
        self.nav_y_pid.setpoint = targets[1]
        self.nav_yaw_pid.setpoint = targets[2]

    def update(self, measurements: List[float]) -> (float, float, float):
        return (self.nav_x_pid.update(measurements[0]), self.nav_y_pid.update(measurements[1]), self.nav_yaw_pid.update(measurements[2]))
