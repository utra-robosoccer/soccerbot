from soccer_common import PID


class VelocityPathController:
    def __init__(self):
        self.max_vel = 0.05

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
            output_limits=(-self.max_vel, self.max_vel),
        )  # TODO could also mod if balance is decreasing

        self.nav_yaw_pid = PID(
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-1, 1),
        )

    def reset(self):
        self.nav_x_pid.reset()
        self.nav_y_pid.reset()
        self.nav_yaw_pid.reset()

    def setpoint(self, x,y,yaw):
        self.nav_x_pid.setpoint=x
        self.nav_y_pid.setpoint=y
        self.nav_yaw_pid.setpoint=yaw


    def update(self, x,y,yaw):
        return self.nav_x_pid.update(x), self.nav_y_pid.update(y), self.nav_yaw_pid.update(yaw)
