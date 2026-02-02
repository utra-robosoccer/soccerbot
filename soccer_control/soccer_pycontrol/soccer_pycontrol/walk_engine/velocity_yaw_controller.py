from soccer_common import PID


class VelocityYawController:
    def __init__(self):
        self.max_vel = 0.05

        self.nav_yaw_pid = PID(
            Kp=-0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )

    def reset(self):
        self.nav_yaw_pid.reset()

    def setpoint(self,yaw):
        self.nav_yaw_pid.setpoint=yaw


    def update(self, yaw):
        return  self.nav_yaw_pid.update(yaw)
