from soccer_common import PID


class HeadControl:
    def __init__(self, bez):
        self.ball_dx = 0
        self.ball_dy = 0.7
        self.last_ball_pixel = [0, 0]
        self.bez = bez
        self.ball_x_pid = PID(
            Kp=0.05,
            Kd=0,
            Ki=0.03,
            setpoint=0,
            output_limits=(-1.57, 1.57),
        )

        self.ball_y_pid = PID(
            Kp=-0.05,
            Kd=0,
            Ki=-0.03,
            setpoint=2.4,
            output_limits=(0.1, 1.3),
        )

        ctrl_dt = 1/50.0
        self.n_substeps = int(round(ctrl_dt/self.bez.world.dt))
        self.counter = 0


    def track_ball(self,ball_pixel=[0, 0]):
        self.counter += 1
        if self.counter % self.n_substeps == 0:
            if ball_pixel != self.last_ball_pixel:

                self.last_ball_pixel = ball_pixel
                self.ball_dx = self.ball_x_pid.update(3.2 - ball_pixel[0] / 100.0)
                self.ball_dy = self.ball_y_pid.update(ball_pixel[1] / 100.0)
            print(f"{ball_pixel}, {self.ball_dx}, {self.ball_dy}")
            # self.bez.motor_control.set_head_target_angles([-self.ball_dx, self.ball_dy])
            self.bez.motor_control.set_motor()
