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
            output_limits=(-3.14, 3.14),
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
        self.time_since_last_center = 0
        self.time_since_last_ball = 0


    def track_ball(self,ball_pixel=[0, 0]):
        self.counter += 1
        self.time_since_last_center += 1
        self.time_since_last_ball += 1
        ret_msg = -self.ball_dx
        if self.counter % self.n_substeps == 0:
            if ball_pixel != self.last_ball_pixel:
                self.time_since_last_ball = 0
                # print(f"here: {ball_pixel}, {self.ball_dx}, {self.ball_dy}")
                self.last_ball_pixel = ball_pixel
                self.ball_dx = self.ball_x_pid.update(3.2 - ball_pixel[0] / 100.0) # TODO change with center pixel from cam settign
                self.ball_dy = self.ball_y_pid.update(ball_pixel[1] / 100.0)
            if abs(ball_pixel[0] - 320.5) < 30:
                self.time_since_last_center = 0

            # print(f"{ball_pixel}, {self.ball_dx}, {self.ball_dy}")
            self.bez.motor_control.set_head_target_angles([-self.ball_dx, self.ball_dy])
            # self.bez.motor_control.set_single_motor("head_pitch", self.ball_dy)
            self.bez.motor_control.set_motor()
        # print(f"here: {self.time_since_last_center * self.bez.world.dt}, {self.time_since_last_ball * self.bez.world.dt} {self.ball_dx}, {self.ball_dy}")
        if self.time_since_last_center * self.bez.world.dt > 1.0:
            ret_msg =  -self.ball_dx
        if self.time_since_last_ball * self.bez.world.dt > 1.0:
            self.bez.motor_control.set_head_target_angles([0, 0.4])
            self.bez.motor_control.set_motor()
            ret_msg = 0 #-self.bez.sensors.get_pose().orientation_euler[0]
        return ret_msg
