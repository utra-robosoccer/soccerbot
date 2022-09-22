class PID:
    """
    Useful reusable PID class
    """

    def __init__(self, Kd: float, Kp: float, Ki: float, setpoint=0.0, output_limits=None):
        """
        Initializes the PID controller

        :param Kd: The derivative portion of the PID controller
        :param Kp: The proportial portion of the PID controller
        :param Ki: The integral portion of the PID controller
        :param setpoint: The point that the PID is set to
        :param output_limits: Max and minimum setting for the PID controller
        :type output_limits: [float, float]
        """
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki
        self.setpoint = setpoint
        self.output_limits = output_limits

        self.integral = 0
        self.last_error = 0
        self.last_value = 0
        pass

    def reset(self) -> None:
        """
        Resets the PID controller to be used again
        """
        self.integral = 0
        self.last_error = 0
        self.last_value = 0

    def update(self, measurement: float, delta_t: float = 1) -> float:
        """
        Takes a measurement and updates the PID, returns the gain used by the PID controller

        :param measurement: The measurement used to compare against the setpoint
        :param delta_t: The time passed since the measurement
        :return: The gain to apply to the PID controller as a result
        """
        error = self.setpoint - measurement
        derivative = error - self.last_error

        value = ((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)) * delta_t

        if self.output_limits is not None:
            value = min(max(value, self.output_limits[0]), self.output_limits[1])

        self.last_value = value
        self.last_error = error
        self.integral = self.integral + error

        return value
