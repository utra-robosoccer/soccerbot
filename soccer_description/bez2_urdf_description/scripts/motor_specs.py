import math
VAR_LENGTH = 30
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
class JointSpecs:
    def __init__(self, name, max_tor_12V, max_tor_14_8V, max_vel_12V_rpm, max_vel_14_8V_rpm,
    NT_curve_point_1, NT_curve_point_2):
        self.name = name
        self.max_tor_12V = max_tor_12V
        self.max_tor_14_8V = max_tor_14_8V
        self.max_vel_12V_rpm = max_vel_12V_rpm
        self.max_vel_14_8V_rpm = max_vel_14_8V_rpm
        self.NT_curve_point_1_12V = NT_curve_point_1
        self.NT_curve_point_2_12V = NT_curve_point_2

        # convert to rad/s
        self.max_vel_12V = self.max_vel_12V_rpm * math.tau / 60
        self.max_vel_14_8V = self.max_vel_14_8V_rpm * math.tau / 60
        self.NT_curve_point_1_12V.y = self.NT_curve_point_1_12V.y * math.tau / 60
        self.NT_curve_point_2_12V.y = self.NT_curve_point_2_12V.y * math.tau / 60
        # scale N-T points from 12V to 14_8V
        self.scale_factor_tor = self.max_tor_14_8V / self.max_tor_12V
        self.scale_factor_vel = self.max_vel_14_8V / self.max_vel_12V
        self.NT_curve_point_1_14_8V = Point(
            self.NT_curve_point_1_12V.x * self.scale_factor_tor,
            self.NT_curve_point_1_12V.y * self.scale_factor_vel)
        self.NT_curve_point_2_14_8V = Point(
            self.NT_curve_point_2_12V.x * self.scale_factor_tor,
            self.NT_curve_point_2_12V.y * self.scale_factor_vel)
    def get_values(self, use_14_8V):
        # choose correct values based on voltage
        if use_14_8V:
            stall_torque = self.max_tor_14_8V
            vel = self.max_vel_14_8V
            point_1 = self.NT_curve_point_1_14_8V
            point_2 = self.NT_curve_point_2_14_8V
        else:
            stall_torque = self.max_tor_12V
            vel = self.max_vel_12V
            point_1 = self.NT_curve_point_1_12V
            point_2 = self.NT_curve_point_2_12V

        vel_diff = (point_2.y - point_1.y)
        torque_diff = (point_2.x - point_1.x)
        a = vel_diff / torque_diff
        b = point_1.y - (point_1.x * a)
        # compute torque at vel=0
        # 0 = a*x+b -> x = -b / a
        torque_vel0 = -b / a
        friction = stall_torque - torque_vel0
        damping_constant = -torque_diff / vel_diff
        self.print_value("torque", stall_torque, use_14_8V)
        self.print_value("vel", vel, use_14_8V)
        self.print_value("damping", damping_constant, use_14_8V)
        self.print_value("friction", friction, use_14_8V)
        print("")
    def print_value(self, property_name, value, use_14_8V):
        v_text = "14.8V" if use_14_8V else "12V"
        var_name = (f"{self.name}-{property_name}-{v_text}").ljust(VAR_LENGTH)
        print(f"{var_name}{value:5.2f}")
    def print_all_values(self):
        self.get_values(False)
        self.get_values(True)

# max_tor_12V, max_tor_14_8V, max_vel_12V_rpm, max_vel_14_8V_rpm, NT_curve_point_1, NT_curve_point_2
JointSpecs("MX28", 2.5, 3.1, 55, 67, Point(0.112, 49),
Point(1.288, 12)).print_all_values()
JointSpecs("MX64", 6.0, 7.3, 63, 78, Point(0.15, 64),
Point(2.85, 25)).print_all_values()
JointSpecs("MX106", 8.4, 10, 45, 55, Point(0.7, 42),
Point(5.6, 5)).print_all_values()
JointSpecs("XH540W270", 10.6, 12.9, 30, 37, Point(0.4, 29),
Point(8.6, 2.5)).print_all_values()

JointSpecs("XM430", 4.1, 4.8, 46, 57, Point(0.07, 44),
Point(2.968, 5.5)).print_all_values()
JointSpecs("XL430", 1.5, 1.5, 61, 61, Point(0.06, 49.6),
Point(0.84, 3.6)).print_all_values()