import time
import rospy as rp
from wait_for_ms import WaitForMs
from prettytable import PrettyTable
from transmitter import Transmitter
from receiver import Receiver
from utility import *
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from transformations import *


class Communication:
    def __init__(self, ser, step_is_on, wait_feedback_is_on):
        self._last_angles = np.ndarray
        self._last_imu = np.ndarray

        # MCU Callbacks
        self._tx_thread = Transmitter(name="tx_th", ser=ser)
        self._rx_thread = Receiver(name="rx_th", ser=ser)
        self._rx_thread.set_timeout(0.010)
        self._rx_thread.bind(self.receive_callback)

        self._pub_imu = rp.Publisher('imu', Imu, queue_size=1)
        self._motor_map = rp.get_param("~motor_mapping")
        for motor in self._motor_map:
            self._motor_map[motor]["subscriber"] = rp.Subscriber(motor + "/command", Float64, self.trajectory_callback, motor)
            self._motor_map[motor]["publisher"] = rp.Publisher(motor + "/state", JointControllerState, queue_size=1)
            self._motor_map[motor]["value"] = 0.0

        self._publish_timer = rp.Timer(rp.Duration(nsecs=10000000), self.send_angles)

    def run(self):
        self._rx_thread.start()
        self._tx_thread.start()

        tx_cycle = WaitForMs(10)
        tx_cycle.set_e_gain(1.5)
        # Never need to wait longer than the target time, but allow calls to
        # time.sleep for down to 3 ms less than the desired time
        tx_cycle.set_e_lim(0, -3.0)
        rp.spin()

    def trajectory_callback(self, robot_goal, motor):
        self._motor_map[motor]["value"] = robot_goal.data

    def send_angles(self, event):
        motor_angles = [0] * len(self._motor_map)
        for motor in self._motor_map:
            motor_angles[int(self._motor_map[motor]["id"])] = (np.rad2deg(self._motor_map[motor]["value"] * float(self._motor_map[motor]["direction"])) + float(self._motor_map[motor]["offset"]))
        self._tx_thread.send(motor_angles)

    def print_imu(self):
        """ Prints out a numpy vector interpreted as data from the IMU, in the
            order X-gyro, Y-gyro, Z-gyro, X-accel, Y-accel, Z-accel.
        """
        received = self._last_imu

        t = PrettyTable(['', 'Gyro (deg/s)', 'Accel (m/s^2)'])
        t.add_row(["X", round(received[0][0], 2), round(received[3][0], 2)])
        t.add_row(["Y", round(received[1][0], 2), round(received[4][0], 2)])
        t.add_row(["Z", round(received[2][0], 2), round(received[5][0], 2)])
        print(t)

    def receive_callback(self, received_angles, received_imu):
        self._last_angles = received_angles
        self._last_imu = received_imu
        self.publish_sensor_data(received_angles, received_imu)

    def publish_sensor_data(self, received_angles, received_imu):
        # IMU FEEDBACK
        imu = Imu()
        imu.angular_velocity = Vector3(-received_imu[2][0], received_imu[1][0], received_imu[0][0])
        imu.linear_acceleration = Vector3(received_imu[5][0], received_imu[4][0], received_imu[3][0])
        self._pub_imu.publish(imu)

        # MOTOR FEEDBACK
        # Convert motor array from the embedded coordinate system to that
        # used by controls
        ctrl_angle_array = mcuToCtrlAngles(received_angles)
        robot_state = RobotState()
        for i in range(12):
            robot_state.joint_angles[i] = ctrl_angle_array[i][0]

        # Convert motor array from the embedded order and sign convention
        # to that used by controls
        m = getCtrlToMcuAngleMap()
        robot_state.joint_angles[0:12] = np.linalg.inv(m).dot(robot_state.joint_angles[0:18])[0:12]
        self._pub_angles.publish(robot_state)
