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
    def __init__(self, ser):
        self._last_angles = np.ndarray
        self._last_imu = np.ndarray

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
        for motor in self._motor_map:
            if int(self._motor_map[motor]["id"]) < 12:
                angle = received_angles[int(self._motor_map[motor]["id"])]
                angle = (angle - float(self._motor_map[motor]["offset"])) / float(self._motor_map[motor]["direction"])
                angle = np.deg2rad(angle)
            else:
                angle = self._motor_map[motor]["value"]
            state = JointControllerState()
            state.process_value = angle
            state.command = self._motor_map[motor]["value"]
            state.error = angle - self._motor_map[motor]["value"]
            state.process_value_dot = 0 # TODO PID settings and process value dot
            state.header.stamp = rp.rostime.get_rostime()
            self._motor_map[motor]["publisher"].publish(state)
