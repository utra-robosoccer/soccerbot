import math
from threading import Lock

import rospy as rp
import serial
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import Vector3
from imu_receiver import IMUReceiver
from motor_receiver import MotorReceiver
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from transformations import *
from transmitter import Transmitter
from wait_for_ms import WaitForMs


class Communication:
    def __init__(self, servo_ser, imu_ser):
        self._last_angles = None
        self._last_imu = None

        # https://www.pieter-jan.com/node/11
        self.pitch_acc = 0
        self.roll_acc = 0
        self.pitch = 0
        self.roll = 0

        servo_ser._motor_lock = (
            Lock()
        )  # TODO improve on this hacky exclusive lock over the serial port for motor TX/RX (which always requires flushing the RX buffer via the state machine due to echo, hence exclusive lock)
        self._tx_servo_thread = Transmitter(name="tx_servo_th", ser=servo_ser)
        self._rx_servo_thread = MotorReceiver(name="rx_servo_th", ser=servo_ser)
        self._rx_servo_thread.set_timeout(0.015)
        self._rx_servo_thread.bind(self.receive_servo_callback)

        self._rx_imu_thread = IMUReceiver(name="rx_imu_th", ser=imu_ser)
        self._rx_imu_thread.set_timeout(0.010)
        self._rx_imu_thread.bind(self.receive_imu_callback)

        self._pub_imu = rp.Publisher("~imu_raw", Imu, queue_size=1)
        self._pub_joint_states = rp.Publisher("~joint_states", JointState, queue_size=1)

        self._imu_calibration = rp.get_param("~imu_calibration")
        self._motor_map = rp.get_param("~motor_mapping")

        self._joint_command_sub = rp.Subscriber("~joint_command", JointState, self.joint_command_callback)

        for motor in self._motor_map:
            self._motor_map[motor]["value"] = 0.0
        #     self._motor_map[motor]["subscriber"] = rp.Subscriber(motor + "/command", Float64, self.trajectory_callback, motor)
        #     self._motor_map[motor]["publisher"] = rp.Publisher(motor + "/state", JointControllerState, queue_size=1)

        self._publish_timer = rp.Timer(rp.Duration(nsecs=10000000), self.send_angles)

    def run(self):
        self._rx_servo_thread.start()
        self._rx_imu_thread.start()
        self._tx_servo_thread.start()

        tx_cycle = WaitForMs(10)
        tx_cycle.set_e_gain(1.5)
        # Never need to wait longer than the target time, but allow calls to
        # time.sleep for down to 3 ms less than the desired time
        tx_cycle.set_e_lim(0, -3.0)
        rp.spin()

    def joint_command_callback(self, joint_command):
        for motor_name, target in zip(joint_command.name, joint_command.position):
            if motor_name in self._motor_map:
                self._motor_map[motor_name]["value"] = target

    def send_angles(self, event):
        motor_angles = []  # [0] * len(self._motor_map)
        for motor_name, motor in self._motor_map.items():
            angle = np.rad2deg(motor["value"] * float(motor["direction"])) + float(motor["offset"])
            if "limits" in motor and motor["limits"] is not None:
                angle = max(motor["limits"][0], min(motor["limits"][1], angle))
            motor_angles.append(((motor_name, motor), angle))
        self._tx_servo_thread.send(motor_angles)

    def receive_servo_callback(self, received_angles):
        self._last_angles = received_angles
        self.publish_sensor_data(self._last_angles, self._last_imu)

    def receive_imu_callback(self, received_imu):
        # print(received_imu)
        self._last_imu = np.array(received_imu).reshape((6, 1))
        self.publish_sensor_data(self._last_angles, self._last_imu)

    def publish_sensor_data(self, received_angles, received_imu):
        if received_imu is None or received_angles is None:
            return

        # IMU FEEDBACK
        imu = Imu()
        imu.header.stamp = rp.rostime.get_rostime()
        imu.header.frame_id = "imu_link"

        # TODO autocalibrate
        imu.angular_velocity = Vector3(
            (-received_imu[2][0] - self._imu_calibration["gyro_offset"][0]) * self._imu_calibration["gryo_scale"][0],
            (received_imu[1][0] - self._imu_calibration["gyro_offset"][1]) * self._imu_calibration["gryo_scale"][1],
            (received_imu[0][0] - self._imu_calibration["gyro_offset"][2]) * self._imu_calibration["gryo_scale"][2],
        )
        imu.linear_acceleration = Vector3(
            (received_imu[5][0] - self._imu_calibration["acc_offset"][0]) * self._imu_calibration["acc_scale"][0],
            (received_imu[4][0] - self._imu_calibration["acc_offset"][1]) * self._imu_calibration["acc_scale"][1],
            (received_imu[3][0] - self._imu_calibration["acc_offset"][2]) * self._imu_calibration["acc_scale"][2],
        )
        imu.orientation_covariance[0] = -1
        # print(imu)
        self._pub_imu.publish(imu)

        # MOTOR FEEDBACK
        joint_state = JointState()
        joint_state.header.stamp = rp.rostime.get_rostime()
        # print(received_angles)
        for motor in self._motor_map:
            servo_idx = int(self._motor_map[motor]["id"])
            # print(servo_idx, str(type(list(received_angles.keys())[0])), servo_idx in received_angles)
            if int(servo_idx) < 12 and (servo_idx + 1) in received_angles:
                angle = received_angles[servo_idx + 1]
                if math.isnan(angle):  # TODO fix this
                    continue
                angle = (angle - float(self._motor_map[motor]["offset"])) * float(self._motor_map[motor]["direction"])
                angle = np.deg2rad(angle)
            else:
                angle = self._motor_map[motor]["value"]

            # # Joint controller state
            # state = JointControllerState()
            # state.process_value = angle
            # state.command = self._motor_map[motor]["value"]
            # state.error = angle - self._motor_map[motor]["value"]
            # state.process_value_dot = 0  # TODO PID settings and process value dot
            # state.header.stamp = rp.rostime.get_rostime()
            # self._motor_map[motor]["publisher"].publish(state)

            # Joint State
            joint_state.name.append(motor)
            joint_state.position.append(angle)
        # print(joint_state)
        self._pub_joint_states.publish(joint_state)


if __name__ == "__main__":
    Communication(*[serial.Serial(*a) for a in [("/dev/ttyUSB0", 1000000), ("/dev/ttyACM0", 230400)]])
