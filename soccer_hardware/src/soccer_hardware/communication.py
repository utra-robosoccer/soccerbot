import time
import rospy as rp
from wait_for_ms import WaitForMs
from prettytable import PrettyTable
from transmitter import Transmitter
from receiver import Receiver
from utility import *
from soccer_msgs.msg import RobotGoal
from soccer_msgs.msg import RobotState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from transformations import *


class Communication:
    def __init__(self, ser, step_is_on, wait_feedback_is_on):
        self._started = False
        self._last_angles = np.ndarray
        self._last_imu = np.ndarray
        self._num_rx = 0
        self._num_tx = 0
        self._t_start = 0

        self._last_print_time = time.time()
        self._step_is_on = step_is_on
        self._wait_feedback_is_on = wait_feedback_is_on
        self._use_trajectory = False

        self._tx_thread = Transmitter(name="tx_th", ser=ser)
        self._rx_thread = Receiver(name="rx_th", ser=ser)
        self._rx_thread.set_timeout(0.010)
        self._rx_thread.bind(self.receive_callback)

        rp.init_node('soccer_hardware', anonymous=True)
        rp.Subscriber("robotGoal", RobotGoal, self.trajectory_callback, queue_size=1)
        self._pub_imu = rp.Publisher('soccerbot/imu', Imu, queue_size=1)
        self._pub_angles = rp.Publisher('soccerbot/robotState', RobotState, queue_size=1)

    def __del__(self):
        if self._started:
            log_string("Cleaning up threads...")
            self._rx_thread.stop()
            self._tx_thread.stop()
            self._num_rx, num_rx_failures = self._rx_thread.get_num_rx()
            self._num_tx = self._tx_thread.get_num_tx()
            self._rx_thread.join()
            log_string("Rx thread joined")
            self._tx_thread.join()
            log_string("Tx thread joined")
            t_stop = time.time()
            if self._num_rx != 0 and self._num_tx != 0:
                log_string("Transmitted {0} packets and received {1} ({2}% success)".format(
                    self._num_tx, self._num_rx, np.round(self._num_rx * 100.0 / self._num_tx, 2)
                )
                )

                tx_period = (t_stop - self._t_start) / self._num_tx
                log_string("Transmitted {0} packets/s on average (period = {1} ms)".format(
                    np.round(1 / tx_period, 2), np.round(tx_period * 1000, 2)
                )
                )

                rx_period = (t_stop - self._t_start) / self._num_rx
                log_string("Received {0} packets/s on average (period = {1} ms)".format(
                    np.round(1 / rx_period, 2), np.round(rx_period * 1000, 2)
                )
                )

                log_string("{0} reception(s) timed out".format(num_rx_failures))

    def run(self):
        self._rx_thread.start()
        self._tx_thread.start()
        self._t_start = time.time()
        self._started = True

        tx_cycle = WaitForMs(10)
        tx_cycle.set_e_gain(1.5)
        # Never need to wait longer than the target time, but allow calls to
        # time.sleep for down to 3 ms less than the desired time
        tx_cycle.set_e_lim(0, -3.0)
        rp.spin()

    def print_angles(self):
        """ Prints out 2 numpy vectors side-by-side, where the first vector entry
            is interpreted as belonging to motor 1, the seconds to motor 2, etc.
        """
        sent = self._goal_angles[0:12]
        received = self._last_angles[0:12]

        assert sent.shape[0] == received.shape[0]
        t = PrettyTable(['Motor Number', 'Sent', 'Received'])
        for i in range(sent.shape[0]):
            t.add_row([str(i + 1), round(sent[i][0], 4), round(received[i][0], 2)])
        print(t)

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

    def print_handler(self):
        current_time = time.time()
        if current_time - self._last_print_time >= 1:
            self._last_print_time = current_time
            print('\n')
            self._num_rx = self._rx_thread.get_num_rx()[0]
            log_string("Received: {0}".format(self._num_rx))
            self._num_tx = self._tx_thread.get_num_tx()
            log_string("Transmitted: {0}\n".format(self._num_tx))
            if self._num_rx > 0:
                # Prints the last valid data received
                self.print_angles()
                self.print_imu()

    def trajectory_callback(self, robot_goal):
        """
        Used by ROS. Converts the motor array from the order and sign convention
        used by controls to that used by embedded
        """
        m = getCtrlToMcuAngleMap()
        self._goal_angles = m.dot(robot_goal.trajectories[0:18])
        self._goal_angles = self._goal_angles[:, np.newaxis]
        self._tx_thread.send(self._goal_angles)

    def receive_callback(self, received_angles, received_imu):
        self._last_angles = received_angles
        self._last_imu = received_imu
        self.publish_sensor_data(received_angles, received_imu)

    def publish_sensor_data(self, received_angles, received_imu):
        # IMU FEEDBACK
        imu = Imu()
        vec1 = Vector3(-received_imu[2][0], received_imu[1][0], received_imu[0][0])
        imu.angular_velocity = vec1
        vec2 = Vector3(received_imu[5][0], received_imu[4][0], received_imu[3][0])
        imu.linear_acceleration = vec2
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
