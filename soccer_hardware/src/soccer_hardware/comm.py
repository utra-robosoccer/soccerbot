# Comm.py

import time
import os
import numpy as np
from prettytable import PrettyTable
from Transmitter import Tx
from Receiver import Rx
from utility import *

try:
    import rospy
    from soccer_msgs.msg import RobotGoal
    from soccer_msgs.msg import RobotState
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Vector3
    from geometry_msgs.msg import Quaternion
    from tf.msg import tfMessage
    from tf.transformations import quaternion_from_euler
    from transformations import *
except:
    pass

class WaitForMs:
    '''
    Delays using time.sleep() have a tendency to overshoot the target wait time.
    This class uses feedback to adjust the wait time to achieve the target.
    '''
    def __init__(self, ms):
        self._t_ref = ms / 1000.0
        self._t_err = 0
        self._e_gain = 0.0
        self._e_lim_high = 0.0
        self._e_lim_low = 0.0
        self._debug = False

    def set_e_gain(self, e):
        '''
        Sets the error amplification factor.
        '''
        self._e_gain = e

    def set_e_lim(self, high, low):
        '''
        Sets the max and min error bounds. Arguments in ms.
        '''
        self._e_lim_high = high / 1000.0
        self._e_lim_low = low / 1000.0

    def _limit(self, err):
        '''
        Bounds the error feedback, if necessary, so that
        _e_lim_high >= err >= _e_lim_low.
        '''
        if err > self._e_lim_high:
            return self._e_lim_high
        elif err < self._e_lim_low:
            return self._e_lim_low
        else:
            return err

    def wait(self):
        '''
        Wait with feedback and limiting. Behaves like time.sleep if the gain and
        limits are not set.
        '''
        ts = time.time()

        # Adjust wait time based on previous error
        t_wait = self._t_ref + self._t_err
        time.sleep(t_wait)

        # Update error term
        t_waited = time.time() - ts
        err = self._t_ref - t_waited
        self._t_err = self._t_err + self._e_gain*err
        self._t_err = self._limit(err)

        if self._debug:
            print(
                np.round(t_wait*1000,4), np.round(t_waited*1000,4),
                np.round(err*1000,4), np.round(self._t_err*1000,4)
            )


class Comm:
    def __init__(self):
        self._started = False
        self._last_angles = np.ndarray
        self._last_imu = np.ndarray
        self._num_rx = 0
        self._num_tx = 0
        self._t_start = 0
        return

    def _print_angles(self, sent, received):
        ''' Prints out 2 numpy vectors side-by-side, where the first vector entry
            is interpreted as belonging to motor 1, the seconds to motor 2, etc.
        '''
        assert sent.shape[0] == received.shape[0]
        t = PrettyTable(['Motor Number', 'Sent', 'Received'])
        for i in range(sent.shape[0]):
            t.add_row([str(i + 1), round(sent[i][0], 4), round(received[i][0], 2)])
        print(t)

    def _print_imu(self, received):
        ''' Prints out a numpy vector interpreted as data from the IMU, in the
            order X-gyro, Y-gyro, Z-gyro, X-accel, Y-accel, Z-accel.
        '''

        t = PrettyTable(['', 'Gyro (deg/s)', 'Accel (m/s^2)'])
        t.add_row(["X", round(received[0][0], 2), round(received[3][0], 2)])
        t.add_row(["Y", round(received[1][0], 2), round(received[4][0], 2)])
        t.add_row(["Z", round(received[2][0], 2), round(received[5][0], 2)])
        print(t)

    def _print_handler(self):
        current_time = time.time()
        if(current_time - self._last_print_time >= 1):
            self._last_print_time = current_time
            print('\n')
            self._num_rx = self._rx_thread.get_num_rx()[0]
            logString("Received: {0}".format(self._num_rx))
            self._num_tx = self._tx_thread.get_num_tx()
            logString("Transmitted: {0}\n".format(self._num_tx))
            if(self._num_rx > 0):
                # Prints the last valid data received
                self._print_angles(self._goal_angles[0:12], self._last_angles[0:12])
                self._print_imu(self._last_imu)

    def _trajectory_callback(self, robotGoal):
        '''
        Used by ROS. Converts the motor array from the order and sign convention
        used by controls to that used by embedded
        '''
        m = getCtrlToMcuAngleMap()
        self._goal_angles = m.dot(robotGoal.trajectories[0:18])
        self._goal_angles = self._goal_angles[:, np.newaxis]
        self._tx_thread.send(self._goal_angles)

    def _publish_sensor_data(self, received_angles, received_imu):
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
        ctrlAngleArray = mcuToCtrlAngles(received_angles)
        robotState = RobotState()
        for i in range(12):
            robotState.joint_angles[i] = ctrlAngleArray[i][0]
        # Convert motor array from the embedded order and sign convention
        # to that used by controls
        m = getCtrlToMcuAngleMap()
        robotState.joint_angles[0:12] = np.linalg.inv(m).dot(robotState.joint_angles[0:18])[0:12]
        self._pub_angles.publish(robotState)

    def _receive_callback(self, received_angles, received_imu):
        self._last_angles = received_angles
        self._last_imu = received_imu
        if self._ros_is_on == True:
            self._publish_sensor_data(received_angles, received_imu)

    def init(self, ser, ros_is_on, traj, step_is_on, wait_feedback_is_on):
        self._last_print_time = time.time()
        self._ros_is_on = ros_is_on
        self._step_is_on = step_is_on
        self._wait_feedback_is_on = wait_feedback_is_on
        self._use_trajectory = False

        self._tx_thread = Tx(name="tx_th", ser=ser)
        self._rx_thread = Rx(name="rx_th", ser=ser)
        self._rx_thread.set_timeout(0.010) # 10 ms
        self._rx_thread.bind(self._receive_callback)

        if(self._ros_is_on == True):
            rospy.init_node('soccer_hardware', anonymous=True)
            rospy.Subscriber("robotGoal", RobotGoal, self._trajectory_callback, queue_size=1)
            self._pub_imu = rospy.Publisher('soccerbot/imu', Imu, queue_size=1)
            self._pub_angles = rospy.Publisher('soccerbot/robotState', RobotState, queue_size=1)
        else:
            trajectories_dir = os.path.join("trajectories", traj)
            try:
                self._trajectory = np.loadtxt(
                    open(trajectories_dir, "rb"),
                    delimiter=",",
                    skiprows=0
                )

                logString("Opened trajectory {0}".format(traj))
                self._use_trajectory = True
            except IOError as err:
                logString("Error: Could not open trajectory: {0}".format(err))
                logString("(Is your shell running in the soccer-communication directory?)")
                logString("Standing pose will be sent instead...")

    def begin_event_loop(self):
        self._rx_thread.start()
        self._tx_thread.start()
        self._t_start = time.time()
        self._started = True

        tx_cycle = WaitForMs(10)
        tx_cycle.set_e_gain(1.5)
        # Never need to wait longer than the target time, but allow calls to
        # time.sleep for down to 3 ms less than the desired time
        tx_cycle.set_e_lim(0, -3.0)

        if self._ros_is_on == True:
            rospy.spin()
        elif self._use_trajectory:
            while True:
                # Iterate through the static trajectory forever
                # TODO: If the static trajectories are ever re-generated, we will need
                # TODO: to dot the trajectories with the ctrlToMcuAngleMap to shuffle
                # TODO: them properly
                for i in range(self._trajectory.shape[1]):
                    if self._step_is_on:
                        wait = input('Press enter to send next pose')
                    elif self._wait_feedback_is_on == True:
                        tx_cycle.wait()
                    else:
                        time.sleep(0.010)
                    self._goal_angles = self._trajectory[:, i:i+1]
                    self._tx_thread.send(self._goal_angles)
                    self._print_handler()
        else:
            # Send standing pose
            self._goal_angles = np.zeros((18,1));
            while True:
                if self._step_is_on:
                    wait = input('Press enter to send next pose')
                elif self._wait_feedback_is_on == True:
                    tx_cycle.wait()
                else:
                    time.sleep(0.010)
                self._tx_thread.send(self._goal_angles)
                self._print_handler()

    def cleanup(self):
        if self._started:
            logString("Cleaning up threads...")
            self._rx_thread.stop()
            self._tx_thread.stop()
            self._num_rx, num_rx_failures = self._rx_thread.get_num_rx()
            self._num_tx = self._tx_thread.get_num_tx()
            self._rx_thread.join()
            logString("Rx thread joined")
            self._tx_thread.join()
            logString("Tx thread joined")
            t_stop = time.time()
            if self._num_rx != 0 and self._num_tx != 0:
                logString("Transmitted {0} packets and received {1} ({2}% success)".format(
                    self._num_tx, self._num_rx, np.round(self._num_rx*100.0/self._num_tx, 2)
                )
                )

                tx_period = (t_stop - self._t_start) / self._num_tx
                logString("Transmitted {0} packets/s on average (period = {1} ms)".format(
                    np.round(1/tx_period, 2), np.round(tx_period * 1000, 2)
                )
                )

                rx_period = (t_stop - self._t_start) / self._num_rx
                logString("Received {0} packets/s on average (period = {1} ms)".format(
                    np.round(1/rx_period, 2), np.round(rx_period * 1000, 2)
                )
                )

                logString("{0} reception(s) timed out".format(num_rx_failures))