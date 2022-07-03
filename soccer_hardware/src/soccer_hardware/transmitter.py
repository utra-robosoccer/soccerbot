# Transmitter.py

import struct
from queue import Queue
from threading import Event, Lock, Thread

import numpy as np
import rospy as rp
import serial
from jx_servo_util import CMD_HEADS, CMDS, RWS
import jx_servo_util

import gobilda_servo_util
from utility import log_string

ANGLE2JX_POT_VOLTS = 0xFFF / 180.0
ANGLE2GOBILDA_PWM = 0x7F / 300.0

JX_JOINTS = ['right_leg_motor_0', 'right_leg_motor_1', 'right_leg_motor_2', 'right_leg_motor_3', 'right_leg_motor_4', 'right_leg_motor_5', 'left_leg_motor_0', 'left_leg_motor_1', 'left_leg_motor_2', 'left_leg_motor_3', 'left_leg_motor_4', 'left_leg_motor_5']
GOBILDA_JOINTS = ['left_arm_motor_0', 'left_arm_motor_1', 'right_arm_motor_0', 'right_arm_motor_1', 'head_motor_0', 'head_motor_1',]
class Transmitter(Thread):
    def __init__(self, ser, dry_run=False, group=None, target=None, name=None):
        super(Transmitter, self).__init__(group=group, target=target, name=name)
        self._name = name
        self._cmd_queue = Queue(1)
        self._stop_event = Event()
        self._dry_run = dry_run
        self._ser = ser
        self._num_tx = 0
        self._num_tx_lock = Lock()

    def start(self, *args, **kwargs):
        with self._ser._motor_lock:
            uart_transact(self._ser, [1600, 1, 15] * 13, CMDS.PID_COEFF, RWS.WRITE)  # push initial PID gains
            uart_transact(self._ser, [1700] * 13, CMDS.MAX_DRIVE, RWS.WRITE)  # push initial maximum drive (out of 4096)
        super().start(*args, **kwargs)

    def stop(self):
        """
        Prevents any more commands from being added to the queue and causes the
        thread to exit once the queue is empty.
        """
        self._stop_event.set()

    def _stopped(self):
        return self._stop_event.is_set()

    def _send_packet_to_mcu(self, goal_motor_angles):
        """Sends bytes to the MCU with the header sequence attached. Angles in degrees."""
        with self._ser._motor_lock:  # NOTE: possibly replace with acquire-release with timeout
            # print('**************', goal_motor_angles)
            jx_goal_angles, gobilda_goal_angles = [
                np.array(
                    [(int(motor['id']), angle)
                        for (motor_name, motor), angle in goal_motor_angles
                        if motor_name in js
                    ]
                ) for js in [JX_JOINTS, GOBILDA_JOINTS]
            ]
            
            flat_jx_goal_angles = [0] * (np.amax(jx_goal_angles[:,0]) + 1)
            for idx, v in jx_goal_angles:
                flat_jx_goal_angles[idx] = int(v * ANGLE2JX_POT_VOLTS)
            jx_servo_util.uart_transact(self._ser, flat_jx_goal_angles, CMDS.POSITION, RWS.WRITE, 0.01)
            
            GOBILDA_IDX_BASE = np.amin(gobilda_goal_angles[:,0])
            flat_gobilda_goal_angles = [0] * (np.amax(gobilda_goal_angles[:,0]) - GOBILDA_IDX_BASE + 1)
            for idx, v in gobilda_goal_angles:
                flat_gobilda_goal_angles[idx - GOBILDA_IDX_BASE] = int(v * ANGLE2GOBILDA_PWM)
            gobilda_servo_util.uart_transact(self._ser, flat_gobilda_goal_angles)
            

    def get_num_tx(self):
        with self._num_tx_lock:
            return self._num_tx

    def send(self, goal_motor_angles):
        """
        Adds a set of goal angles to the command queue.
        """
        if not self._stopped():
            self._cmd_queue.put(goal_motor_angles)

    def run(self):
        """
        Services the command queue; sends packets to the microcontroller.
        """
        log_string("Starting Tx thread ({0})...".format(self._name))
        try:
            while not rp.is_shutdown():
                while not self._cmd_queue.empty():
                    goal_motor_angles = self._cmd_queue.get()
                    # print(goal_motor_angles)
                    self._send_packet_to_mcu(goal_motor_angles)
                    with self._num_tx_lock:
                        self._num_tx = self._num_tx + 1
        except serial.serialutil.SerialException as e:
            log_string("Serial exception in thread {0}".format(self._name))
            print(e)
        log_string("Stopping Tx thread ({0})...".format(self._name))
