#!/usr/bin/env python3

import os
import rospy
import serial.tools.list_ports
from communication import Communication
from utility import *
import time
import glob
import subprocess

class SerialWrapper:
    _ser_wrap_pre = ""
    _ser_wrap_ser = None
    _ser_wrap_args = []
    _ser_wrap_kwargs = {}
    lock = Lock()
    MAX_REOPEN_TRIES = 5
    SER_RETRY_DELAY = 0.1

    def __init__(self, pre, *args, **kwargs):
        self._ser_wrap_pre = pre
        self._ser_wrap_args = args
        self._ser_wrap_kwargs = kwargs
        self.reopen()

    def reopen(self):
        # NOTE: UPDATE PATH IF THIS EVER LEAVES `soccer_hardware`
        # Set all relevant UART ports to low-latency
        # TODO: consider enabling sudo for `setserial` in general and iterating over all `pre*`
        subprocess.run(['%s/usb.sh' % os.path.dirname(os.path.realpath(__file__))]) # NOTE: this will sometimes say "/dev/tty... No such file or directory" as the OS scrambles to reopen the port, so don't be confused from the output
        with self.lock:
            if self._ser_wrap_ser is not None:
                self._ser_wrap_ser.close()

            e = serial.serialutil.SerialException('No working ports with prefix `%s`' % self._ser_wrap_pre)
            for _ in range(self.MAX_REOPEN_TRIES):
                ser = None
                for p in sorted(glob.glob(self._ser_wrap_pre)):
                    try:
                        ser = serial.Serial(p, *self._ser_wrap_args, **self._ser_wrap_kwargs)
                        self._ser_wrap_ser = ser
                        return
                    except PermissionError as _e:
                        # It's possible for this to be a transient error, seen during testing. I think it also flows into a serial exception on attempt of an actual read/write
                        e = _e
                        pass
                    except OSError as _e:
                        e = _e
                        if e.errno in [5, 19]:
                            # 5: Input/output error, caused by misc serial device problems
                            # 19: No such device, probably caused by race condition on the device waking up
                            # 
                            pass
                        else:
                            raise e
                    except serial.serialutil.SerialException as _e:
                        e = _e
                        pass

                if ser is None:
                    time.sleep(self.SER_RETRY_DELAY)
            raise e

    def __getattr__(self, key):
        # note: falls through to this if the access fails in self object, so no need to check
        try:
            # Note: often the lock is held to wrap a transaction rather than just a single send-receive, so don't be opinionated and leave the lock be there for user purposes
            # if hasattr(self.ser, key) and callable(getattr(self.ser, key)):
            #     def wrap(*args, **kwargs):
            #         # enforce single-thread lock on all method calls
            #         with self.lock:
            #             return getattr(self.ser, key)(*args, **kwargs)
            #     return wrap
            # else:
            #     return getattr(self.ser, key)
            return getattr(self._ser_wrap_ser, key)
        except RecursionError:
            raise ValueError('self.ser is not defined, leading to inf recursion in __getattr__')

    def __setattr__(self, key, value):
        # a little bit risky, mixing all parameters of this wrapper and the serial object together. the child object takes precedence. it's a hack to be sure, but one that's easy to use down the road as a plain serial object with lock and restart functionality
        if not hasattr(self._ser_wrap_ser, key):
            super(SerialWrapper, self).__setattr__(key, value) # thanks to https://stackoverflow.com/a/17020163
        else:
            setattr(self._ser_wrap_ser, key, value)

def open_ser():
    imu_port = rospy.get_param("~imu_port", "/dev/ttyACM*")
    servo_port = rospy.get_param("~servo_port", "/dev/ttyUSB*")
    imu_baud = rospy.get_param("~imu_baud_rate", 230400)
    servo_baud = rospy.get_param("~servo_baud_rate", 1000000)

    log_string("Connecting To Embedded system")
    log_string("\tServo Port: " + servo_port)
    log_string("\tServo Baud rate: " + str(servo_baud))
    log_string("\tIMU Port: " + imu_port)
    log_string("\tIMU Baud rate: " + str(imu_baud))

    imu_ser = None
    servo_ser = None
    try:
        imu_ser = SerialWrapper(imu_port, imu_baud)
        servo_ser = SerialWrapper(servo_port, servo_baud)
    except serial.serialutil.SerialException:
        rospy.logerr(
            f"Opening either the IMU port at {imu_port} or the servo port at {servo_port} failed"
        )

    return (servo_ser, imu_ser)

if __name__ == "__main__":
    rospy.init_node("soccer_hardware")

    attempt = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        sers = tuple()
        try:
            sers = open_ser()
            if not all(sers):
                raise serial.serialutil.SerialException()

            comm = Communication(*sers)
            comm.run()
            break

        except serial.serialutil.SerialException as e:
            for ser in sers:
                ser.close()
            log_string("Serial exception. " + str(e) + " Retrying...(attempt {0})".format(attempt))
            attempt += 1
            rate.sleep()
