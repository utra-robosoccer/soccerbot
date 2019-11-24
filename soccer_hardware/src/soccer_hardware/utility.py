# utility.py

import argparse
import serial.tools.list_ports
import os
import sys
from threading import Lock

from datetime import datetime

has_ros = True
try:
    import rospy
except ImportError:
    has_ros = False

print_lock = Lock()


def logString(userMsg):
    """ Prints the desired string to the shell, precedded by the date and time.
    """
    with print_lock:
        print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)


def list_ports():
    ports = serial.tools.list_ports.comports()
    msg = ""
    if len(ports) == 0:
        msg = "Error: No COM ports have been detected"
    else:
        ports = [port.device for port in ports]
        msg = "Available ports are: " + " ".join(ports)
    return msg


def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))


def parse_args():
    os.chdir(get_script_path())
    logString("Starting PC-side application")

    parser = argparse.ArgumentParser(description='Soccer hardware')
    parser.add_argument(
        '-r',
        '--ros',
        help='Subscribes and publishes to ROS nodes (event-based). This is '
             'the flow used on the actual robot. Default: True if you have '
             'ROS installed, otherwise false',
        default=True
    )
    parser.add_argument(
        '--port',
        help='The serial port used for communication. Default: /dev/ttyUSB0',
        default='/dev/ttyACM0'
    )

    parser.add_argument(
        '--baud',
        help='Serial port baud rate. Default: 230400',
        default=230400
    )

    parser.add_argument(
        '--traj',
        help='The trajectory to be used, if not in ROS mode. Default: '
             'standing.csv',
        default='standing.csv'
    )

    parser.add_argument(
        '--step',
        help='Causes goal angles to be sent when enter is pressed, if not '
             'in ROS mode. Default: False',
        default=False
    )

    parser.add_argument(
        '--use_wait_feedback',
        help='Causes delays between transmissions to adjust on-the-fly to '
             'achieve desired wait times, for non-ROS modes. Default: True',
        default=True
    )

    parser.add_argument(
        '__name',
        nargs='?',
        help='ROS argument'
    )

    parser.add_argument(
        '__log',
        nargs='?',
        help='ROS argument'
    )

    args = vars(parser.parse_args())

    arg_str = ""
    for k in args.keys():
        if k == 'ros' and not has_ros:
            original_arg = args[k]
            args[k] = False

            arg_str = arg_str + k + "=" + str(False)
            if original_arg == True:
                arg_str = arg_str + " (ROS is not installed)"

            arg_str = arg_str + ", "
        else:
            arg_str = arg_str + k + "=" + str(args[k]) + ", "

    logString("Starting script with " + arg_str[:len(arg_str) - 2])

    return args