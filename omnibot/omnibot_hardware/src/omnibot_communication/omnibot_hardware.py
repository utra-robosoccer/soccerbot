#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created June 6 2018

Author: Tyler and Jason
"""

import argparse
import serial
import time
import os
import sys
import numpy as np
import struct

from datetime import datetime
from prettytable import PrettyTable

# SR: needs cmd_vel msg?
try:
    import rospy
    #from geometry_msgs.msg import Vector3
    #from geometry_msgs.msg import Quaternion
    from geometry_msgs.msg import Twist
    from omnibot_msgs.msg import OmnibotGoal
    #from tf.msg import tfMessage
    #from tf.transformations import quaternion_from_euler
except:
    print("No ROS")


def logString(userMsg):
    ''' Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)
    

class omnibot_hardware:
    def __init__(self):
        logString("Starting PC-side application")
        
        parser = argparse.ArgumentParser(description='Soccer hardware')
        parser.add_argument(
            '-r',
            '--ros',
            help='Imports ROS-related dependencies if True or omitted. Default: True',
            default=True
        )
        parser.add_argument(
            '--port',
            help='Specifies the port argument to the serial.Serial constructor. Default: /dev/ttyUSB0',
            default='/dev/ttyACM0'
        )
        
        parser.add_argument(
            '--baud',
            help='Specifies the serial port baud rate. Default: 115200',
            default=115200
        )

        parser.add_argument(
            '--dryrun',
            help='Dryrun, does not need hardware connected'
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
        
        self.isROSmode = args['ros']
        self.port = args['port']
        self.baud = args['baud']
        #self.dryrun = args['dryrun']
        self.dryrun = False
        
        
        logString("Started with ROS = {0}".format(args['ros'] == True))
        
    def connectToEmbedded(self):
        print(self.dryrun.lower())
        if self.dryrun.lower() == 'true': # Needs improvement, but now works with roslaunch file.
            logString("Dryrun invoked, will not connect to real hardware")
        else:
            logString(
                "Attempting connection to embedded systems via port {0} with baud rate {1}".format(
                    self.port,
                    self.baud
                )
            )
        
            try:
                self.ser = serial.Serial(self.port, self.baud,timeout=100)
            except:
                logString("Connection failed. Exiting")
                sys.exit(1)
            
            logString("Connected")

    def fsm(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        w = msg.angular.z

        x_adj = abs(x)
        y_adj = abs(y)

        # hardcoded
        lin_step = 0.2
        lin_max = lin_step * 3 # 2bits, one state is zero vel

        if x_adj > lin_max:
            x_adj = lin_max

        if y_adj > lin_max:
            y_adj = lin_max


        x_str = format(int(round(x_adj / lin_step)) ,'02b')
        y_str = format(int(round(y_adj / lin_step)) ,'02b')

        x_sign = '0' if x > 0 else '1'
        y_sign = '0' if y > 0 else '1'

        w_string = '01' if w > 0 else '11'
        w_string = '00' if w == 0 else w_string
        t = PrettyTable(['Type', 'Value', 'Corresponding Bits'])
        t.add_row(['Linear  X', str(x), x_sign + x_str])
        t.add_row(['Linear  Y', str(y), y_sign + y_str])
        t.add_row(['Angular Z', str(w), w_string])
        t.add_row(['Msg Sent:', '----', w_string + y_sign + y_str + x_sign + x_str])
        print(t)
        if not self.dryrun:
            #return bytes('', int(w_string + y_sign + y_str + x_sign + x_str,2))
            val = int(w_string + y_sign + y_str + x_sign + x_str, 2)
            b = struct.pack('<B', val)
            return b


    def cmd_callback(self, msg):
        if self.dryrun:
            self.fsm(msg)
        else:
            if not self.ser.isOpen():
                logString("Disconnected!!!!!")
                sys.exit(1)
        
            self.ser.write(self.fsm(msg))
    
if __name__ == "__main__":
    sh = omnibot_hardware()
    sh.connectToEmbedded()
    
    if(sh.isROSmode == True):
        rospy.init_node('omnibot_hardware', anonymous=True)
        rospy.Subscriber("/omnibot/cmd_vel", Twist, sh.cmd_callback, queue_size=1)
        rospy.Subscriber("/omnibot/camera_angles", OmnibotGoal, sh.camera_callback, queue_size=1)       
        #rospy.Subscriber("robotGoal", RobotGoal, sh.cmd_callback, queue_size=1)
        rospy.spin() 
    else:
        logString("No ROS mode not supported yet, exiting the script...")
        sys.exit(1)      
