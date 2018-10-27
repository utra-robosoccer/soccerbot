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

try:
    import rospy
    from soccer_msgs.msg import RobotGoal
    from soccer_msgs.msg import RobotState
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Vector3
    from geometry_msgs.msg import Quaternion
    from tf.msg import tfMessage
    from tf.transformations import quaternion_from_euler
except:
    print("No ROS")

def rxDecoder(raw, decodeHeader=True):
    ''' Decodes raw bytes received from the microcontroller. As per the agreed
        upon protocol, the first 4 bytes are for a header while the remaining
        80 bytes contain floats for each motor.
    '''
    motors = list()
    imu = list()

    for i in range(12):
        # Here, we only unpack for 12 motors since that's all we have connected
        # in our current setup
        motors.append(struct.unpack('<f',raw[4 + i * 4:8 + i * 4])[0])
    for i in range(6):
        # Unpack IMU Data
        imu.append(struct.unpack('<f', raw[52 + i * 4: 56 + i * 4])[0])
    return (motors, imu)
    
def logString(userMsg):
    ''' Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def sendPacketToMCU(ser, byteStream):
    ''' Sends bytes to the MCU with the header sequence attached.
    '''
    header = struct.pack('<L', 0xFFFFFFFF)
    id = struct.pack('<I', 0x1234)
    padding = bytes(''.encode())
    footer = struct.pack('<L', 0x00000000)
    
    numBytes = len(byteStream)
    if(numBytes < 80):
        padding = struct.pack('<B', 0x00) * (80 - numBytes)
        
    ser.write(header + id + byteStream + padding + footer)
    
def vec2bytes(vec):
    ''' Transforms a numpy vector to a byte array, with entries interpreted as
        32-bit floats.
    '''
    byteArr = bytes(''.encode())
    for element in vec:
        byteArr = byteArr + struct.pack('f', element)
    return byteArr

def printAsAngles(vec1, vec2):
    ''' Prints out 2 numpy vectors side-by-side, where the first vector entry
        is interpreted as belonging to motor 1, the seconds to motor 2, etc.
    '''
    assert vec1.shape[0] == vec2.shape[0]
    t = PrettyTable(['Motor Number', 'Sent', 'Received'])
    
    for i in range(vec1.shape[0]):
        t.add_row([str(i + 1), round(vec1[i][0], 4), round(vec2[i][0], 2)])
    
    print(t)

def printAsIMUData(vec1):
    ''' Prints out a numpy vector interpreted as data from the IMU, in the
        order X-gyro, Y-gyro, Z-gyro, X-accel, Y-accel, Z-accel.
    '''
    
    t = PrettyTable(['', 'Gyro (deg/s)', 'Accel (m/s^2)'])
    
    t.add_row(["X", round(vec1[0][0], 2), round(vec1[3][0], 2)])
    t.add_row(["Y", round(vec1[1][0], 2), round(vec1[4][0], 2)])
    t.add_row(["Z", round(vec1[2][0], 2), round(vec1[5][0], 2)])
    
    print(t)
    
def receivePacketFromMCU(ser):
    ''' Receives 80 bytes of the MCU provided that there is a valid 4-byte 
        header attached to the front. Returns the list of data interpreted as
        32-bit floats.
    '''
    
    BUFF_SIZE = 4
    totalBytesRead = 0
    startSeqCount = 0
    buff = bytes(''.encode())
    
    while(True):
        while(ser.in_waiting < BUFF_SIZE):
            time.sleep(0.001)
        rawData = ser.read(BUFF_SIZE)
        
        for i in range(BUFF_SIZE):
            if(startSeqCount == 4):
                buff = buff + rawData[i:i+1]
                totalBytesRead = totalBytesRead + 1
                if(totalBytesRead == 84):
                    break
            else:
                if(struct.unpack('<B', rawData[i:i+1])[0] == 0xFF):
                    startSeqCount = startSeqCount + 1
                else:
                    startSeqCount = 0
        if(totalBytesRead == 84):
            break
    return buff
    
def receiveWithChecks(ser, isROSmode, numTransfers, angles):
    ''' Receives a packet from the MCU and performs basic checks on the packet
        for data integrity. Also decodes the packet and prints a data readout 
        every so often.
    '''
    
    (recvAngles, recvIMUData) = rxDecoder(receivePacketFromMCU(ser),
                                            decodeHeader=False)

    imuArray = np.array(recvIMUData).reshape((6, 1))
    
    if(isROSmode == True):
        ''' IMU Feedback '''
        imu = Imu()
        vec1 = Vector3(-imuArray[2][0], imuArray[1][0], imuArray[0][0])
        imu.angular_velocity = vec1
        vec2 = Vector3(imuArray[5][0], imuArray[4][0], imuArray[3][0])
        imu.linear_acceleration = vec2
        pub.publish(imu)
        
        ''' Motor Feedback '''
        robotState = RobotState()
        for i in range(0,12):
			recvAngles[i] = (recvAngles[i] - 150) / 180 * 3.1415926
			robotState.joint_angles[i] = recvAngles[i]
		
        recvAngles[3] = -recvAngles[3]
        recvAngles[4] = -recvAngles[4]
        recvAngles[7] = -recvAngles[7]
        recvAngles[9] = -recvAngles[9]
        recvAngles[10] = -recvAngles[10]

        pub2.publish(robotState)
        
    
    if(numTransfers % 50 == 0):
        
        print('\n')
        logString("Received valid data")
        printAsAngles(angles[0:12], np.array(recvAngles).reshape((12, 1)))
        printAsIMUData(np.array(recvIMUData).reshape((6, 1)))

def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))
    
class soccer_hardware:
    def __init__(self):
        os.chdir(get_script_path())
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
            default='/dev/ttyUSB0'
        )
        
        parser.add_argument(
            '--baud',
            help='Specifies the serial port baud rate. Default: 230400',
            default=230400
        )
        
        parser.add_argument(
            '--traj',
            help='Specifies the trajectory to use by default. Default: standing.csv',
            default='standing.csv'
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
        self.traj = args['traj']
        
        
        logString("Started with ROS = {0}".format(args['ros'] == True))
        
        logString("Attempting to open trajectory file \'{0}\'".format(self.traj))
        
        trajectories_dir = os.path.join("trajectories", self.traj)
        
        self.trajectory = np.loadtxt(
            open(trajectories_dir, "rb"),
            delimiter=",",
            skiprows=0
        )
        
        logString("Initialized soccer hardware with trajectory {0}".format(
                self.traj
            )
        )
                
    def connectToEmbedded(self):
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
        
    def loopTrajectory(self):
        numTransfers = 0
        try:
            while(self.ser.isOpen()):
                for i in range(self.trajectory.shape[1]):
                    angles = self.trajectory[:, i:i+1]
                    
                    sendPacketToMCU(self.ser, vec2bytes(angles))
                    numTransfers = numTransfers + 1
                    receiveWithChecks(self.ser, self.isROSmode, numTransfers, angles)
                    
        except serial.serialutil.SerialException as e:
            logString("Serial exception {0}. Exiting".format(e))
            sys.exit(1)
        except Exception as e:
            logString(str(e))
            sys.exit(1)

    def trajectory_callback(self, robotGoal):
        numTransfers = 0
        if not self.ser.isOpen():
            return
        angles = np.zeros((18,1))
        angles[0:18,0] = robotGoal.trajectories[0:18]
        
        angles[1,0] =-(angles[1,0])
        angles[5,0] =-(angles[5,0])
        angles[6,0] =-(angles[6,0])

        angles[0:6,0] = np.flipud(angles[0:6,0])
        
        sendPacketToMCU(self.ser, vec2bytes(angles))
        receiveWithChecks(self.ser, self.isROSmode, numTransfers, angles)
    
if __name__ == "__main__":
    sh = soccer_hardware()
    sh.connectToEmbedded()
    
    if(sh.isROSmode == True):
        rospy.init_node('soccer_hardware', anonymous=True)
        rospy.Subscriber("robotGoal", RobotGoal, sh.trajectory_callback, queue_size=1)
        pub = rospy.Publisher('soccerbot/imu', Imu, queue_size=1)
        pub2 = rospy.Publisher('soccerbot/robotState', RobotState, queue_size=1)
        rospy.spin() 
    else:
        sh.loopTrajectory()
    
    sys.exit(0)      
