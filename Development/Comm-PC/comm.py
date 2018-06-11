# -*- coding: utf-8 -*-
"""
Created June 6 2018

Author: Tyler
"""

import serial
import serial.tools.list_ports
import time
import os
import numpy as np
import struct
from datetime import datetime
from prettytable import PrettyTable

def rxDecoder(raw, decodeHeader=True):
    ''' Decodes raw bytes received from the microcontroller. As per the agreed
        upon protocol, the first 4 bytes are for a header while the remaining
        80 bytes contain floats for each motor.
    '''
    motors = list()
    imu = list()
    
    if(decodeHeader):
        header = struct.unpack('<L',raw[0:4])[0]
        for i in range(12):
            # Here, we only unpack for 12 motors since that's all we have connected
            # in our current setup
            motors.append(struct.unpack('<f',raw[8 + i * 4:12 + i * 4])[0])
        for i in range(6):
            # Unpack IMU Data
            imu.append(struct.unpack('<f', raw[52 + i * 4: 56 + i * 4])[0])
        return (header, motors, imu)
    else:
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

def sendPacketToMCU(byteStream):
    ''' Sends bytes to the MCU with the header sequence attached.
    '''
    header = struct.pack('L', 0xFFFFFFFF)
    id = struct.pack('I', 0x1234)
    padding = bytes(''.encode())
    footer = struct.pack('L', 0x00000000)
    
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
        t.add_row([str(i + 1), str(vec1[i][0]), str(vec2[i][0])])
    
    print(t)

def printAsIMUData(vec1):
    ''' Prints out a numpy vector interpreted as data from the IMU, in the
        order X-gyro, Y-gyro, Z-gyro, X-accel, Y-accel, Z-accel.
    '''
    
    t = PrettyTable(['', 'Gyro (deg/s)', 'Accel (m/s^2)'])
    
    t.add_row(["X", str(vec1[0][0]), str(vec1[3][0])])
    t.add_row(["Y", str(vec1[1][0]), str(vec1[4][0])])
    t.add_row(["Z", str(vec1[2][0]), str(vec1[5][0])])
    
    print(t)
    
    
# TODO: Test (this function is not safe to use yet)
def receivePacketFromMCU():
    ''' Receives 80 bytes of the MCU provided that there is a valid 4-byte 
        header attached to the front. Returns the list of data interpreted as
        32-bit floats.
    '''
    BUFF_SIZE = 23 # Some factor of 92
    totalBytesRead = 0
    startSeqCount = 0
    buff = bytes(''.encode())
    
    while(True):
        while(ser.in_waiting < BUFF_SIZE):
            time.sleep(0.001)
        rawData = ser.read(BUFF_SIZE)
        
        for i in range(BUFF_SIZE):
            if(startSeqCount == 4):
                buff = buff + rawData[i * 4:4 + i * 4]
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

#os.chdir('D:/users/tyler/documents/stm/embedded/soccer-embedded/development/comm-pc')

if __name__ == "__main__":
    os.chdir('D:/users/tyler/documents/stm/embedded/soccer-embedded/development/comm-pc')
    logString("Starting PC-side application")
    
    walking = np.loadtxt(open("walking.csv", "rb"), delimiter=",", skiprows=0)
    
    with serial.Serial('COM7',230400,timeout=100) as ser:
        logString("Opened port " + ser.name)
        
        numTransfers = 0
        while(ser.isOpen()):
            for i in range(walking.shape[1]):
                angles = walking[:, i:i+1]
                t1 = datetime.now() # Start tracking time
                sendPacketToMCU(vec2bytes(angles))
                
                numTransfers = numTransfers + 1
                    
                while(ser.in_waiting < 92):
                    #pass
                    time.sleep(0.001)
                rawData = ser.read(92)
                t2 = datetime.now() # Finish tracking time
                
                (header, recvAngles, recvIMUData) = rxDecoder(rawData)
                
                    # Forward to control application
                if(numTransfers % 50 == 0):
                    print('\n')
                    logString("Header matches sequence: " + 
                                str(header == 0xFFFFFFFF)
                        )
                    printAsAngles(angles, 
                                np.array(recvAngles).reshape(angles.shape)
                        )
                    printAsIMUData(np.array(recvIMUData).reshape((6, 1)))
                    print("Time delta: " + str((t2 - t1)))
                        
                if(header == 0xFFFFFFFF):
                    continue
                
                # (recvAngles, recvIMUData) = rxDecoder(receivePacketFromMCU())
                # 
                # if(numTransfers % 50 == 0):
                #     logString("Received valid data")
                #     printAsAngles(angles, 
                #                 np.array(recvAngles).reshape(angles.shape)
                #         )