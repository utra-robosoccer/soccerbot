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

def rxDecoder(raw):
    ''' Decodes raw bytes received from the microcontroller. As per the agreed
        upon protocol, the first 4 bytes are for a header while the remaining
        80 bytes contain floats for each motor.
    '''
    motors = list()
    header = struct.unpack('<L',raw[0:4])[0]
    for i in range(12):
        # Here, we only unpack for 12 motors since that's all we have connected
        # in our current setup
        motors.append(struct.unpack('<f',raw[4 + i * 4:8 + i * 4])[0])
    return (header, motors)
    

def logString(userMsg):
    ''' Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def sendStrToMCU(msg):
    ''' Sends the desired string to the microcontroller.
    '''
    ser.write(bytes(msg.encode()))
    
def vec2bytes(vec):
    ''' Transforms a numpy vector to a byte array, with entries interpreted as
        32-bit floats.
    '''
    byteArr=bytes(''.encode())
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

#os.chdir('D:/users/tyler/documents/stm/embedded/soccer-embedded/development/comm-pc')

if __name__ == "__main__":
    logString("Starting PC-side application")
    
    walking = np.loadtxt(open("walking.csv", "rb"), delimiter=",", skiprows=0)
    
    with serial.Serial('COM3',230400,timeout=100) as ser:
        logString("Opened port " + ser.name)
        
        numTransfers = 0
        while(ser.isOpen()):
            for i in range(walking.shape[1]):
                angles = walking[:, i:i+1]
                ser.write(struct.pack('L', 0xFFFFFFFF))
                ser.write(vec2bytes(angles))
                
                numTransfers = numTransfers + 1
                    
                while(ser.in_waiting < 84):
                    time.sleep(0.001)
                rawData = ser.read(84)
                
                (header, recvAngles) = rxDecoder(rawData)
                
                if(numTransfers % 50 == 0):
                    logString("Header matches sequence: " + 
                                str(header == 0xFFFFFFFF)
                        )
                    printAsAngles(angles, 
                                np.array(recvAngles).reshape(angles.shape)
                        )
                    
                if(header == 0xFFFFFFFF):
                    continue
                    # Forward to control application