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
import matplotlib.pyplot as plt

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
            imu.append(struct.unpack('<f', raw[56 + i * 4: 60 + i * 4])[0])
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
    
def receivePacketFromMCU():
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

def receiveWithChecks(file):
    ''' Receives a packet from the MCU and performs basic checks on the packet
        for data integrity. Also decodes the packet and prints a data readout 
        every so often.
    '''
    (recvAngles, recvIMUData) = rxDecoder(receivePacketFromMCU(),
                                            decodeHeader=False)
                                            
    file.write(datetime.now().strftime('%H.%M.%S.%f') + " ")
    for item in recvIMUData:
        file.write(str(item) + "\t")
    file.write("\n")
    
    if(numTransfers % 50 == 0):
        print('\n')
        logString("Received valid data")
        printAsAngles(angles[0:12], 
                    np.array(recvAngles).reshape((12, 1))
            )
        printAsIMUData(np.array(recvIMUData).reshape((6, 1)))
        
def receiveWithoutChecks():
    ''' Receives a packet from the MCU by completely trusting the data integrity
        and doing no checks whatsoever. Also decodes the packet and prints
        a data readout every so often.
    '''
    while(ser.in_waiting < 92):
        time.sleep(0.001)
    rawData = ser.read(92)
                    
    (header, recvAngles, recvIMUData) = rxDecoder(rawData)

    
    if(numTransfers % 50 == 0):
        print('\n')
        logString("Header matches sequence: " + 
                    str(header == 0xFFFFFFFF)
            )
        printAsAngles(angles[0:12], 
                    np.array(recvAngles).reshape((12,1))
            )
        printAsIMUData(np.array(recvIMUData).reshape((6, 1)))
           
    # Forward to control application
    # if(header == 0xFFFFFFFF):
    #     # TODO

if __name__ == "__main__":
    #os.chdir('/home/shahryar/soccer-embedded/Development/Comm-PC')
    os.chdir('D:/users/tyler/documents/stm/embedded/soccer-embedded/development/comm-pc')
    logString("Starting PC-side application")
    
    trajectory = np.loadtxt(open("walking.csv", "rb"), delimiter=",", skiprows=0)
    
    data_root = 'IMU_data\\' + time.strftime('%Y_%m_%d')+'\\'
    if not os.path.exists(data_root):
        os.makedirs(data_root)
        
    with open(data_root + time.strftime('%Y_%m_%d_%H_%M_%S') + ".txt", 'w') as f:
        with serial.Serial('COM7',230400,timeout=100) as ser:
            logString("Opened port " + ser.name)
            
            numTransfers = 0
            while(ser.isOpen()):
                for i in range(trajectory.shape[1]):
                    #dummy=input('') # Uncomment this if you want to step through the trajectories via user input
                    #angles = trajectory[:, i:i+1]
                    angles = np.zeros((18, 1))
                    sendPacketToMCU(vec2bytes(angles))
                    
                    numTransfers = numTransfers + 1
                    
                    receiveWithChecks(f)
                    
                    
def analyze():
    import serial
    import serial.tools.list_ports
    import time
    import os
    import numpy as np
    import struct
    from datetime import datetime
    from prettytable import PrettyTable
    import matplotlib.pyplot as plt
    
    root1 = 'D:/Users/Tyler/Documents/STM/embedded/soccer-embedded/Development/Comm-PC/IMU_data/2018_07_21_16_03_15'
    root2 = 'D:/Users/Tyler/Documents/STM/embedded/soccer-embedded/Development/Comm-PC/IMU_data/2018_07_21_16_46_49'
    root3 = 'D:/Users/Tyler/Documents/STM/embedded/soccer-embedded/Development/Comm-PC/IMU_data/2018_07_21_16_49_14'
    root4 = 'D:/Users/Tyler/Documents/STM/embedded/soccer-embedded/Development/Comm-PC/IMU_data/2018_07_21_16_54_39'
    root5 = 'D:/Users/Tyler/Documents/STM/embedded/soccer-embedded/Development/Comm-PC/IMU_data/2018_07_21_16_59_20'
    root6 = 'D:/Users/Tyler/Documents/STM/embedded/soccer-embedded/Development/Comm-PC/IMU_data/2018_07_21'
    root = root6
    os.chdir(root)
    
    files = os.listdir(os.getcwd())
    files = [file for file in files if file.endswith('.txt')]
    filename = files[1]
    data = np.loadtxt(fname=filename, usecols=(1,2,3,4,5,6))
    time = np.arange(0, data.shape[0])

    plt.figure(figsize=(70, 10))
    plt.title('Data for {0}'.format(filename))
    plt.xlabel('Time (s)')
    plt.plot(time, data[:,0], color='red', label='Vx')
    plt.plot(time, data[:,1], color='green', label='Vy')
    plt.plot(time, data[:,2], color='blue', label='Vz')
    plt.plot(time, data[:,3], color='purple', label='Ax')
    plt.plot(time, data[:,4], color='brown', label='Ay')
    plt.plot(time, data[:,5], color='orange', label='Az')
    plt.legend(loc='best')
    plt.savefig(root + '/' + filename + '.png')
    plt.close()
    