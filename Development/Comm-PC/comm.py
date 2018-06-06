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

def rxDecoder(raw_bytes):
    motors = list()
    header=struct.unpack('<H',raw_bytes[0:4])[0]
    for i in range(12):
        motors.append(struct.unpack('<f',raw_bytes[4 + i * 4:8 + i * 4])[0])
    return (header, motors)
    
def logString(userMsg):
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def sendToMCU(msg):
    ser.write(bytes(msg.encode()))

if __name__ == "__main__":
    logString("Starting PC-side application")
    
    walking= np.loadtxt(open("walking.csv", "rb"), delimiter=",", skiprows=1)
    
    with serial.Serial('COM3',230400,timeout=100) as ser:
        logString("Opened port " + ser.name)
        
        