'''
Real-time data plotter for a stream of floats from a virtual COM port

Author: Tyler
'''

import sys
import numpy as np
import serial
import struct
import ctypes
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QDesktopWidget
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSignal, QObject, QThread
    
def bytes2fvec(byteArr):
    ''' Transforms a byte array, with entries interpreted as 32-bit floats, to
        a Numpy vector.
    '''
    
    assert len(byteArr) % 4 == 0 # Make sure we are dealing with floats
    
    numFloats = len(byteArr) // 4
    
    vec = np.zeros((numFloats, 1))
    
    for i in range(numFloats):
        vec[i] = struct.unpack('<f', byteArr[i * 4:(i + 1) * 4])[0]
        
    return vec
    
class SerialReader(QThread):
    """ Class that reads data from a serial port """
    signal_serial_status = pyqtSignal(str)
    
    def __init__(self, com_port):
        super(SerialReader, self).__init__()
        self.com_port = com_port
    
    def run(self):
        self.initReader()
        #receiveFromMCU()
        
    def initReader(self):
        connected = False
        num_tries = 0
        while(not connected):
            try:
                ser = serial.Serial(self.com_port, 230400, timeout=100)
                self.ser.reset_output_buffer()
                self.ser.reset_input_buffer()
                
                connected = True
                
                self.signal_serial_status.emit(self.signal_serial_status, 
                        "SUCCESS: connected to {0}".format(self.com_port))
            except:
                num_tries = num_tries + 1
                
                self.signal_serial_status.emit(
                    "FAIL: Could not connect to {0}. Number of tries: {1}".
                    format(self.com_port, num_tries)
                )
            
            QThread.msleep(100)
        
    def receiveFromMCU():
        # TODO
        while(ser.in_waiting < 4):
            QThread.msleep(1)
    
        raw = ser.read(4)
        v = bytes2fvec(raw)
        
        
# In the future, could extend this to accept an argument for the number of
# channels being multiplexed in the stream
class DataPlotter(QMainWindow):
    def __init__(self, com_port):
        super(DataPlotter, self).__init__()
        self.initUI(com_port)
    
    def initUI(self, com_port):
        # Init the GUI window
        self.resize(1000, 750)
        self.center()
        
        self.setWindowTitle('Robosoccer Data Plotter')
        self.setWindowIcon(QIcon('icon.png'))
        myappid = 'utra.robosoccer.plotter'
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
        
        self.statusBar().showMessage('Ready')
        self.show()
        
        # Init the serial data reader
        self.serialThread = SerialReader(com_port)
        self.serialThread.signal_serial_status.connect(self.serial_status_signal_callback)
        self.serialThread.start()
        
    def center(self):
        windowGeometry = self.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        windowGeometry.moveCenter(centerPoint)
        self.move(windowGeometry.topLeft())
        
    def serial_status_signal_callback(self, status):
        self.statusBar().showMessage(status)
        
        
if __name__ == "__main__":
    com = 'COM7'
    app = QApplication(sys.argv)
    dataPlotter = DataPlotter(com)
    sys.exit(app.exec_())
    