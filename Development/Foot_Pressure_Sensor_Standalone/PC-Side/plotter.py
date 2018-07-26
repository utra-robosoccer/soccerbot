'''
Real-time data plotter for 1 float via virtual COM port
'''

import sys
import numpy as np
import serial
import struct
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
    
class SerialReader(QObject):
    """ Class that reads data from a serial port """
    
    def __init__(self):
        super().__init__()
        self.initReader()
    
    def initReader(self):
        # In the future, emit a signal here that updates the status bar
        try:
            ser = serial.Serial('COM7',230400,timeout=100)
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
        except:
            print("Serial port connection failure")

    def receiveFromMCU():
        # TODO
        while(ser.in_waiting < 4):
            time.sleep(0.001)
    
        raw = ser.read(4)
        v = bytes2fvec(raw)
        
class DataPlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
    
    def initUI(self):
        # Init the GUI window
        self.resize(1000, 750)
        self.center()
        self.setWindowTitle('Data Plotter')
        self.setWindowIcon(QIcon('icon.png'))
        self.statusBar().showMessage('Ready')
        self.show()
        
        # Init the serial data reader
        self.serialReaderThread = QThread()
        self.serialReader = SerialReader()
        self.serialReader.moveToThread(self.serialReaderThread)
        
    def center(self):
        windowGeometry = self.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        windowGeometry.moveCenter(centerPoint)
        self.move(windowGeometry.topLeft())
        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    dataPlotter = DataPlotter()
    sys.exit(app.exec_())
    
    # with serial.Serial('COM7',230400,timeout=100) as ser: