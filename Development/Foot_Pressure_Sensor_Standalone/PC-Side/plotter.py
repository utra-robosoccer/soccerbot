'''
Real-time data plotter for a stream of floats from a virtual COM port

Author: Tyler

Resources used:
- http://zetcode.com/gui/pyqt5/
- https://stackoverflow.com/questions/9614947/pyqt4-emiting-signals-in-threads-to-slots-in-main-thread
- http://www.qtcentre.org/threads/4486-QThread-run()-Vs-start()
- https://stackoverflow.com/questions/12432637/pyqt4-set-windows-taskbar-icon
- https://matplotlib.org/gallery/user_interfaces/embedding_in_qt_sgskip.html
'''

import sys
import numpy as np
import serial
import struct

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QDesktopWidget, QSizePolicy, QVBoxLayout
from PyQt5.QtCore import pyqtSignal, QObject, QThread, QDateTime, QTimer

import ctypes
from PyQt5.QtGui import QIcon

from matplotlib.backends.qt_compat import is_pyqt5, QtCore
if is_pyqt5():
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure
    
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
    signal_serial_update = pyqtSignal(np.ndarray)
    
    def __init__(self, comPort):
        super(SerialReader, self).__init__()
        self.comPort = comPort
        self.initReader()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.receiveFromMCU)
        self.timer.start(2)
    
    # def run(self):
    #     self.test()
        
    def test(self):
        while(True):
            v = np.zeros((1, 1))
            v[:] = np.sin(QDateTime.currentMSecsSinceEpoch() / 1000)
            self.signal_serial_update.emit(v)
            QThread.msleep(100)
        
    def initReader(self):
        connected = False
        num_tries = 0
        
        while(not connected):
            try:
                self.ser = serial.Serial(self.comPort, 230400, timeout=100)
                self.ser.reset_output_buffer()
                self.ser.reset_input_buffer()
                
                connected = True
                
                self.signal_serial_status.emit(
                    "SUCCESS: connected to {0}".format(self.comPort)
                )
            except:
                num_tries = num_tries + 1
                
                self.signal_serial_status.emit(
                    "FAIL: Could not connect to {0}. Number of tries: {1}".
                    format(self.comPort, num_tries)
                )
            
            QThread.msleep(100)
        
    def receiveFromMCU(self):
        if(self.ser.in_waiting < 4):
            return
            
        raw = self.ser.read(4)
        v = bytes2fvec(raw)
            
        self.signal_serial_update.emit(v)
        
# In the future, could extend this to accept an argument for the number of
# channels being time multiplexed in the stream
class DataPlotter(QMainWindow):
    def __init__(self, comPort):
        super(DataPlotter, self).__init__()
        self.initUI(comPort)
    
    def initUI(self, comPort):
        # Init the GUI window
        self.resize(1000, 750)
        self.center()
        
        self.setWindowTitle('Robosoccer Data Plotter')
        self.setWindowIcon(QIcon('icon.png'))
        myappid = 'utra.robosoccer.plotter'
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
        
        self.statusBar().showMessage('Ready')
        self.show()
        
        # Init the plotter
        self.mainWindow = QWidget()
        self.setCentralWidget(self.mainWindow)
        layout = QVBoxLayout(self.mainWindow)
        
        canvas = FigureCanvas(Figure(figsize=(5, 3)))
        layout.addWidget(canvas)
        self.addToolBar(QtCore.Qt.BottomToolBarArea,
                        NavigationToolbar(canvas, self))
        self.ax = canvas.figure.subplots()
        
        self.timeIndexes = np.zeros((100,))
        self.hasReceivedData = False
        self.dataBuffer = np.zeros((100,)) # 100 data points to be buffered
        
        # Init the serial data reader
        self.serialThread = SerialReader(comPort)
        self.serialThread.signal_serial_status.connect(
            self.serial_status_signal_callback
        )
        self.serialThread.signal_serial_update.connect(
            self.serial_update_signal_callback
        )
        self.serialThread.start()
        
    def center(self):
        windowGeometry = self.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        windowGeometry.moveCenter(centerPoint)
        self.move(windowGeometry.topLeft())
        
    def serial_status_signal_callback(self, status):
        self.statusBar().showMessage(status)
        
    def serial_update_signal_callback(self, data):
        self.ax.clear()
        
        # Shift data buffers left, add new data to rightmost slot, then redraw
        self.timeIndexes = np.roll(self.timeIndexes, -1)
        if not self.hasReceivedData:
            self.startTime = QDateTime.currentMSecsSinceEpoch()
            self.timeIndexes[-1:] = 0
            self.hasReceivedData = True
        else:
            self.timeIndexes[-1:] = (
                QDateTime.currentMSecsSinceEpoch() - self.startTime
            )
            self.ax.set_xlim(self.timeIndexes[0], self.timeIndexes[-1])
        
        self.dataBuffer = np.roll(self.dataBuffer, -1)
        self.dataBuffer[-1:] = data
        
        self.ax.plot(self.timeIndexes, self.dataBuffer)
        self.ax.figure.canvas.draw()
        
if __name__ == "__main__":
    com = 'COM8'
    app = QApplication(sys.argv)
    dataPlotter = DataPlotter(com)
    sys.exit(app.exec_())
    