####################################################################################################
# Python script to read & process data from a BLE Mesh network of Thingy52s and Particle Argons. 
# Various other modules used in this project are also imported and used here.
#
# Author: Hugh Robertson & Oliver Roman
# Date: 13/05/2022
####################################################################################################
import time
import sys
import serial
import serial.tools.list_ports as list_ports
from PyQt5.QtCore import Qt, pyqtSignal, QThread

#Thread that waits for data to appear on serial port and send it to GUI
class SerialPort(QThread):

    newData = pyqtSignal(str)

    def __init__(self, port=None):
        # Initialise the thread

        super(SerialPort, self).__init__()
        self.port = port
        self.ser = serial.Serial()
        try:
            self.ser = serial.Serial(self.port)
        except serial.SerialException:
            print("Couldn't open serial port")

    def run(self):
        # Continuously try and read from serial port and send data to to GUI
        while True:
            
            try:
                line = self.ser.readline().decode()
           
                self.newData.emit(line)
            except serial.SerialException:
                print("Couldn't read serial")
                self.ser.close()
                self.port = None
                break
            time.sleep(0.01)