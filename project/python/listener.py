####################################################################################################
# Python script to read & process data from a BLE Mesh network of Thingy52s and Particle Argons. 
# Various other modules used in this project are also imported and used here. The script also pushes
# the data it gets from serial to the web dashboard (hosted on influxdb)
#
# Author: Hugh Robertson & Oliver Roman
# Date: 13/05/2022
####################################################################################################
import sys
import serial
import serial.tools.list_ports as list_ports
from PyQt5.QtCore import Qt, pyqtSignal, QThread
import json
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

token = os.environ.get("INFLUXDB_TOKEN")
org = "o.roman@uqconnect.edu.au"
url = "https://us-east-1-1.aws.cloud2.influxdata.com"
bucket = "weatherData"
client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)

write_api = client.write_api(write_options=SYNCHRONOUS)
sensor_fields = ["Temperature", "Humidity", "Pressure", "VOC", "CO2", "PM10"]
data = {}


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
                                data = json.loads(line)
                                # Data is sent off to dash board as soon as its read data should 
                                # come every 5 minutes by default but not necessarily
                                point = (Point("weatherData").field(sensor_fields[0]=data[sensor_fields[0]],\
                                         sensor_fields[1]=data[sensor_fields[1]],\
                                         sensor_fields[2]=data[sensor_fields[2]],\
                                         sensor_fields[3]=data[sensor_fields[3]],\
                                         sensor_fields[4]=data[sensor_fields[4]],\
                                         sensor_fields[5]=data[sensor_fields[5]]))
                                write_api.write(bucket=bucket, org="o.roman@uqconnect.edu.au",\
                                                record=point)
                                self.newData.emit(line)
                        except serial.SerialException:
                                print("Couldn't read serial")
                                self.ser.close()
                                self.port = None
                                break
                        time.sleep(0.01)


class GUI(QMainWindow):
        # Main GUI class
        def __init__(self, parent=None):
                
                super(GUI, self).__init__(parent)
                self.setWindowTitle("GUI")
                self.serialPort = SerialPort('/dev/ttyACM1')
                self.serialPort.start()
                self.serialPort.newData.connect(self.receiveLine)


        def setupUi(self, MainWindow): 
                
                MainWindow.setObjectName("MainWindow")
                MainWindow.resize(1000,1000)
                self.centralwidget = QtWidgets.QWidget(MainWindow)
                self.centralwidget.setObjectName("centralwidget")
                self.graphicsView = PlotWidget(self.centralwidget)
                self.graphicsView.setObjectName("graphicsView")
                MainWindow.setCentralWidget(self.graphicsView)
                loop = asyncio.get_event_loop()


        def gui_update(self, result):

                x_data = [1]
                y_data = [random.randrange(10)]

                scatter = pg.ScatterPlotItem(x_data, y_data, size=10,\
                                             brush=[pg.mkBrush(c) for c in "r"])
                self.graphicsView.clear()
                self.graphicsView.addItem(scatter)
                self.graphicsView.setXRange(0,18, padding=0)
                self.graphicsView.setYRange(0,12, padding=0)


        def receiveLine(self, line):
                self.graphicsView.clear()
                self.graphicsView.setXRange(0,400, padding=0)
                self.graphicsView.setYRange(0,400, padding=0)
                self.graphicsView.showGrid(x=True,y=True)
                self.graphicsView.setTitle("Mobile Node Position", color="b", size="30pt")
                styles = {"color": "#f00", "font-size": "20px"}
                self.graphicsView.setLabel("left", "y coordinate (cm)", **styles)
                self.graphicsView.setLabel("bottom", "x coordinate (cm)", **styles)
                self.graphicsView.addLegend()
                pen = pg.mkPen(color=(255, 0, 0))
                self.graphicsView.plot(x_data, y_data, name="RSSI", pen=pen, symbol='+',\
                                       symbolSize=30, symbolBrush=('b'))
                self.graphicsView.plot(x_dataKal, y_dataKal, name="Kalman", pen=pen, symbol='+',\
                                       symbolSize=30, symbolBrush=('r'))


if __name__ == "__main__":
        
        app = QApplication(sys.argv)
        loop = QEventLoop(app)
        MainWindow = QtWidgets.QMainWindow()
        gui = GUI()
        gui.setupUi(MainWindow)
        MainWindow.show()
        with loop: 
                loop.run_forever()
        sys.exit(app.exec_())