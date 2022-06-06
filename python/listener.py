####################################################################################################
# Python script to read & process data from a BLE Mesh network of Thingy52s and Particle Argons. 
# Various other modules used in this project are also imported and used here. The script also pushes
# the data it gets from serial to the web dashboard (hosted on influxdb)
#
# Author: Hugh Robertson & Oliver Roman
# Date: 13/05/2022
####################################################################################################
import sys, serial, os, time, json
import serial.tools.list_ports as list_ports
from PyQt5.QtCore import Qt, pyqtSignal, QThread
from datetime import datetime as dt
#import influxdb_client
#from influxdb_client import InfluxDBClient, Point, WritePrecision
#from influxdb_client.client.write_api import SYNCHRONOUS

#token = os.environ.get("INFLUXDB_TOKEN")
#print(token)
#org = "o.roman@uqconnect.edu.au"
#url = "https://us-east-1-1.aws.cloud2.influxdata.com"
#bucket = "Weather Data"
#client = InfluxDBClient(url=url, token=token, org=org)
#write_api = client.write_api(write_options=SYNCHRONOUS)

# Format of data will come in as: {"UUID":X, "Time":X, "Reading":{ "Temperature":X, etc}}
data = {}

import serial
import threading
import re
import argparse
from datetime import datetime as dt
import sys, serial, os, time, json

# Format of data will come in as: {"UUID":X, "Time":X, "Reading":{ "Temperature":X, etc}}
data = {}

# UUID to node mapping, any new nodes will need to be hardcode added here
nodes = { 
         "3e2f23dd6fa1b0a00000000000000000": "A", 
         "4450023f5fb84eae0000000000000000": "B", 
         "36e2175949d1d4850000000000000000": "C",
         "d684939c4e9abd390000000000000000": "D"
        }


readings = {
        "1" : "Temperature",
        "2" : "Humidity",
        "3" : "Pressure",
        "4" : "VOC",
        "5" : "CO2",
        "6" : "PM10"
}

# 7-bit C1 ANSI sequences
ansi_escape = re.compile(r'''
    \x1B  # ESC
    (?:   # 7-bit C1 Fe (except CSI)
        [@-Z\\-_]
    |     # or [ for CSI, followed by a control sequence
        \[
        [0-?]*  # Parameter bytes
        [ -/]*  # Intermediate bytes
        [@-~]   # Final byte
    )
''', re.VERBOSE)

json_pattern = '\{*\}'
prog = re.compile(json_pattern)

def escape_ansi(line):
    ansi_escape =re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')
    return ansi_escape.sub('', line)

ser = serial.Serial("/dev/ttyACM0")

def read_data(): 
    while True: 
        line = ser.readline().decode('utf-8').replace("base:~$", "").strip('\n\r')

        result = ansi_escape.sub('', line)

        try: 
            data = json.loads(result)

            reading = data["reading"]
            
            device_id = list(reading.keys())[0]

            device_name = readings[device_id]

            measurement = reading[device_id]
            

            reading2 = {device_name: measurement}

            uuid = data["UUID"]


            point_data = {"measurement": nodes[uuid],
                         "time": dt.utcnow(),
                        "fields": reading2     
                        }

            print(point_data)
        except Exception as e: 
            print("Something went wrong", e)

def main(args): 
    

    p1 = threading.Thread(target=read_data, args=())
    p1.start()

    while True: 
        command = input("$:")
        if command != "": 
            if command == "exit": 
                ser.close()
                break
            command = command + "\n"
            ser.write(command.encode("ascii"))


if __name__ == "__main__": 

    parser = argparse.ArgumentParser()
    parser.add_argument('-v', action='store_true', dest='verbose', required=False)
    parser.add_argument('-H', action='store', dest='host', required=False, default="localhost")
    parser.add_argument('-p', action='store', dest='port', type=int, required=False, default="1883")
    parser.add_argument('-t', action='store', dest='topic', required=False)
    args = parser.parse_args()

    main(args)

    

#write_api.write(bucket=bucket, org="o.roman@uqconnect.edu.au",\
#                record=point_data)