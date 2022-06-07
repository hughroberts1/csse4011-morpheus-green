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
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
import threading
import re
import argparse
from datetime import datetime as dt
import csv

token = os.environ.get("INFLUXDB_TOKEN")
print(token)
org = "o.roman@uqconnect.edu.au"
url = "https://us-east-1-1.aws.cloud2.influxdata.com"
bucket = "Weather Data"
client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)


# Format of data will come in as: {"UUID":X, "Time":X, "Reading":{ "Temperature":X, etc}}
data = {}


nodes = {}
readings = {}

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
                measurement = float(reading[device_id])
                reading2 = {device_name: measurement}
                uuid = data["UUID"]
                point_data = {"measurement": nodes[uuid],
                                "time": dt.utcnow(),
                                "fields": reading2     
                                }

                print(point_data)
                write_api.write(bucket=bucket, org="o.roman@uqconnect.edu.au",record=point_data)
        except Exception as e: 
            pass

def main(): 

    p1 = threading.Thread(target=read_data)
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

    if (len(sys.argv) == 1): 
        print("Provide a serial port")
        sys.exit(-1)
    elif (len(sys.argv) == 2): 
        serialPort = sys.argv[1]
    else: 
        sys.exit(-2)

    ser = serial.Serial(serialPort)

    with open('nodes.csv') as node_file: 
        csv_reader = csv.reader(node_file, delimiter=',')
        line_count = 0
        for row in csv_reader: 
            if line_count == 0: 
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else: 
                nodes[row[0]] = row[1]
                line_count += 1
        print(f'Processed {line_count} lines.')
        print(nodes)

    with open('devices.csv') as node_file: 
        csv_reader = csv.reader(node_file, delimiter=',')
        line_count = 0
        for row in csv_reader: 
            if line_count == 0: 
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else: 
                readings[row[0]] = row[1]
                line_count += 1
        print(f'Processed {line_count} lines.')
        print(readings)

    main()
