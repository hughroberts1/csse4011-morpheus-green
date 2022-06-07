import sys, serial, os, time, json
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime as dt
import csv
token = os.environ.get("INFLUXDB_TOKEN")
org = "o.roman@uqconnect.edu.au"
url = "https://us-east-1-1.aws.cloud2.influxdata.com"
bucket = "Weather Data"
client = InfluxDBClient(url=url, token=token, org=org)

query_api = client.query_api()

hr_query = """from(bucket: "Weather Data")
 |> range(start: -6h)
 |> sort(columns: ["_time"], desc:false)
"""


queries_dict = []

tables = query_api.query(hr_query, org="o.roman@uqconnect.edu.au")
for table in tables:
        for record in table.records:
                query_result = {}
                query_result = {"Time":(record.get_time()).strftime("%d-%m-%y%h:%m:%s"), "Measurement":record.get_measurement(),\
                                "Field":record.get_field(), "Value":record.get_value()}
                queries_dict.append(query_result)


with open('real_time_weather_data.csv', "a", newline='') as weather_file:
        dict_writer = csv.DictWriter(weather_file, ["Time", "Measurement", "Field", "Value"])
        for query in queries_dict:
                print(query)
                dict_writer.writerows(query)
                
        