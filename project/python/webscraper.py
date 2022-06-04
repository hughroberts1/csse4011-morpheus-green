####################################################################################################
# Webscraper made for http://weather.science.uq.edu.au/Archive/ to scrape weather data from the UQ
# weather station to feed in to the ML algorithm to help predict the event of rain for the following
# day.
# 
# Author: Oliver Roman
# Date: 01/06/2022
####################################################################################################

from urllib.request import urlopen
from bs4 import BeautifulSoup as bs
import numpy as np
import csv
import re
import datetime as dt

# Opens the url given through params and returns the open page_html file
def url_parse(link):
        uClient = urlopen(link)
        page_html = uClient.read()
        uClient.close()
        return page_html


# Parses through .txt files that can be accessed by giving a url and returns all the weather data
# scraped from the .txt file in a list of dictionaries
def url_txt_parse(link, columns):    
        weather_data = []
        file_text = url_parse(link)
        file_text = file_text.decode()
        lines = file_text.split('\r\n')

        for line in lines:
                # Check that the start of the line is valid for error checking
                if re.search("^\d\d-\d\d-\d\d",line):
                        data_dict = {} # dictionary to store file data (each line)
                        data = []
                        total_data = [item.strip() for item in line.split('\t')]
                        data.append(total_data[0])  # date
                        data.append(total_data[2])  # temp values
                        data.append(total_data[5])  # hum values
                        data.append(total_data[16]) # pressure values
                        data.append(total_data[17]) # rain amount values
                        data.append(total_data[18]) # rain rate values

                        for index, elem in enumerate(data):
                                data_dict[columns[index]] = data[index]
                        weather_data.append(data_dict) # append dictionary to list
        return weather_data


# Parses through .csv files that can be accessed by giving a url and returns all the weather data
# scraped from the .csv file in a list of dictionaries
def url_csv_parse(link, columns):
        weather_data = []
        csv_file = urlopen(link)
        rows = [r.decode('utf-8') for r in csv_file.readlines()]
        csv_reader = csv.reader(rows)

        i = 1
        for row in csv_reader:
                if i == 1:
                        i += 1
                else:
                        data_dict = {} # dictionary to store file data (each row)
                        data = []
                        # Needed to format date the same as the txt files
                        data.append(dt.datetime.strptime(row[0].split(' ')[0], "%Y/%m/%d").\
                                    strftime("%d-%m-%y"))  # date and time
                        data.append(row[6])  # temp values
                        data.append(row[7])  # hum values
                        data.append(row[8])  # pressure values
                        data.append(row[9])  # rain amount values
                        data.append(row[10]) # rain rate values

                        for index, elem in enumerate(data):
                                data_dict[columns[index]] = data[index]
                        weather_data.append(data_dict) # append dictionary to list
                        i += 1
        return weather_data


# Run data through this to compute daily averages all fields and min/max values of temperature
# returns as a new list of dictionary data 
def process_data(data_points):
        dates = []
        processed_data_points = []
        
        for data in data_points:
                # Make a list of dates, then for each date get all readings and average/min/max them
                if not (data["Date"] in dates):
                        dates.append(data["Date"])
        for date in dates:
                temps, hums, pres, rain = [], [], [], []
                for data in data_points:
                        # Check that the date for all data points is the same and there is no null
                        # values in the dictionary otherwise we throw this data value away
                        if (data["Date"] == date) and not ("---" in data.values()):
                                temps.append(data["Temperature"])
                                hums.append(data["Relative Humidity"])
                                pres.append(data["Sea Level Pressure"])
                                rain.append(data["Rain"])
                if(temps and hums and pres and rain):
                        mean_temp = np.array(temps, dtype=np.float32).mean()
                        min_temp = np.array(temps, dtype=np.float32).min()
                        max_temp = np.array(temps, dtype=np.float32).max()
                        mean_hum = np.array(hums, dtype=np.float32).mean()
                        mean_pres = np.array(pres, dtype=np.float32).mean()
                        mm_rain = np.array(rain, dtype=np.float32).sum()
                        values = [date, mean_temp, max_temp, min_temp, mean_hum, mean_pres, mm_rain]

                        processed_data = {}
                        for index, elem in enumerate(values):
                                processed_data[condensed_headers[index]] = values[index]
                        processed_data_points.append(processed_data) # append dictionary to list
        return processed_data_points

if __name__ == "__main__":
        # Webscrape section
        my_url = 'http://weather.science.uq.edu.au/Archive/'

        homepage_html = url_parse(my_url)
        homepage_urls = []
        homepage_soup = bs(homepage_html, "html.parser")

        for link in homepage_soup.find_all('a'):
                if re.search("^20",link.get('href')):
                        homepage_urls.append(my_url + link.get('href'))

        subpage_urls = []
        txt_urls = []
        csv_urls = []
        for link in homepage_urls:
                subpage_html = url_parse(link)
                subpage_soup = bs(subpage_html, "html.parser")
                for sublink in subpage_soup.find_all('a'):
                        if re.search("20\d\d-\d\d\.txt",sublink.get('href')):
                                txt_urls.append(link + sublink.get('href'))
                        elif re.search("[01]\d/$",sublink.get('href')):
                                subpage_urls.append(link + sublink.get('href'))

        # Remove because these are doubles (there are csv and txt files covering these date ranges)
        subpage_urls.remove('http://weather.science.uq.edu.au/Archive/2017/2017_03/')

        for link in subpage_urls:
                subsubpage_html = url_parse(link)
                subsubpage_soup = bs(subsubpage_html, "html.parser")
                for sublink in subsubpage_soup.find_all('a'):
                        if re.search("\.csv",sublink.get('href')):
                                csv_urls.append(link + sublink.get('href'))

        # Remove this aberrant scum >:(
        csv_urls.\
        remove('http://weather.science.uq.edu.au/Archive/2019/2019_10/20191002_to_1009_T140258.csv')

        headers = ["Date","Temperature","Relative Humidity","Sea Level Pressure","Rain",\
                "Rain Intensity"]

        condensed_headers = ["Date", "Avg Temperature" ,"Max Temperature", "Min Temperature",\
                        "Avg Humidity","Avg Pressure","mm of Rain"]
        condensed_data = {}


        with open("weather_data.csv", "w", newline = '') as weather_file:
                        dict_writer = csv.DictWriter(weather_file, condensed_headers)
                        dict_writer.writeheader()

        for link in txt_urls:
                data_chunk = process_data(url_txt_parse(link, headers))

                # Write all the data to a csv file called weather data
                with open("weather_data.csv", "a", newline = '') as weather_file:
                        dict_writer = csv.DictWriter(weather_file, condensed_headers)
                        dict_writer.writerows(data_chunk)


        for link in csv_urls:
                data_chunk = process_data(url_csv_parse(link, headers))
                # Write all the data to a csv file called weather data
                with open("weather_data.csv", "a", newline = '') as weather_file:
                        dict_writer = csv.DictWriter(weather_file, condensed_headers)
                        dict_writer.writerows(data_chunk)

        print("Done")