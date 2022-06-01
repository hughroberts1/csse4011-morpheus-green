from urllib.request import urlopen
from bs4 import BeautifulSoup as bs
import re

def url_parse(url):
        uClient = urlopen(url)
        page_html = uClient.read()
        uClient.close()
        return page_html
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
for url in homepage_urls:
        subpage_html = url_parse(url)
        subpage_soup = bs(subpage_html, "html.parser")
        for link in subpage_soup.find_all('a'):
                if re.search("20\d\d-\d\d\.txt",link.get('href')):
                        txt_urls.append(url + link.get('href'))
                elif re.search("[01]\d/$",link.get('href')):
                        subpage_urls.append(url + link.get('href'))
# Remove because these are doubles (there are csv and txt files covering these date ranges)
subpage_urls.remove('http://weather.science.uq.edu.au/Archive/2017/2017_03/')
subpage_urls.remove('http://weather.science.uq.edu.au/Archive/2017/2017_04/')
subpage_urls.remove('http://weather.science.uq.edu.au/Archive/2017/2017_05/')

for url in subpage_urls:
        subsubpage_html = url_parse(url)
        subsubpage_soup = bs(subsubpage_html, "html.parser")
        for link in subsubpage_soup.find_all('a'):
                if re.search("\.csv",link.get('href')):
                        csv_urls.append(url + link.get('href'))

# Remove this abberant scum >:(
csv_urls.remove('http://weather.science.uq.edu.au/Archive/2019/2019_10/20191002_to_1009_T140258.csv')

dates = []
times = []
max_temp = []
min_temp = []
rel_hum = []
pressure = []
rain = []
weather_data_entry = {}
weather_data = []

file = open('test.txt', mode = 'r', encoding = 'utf-8-sig')
lines = file.readlines()
file.close()

#for line in lines:
#        line = line.split('\t')
#        line[:] = [x for x in line if x != '']



i = 1
for line in lines:
        line = line.strip() # remove leading/trailing white spaces
        if line:
                if i == 1:
                        columns1 = [item.strip() for item in line.split('\t')]
                        i = i + 1
                elif i == 2:
                        columns2 = [item.strip() for item in line.split('\t')]
                        for item in columns1:
                                
                else:
                        d = {} # dictionary to store file data (each line)
                        data = [item.strip() for item in line.split(',')]
                        for index, elem in enumerate(data):
                                d[columns[index]] = data[index]
                        my_list.append(d) # append dictionary to list
filename = "weather_data.csv"
open(filename,w)