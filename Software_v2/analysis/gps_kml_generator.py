import numpy as np
import csv

init = False

latitude = 0
longitude = 0

with open('data_2.txt','r') as csvfile:
    with open('earth.csv','w') as kml:
        spamreader = csv.reader(csvfile, delimiter=',')
        for row in spamreader:
            if not init:
                calib_time = float(row[0])
                calib_tmep = float(row[1])
                calib_alt = float(row[2])
                ram_offset = float(row[3])
            
            if row[0] == str(1):
                last_latitude = latitude
                last_longitude = longitude
                
                latitude = float(row[1]) + float(row[2])/60 + float(row[3])/3600
                longitude = float(row[5]) + float(row[6])/60 + float(row[7])/3600
                
                if last_latitude != latitude or last_longitude != longitude:
                    kml.write(str(longitude) + ',' + str(latitude) + '\n')
        
kml.close()