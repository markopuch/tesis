#!/usr/bin/env python3

import serial
import time
import csv

#I want  code that can read the serial port and write the data to a csv file
       
com = "/dev/ttyUSB0"
baud = 9600

x = serial.Serial(com, baud)

while x.isOpen() is True:
    data = str(x.readline().decode('utf-8')).rstrip()
    
    if data is not "":
        print(data)
        with open('data.csv', 'a') as csv_file:
            csv_file.write(data+ '\n')
    
