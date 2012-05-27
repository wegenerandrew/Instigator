#!/usr/bin/env python

import serial

ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

line = ser.readline()
print line

array = line.split(',')
print array


