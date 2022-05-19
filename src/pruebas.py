#!/usr/bin/python

import serial
import time

serialPort = serial.Serial(port="/dev/ttyACM0", baudrate=57600)
print("ok")
response = serialPort.readline()
print(response)

a = str(input("a: "))
print(a)
a = a.encode()
print(a)
serialPort.write(a)
time.sleep(1)
response = serialPort.readline()
print(response)
response = serialPort.readline()
print(response)
serialPort.close()

a = a.decode()
print(a)
