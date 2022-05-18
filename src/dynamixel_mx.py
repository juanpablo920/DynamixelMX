#!/usr/bin/python

import serial
import time

serialPort = serial.Serial(port="/dev/ttyACM0", baudrate=57600)
print("ok")
time.sleep(0.5)

response = serialPort.readline()
print(response)
time.sleep(0.5)
a = "1"
a = bytes(a, "ascii")
serialPort.write(a)
time.sleep(1)

response = serialPort.readline()
print(response)
time.sleep(0.5)
serialPort.write(b"2\r\n")
time.sleep(1)

response = serialPort.readline()
print(response)
time.sleep(0.5)
serialPort.write(b"3\r\n")
time.sleep(1)

serialPort.close()

# serialPort = serial.Serial(port="/dev/ttyACM0", baudrate=57600)
# print("serialPort ok")
# serialPort.close()
# exit()
# goal_position_tmp = ""
# while(goal_position_tmp != "q"):

#     response = serialPort.readline()
#     print(response)

#     goal_position_tmp = str(input("goal_position_tmp: "))
#     serialPort.write(format(ord(goal_position_tmp), 'b'))

#     time.sleep(0.5)

# serialPort.close()
