#!/usr/bin/python

import rospy
import serial

port = "/dev/ttyACM0"
baudrate = 57600

# rospy.init_node('dynamixel_mx')

try:
    serialPort = serial.Serial(port=port, baudrate=baudrate)
    print("Successful connection to board")
    print("ok")
except:
    print("Failed connection to board")

response = serialPort.readline()
print(response)

a = "1"
a = bytes(a, "ascii")
serialPort.write(a)

response = serialPort.readline()
print(response)

serialPort.write(b"2\r\n")

response = serialPort.readline()
print(response)

serialPort.write(b"3\r\n")

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
