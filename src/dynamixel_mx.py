#!/usr/bin/python

import rospy
import time
import serial


class DynamixelMx:

    def __init__(self):
        self.port = "/dev/ttyACM0"
        self.baudrate_board = 115200
        self.init_pose = 1000
        self.final_pose = 2000
        self.sleep_pose = 10

        self.present_pose = 0
        self.goal_pose = 0

    def setting_serial_port(self):
        self.serialPort = serial.Serial(
            port=self.port,
            baudrate=self.baudrate_board)
        rospy.loginfo("Successful_connection_board")

    def setting_board(self):
        output_tmp = "ok".encode()

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        rospy.loginfo(input_tmp)
        if (input_tmp == "succeeded_open_port"):
            self.serialPort.write(output_tmp)
        else:
            exit()

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        rospy.loginfo(input_tmp)
        if (input_tmp == "succeeded_change_baudrate"):
            self.serialPort.write(output_tmp)
        else:
            exit()

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        rospy.loginfo(input_tmp)
        if (input_tmp == "dynamixel_successfully_connected"):
            pass
        else:
            exit()

    def get_present_pose(self):
        output_tmp = "get_present_pose".encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")

        if (input_tmp == "failed_get_present_pose"):
            print(input_tmp)
            exit()

        self.present_pose = int(input_tmp)

    def set_preset_pose(self, new_present_pose):
        output_tmp = "set_preset_pose".encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()

        output_tmp = str(new_present_pose).encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")

        if (input_tmp == "failed_set_preset_pose"):
            print(input_tmp)
            exit()

    def exit_board(self):
        output_tmp = "exit".encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        print(input_tmp)
        exit()


if __name__ == '__main__':
    rospy.init_node('dynamixel_mx', anonymous=True)

    dynamixe_mx = DynamixelMx()
    dynamixe_mx.setting_serial_port()
    dynamixe_mx.setting_board()

    for new_pos in range(1834, 810, -50):
        dynamixe_mx.get_present_pose()
        dynamixe_mx.set_preset_pose(new_pos)

    dynamixe_mx.exit_board()
