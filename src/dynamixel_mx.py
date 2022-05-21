#!/usr/bin/python

import rospy
from std_msgs.msg import Int16

import serial
import time


class DynamixelMx:

    def __init__(self):
        self.port = "/dev/ttyACM0"
        self.baudrate_board = 115200
        self.init_pose = 1834
        self.final_pose = 810
        self.sleep_org = 30
        self.sleep_scan = 10
        self.delta_error = 5

        self.pose = 0

    def setting_publisher(self):
        self.pub_pose = rospy.Publisher(
            "/dynamixel_mx/pose",
            Int16)

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
            self.serialPort.close()
            exit()

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        rospy.loginfo(input_tmp)
        if (input_tmp == "succeeded_change_baudrate"):
            self.serialPort.write(output_tmp)
        else:
            self.serialPort.close()
            exit()

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        rospy.loginfo(input_tmp)
        if (input_tmp == "dynamixel_successfully_connected"):
            pass
        else:
            self.serialPort.close()
            exit()

    def publish_pose(self):
        self.pub_pose.publish(self.pose)

    def get_pose(self):
        output_tmp = "get_pose".encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")

        if (input_tmp == "failed_get_pose"):
            rospy.loginfo(input_tmp)
            self.serialPort.close()
            exit()

        self.pose = int(input_tmp)

    def set_pose(self, new_pose):
        output_tmp = "set_pose".encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()

        output_tmp = str(new_pose).encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")

        if (input_tmp == "failed_set_pose"):
            rospy.loginfo(input_tmp)
            self.serialPort.close()
            exit()

    def exit_board(self):
        output_tmp = "exit".encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        rospy.loginfo(input_tmp)
        self.serialPort.close()
        exit()

    def go_origin(self):
        rospy.loginfo("go_origin")
        output_tmp = "go_origin".encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        output_tmp = str(self.init_pose).encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        output_tmp = str(self.sleep_org).encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        output_tmp = str(self.delta_error).encode()
        self.serialPort.write(output_tmp)

        input_tmp = self.serialPort.readline()
        input_tmp = input_tmp.decode()
        input_tmp = input_tmp.rstrip("\r\n")
        rospy.loginfo(input_tmp)

        if (input_tmp == "failed_go_origin"):
            self.serialPort.close()
            exit()

    def go_scan(self):
        rospy.loginfo("go_scan")
        sleep = self.sleep_scan
        self.get_pose()
        self.publish_pose()
        delta_poses = abs(self.pose - self.final_pose)

        while (delta_poses > self.delta_error):

            if delta_poses < sleep:
                sleep = delta_poses

            if (self.pose > self.final_pose):
                new_pos = self.pose - sleep
            else:
                new_pos = self.pose + sleep

            self.set_pose(new_pos)
            self.get_pose()
            self.publish_pose()
            delta_poses = abs(self.pose - self.final_pose)

        rospy.loginfo("arrived_scan")


if __name__ == '__main__':
    rospy.init_node('dynamixel_mx')
    dynamixe_mx = DynamixelMx()
    dynamixe_mx.setting_serial_port()
    dynamixe_mx.setting_board()
    try:
        dynamixe_mx.go_origin()
        dynamixe_mx.go_scan()
        dynamixe_mx.exit_board()
    except rospy.ROSInterruptException:
        dynamixe_mx.exit_board()
