#!/usr/bin/python

import rospy
import rospkg
from geometry_msgs.msg import PointStamped

import serial
import yaml
import time


class DynamixelMx:

    def __init__(self):
        r = rospkg.RosPack()
        pwd_params = r.get_path('dynamixel_mx')+'/config/params.yaml'

        with open(pwd_params, 'r') as f:
            params = yaml.load(f)["dynamixel_mx"]

        self.port = params["port"]
        self.baudrate_board = params["baudrate_board"]
        self.init_pose = params["init_pose"]
        self.final_pose = params["final_pose"]
        self.step_org = params["step_org"]
        self.step_scan = params["step_scan"]
        self.delta_error = params["delta_error"]
        self.resolution = params["resolution"]
        self.base_pose = params["base_pose"]

        self.pose = 0

    def setting_publisher(self):
        self.pub_pose = rospy.Publisher(
            "/dynamixel_mx/pose",
            PointStamped)

    def setting_serial_port(self):
        self.serialPort = serial.Serial(
            port=self.port,
            baudrate=self.baudrate_board)
        rospy.loginfo("Successful_connection_board")

    def setting_board(self):
        output_tmp = "1".encode()

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
        pose_mgs = PointStamped()
        pose_mgs.point.x = (int(self.pose) - self.base_pose)/self.resolution
        pose_mgs.header.stamp = rospy.Time.now()
        self.pub_pose.publish(pose_mgs)

        # pose_mgs.point.x = 0
        # pose_mgs.header.stamp = rospy.Time.now()
        # self.pub_pose.publish(pose_mgs)

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
        output_tmp = "set_pose"
        output_tmp += ":"
        output_tmp += str(new_pose)
        output_tmp = output_tmp.encode()
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
        output_tmp = "go_origin"
        output_tmp += ":"
        output_tmp += str(self.init_pose)
        output_tmp += ":"
        output_tmp += str(self.step_org)
        output_tmp += ":"
        output_tmp += str(self.delta_error)
        output_tmp = output_tmp.encode()

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
        output_tmp = "go_scan"
        output_tmp += ":"
        output_tmp += str(self.final_pose)
        output_tmp += ":"
        output_tmp += str(self.step_scan)
        output_tmp += ":"
        output_tmp += str(self.delta_error)
        output_tmp = output_tmp.encode()

        self.serialPort.write(output_tmp)

        output_tmp = "1".encode()

        state_set_pose = 1
        while (state_set_pose == 1):
            input_tmp = self.serialPort.readline()
            input_tmp = input_tmp.decode()
            input_tmp = input_tmp.rstrip("\r\n")

            if (input_tmp == "arrived_scan"):
                state_set_pose = 0
                rospy.loginfo(input_tmp)
            elif (input_tmp == "failed_go_scan"):
                rospy.loginfo(input_tmp)
                self.serialPort.close()
                exit()
            else:
                self.pose = int(input_tmp)
                self.publish_pose()
                time.sleep(0.1)
                self.serialPort.write(output_tmp)


if __name__ == '__main__':
    rospy.init_node('dynamixel_mx')
    dynamixe_mx = DynamixelMx()
    dynamixe_mx.setting_publisher()
    dynamixe_mx.setting_serial_port()
    dynamixe_mx.setting_board()

    dynamixe_mx.go_origin()
    dynamixe_mx.go_scan()
    rospy.loginfo("control + C para salir")
    while not rospy.is_shutdown():
        pass
    dynamixe_mx.exit_board()
