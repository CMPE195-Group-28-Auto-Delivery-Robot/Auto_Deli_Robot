#!/usr/bin/python3
# -*- coding: utf-8 -*-
from lib2to3.pgen2 import driver
from time import sleep
import rospy
from sensor_msgs.msg import Imu, MagneticField


class Positioner:
    def __init__(self):
        self.imu = None
        self.mag = None

    def callback_IMU(self, data: Imu):
        self.imu = data

    def callback_MAG(self, data: MagneticField):
        self.mag = data
    
def RobotDriverMain():

    rospy.init_node('Robot_Positionor', anonymous=True)

    robotPostionor = Positioner()

    rospy.Subscriber("zed_node/imu/mag", MagneticField, Positioner.callback_MAG, Positioner)

    rospy.spin()

if __name__ == '__main__':
    RobotDriverMain()