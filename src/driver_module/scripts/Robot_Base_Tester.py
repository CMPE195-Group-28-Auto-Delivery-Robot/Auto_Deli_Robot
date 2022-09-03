#!/usr/bin/python3
# -*- coding: utf-8 -*-
from lib2to3.pgen2 import driver
from time import sleep
import rospy
from geometry_msgs.msg import Twist 
import board
import busio
import adafruit_pca9685
import adafruit_motor.servo
from adafruit_servokit import ServoKit

def MAX_SERVO_DEGREE():
    return 125;

def MIN_SERVO_DEGREE():
    return 45;

def RANGE_SERVO_DEGREE():
    return (MAX_SERVO_DEGREE()-MIN_SERVO_DEGREE());

def CENTER_SERVO_DEGREE():
    return (RANGE_SERVO_DEGREE())/2+MIN_SERVO_DEGREE();


class i2CPWMDriver:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.pca.frequency = 50
        self.Channels = ServoKit(channels=16)
        print("Robot Driver Initalize Completed")
    
    def SetChannelAngle(self, Channel:int, Angle:int):
        self.Channels.servo[Channel].angle = Angle
        print("Robot Turn to Angle: "+str(Angle-CENTER_SERVO_DEGREE()))
    
def RobotTesterMain():
    driverBoard = i2CPWMDriver()
    Channel_Want = input("Playing Channel: ")
    while True:
        Angle_Want = input("Angle you want: ")
        driverBoard.SetChannelAngle(int(Channel_Want), int(Angle_Want));


if __name__ == '__main__':
    RobotTesterMain()