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
    return 130;

def MIN_SERVO_DEGREE():
    return 40;

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
        rospy.loginfo("Robot Driver Initalize Completed")
    
    def SetTurningAngle(self, Angle:int):
        self.Channels.servo[0].angle = Angle
        rospy.loginfo("Robot Turn to Angle: "+str(Angle-CENTER_SERVO_DEGREE()))

    def SetThrottle(self, Power:float):
        self.Channels.servo[1].fraction = Power
        rospy.loginfo("Robot Throttle Power: "+str(Power*100))
    

def callback(data: Twist, device: i2CPWMDriver):
    turnDegree = (((data.angular.z + 3) / 6) * RANGE_SERVO_DEGREE() + MIN_SERVO_DEGREE())
    device.SetTurningAngle(turnDegree)
    robotThrottle = (data.linear.x + 3) / 6
    device.SetThrottle(robotThrottle)
    
def RobotDriverMain():

    rospy.init_node('Robot_Base_Driver', anonymous=True)

    driverBoard = i2CPWMDriver()

    driverBoard.SetTurningAngle(CENTER_SERVO_DEGREE())
    driverBoard.SetThrottle(0.5)

    rospy.Subscriber("cmd_vel", Twist, callback, driverBoard)
    # rospy.Subscriber("controlVel", Twist, callback, driverBoard)

    rospy.spin()

if __name__ == '__main__':
    RobotDriverMain()