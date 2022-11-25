#!/usr/bin/python3
# -*- coding: utf-8 -*-
from lib2to3.pgen2 import driver
from time import sleep
import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu 
import board
import busio
import adafruit_bno055

class i2cBNO055:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
        rospy.loginfo("BNO055 IMU Initalize Completed")
    
    def GetImuMsg(self) -> Imu :
        tempMsg = Imu()
        # print("Linear acceleration (m/s^2): {}".format(self.bno.linear_acceleration))
        tempMsg.header.stamp = rospy.Time.now()
        tempMsg.header.frame_id = "base_link"
        roll = math.radians(self.bno.euler[0])
        pitch = math.radians(self.bno.euler[1])
        yaw = math.radians(self.bno.euler[2])
        tempMsg.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        tempMsg.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        tempMsg.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        tempMsg.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        tempMsg.linear_acceleration.x = self.bno.linear_acceleration[0]
        tempMsg.linear_acceleration.y = self.bno.linear_acceleration[1]
        tempMsg.linear_acceleration.z = self.bno.linear_acceleration[2]
        tempMsg.angular_velocity.x = self.bno.gyro[0]
        tempMsg.angular_velocity.y = self.bno.gyro[1]
        tempMsg.angular_velocity.z = self.bno.gyro[2]
        return tempMsg
        
    
def RobotDriverMain():

    rospy.init_node('Robot_Imu_Node', anonymous=True)

    imuBoard = i2cBNO055()
    imuPublisher = rospy.Publisher('sensors/imu', Imu, queue_size=10)

    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        imuPublisher.publish(imuBoard.GetImuMsg())
        rate.sleep()

    rospy.loginfo("BNO055 IMU Node End")

if __name__ == '__main__':
    RobotDriverMain()