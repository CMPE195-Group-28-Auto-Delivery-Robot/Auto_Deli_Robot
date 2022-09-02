#!/usr/bin/python3
# -*- coding: utf-8 -*-

import ctypes
# import pylibi2c
import rospy
from geometry_msgs.msg import Twist 


def i2c_PWM_write(device, PWM_CH, value):
    if( (PWM_CH > 15) or (PWM_CH < 0) or (value > 4095) or (value < 0) ):
        return False
    OFF_Hi  = (PWM_CH & 0xFF)*4 + 0x09
    ON_Hi   = (PWM_CH & 0xFF)*4 + 0x07
    ON_Lo   = (PWM_CH & 0xFF)*4 + 0x06

    device.write(OFF_Hi, ((0x10).to_bytes(1,'little')))
    PWM_Lo= (value & 0xFF).to_bytes(1, "little")
    PWM_Hi= ((value >> 8) & 0x0F).to_bytes(1, "little")
    print(PWM_Lo)
    print(PWM_Hi)
    device.write(ON_Lo, PWM_Lo)
    device.write(ON_Hi, PWM_Hi)
    return True

def i2C_Main():
    i2c_PWM = pylibi2c.I2CDevice("/dev/i2c-1", 0x40)

    print(i2c_PWM.ioctl_read(0x6, 1))
    print(i2c_PWM.ioctl_read(0x7, 1))
    print(i2c_PWM.ioctl_read(0x8, 1))
    print(i2c_PWM.ioctl_read(0x9, 1))
    i2c_PWM_write(i2c_PWM, 1, 3003)

    print(i2c_PWM.ioctl_read(0x6, 1))
    print(i2c_PWM.ioctl_read(0x7, 1))
    print(i2c_PWM.ioctl_read(0x8, 1))
    print(i2c_PWM.ioctl_read(0x9, 1))

def callback(data: Twist):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)
    
def listener():
    rospy.init_node('Robot_Base_Driver', anonymous=True)

    rospy.Subscriber("deli_robot/cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()