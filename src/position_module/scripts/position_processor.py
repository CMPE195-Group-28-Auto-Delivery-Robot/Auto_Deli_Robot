#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Positioner:
    def __init__(self):
        self.odom = None
        self.cmdVel = None

    def callback_CmdVel(self, data: Twist):
        self.cmdVel = data
        rospy.loginfo("Speed: x:"+str(self.Odom.twist.twist.linear.x))

    def callback_Odom(self, data: Odometry):
        self.odom = data
        rospy.loginfo("Speed: x:"+str(self.odom.twist.twist.linear.x))
    
def RobotPoseMain():

    rospy.init_node('Robot_Positionor', anonymous=True)

    robotPostionor = Positioner()

    rospy.Subscriber("zed_node/odom", Odometry, Positioner.callback_Odom, Positioner)
    rospy.Subscriber("cmd_vel", Twist, Positioner.callback_CmdVel, Positioner)

    rospy.spin()

if __name__ == '__main__':
    RobotPoseMain()