#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    for distance in data.ranges:
        #print(distance)
        if distance <= 0.5:
            rospy.loginfo("Too close")
    
    
def lidar_analysis():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lidar_analysis_node', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lidar_analysis()
