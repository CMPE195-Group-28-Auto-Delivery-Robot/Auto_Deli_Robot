#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Point, Pose, PointStamped, PoseStamped
from nav_msgs.msg import Odometry


def get_next_coordinate(target_coordinate):
    target_point = PoseStamped()
    target_point.header.stamp = rospy.Time.now()
    target_point.header.frame_id = "map"
    target_point.pose.position.x = target_coordinate[0]
    target_point.pose.position.y = target_coordinate[1]
    target_point.pose.position.z = 0.0
    target_point.pose.orientation.x = 0
    target_point.pose.orientation.y = 0
    target_point.pose.orientation.z = 0
    target_point.pose.orientation.w = 1
    return target_point

class setpose:

    def __init__(self):
        rospy.init_node('setpose')
        target_point = [2, 0]
        path_publisher = rospy.Publisher('/deli_robot/goalPosition', PoseStamped, queue_size=10)
        rate = rospy.Rate(5)
        # main function
        while not rospy.is_shutdown():
            path_publisher.publish(get_next_coordinate(target_point))
            print(target_point)
            rate.sleep()
            


if __name__ == '__main__':
    try:
        setpose()
    except rospy.ROSInterruptException:
        pass
