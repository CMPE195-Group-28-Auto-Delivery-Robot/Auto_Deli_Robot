#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from laser_line_extraction.msg import LineSegment, LineSegmentList
from zed_interfaces.msg import ObjectsStamped, Object
from geometry_msgs.msg import Quaternion, Point, Pose, PointStamped, TransformStamped
from sensor_msgs.msg import NavSatFix
from robot_msgs.msg import dest_list_msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler

import autopilot

def get_next_coordinate(target_coordinate):
    target_point = Odometry()
    target_point.header.stamp = rospy.Time.now()
    target_point.header.frame_id = "map"
    target_point.pose.pose.position.x = target_coordinate[0]
    target_point.pose.pose.position.y = target_coordinate[1]
    target_point.pose.pose.position.z = 0.0
    target_point.pose.pose.orientation.x = 0.0
    target_point.pose.pose.orientation.y = 0.0
    target_point.pose.pose.orientation.z = 0.0
    target_point.pose.pose.orientation.w = 1.0
    return target_point

class setpose:

    def __init__(self):
        rospy.init_node('setpose')
        target_point = [4, 0]
        path_publisher = rospy.Publisher('/deli_robot/set_pose', Odometry, queue_size=10)
        rate = rospy.Rate(1)
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
