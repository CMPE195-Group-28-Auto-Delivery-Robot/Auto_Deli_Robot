#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from robot_msgs.msg import dest_list_msg

import utm


class gps_tf2_utm:

    def set_gps_to_utm(self, utm_data, alt):
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'gps'
        transform.child_frame_id = 'utm'
        transform.transform.translation.x = utm_data[0]
        transform.transform.translation.y = utm_data[1]
        transform.transform.translation.z = alt
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def publish_utm(self, utm_data, alt):
        utm_point = PointStamped()
        utm_point.header.stamp = rospy.Time.now()
        utm_point.header.frame_id = 'utm'
        utm_point.point.x = utm_data[0]
        utm_point.point.y = utm_data[1]
        utm_point.point.z = alt
        self.utm_publisher.publish(utm_point)

    def gps_callback(self, msg):
        utm_data = utm.from_latlon(msg.latitude, msg.longitude)
        self.set_gps_to_utm(utm_data, msg.altitude)
        self.publish_utm(utm_data, msg.altitude)
        
    def destination_callback(self, msg):
        for dest_point in msg.dest_list:
            utm_data = utm.from_latlon(dest_point.lat, dest_point.lng)
            self.set_gps_to_utm(utm_data)

    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.utm_publisher = rospy.Publisher('/utm', PointStamped, queue_size=10)
        self.run()
    
    def run(self):
        rospy.init_node('gps_tf2_utm')
        rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/dest',  dest_list_msg, self.destination_callback)
        rospy.spin()

if __name__ == '__main__':
    gps_tf2_utm()
