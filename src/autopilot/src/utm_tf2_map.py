#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, Point

import utm

def get_gps_to_utm(x, y):
    temp_point = PointStamped()
    temp_point.header.stamp = rospy.Time()
    temp_point.header.frame_id = "gps"
    temp_point.point = Point(x, y, 0)
    try:
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform("utm", "gps", rospy.Time(), rospy.Duration(1))
        temp_point = do_transform_point(temp_point, trans)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    return [temp_point.point.x, temp_point.point.y]


class utm_tf2_map:

    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.start_point = [0, 0]
        self.init = False
        self.run()

    def set_utm_to_map(self, x, y):
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'utm'
        transform.child_frame_id = 'map'
        transform.transform.translation.x = self.start_point[0]
        transform.transform.translation.y = self.start_point[1]
        transform.transform.translation.z = 0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def gps_callback(self, msg):
        if not self.init:
            self.start_point = get_gps_to_utm(msg.latitude, msg.longitude)
            print(self.start_point)
            self.init = True

    def utm_callback(self, msg):
        if self.init:
            self.set_utm_to_map(msg.point.x, msg.point.y)

    def run(self):
        rospy.init_node('utm_tf2_map')
        rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/utm', PointStamped, self.utm_callback) 
        rospy.spin()


if __name__ == '__main__':
    utm_tf2_map()