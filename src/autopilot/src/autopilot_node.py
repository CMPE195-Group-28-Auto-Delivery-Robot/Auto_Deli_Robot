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

import utm


import autopilot

obstacles = []
goal_position = []
current_position = []

tf_broadcaster = tf2_ros.TransformBroadcaster()
quaternion = quaternion_from_euler(0, 0, 0)

'''
def gps2etm(lat, lon, alt):
    utm_data = utm.from_latlon(lat, lon)
    utm_x = utm_data[0]
    utm_y = utm_data[1]
    utm_zone = utm_data[2]
    itm_band = utm_data[3]
    transform = TransformStamped()
    transform.header.stamp = rospy.Time(1)
    transform.header.frame_id = 'utm'
    transform.child_frame_id = 'gps'
    transform.transform.translation.x = utm_x
    transform.transform.translation.y = utm_y
    transform.transform.translation.z = alt
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    tf_broadcaster.sendTransform(transform)
'''

def NavSatFix2Point(msg, souce, target):
    temp_point = PointStamped()
    temp_point.header.stamp = msg.header.stamp
    temp_point.header.frame_id = souce
    temp_point.point = Point(msg.latitude, msg.longitude, msg.altitude)
    try:
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
        temp_point = tfBuffer.transform(temp_point, target, rospy.Duration(2))
        #print(temp_point)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #print(e)
        pass
    return temp_point


def Odometry2Point(msg, souce, target):
    temp_point = PointStamped()
    temp_point.header.stamp = msg.header.stamp
    temp_point.header.frame_id = souce
    temp_point.point = Point(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    try:
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
        temp_point = tfBuffer.transform(temp_point, target, rospy.Duration(0.05))
        #print(temp_point)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #print(e)
        pass
    return temp_point
    

class autopilot_node:

    def __init__(self):
        self.status = False
        self.target_list = []
        self.target_point = [30, 30]
        self.curren_point = [0.0, 0.0]
        self.objects = []
        self.obstacles = []
        self.restricted_areas = []
        self.step_length = 2
        self.lost_gps = False
        self.node = autopilot.autopilot()

        self.local_point = Point()
        self.dest_point = Point()

        self.run()

    # Callback function to receive the current gps coordinates
    def gps_callback(self, msg):

        #gps2etm(msg.latitude, msg.longitude, msg.altitude)
        self.local_point = Odometry2Point(msg, "odom" ,"base_link").point
        print(self.local_point)
        self.curren_point[0] = self.local_point.x
        self.curren_point[1] = self.local_point.y
        '''
        if msg.status.status == 0:
            self.lost_gps = False
        else:
            self.lost_gps = True
        '''

    # Callback function to receive target coordinates destination
    def destination_callback(self, msg):
        #self.target_point = NavSatFix2Point(msg, "gps" ,"map").point
        '''
        target_list = []
        for dest_point in msg.dest_list:
            target_list.append([dest_point.lat, dest_point.lng])
        self.target_point = target_list.pop(0)
        '''

    # Callback function to receive the coordinates of the obstacle
    def obstacles_callback(self, msg):
        self.obstacles = []
        for line_segment in msg.line_segments:
            self.obstacles.append((line_segment.start, line_segment.end))


    # Callback function to receive object recognition information
    def object_callback(self, msg):
        self.objects = []
        for temp_object in msg.objects:
            self.objects.append(temp_object)
        if "Person" in self.objects or "Vehicle" in self.objects:
            self.step_length = 1
        else:
            self.step_length = 2

    

    def run(self):
        rospy.init_node('autopilot_node')
        # Subscribe to topics
        rospy.Subscriber('/gps', Odometry, self.gps_callback)
        rospy.Subscriber('/obstacles', LineSegmentList, self.obstacles_callback)

        #rospy.Subscriber('/destination', dest_list_msg, self.destination_callback)
        #rospy.Subscriber('/objects', ObjectsStamped, self.object_callback)
        #rospy.Subscriber('/lawn', ObjectsStamped, self.object_callback)

        # Publish results
        # /deli_robot/set_pose
        #path_publisher = rospy.Publisher('/path', Odometry, queue_size=10)
        # /deli_robot/emoji_message
        #msg_publisher = rospy.Publisher('/emoji_message', String, queue_size=10)

        test1 = rospy.Publisher('/test1', Point, queue_size=10)
        test2 = rospy.Publisher('/test2', Point, queue_size=10)
       
        # 1hz
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            test1.publish(self.local_point)
            
            rate.sleep()

        # main function
        while not rospy.is_shutdown():
            # start work if gps working and get order
            if self.status and not self.lost_gps:
                path, done = self.node.get_next(self.obstacles, self.restricted_areas, self.step_length, self.curren_point, self.target_point)
                # check point
                if done:
                    #path_publisher.publish(self.target_point)
                    # not finish all point
                    if self.target_list:
                        self.target_point = self.target_list.pop(0)
                        self.node.start()
                        print("log: check")
                    # finish all point
                    else:
                        self.status = False
                        print("log: done")
                # next pose
                elif path:
                    path[0] += self.curren_point[0]
                    path[1] += self.curren_point[1]
                    #path_publisher.publish(path)
                    print("======")
                    print(path)
                # error
                else:
                    self.status = False
                    #msg_publisher.publish("error")
                    print("error: cannot move")
            # get order
            elif self.target_point:
                self.status = True
                self.node.start()
            rate.sleep()


if __name__ == '__main__':
    try:
        autopilot_node()
    except rospy.ROSInterruptException:
        pass
