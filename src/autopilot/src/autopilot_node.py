#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Float32
from tf2_geometry_msgs import do_transform_point
from laser_line_extraction.msg import LineSegment, LineSegmentList
from zed_interfaces.msg import ObjectsStamped, Object
from geometry_msgs.msg import Quaternion, Point, Pose, PointStamped, TransformStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from robot_msgs.msg import dest_list_msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler

from robot_msgs.srv import ChangeSpeed

import autopilot

test_mode = rospy.get_param('test_status')
if test_mode:
    test_x = rospy.get_param('test_x')
    test_y = rospy.get_param('test_y')

def gps_to_map(coordinate):
    x, y = coordinate
    temp_point = PointStamped()
    temp_point.header.stamp = rospy.Time()
    temp_point.header.frame_id = "map"
    temp_point.point = Point(x, y, 0)
    try:
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform("map", "gps", rospy.Time(), rospy.Duration(1))
        temp_point = do_transform_point(temp_point, trans)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    return [temp_point.point.x, temp_point.point.y]


def laser_to_map(coordinate, trans):
    x, y = coordinate
    temp_point = PointStamped()
    temp_point.header.stamp = rospy.Time.now()
    temp_point.header.frame_id = "odom"
    temp_point.point = Point(x, y, 0)
    temp_point = do_transform_point(temp_point, trans)
    return [temp_point.point.x, temp_point.point.y]
        

def obstacles_convet(obstacles):
    temp_obstacles = []
    try:
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform("odom", "laser", rospy.Time(), rospy.Duration(50))
        for obstacle in obstacles:
            temp_obstacles.append([laser_to_map(obstacle[0], trans), laser_to_map(obstacle[1], trans), 0])
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    return temp_obstacles
    

def to_map_coordinate(target_coordinate):
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

class autopilot_node:

    def __init__(self):
        self.status = False
        self.lost_gps = False
        self.target_list = []
        self.curren_point = [0, 0]
        self.objects = []
        self.obstacles = []
        self.restricted_areas = []
        self.speed = 2
        self.local_point = []
        self.node = autopilot.autopilot()
        self.target_point = []
        self.start_point = []
        self.service_client = rospy.ServiceProxy('Robot_PID_Ctrl/ChangeSpeed', ChangeSpeed)
        self.run()


    # Callback function to receive the current map coordinates
    def map_callback(self, msg):
        self.curren_point[0] = msg.pose.pose.position.x
        self.curren_point[1] = msg.pose.pose.position.y


    # Callback function to receive target coordinates destination
    def destination_callback(self, msg):
        self.target_list = []
        for dest_point in msg.dest_list:
            self.target_list.append(gps_to_map([dest_point.lat, dest_point.lng]))
        rospy.loginfo(self.target_list)
        self.target_point = self.target_list.pop(0)
        self.start_point = self.curren_point

    # Callback function to receive the coordinates of the obstacle
    def obstacles_callback(self, msg):
        temp_obstacles = []
        for line_segment in msg.line_segments:
            temp_obstacles.append([line_segment.start, line_segment.end])
        self.obstacles = temp_obstacles

    # Callback function to receive object recognition information
    def object_callback(self, msg):
        self.objects = []
        for temp_object in msg.objects:
            self.objects.append(temp_object.label)
        if "Person" in self.objects:
            if self.speed != 1:
                rospy.logwarn("Log: People in Front")
                request = ChangeSpeed()
                request = 0.3
                response = self.service_client(request)
                if response.result:
                    rospy.logwarn("Log: Slow Done")
                    self.speed = 1
        elif "Vehicle" in self.objects:
            if self.speed != 1:
                rospy.loginfo("Log: Vehicle in Front")
                request = ChangeSpeed()
                request = 0.4
                response = self.service_client(request)
                if response.result:
                    rospy.logwarn("Log: Slow Done")
                    self.speed = 1
        else:
            if self.speed != 2:
                rospy.logwarn("Log: Road Clear")
                request = ChangeSpeed()
                request = 0.5
                response = self.service_client(request)
                if response.result:
                    rospy.logwarn("Log: Speed Up")
                    self.speed = 2

    def run(self):
        rospy.init_node('autopilot_node')

        # Subscribe to topics
        rospy.Subscriber('map', Odometry, self.map_callback)
        rospy.Subscriber('obstacles', LineSegmentList, self.obstacles_callback)
        rospy.Subscriber('destination', dest_list_msg, self.destination_callback)
        rospy.Subscriber('objects', ObjectsStamped, self.object_callback)

        # Publish results
        path_publisher = rospy.Publisher('path', PoseStamped, queue_size=10)
        #msg_publisher = rospy.Publisher('emoji_message', String, queue_size=10)
            

        if test_mode:
            self.target_point = [test_x, test_y]
            self.start_point = self.curren_point
            self.status = True

        # 1hz
        rate = rospy.Rate(1)
        # main function
        while not rospy.is_shutdown():
            # start work if gps working and get order
            #if self.status and not self.lost_gps:
            if self.status and self.curren_point[0] != 0:
                rospy.loginfo("==================================================")
                self.obstacles = obstacles_convet(self.obstacles)
                path, done = self.node.get_next(self.obstacles, self.restricted_areas, self.curren_point, self.start_point, self.target_point)
                # check point
                if done:
                    path_publisher.publish(to_map_coordinate(self.target_point))
                    rospy.loginfo("speed: ")
                    rospy.loginfo([abs(abs(self.curren_point[0]) - abs(self.target_point[0])), abs(abs(self.curren_point[1]) - abs(self.target_point[1]))])
                    rospy.loginfo("next_point: ")
                    rospy.loginfo(self.target_point)
                    # not finish all point
                    if self.target_list:
                        self.target_point = self.target_list.pop(0)
                        self.start_point = self.curren_point
                        self.node.start()
                        rospy.loginfo("log: check point")
                    # finish all point
                    else:
                        self.status = False
                        self.target_point = []
                        rospy.loginfo("log: finish all point")
                # next pose
                elif path:
                    rospy.loginfo("speed: ")
                    rospy.loginfo([abs(self.curren_point[0] - path[0]), abs(self.curren_point[1] + path[1])])
                    rospy.loginfo("next_point: ")
                    rospy.loginfo(path)
                    path_publisher.publish(to_map_coordinate(path))
                # error
                else:
                    self.status = False
                    #msg_publisher.publish("error")
                    rospy.logwarn("error: cannot move")
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
