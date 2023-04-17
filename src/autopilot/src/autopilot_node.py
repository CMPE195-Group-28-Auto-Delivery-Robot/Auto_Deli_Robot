#!/usr/bin/env python

import rospy
from tf2_ros import TransformListener
from laser_line_extraction.msg import LineSegment, LineSegmentList
from zed_interfaces.msg import ObjectsStamped, Object
from geometry_msgs.msg import Quaternion, Point, Pose
from robot_msgs.msg import dest_list_msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header

import autopilot

obstacles = []
goal_position = []
current_position = []


class autopilot_node:

    def __init__(self):
        self.status = False
        self.target_list = []
        self.target_point = []
        self.curren_point = []
        self.objects = []
        self.obstacles = []
        self.restricted_areas = []
        self.step_length = 2
        self.lost_gps = False
        self.node = autopilot.autopilot()
        self.local_point = Point()
        self.run()

    # Callback function to receive the current gps coordinates
    def gps_callback(self, msg):
        self.curren_point[0] = msg.latitude
        self.curren_point[1] = msg.longitude
        if msg.status.status == 0:
            self.lost_gps = False
        else:
            self.lost_gps = True

    # Callback function to receive the coordinates of the obstacle
    def obstacles_callback(self, msg):

        gps_point = Point()
        gps_point.x = msg.latitude
        gps_point.y = msg.longitude
        gps_point.z = msg.altitude

        print(gps_point)

        transform_listener = TransformListener(rospy.Duration(1))
        transform = transform_listener.lookup_transform("map", "base_link", rospy.Time())
        self.local_point = do_transform_point(gps_point, transform)


        '''
        self.obstacles = []
        for line_segment in msg.line_segments:
            self.obstacles.append((line_segment.start, line_segment.end))
        '''

    # Callback function to receive object recognition information
    def object_callback(self, msg):
        self.objects = []
        for temp_object in msg.objects:
            self.objects.append(temp_object)
        if "Person" in self.objects or "Vehicle" in self.objects:
            self.step_length = 1
        else:
            self.step_length = 2

    # Callback function to receive target coordinates command
    def command_callback(self, msg):
        target_list = []
        for dest_point in msg.dest_list:
            target_list.append([dest_point.lat, dest_point.lng])
        self.target_point = target_list.pop(0)

    def run(self):
        rospy.init_node('autopilot_node')
        # Subscribe to topics
        rospy.Subscriber('/gps', dest_list_msg, self.gps_callback)
        rospy.Subscriber('/destination', dest_list_msg, self.command_callback)
        rospy.Subscriber('/obstacles', LineSegmentList, self.obstacles_callback)
        rospy.Subscriber('/objects', ObjectsStamped, self.object_callback)
        # rospy.Subscriber('/lawn', ObjectsStamped, self.object_callback)

        # Publish results
        # /deli_robot/set_pose
        path_publisher = rospy.Publisher('/path', Odometry, queue_size=10)

        # /deli_robot/emoji_message
        msg_publisher = rospy.Publisher('/emoji_message', String, queue_size=10)

        test1 = rospy.Publisher('/test1', Point, queue_size=10)
        
        # 1hz
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            test1.publish(self.local_point)
            rate.sleep()
        '''

        # main function
        while not rospy.is_shutdown():
            # start work if gps working and get order
            if self.status and not self.lost_gps:
                temp_target_point = []
                temp_target_point[0] = self.target_point[0] - self.curren_point[0]
                temp_target_point[1] = self.target_point[1] - self.curren_point[1]
                path, done = self.node.get_next(self.obstacles, self.restricted_areas, self.step_length, temp_target_point)
                # check point
                if done:
                    path_publisher.publish(self.target_point)
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
                    path_publisher.publish(path)
                    print("======")
                    print(path)
                # error
                else:
                    self.status = False
                    msg_publisher.publish("error")
                    print("error: cannot move")
            # get order
            elif self.target_point:
                self.status = True
                self.node.start()
            rate.sleep()
        '''

if __name__ == '__main__':
    try:
        autopilot_node()
    except rospy.ROSInterruptException:
        pass
