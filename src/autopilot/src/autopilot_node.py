#!/usr/bin/env python

import rospy
from laser_line_extraction.msg import LineSegment, LineSegmentList
from zed_interfaces.msg import ObjectsStamped, Object
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

    # 接收当前gps坐标
    def gps_callback(self, msg):
        self.curren_point[0] = msg.latitude
        self.curren_point[1] = msg.longitude
        if msg.status.status == 0:
            self.lost_gps = False
        else:
            self.lost_gps = True

    # 接收障碍物坐标的回调函数
    def obstacles_callback(self, msg):
        self.obstacles = []
        for line_segment in msg.line_segments:
            self.obstacles.append((line_segment.start, line_segment.end))

    # 接收物体识别信息
    def object_callback(self, msg):
        self.objects = []
        for temp_object in msg.objects:
            self.objects.append(temp_object)
        if "Person" in self.objects or "Vehicle" in self.objects:
            self.step_length = 1
        else:
            self.step_length = 2

    # 接收目标坐标指令
    def command_callback(self, msg):
        target_list = []
        for dest_point in msg.dest_list:
            target_list.append([dest_point.lat, dest_point.lng])
        self.target_point = target_list.pop(0)

    def autopilot_node(self):
        # 初始化节点
        rospy.init_node('path_planner_node')
        # 订阅障碍物坐标和当前坐标
        # /deli_robot/UbloxF9P/fix
        rospy.Subscriber('/gps', dest_list_msg, self.gps_callback)
        # /deli_robot/destList_array
        rospy.Subscriber('/destination', dest_list_msg, self.command_callback)
        # /deli_robot / line_segments
        rospy.Subscriber('/obstacles', LineSegmentList, self.obstacles_callback)
        # /deli_robot/zed_node/obj_det/objects
        rospy.Subscriber('/objects', ObjectsStamped, self.object_callback)
        # rospy.Subscriber('/lawn', ObjectsStamped, self.object_callback)

        # 发布路径规划结果
        # /deli_robot/set_pose
        path_publisher = rospy.Publisher('/path', Odometry, queue_size=10)

        # /deli_robot/emoji_message
        msg_publisher = rospy.Publisher('/emoji_message', String, queue_size=10)

        # 1hz
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            # 如果有任务并且gps信号正常
            if self.status and not self.lost_gps:
                temp_target_point = []
                temp_target_point[0] = self.target_point[0] - self.curren_point[0]
                temp_target_point[1] = self.target_point[1] - self.curren_point[1]
                path, done = self.node.get_next(self.obstacles, self.restricted_areas, self.step_length, temp_target_point)
                if done:
                    path_publisher.publish(self.target_point)
                    if self.target_list:
                        self.target_point = self.target_list.pop(0)
                        self.node.start()
                        print("log: check")
                    else:
                        self.status = False
                        print("log： done")
                elif path:
                    path[0] += self.curren_point[0]
                    path[1] += self.curren_point[1]
                    path_publisher.publish(path)
                    print("======")
                    print(path)
                else:
                    self.status = False
                    msg_publisher.publish("error")
                    print("error: cannot move")
            elif self.target_point:
                self.status = True
                self.node.start()
            rate.sleep()


if __name__ == '__main__':
    try:
        autopilot_node()
    except rospy.ROSInterruptException:
        pass
