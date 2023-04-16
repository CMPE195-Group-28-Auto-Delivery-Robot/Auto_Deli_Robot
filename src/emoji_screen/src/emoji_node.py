#!/usr/bin/env python

import rospy
import threading
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import emoji


class emoji_node:

    def __init__(self):
        rospy.init_node('emoji_node')
        rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/test', String, self.msg_callback)
        self.screen = emoji.emoji()
        self.thread = threading.Thread(target=self.screen.run())
        self.thread.start()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def gps_callback(self, msg):
        if msg.status.status == 0:
            if self.screen.check_order() != 4:
                self.screen.add_order(4)
                #print("lost gps")
        else:
            if self.screen.check_order() != 0:
                self.screen.add_order(0)
                #print("get gps")

    def msg_callback(self, msg):
        if msg.data == "a":
            if self.screen.check_order() != 3:
                self.screen.add_order(3)
                #print("error")
        elif msg.data == "b":
            if self.screen.check_order() != 0:
                self.screen.add_order(0)
                #print("pass")
        elif msg.data == "c":
            self.screen.add_order(5)
            #print("end")


if __name__ == '__main__':
    try:
        emoji_node()
    except rospy.ROSInterruptException:
        pass
