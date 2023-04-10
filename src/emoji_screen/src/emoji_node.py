#!/usr/bin/env python

import rospy
import threading
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import emoji


class emoji_node:
    def __init__(self):
        rospy.init_node('emoji')
        thread = threading.Thread(target=emoji.run())
        thread.start()
        rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/test', String, self.msg_callback)

    def gps_callback(self, msg):
        if msg.position_covariance_type == 0:
            if emoji.check_order != 4:
                emoji.add_order(4)
                print("lost gps")
        else:
            if emoji.check_order == 4:
                emoji.add_order(0)
                print("get gps")

    def msg_callback(self, msg):
        if msg.data == "a":
            if emoji.check_order != 3:
                emoji.add_order(3)
                print("error")
        elif msg.data == "b":
            if emoji.check_order == 3:
                emoji.add_order(0)
                print("pass")


if __name__ == '__main__':
    try:
        emoji_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
