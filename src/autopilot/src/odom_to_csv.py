#!/usr/bin/env python

import rospy
import pandas as pd
from nav_msgs.msg import Odometry

class OdomToCSV():
    def __init__(self):
        rospy.init_node('odom_to_csv', anonymous=True)
        rospy.Subscriber('odom', Odometry, self.callback)
        self.df = pd.DataFrame(columns=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])

    def callback(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        euler_angles = self.quaternion_to_euler(orientation)
        row = {'x': position.x, 'y': position.y, 'z': position.z, 
               'roll': euler_angles[0], 'pitch': euler_angles[1], 'yaw': euler_angles[2]}
        self.df = self.df.append(row, ignore_index=True)

    def quaternion_to_euler(self, quat):
        import math
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        roll = math.atan2(2*(w*x + y*z), 1-2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
        return [roll, pitch, yaw]

    def save_to_csv(self):
        filename = "~/odom.csv"
        print(filename)
        self.df.to_csv(filename, index=False)

if __name__ == '__main__':
    try:
        odom_to_csv = OdomToCSV()
        rospy.on_shutdown(odom_to_csv.save_to_csv)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
