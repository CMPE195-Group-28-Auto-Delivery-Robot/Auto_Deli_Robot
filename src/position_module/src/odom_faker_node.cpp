// Originally located at:
// http://controls.ece.unm.edu/documents/gpsTest.cpp
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "robot_msgs/dest_list_msg.h"
#include "geometry_msgs/PoseStamped.h"

nav_msgs::Odometry odom;

void OdomCallback(const geometry_msgs::PoseStamped::ConstPtr &odomMsg){
    geometry_msgs::PoseStamped m_robotOdometryMsg = *(odomMsg.get());
    odom.pose.pose.position.x = (double)m_robotOdometryMsg.pose.position.x;
    odom.pose.pose.position.y = (double)m_robotOdometryMsg.pose.position.y;
}

int main(int argc, char** argv)
{
    odom.pose.pose.position.x = 50;
    odom.pose.pose.position.y = 50;
    ros::init(argc, argv, "odom_faker");
    ros::NodeHandle n;
    ros::Subscriber fakeOdomSus = n.subscribe<geometry_msgs::PoseStamped>("nextStep", 1000, &OdomCallback);
    ros::Publisher fakeGPSpub = n.advertise<nav_msgs::Odometry>("currentPosition", 1000);

    ros::Rate loop_rate(5);

    while(ros::ok()){
        fakeGPSpub.publish(odom);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}