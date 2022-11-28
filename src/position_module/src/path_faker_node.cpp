// Originally located at:
// http://controls.ece.unm.edu/documents/gpsTest.cpp
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "robot_msgs/dest_list_msg.h"
#include "robot_msgs/destPoint_msg.h"

robot_msgs::dest_list_msg FakePath(){
    robot_msgs::dest_list_msg fakeMsg;
    robot_msgs::destPoint_msg path1;
    path1.lat = 50;
    path1.lng = 50;
    robot_msgs::destPoint_msg path2;
    path2.lat = 60;
    path2.lng = 60;
    fakeMsg.dest_list.push_back(path1);
    fakeMsg.dest_list.push_back(path2);

    
    //ROS_INFO("Fake GPS: %f, %f", fakeMsg.latitude, fakeMsg.longitude);
    return fakeMsg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_faker");
    ros::NodeHandle n;

    ros::Publisher fakeGPSpub = n.advertise<robot_msgs::dest_list_msg>("destList_array",1000);

    ros::Rate loop_rate(1);

    while(ros::ok()){
        fakeGPSpub.publish(FakePath());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}