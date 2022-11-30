// Originally located at:
// http://controls.ece.unm.edu/documents/gpsTest.cpp
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "robot_msgs/dest_list_msg.h"
#include "robot_msgs/destPoint_msg.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/LineSegment.h"

robot_msgs::dest_list_msg FakePath(){
    robot_msgs::dest_list_msg fakeMsg;
    robot_msgs::destPoint_msg path1;
    path1.lat = 60;
    path1.lng = 75;
    robot_msgs::destPoint_msg path2;
    path2.lat = 35;
    path2.lng = 50;
    fakeMsg.dest_list.push_back(path1);
    fakeMsg.dest_list.push_back(path2);
    return fakeMsg;

}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "path_faker");
    ros::NodeHandle n;

    ros::Publisher fakeObspub = n.advertise<robot_msgs::dest_list_msg>("destList_array",1000);

    ros::Rate loop_rate(5);
    while(ros::ok()){
        fakeObspub.publish(FakePath());
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}