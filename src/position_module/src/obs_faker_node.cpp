#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/LineSegment.h"

laser_line_extraction::LineSegmentList FakeObstacle(){
    laser_line_extraction::LineSegmentList fakeObsList;
    laser_line_extraction::LineSegment line1;
    line1.start = {40,70};
    line1.end = {55,65};
    laser_line_extraction::LineSegment line2;
    line2.start = {40,41};
    line2.end = {40,60};
    fakeObsList.line_segments.push_back(line1);
    fakeObsList.line_segments.push_back(line2);
    
    //ROS_INFO("Fake GPS: %f, %f", fakeMsg.latitude, fakeMsg.longitude);
    return fakeObsList;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "obs_faker");
    ros::NodeHandle n;

    ros::Publisher fakeObspub = n.advertise<laser_line_extraction::LineSegmentList>("line_segments",1000);
    
    ros::Rate loop_rate(5);
    while(ros::ok()){
        fakeObspub.publish(FakeObstacle());
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}