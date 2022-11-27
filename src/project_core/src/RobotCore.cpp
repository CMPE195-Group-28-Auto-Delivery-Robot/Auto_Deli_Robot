#include <cmath>
#include "ros/ros.h"
#include "infoProcessor.h"


#define RAD2DEG(x) ((x)*180./M_PI)

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "Robot_Core_Node");

    ROS_INFO("Core System Start Running");
    ros::NodeHandle rosHandle;



    infoProcessor controlNode(20, 10, 20);
    
    ros::Publisher goalPosition_pub = rosHandle.advertise<geometry_msgs::PoseStamped>("goalPosition", 1000);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("odometry/map", 1000, &infoProcessor::OdomCallback, &controlNode);
    ros::Subscriber obstacle_sub = rosHandle.subscribe<zed_interfaces::ObjectsStamped>("zed_node/obj_det/objects", 1000, &infoProcessor::ObjectCallback, &controlNode);
    ros::Subscriber line_extra_sub = rosHandle.subscribe<laser_line_extraction::LineSegmentList>("line_segments", 1000, &infoProcessor::LineExtraCallback, &controlNode);
    ros::Subscriber goal_position_sub = rosHandle.subscribe<geometry_msgs::PoseStamped>("destList_array", 1000, &infoProcessor::GoalPositionCallback, &controlNode);
    ros::Rate loop_rate(5);

    while(ros::ok()){
        goalPosition_pub.publish(controlNode.getTarget());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 
