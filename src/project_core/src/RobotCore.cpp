#include <cmath>
#include "ros/ros.h"
#include "infoProcessor.h"



#define RAD2DEG(x) ((x)*180./M_PI)

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "Robot_Core_Node");

    ROS_INFO("Core System Start Running");
    ros::NodeHandle rosHandle;
    int avoid_angle;
    int move_distance;
    int search_distance;
    float min_reach;
    rosHandle.param(ros::this_node::getName()+"/avoid_angle", avoid_angle, 20);
    rosHandle.param(ros::this_node::getName()+"/move_distance", move_distance, 10);
    rosHandle.param(ros::this_node::getName()+"/search_distance", search_distance, 20);
    rosHandle.param(ros::this_node::getName()+"/min_reach_distance", min_reach, (float)5.0);

    infoProcessor controlNode(avoid_angle, move_distance, search_distance, min_reach);
    ROS_INFO("Object created");
    ros::Publisher stepPosition_pub = rosHandle.advertise<geometry_msgs::PoseStamped>("goalPosition", 1);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("odometry/map", 1, &infoProcessor::OdomCallback, &controlNode);
    ros::Subscriber obstacle_sub = rosHandle.subscribe<zed_interfaces::ObjectsStamped>("zed_node/obj_det/objects", 1, &infoProcessor::ObjectCallback, &controlNode);
    ros::Subscriber line_extra_sub = rosHandle.subscribe<laser_line_extraction::LineSegmentList>("line_segments", 1, &infoProcessor::LineExtraCallback, &controlNode);
    ros::Subscriber goal_position_sub = rosHandle.subscribe<robot_msgs::dest_list_msg>("destList_array", 1, &infoProcessor::DestListCallback, &controlNode);
    ros::Rate loop_rate(1);
    while(ros::ok()){
        ros::spinOnce();
        if(controlNode.isInitialized()){
            if(!controlNode.isEnd()){
                stepPosition_pub.publish(controlNode.getNextStep());     
            }   
        }
        loop_rate.sleep();
    }

    return 0;
} 
