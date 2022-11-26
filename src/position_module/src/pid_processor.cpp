#include <cmath>
#include "ros/ros.h"
#include "pidController.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Robot_PID_Node");
    ros::NodeHandle rosHandle;
    pidController pidNode;
    
    ros::Publisher cmdVel_pub = rosHandle.advertise<geometry_msgs::Twist>("controlVel", 1000);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("/deli_robot/odometry/map", 1000, &pidController::OdomCallback, &pidNode);
    ros::Subscriber control_sub = rosHandle.subscribe<geometry_msgs::Twist>("/deli_robot/cmd_vel", 1000, &pidController::ControlCallback, &pidNode);
    ros::Subscriber pose_sub = rosHandle.subscribe<geometry_msgs::PoseStamped>("/deli_robot/goalPosition", 1000, &pidController::TargetCallback, &pidNode);

    ros::Rate loop_rate(15);

    while(ros::ok()){
        cmdVel_pub.publish(pidNode.GetProcessdMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 
