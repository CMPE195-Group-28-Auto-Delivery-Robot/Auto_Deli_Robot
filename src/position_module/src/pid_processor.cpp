#include <cmath>
#include "ros/ros.h"
#include "pidController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Robot_PID_Node");
    ros::NodeHandle rosHandle;
    std::string pidConfigPath;

    if(!rosHandle.getParam(ros::this_node::getName()+"/configPath",pidConfigPath)){
        ROS_ERROR("Config File Parameter Missing");
        return -1;
    }
    
    pidController pidNode(pidConfigPath);

    if(!pidNode.ReadPIDConfig()){
        return -1;
    }
    
    ros::Publisher cmdVel_pub = rosHandle.advertise<geometry_msgs::Twist>("controlVel", 1000);
    ros::Publisher speed_pub = rosHandle.advertise<std_msgs::Float32>("robotSpeed", 1000);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("odometry/map", 1000, &pidController::OdomCallback, &pidNode);
    ros::Subscriber control_sub = rosHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &pidController::ControlCallback, &pidNode);
    ros::Subscriber pose_sub = rosHandle.subscribe<geometry_msgs::PoseStamped>("goalPosition", 1000, &pidController::TargetCallback, &pidNode);
    ros::ServiceServer serviceakp = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateAngularKp", &pidController::UpdateAngularKp, &pidNode);
    ros::ServiceServer serviceaki = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateAngularKi", &pidController::UpdateAngularKi, &pidNode);
    ros::ServiceServer serviceakd = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateAngularKd", &pidController::UpdateAngularKd, &pidNode);
    ros::ServiceServer serviceskp = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateSpeedKp", &pidController::UpdateSpeedKp, &pidNode);
    ros::ServiceServer serviceski = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateSpeedKi", &pidController::UpdateSpeedKi, &pidNode);
    ros::ServiceServer serviceskd = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateSpeedKd", &pidController::UpdateSpeedKd, &pidNode);
    ros::ServiceServer servicedkp = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateDistanceKp", &pidController::UpdateDistanceKp, &pidNode);
    ros::ServiceServer servicedki = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateDistanceKi", &pidController::UpdateDistanceKi, &pidNode);
    ros::ServiceServer servicedkd = rosHandle.advertiseService(ros::this_node::getName()+"/UpdateDistanceKd", &pidController::UpdateDistanceKd, &pidNode);


    ros::Rate loop_rate(15);
    ROS_INFO("PID Node Started");
    while(ros::ok()){
        ros::spinOnce();
        cmdVel_pub.publish(pidNode.GetSpeedCtrlMsg());
        speed_pub.publish(pidNode.GetCurrSpeed());
        loop_rate.sleep();
    }
    ROS_INFO("PID Node Ended");
    return 0;
} 
