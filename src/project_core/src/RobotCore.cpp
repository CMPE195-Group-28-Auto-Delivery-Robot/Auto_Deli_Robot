#include <cmath>
#include "ros/ros.h"
#include "zed_interfaces/ObjectsStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#define RAD2DEG(x) ((x)*180./M_PI)

class infoCollector{
private:
    zed_interfaces::ObjectsStamped m_objectDetectionMsg;
    nav_msgs::Odometry m_robotOdometryMsg;
    const float Kp = 1;
    const float Ki = 1;
    const float Kd = 1;

public:
    infoCollector(){

    }

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        m_robotOdometryMsg = *(odomMsg.get());
    }

    void ObjectCallback(const zed_interfaces::ObjectsStamped::ConstPtr& objectMsg)
    {
        m_objectDetectionMsg = *(objectMsg.get());
    }

    geometry_msgs::PoseStamped GetProcessdMsg(){
        geometry_msgs::PoseStamped nextGoal;
        nextGoal.header.stamp = ros::Time::now();
        nextGoal.pose.position.x = 1;
        nextGoal.pose.position.y = 1;
        nextGoal.pose.position.z = 1;
        return nextGoal;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Robot_Core_Node");

    ROS_INFO("Core System Start Running");
    ros::NodeHandle rosHandle;

    infoCollector controlNode;
    
    ros::Publisher cmdVel_pub = rosHandle.advertise<geometry_msgs::PoseStamped>("goalPosition", 1000);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("/deli_robot/odometry/map", 1000, &infoCollector::OdomCallback, &controlNode);
    ros::Subscriber control_sub = rosHandle.subscribe<zed_interfaces::ObjectsStamped>("/deli_robot/zed_node/obj_det/objects", 1000, &infoCollector::ObjectCallback, &controlNode);

    ros::Rate loop_rate(5);

    while(ros::ok()){
        cmdVel_pub.publish(controlNode.GetProcessdMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 
