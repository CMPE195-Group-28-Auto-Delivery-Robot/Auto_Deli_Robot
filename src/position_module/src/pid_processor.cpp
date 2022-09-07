#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#define RAD2DEG(x) ((x)*180./M_PI)

class pidController{
private:
    geometry_msgs::Twist m_robotProcessMsg;
    geometry_msgs::Twist m_robotControlMsg;
    nav_msgs::Odometry m_robotOdometryMsg;

public:
    pidController(){

    }

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        m_robotOdometryMsg = *(odomMsg.get());
    }

    void ControlCallback(const geometry_msgs::Twist::ConstPtr& controlMsg)
    {
        m_robotControlMsg = *(controlMsg.get());
    }

    geometry_msgs::Twist GetProcessdMsg(){
        ROS_INFO("Speed: %.5f, %.5f, %.5f, %.5f", m_robotControlMsg.linear.x, m_robotControlMsg.angular.z, m_robotOdometryMsg.twist.twist.linear.x, m_robotOdometryMsg.twist.twist.angular.z);
        return m_robotProcessMsg;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Robot_PID_Node");
    ros::NodeHandle rosHandle;

    pidController pidNode;
    
    ros::Publisher cmdVel_pub = rosHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("/zed_node/odom", 1000, &pidController::OdomCallback, &pidNode);

    ros::Rate loop_rate(15);

    while(ros::ok()){
        cmdVel_pub.publish(pidNode.GetProcessdMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 
