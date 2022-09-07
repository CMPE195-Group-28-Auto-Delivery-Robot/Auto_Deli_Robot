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
    sensor_msgs::Imu m_robotImuMsg;
    nav_msgs::Odometry m_robotOdometryMsg;

public:
    pidController(){

    }

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        m_robotOdometryMsg = *(odomMsg.get());
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        m_robotImuMsg = *(imuMsg.get());
    }

    void ControlCallback(const geometry_msgs::Twist::ConstPtr& controlMsg)
    {
        m_robotControlMsg = *(controlMsg.get());
    }

    geometry_msgs::Twist GetProcessdMsg(){
        ROS_INFO("Speed: %.5f, %.5f, %.5f, %.5f", m_robotImuMsg.linear_acceleration.x, m_robotImuMsg.angular_velocity.z, m_robotOdometryMsg.twist.twist.linear.x, m_robotOdometryMsg.twist.twist.angular.z);
        return m_robotProcessMsg;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Robot_PID_Node");
    ros::NodeHandle rosHandle;

    pidController pidNode;
    
    ros::Publisher cmdVel_pub = rosHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("zed_node/odom", 1000, &pidController::OdomCallback, &pidNode);
    ros::Subscriber imu_sub = rosHandle.subscribe<sensor_msgs::Imu>("zed_node/imu/data", 1000, &pidController::IMUCallback, &pidNode);
    ros::Subscriber control_sub = rosHandle.subscribe<geometry_msgs::Twist>("controlVel", 1000, &pidController::ControlCallback, &pidNode);

    ros::Rate loop_rate(15);

    while(ros::ok()){
        cmdVel_pub.publish(pidNode.GetProcessdMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 
