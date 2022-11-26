#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "json.h"
#pragma once

class pidController{
private:
    typedef struct{
        float Kp;
        float Ki;
        float Kd;
    } pidParam;
    geometry_msgs::Twist m_robotControlMsg;
    nav_msgs::Odometry m_robotOdometryMsg;
    geometry_msgs::PoseStamped m_robotTargetPoseMsg;
    pidParam speedPid;
    pidParam anglePid;

public:
    pidController();
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void ControlCallback(const geometry_msgs::Twist::ConstPtr& controlMsg);
    void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr& tagMsg);
    geometry_msgs::Twist GetProcessdMsg();
};