#include <chrono>

#include <jsoncpp/json/json.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

// Custom msgs and srvs
#include "robot_msgs/ChangeOpMode.h"
#include "robot_msgs/GoHome.h"
#include "robot_msgs/UpdateAngularKp.h"
#include "robot_msgs/UpdateAngularKi.h"
#include "robot_msgs/UpdateAngularKd.h"
#include "robot_msgs/UpdateSpeedKp.h"
#include "robot_msgs/UpdateSpeedKi.h"
#include "robot_msgs/UpdateSpeedKd.h"
#include "robot_msgs/UpdateDistanceKp.h"
#include "robot_msgs/UpdateDistanceKi.h"
#include "robot_msgs/UpdateDistanceKd.h"

// Custom Libirary
#include "pidAlgorithm/pidRegulater.h"

#pragma once

class pidController{
private:
    bool m_opMode; // If true PID Mode, else Direct Mode
    bool m_goalSet;
    float m_currspeed;
    float m_arrivalRange;
    geometry_msgs::Twist m_robotControlMsg;
    nav_msgs::Odometry m_robotOdometryMsg;
    geometry_msgs::PoseStamped m_robotTargetPoseMsg;
    pidRegulater m_distancePid;
    pidRegulater m_speedPid;
    pidRegulater m_angularPid;
    std::string m_pidConfigPath;
    ros::Time m_lastCmdRecevied;
    float AngleDiff(float ax, float ay,
                    float bx, float by);

public:
    pidController(std::string pidConfigPath, float range);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void ControlCallback(const geometry_msgs::Twist::ConstPtr& controlMsg);
    void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr& tagMsg);
    bool ReadPIDConfig();
    bool SavePIDConfig();

    geometry_msgs::Twist GetSpeedCtrlMsg();
    std_msgs::Float32 GetCurrSpeed();
    bool IsGoalSet();
    
    bool ChangeOpMode( robot_msgs::ChangeOpMode::Request &req,
                       robot_msgs::ChangeOpMode::Response &res );
    bool GoHome( robot_msgs::GoHome::Request &req,
                 robot_msgs::GoHome::Response &res );
    bool UpdateAngularKp( robot_msgs::UpdateAngularKp::Request &req,
                          robot_msgs::UpdateAngularKp::Response &res );
    bool UpdateAngularKi( robot_msgs::UpdateAngularKi::Request &req,
                          robot_msgs::UpdateAngularKi::Response &res );
    bool UpdateAngularKd( robot_msgs::UpdateAngularKd::Request &req,
                          robot_msgs::UpdateAngularKd::Response &res );
    bool UpdateSpeedKp( robot_msgs::UpdateSpeedKp::Request &req,
                        robot_msgs::UpdateSpeedKp::Response &res );
    bool UpdateSpeedKi( robot_msgs::UpdateSpeedKi::Request &req,
                        robot_msgs::UpdateSpeedKi::Response &res );
    bool UpdateSpeedKd( robot_msgs::UpdateSpeedKd::Request &req,
                        robot_msgs::UpdateSpeedKd::Response &res );
    bool UpdateDistanceKp( robot_msgs::UpdateDistanceKp::Request &req,
                           robot_msgs::UpdateDistanceKp::Response &res );
    bool UpdateDistanceKi( robot_msgs::UpdateDistanceKi::Request &req,
                           robot_msgs::UpdateDistanceKi::Response &res );
    bool UpdateDistanceKd( robot_msgs::UpdateDistanceKd::Request &req,
                           robot_msgs::UpdateDistanceKd::Response &res );

};