#include "pidController.h"
#include <fstream>

pidController::pidController(){
}

void pidController::OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    m_robotOdometryMsg = *(odomMsg.get());
}

void pidController::ControlCallback(const geometry_msgs::Twist::ConstPtr& controlMsg)
{
    m_robotControlMsg = *(controlMsg.get());
}

void pidController::TargetCallback(const geometry_msgs::PoseStamped::ConstPtr& tagMsg)
{
    m_robotTargetPoseMsg = *(tagMsg.get());
}

bool pidController::ReadPIDConfig(std::string pidConfigPath){
    std::ifstream configFile;
    Json::Value j_root, j_speedPid, j_angularPid;
    configFile.open(pidConfigPath);
    configFile >> j_root;
    if((j_speedPid = j_root["speedPid"]) == Json::Value::null){
        ROS_ERROR("Speed Pid Config Missing");
        configFile.close();
        return false;
    }
    m_speedPid.Kp = j_speedPid.get("Kp", 1.0).asFloat();
    m_speedPid.Ki = j_speedPid.get("Ki", 0.0).asFloat();
    m_speedPid.Kd = j_speedPid.get("Kd", 0.0).asFloat();
    if((j_angularPid = j_root["angularPid"]) == Json::Value::null){
        ROS_ERROR("Angular Pid Config Missing");
        configFile.close();
        return false;
    }
    m_angularPid.Kp = j_angularPid.get("Kp", 1.0).asFloat();
    m_angularPid.Ki = j_angularPid.get("Ki", 0.0).asFloat();
    m_angularPid.Kd = j_angularPid.get("Kd", 0.0).asFloat();
    configFile.close();
    return true;
}

geometry_msgs::Twist pidController::GetProcessdMsg(){
    float headAngle, speedAngle, speed_error;
    geometry_msgs::Twist m_robotProcessMsg;
    geometry_msgs::Twist currRobotTwist;
    geometry_msgs::Pose currRobotPose;

    currRobotTwist = m_robotOdometryMsg.twist.twist;
    currRobotPose = m_robotOdometryMsg.pose.pose;
    double currentOrien, currlinSpeed, currAngSpeed;
    
    currentOrien = sqrt(pow(currRobotPose.orientation.x,2)+pow(currRobotPose.orientation.y,2));
    currlinSpeed = sqrt(pow(currRobotTwist.linear.x,2)+pow(currRobotTwist.linear.y,2));
    m_robotProcessMsg.angular.z = m_robotControlMsg.angular.z;

    headAngle = atan2(currRobotPose.orientation.y, currRobotPose.orientation.x);
    speedAngle = atan2(currRobotTwist.linear.y, currRobotTwist.linear.x);
    if(abs((speedAngle * 180)/M_PI)>90){
        currlinSpeed *= -1;
    }
    speed_error = m_robotControlMsg.linear.x - currAngSpeed;
    m_robotProcessMsg.linear.x = m_speedPid.Kp*speed_error;
    // Turn the Robot Speed into absoulate speed and retain the Z angler speed only
    // ROS_INFO("Current Speed: %.2f, %f", currlinSpeed, m_robotControlMsg.angular.z);
    // Compare the current speed with the Input Twist
    return m_robotProcessMsg;
}