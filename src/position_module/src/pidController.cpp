#include "pidController.h"
#include <fstream>

pidController::pidController(std::string pidConfigPath): m_pidConfigPath(pidConfigPath){
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

bool pidController::ReadPIDConfig(){
    std::ifstream configFile;
    Json::Value j_root, j_speedPid, j_angularPid;
    configFile.open(m_pidConfigPath);
    if(!configFile.is_open()){
        ROS_ERROR("Config File cannot Found");
        return false;
    }
    configFile >> j_root;
    if((j_speedPid = j_root["speedPid"]) == Json::Value::null){
        ROS_ERROR("Speed Pid Config Missing");
        configFile.close();
        return false;
    }
    m_speedPid.Kp = j_speedPid.get("Kp", 1.0).asFloat();
    m_speedPid.Ki = j_speedPid.get("Ki", 0.0).asFloat();
    m_speedPid.Kd = j_speedPid.get("Kd", 0.0).asFloat();
    m_speedPid.pre_error = 0;
    m_speedPid.cum_error = 0;
    if((j_angularPid = j_root["angularPid"]) == Json::Value::null){
        ROS_ERROR("Angular Pid Config Missing");
        configFile.close();
        return false;
    }
    m_angularPid.Kp = j_angularPid.get("Kp", 1.0).asFloat();
    m_angularPid.Ki = j_angularPid.get("Ki", 0.0).asFloat();
    m_angularPid.Kd = j_angularPid.get("Kd", 0.0).asFloat();
    m_angularPid.pre_error = 0;
    m_angularPid.cum_error = 0;
    configFile.close();
    return true;
}

bool pidController::SavePIDConfig(){
    std::ofstream configFile;
    Json::Value j_root, j_speedPid, j_angularPid;
    configFile.open(m_pidConfigPath);
    if(!configFile.is_open()){
        ROS_ERROR("Config File cannot Found");
        return false;
    }
    j_speedPid["Kp"] = m_speedPid.Kp;
    j_speedPid["Ki"] = m_speedPid.Ki;
    j_speedPid["Kd"] = m_speedPid.Kd;
    j_root["speedPid"] = j_speedPid;

    j_angularPid["Kp"] = m_angularPid.Kp;
    j_angularPid["Ki"] = m_angularPid.Ki;
    j_angularPid["Kd"] = m_angularPid.Kd;
    j_root["angularPid"] = j_angularPid;

    configFile << j_root;
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
    speed_error = m_robotControlMsg.linear.x - currlinSpeed;
    m_robotProcessMsg.linear.x = m_speedPid.Kp*speed_error;
    // Turn the Robot Speed into absoulate speed and retain the Z angler speed only
    // ROS_INFO("Current Speed: %.2f, %f", currlinSpeed, m_robotControlMsg.angular.z);
    // Compare the current speed with the Input Twist
    return m_robotProcessMsg;
}

bool pidController::UpdateAngularKp( robot_msgs::UpdateAngularKp::Request &req,
                                     robot_msgs::UpdateAngularKp::Response &res ){
    ROS_DEBUG("Change Angular Kp From %f to %f", m_angularPid.Kp, req.value);
    m_angularPid.Kp = req.value;
    SavePIDConfig();
}

bool pidController::UpdateAngularKi( robot_msgs::UpdateAngularKi::Request &req,
                                     robot_msgs::UpdateAngularKi::Response &res ){
    ROS_DEBUG("Change Angular Ki From %f to %f", m_angularPid.Ki, req.value);
    m_angularPid.Ki = req.value;
    SavePIDConfig();
}

bool pidController::UpdateAngularKd( robot_msgs::UpdateAngularKd::Request &req,
                                     robot_msgs::UpdateAngularKd::Response &res ){
    ROS_DEBUG("Change Angular Kd From %f to %f", m_angularPid.Kd, req.value);
    m_angularPid.Kd = req.value;
    SavePIDConfig();
}

bool pidController::UpdateSpeedKp( robot_msgs::UpdateSpeedKp::Request &req,
                                   robot_msgs::UpdateSpeedKp::Response &res ){
    ROS_DEBUG("Change Speed Kp From %f to %f", m_speedPid.Kp, req.value);
    m_speedPid.Kp = req.value;
    SavePIDConfig();
}

bool pidController::UpdateSpeedKi( robot_msgs::UpdateSpeedKi::Request &req,
                                   robot_msgs::UpdateSpeedKi::Response &res ){
    ROS_DEBUG("Change Speed Ki From %f to %f", m_speedPid.Ki, req.value);
    m_speedPid.Ki = req.value;
    SavePIDConfig();
}

bool pidController::UpdateSpeedKd( robot_msgs::UpdateSpeedKd::Request &req,
                                   robot_msgs::UpdateSpeedKd::Response &res ){
    ROS_DEBUG("Change Speed Kd From %f to %f", m_speedPid.Kd, req.value);
    m_speedPid.Kd = req.value;
    SavePIDConfig();
}