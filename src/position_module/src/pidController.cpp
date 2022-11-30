#include "pidController.h"
#include <fstream>

pidController::pidController(std::string pidConfigPath, float range): 
    m_pidConfigPath(pidConfigPath), m_arrivalRange(range){
    m_goalSet = false;
    m_opMode = true;
}

void pidController::OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    m_robotOdometryMsg = *(odomMsg.get());
}

void pidController::ControlCallback(const geometry_msgs::Twist::ConstPtr& controlMsg)
{
    m_lastCmdRecevied = ros::Time::now();
    m_robotControlMsg = *(controlMsg.get());
}

void pidController::TargetCallback(const geometry_msgs::PoseStamped::ConstPtr& tagMsg)
{
    m_robotTargetPoseMsg = *(tagMsg.get());
}

bool pidController::IsGoalSet(){
    return m_goalSet;
}

bool pidController::ReadPIDConfig(){
    std::ifstream configFile;
    Json::Value j_root, j_speedPid, j_angularPid, j_distancePid;
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
    m_speedPid.setKp(j_speedPid.get("Kp", 1.0).asFloat());
    m_speedPid.setKi(j_speedPid.get("Ki", 0.0).asFloat());
    m_speedPid.setKd(j_speedPid.get("Kd", 0.0).asFloat());
    if((j_angularPid = j_root["angularPid"]) == Json::Value::null){
        ROS_ERROR("Angular Pid Config Missing");
        configFile.close();
        return false;
    }
    m_angularPid.setKp(j_angularPid.get("Kp", 1.0).asFloat());
    m_angularPid.setKi(j_angularPid.get("Ki", 0.0).asFloat());
    m_angularPid.setKd(j_angularPid.get("Kd", 0.0).asFloat());
    configFile.close();
    return true;
}

bool pidController::SavePIDConfig(){
    std::ofstream configFile;
    Json::Value j_root, j_speedPid, j_angularPid, j_distancePid;
    configFile.open(m_pidConfigPath);
    m_speedPid.Clear();
    m_angularPid.Clear();
    if(!configFile.is_open()){
        ROS_ERROR("Config File cannot Found");
        return false;
    }
    j_speedPid["Kp"] = m_speedPid.getKp();
    j_speedPid["Ki"] = m_speedPid.getKi();
    j_speedPid["Kd"] = m_speedPid.getKd();
    j_root["speedPid"] = j_speedPid;

    j_angularPid["Kp"] = m_angularPid.getKp();
    j_angularPid["Ki"] = m_angularPid.getKi();
    j_angularPid["Kd"] = m_angularPid.getKd();
    j_root["angularPid"] = j_angularPid;

    configFile << j_root;
    configFile.close();
    return true;
}

float pidController::AngleDiff(float ax, float ay,
                               float bx, float by){
    float det, dot;
    dot = ax*bx + ay*by;
    det = ax*by - ay*bx;
    return atan2(det, dot);
}

geometry_msgs::Twist pidController::GetSpeedCtrlMsg(){
    float goaldist, angleDiff;
    geometry_msgs::Twist robotProcessMsg;
    geometry_msgs::Twist currRobotTwist;
    geometry_msgs::Pose currRobotPose;
    ros::Time currTime = ros::Time::now();

    currRobotPose = m_robotOdometryMsg.pose.pose;
    currRobotTwist = m_robotOdometryMsg.twist.twist;
    
    m_currspeed = sqrt(pow(currRobotTwist.linear.x,2)+pow(currRobotTwist.linear.y,2)); // Calculate CurrSpeed
    if(abs(atan2(currRobotTwist.linear.y,currRobotTwist.linear.x))>(M_PI/2)){ // if the difference between two angle is greater than 90 degree it is going back 
        m_currspeed *= -1;
    }

    if(IsGoalSet() && (currTime-m_lastCmdRecevied).toSec()>5){
        // ROS_INFO("Node In Control");
        float xDiff, yDiff;
        xDiff = currRobotPose.position.x - m_robotTargetPoseMsg.pose.position.x;
        yDiff = currRobotPose.position.y - m_robotTargetPoseMsg.pose.position.y;
        goaldist = sqrt(pow(xDiff,2) + pow(yDiff,2));
        if(goaldist < m_arrivalRange){
            ROS_INFO("Goal Point Arrived");
            m_angularPid.Clear();
            m_goalSet = false;
        }else{ // If GUI didn't control in 5 sec and goal is set go to the goal
	        angleDiff = AngleDiff(xDiff, yDiff,
                                  m_robotOdometryMsg.pose.pose.orientation.x, m_robotOdometryMsg.pose.pose.orientation.y);
            ROS_INFO("P2P Debug: Remaining Distance %f, Angle Difference: %f", goaldist, angleDiff);
	        robotProcessMsg.linear.x = m_speedPid.getResult(m_currspeed, 0.6);
            robotProcessMsg.angular.z = m_angularPid.getResult(angleDiff);
            return robotProcessMsg;
        }
    }

    if(m_opMode){
        // ROS_INFO("PID Speed Log: %f, %f, %f", (m_robotControlMsg.linear.x-m_currspeed), m_speedPid.getcerr(), m_speedPid.getperr());
        robotProcessMsg.linear.x = m_speedPid.getResult(m_currspeed, m_robotControlMsg.linear.x);
    }else{
        robotProcessMsg.linear.x = m_robotControlMsg.linear.x;
    }
    robotProcessMsg.angular.z = m_robotControlMsg.angular.z;
    return robotProcessMsg;
}

std_msgs::Float32 pidController::GetCurrSpeed(){
    std_msgs::Float32 temp;
    temp.data = m_currspeed;
    return temp;
}

bool pidController::ChangeOpMode( robot_msgs::ChangeOpMode::Request &req,
                       robot_msgs::ChangeOpMode::Response &res ){
    m_opMode = !m_opMode;
    if(m_opMode){
        m_speedPid.Clear();
        res.result = "Change to PID Mode";
        ROS_INFO("Change to PID Mode");
    }else{
        res.result = "Change to Direct Mode";
        ROS_INFO("Change to Direct Mode");
    }
    return true;
}

bool pidController::GoHome( robot_msgs::GoHome::Request &req,
                 robot_msgs::GoHome::Response &res ){
    m_robotTargetPoseMsg.header.frame_id = "map";
    m_robotTargetPoseMsg.header.stamp = ros::Time::now();
    m_robotTargetPoseMsg.pose.position.x = 0;
    m_robotTargetPoseMsg.pose.position.y = 0;
    m_robotTargetPoseMsg.pose.position.z = 0;
    m_goalSet = true;
    return true;
}

bool pidController::ClearGoal( robot_msgs::ClearGoal::Request &req,
                    robot_msgs::ClearGoal::Response &res ){
    m_goalSet = false;
    m_angularPid.Clear();
    return true;
}

bool pidController::UpdateAngularKp( robot_msgs::UpdateAngularKp::Request &req,
                                     robot_msgs::UpdateAngularKp::Response &res ){
    ROS_DEBUG("Change Angular Kp From %f to %f", m_angularPid.getKp(), req.value);
    m_angularPid.setKp(req.value);
    return SavePIDConfig();
}

bool pidController::UpdateAngularKi( robot_msgs::UpdateAngularKi::Request &req,
                                     robot_msgs::UpdateAngularKi::Response &res ){
    ROS_DEBUG("Change Angular Ki From %f to %f", m_angularPid.getKi(), req.value);
    m_angularPid.setKi(req.value);
    return SavePIDConfig();
}

bool pidController::UpdateAngularKd( robot_msgs::UpdateAngularKd::Request &req,
                                     robot_msgs::UpdateAngularKd::Response &res ){
    ROS_DEBUG("Change Angular Kd From %f to %f", m_angularPid.getKd(), req.value);
    m_angularPid.setKd(req.value);
    return SavePIDConfig();
}

bool pidController::UpdateSpeedKp( robot_msgs::UpdateSpeedKp::Request &req,
                                   robot_msgs::UpdateSpeedKp::Response &res ){
    ROS_DEBUG("Change Speed Kp From %f to %f", m_speedPid.getKp(), req.value);
    m_speedPid.setKp(req.value);
    return SavePIDConfig();
}

bool pidController::UpdateSpeedKi( robot_msgs::UpdateSpeedKi::Request &req,
                                   robot_msgs::UpdateSpeedKi::Response &res ){
    ROS_DEBUG("Change Speed Ki From %f to %f", m_speedPid.getKi(), req.value);
    m_speedPid.setKi(req.value);
    return SavePIDConfig();
}

bool pidController::UpdateSpeedKd( robot_msgs::UpdateSpeedKd::Request &req,
                                   robot_msgs::UpdateSpeedKd::Response &res ){
    ROS_DEBUG("Change Speed Kd From %f to %f", m_speedPid.getKd(), req.value);
    m_speedPid.setKd(req.value);
    return SavePIDConfig();
}