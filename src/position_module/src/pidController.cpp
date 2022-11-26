#include "pidController.h"

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
    m_robotProcessMsg.linear.x = speedPid.Kp*speed_error;
    // Turn the Robot Speed into absoulate speed and retain the Z angler speed only
    // ROS_INFO("Current Speed: %.2f, %f", currlinSpeed, m_robotControlMsg.angular.z);
    // Compare the current speed with the Input Twist
    return m_robotProcessMsg;
}