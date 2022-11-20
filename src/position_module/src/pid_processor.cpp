#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#define RAD2DEG(x) ((x)*180./M_PI)

class pidController{
private:
    geometry_msgs::Twist m_robotControlMsg;
    nav_msgs::Odometry m_robotOdometryMsg;
    geometry_msgs::PoseStamped m_robotTargetPoseMsg;
    const float Kp = 1;
    const float Ki = 1;
    const float Kd = 1;

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

    void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr& tagMsg)
    {
        m_robotTargetPoseMsg = *(tagMsg.get());
    }

    geometry_msgs::Twist GetProcessdMsg(){
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
        m_robotProcessMsg.linear.x = Kp*speedAngle;
        // Turn the Robot Speed into absoulate speed and retain the Z angler speed only
        // ROS_INFO("Current Speed: %.2f, %f", currlinSpeed, m_robotControlMsg.angular.z);
        // Compare the current speed with the Input Twist
        return m_robotProcessMsg;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Robot_PID_Node");
    ros::NodeHandle rosHandle;

    pidController pidNode;
    
    ros::Publisher cmdVel_pub = rosHandle.advertise<geometry_msgs::Twist>("controlVel", 1000);
    ros::Subscriber odom_sub = rosHandle.subscribe<nav_msgs::Odometry>("/deli_robot/odometry/map", 1000, &pidController::OdomCallback, &pidNode);
    ros::Subscriber control_sub = rosHandle.subscribe<geometry_msgs::Twist>("/deli_robot/cmd_vel", 1000, &pidController::ControlCallback, &pidNode);
    ros::Subscriber pose_sub = rosHandle.subscribe<geometry_msgs::PoseStamped>("/deli_robot/goalPosition", 1000, &pidController::TargetCallback, &pidNode);

    ros::Rate loop_rate(15);

    while(ros::ok()){
        cmdVel_pub.publish(pidNode.GetProcessdMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 
