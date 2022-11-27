#include <string>
#include <stdio.h>
#include <sstream>
#include "ros/ros.h"
#include <vector>
#include "zed_interfaces/ObjectsStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "laser_line_extraction/LineSegmentList.h"
#include <unordered_set>

class infoProcessor{
private:
    int avoid_angle;
    int move_distance;
    int search_distance;
    struct location{
        float x;
        float y;
    } ;
    struct obstacle{
        location start;
        location end;
    } ;
    location current_position;
    location dest_position;
    zed_interfaces::ObjectsStamped m_objectDetectionMsg; 
    nav_msgs::Odometry m_robotOdometryMsg;
    laser_line_extraction::LineSegmentList m_lineExtrationLst;
    geometry_msgs::PoseStamped m_goalPosition;
    std::vector<location> obstacle_map;
    std::vector<obstacle> input_lidar_info;




public:
    infoProcessor(int alert_angle, int step_size, int alert_distance);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void ObjectCallback(const zed_interfaces::ObjectsStamped::ConstPtr& objectMsg);
    void LineExtraCallback(const laser_line_extraction::LineSegmentList::ConstPtr& lineExtraLts);
    void GoalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& lineExtraLts);
    geometry_msgs::PoseStamped getTarget();
    location getPosition(float angle);
    float getAngle();
    bool isClear(location nextStep);
    void addObstacle(obstacle line);
    geometry_msgs::PoseStamped locToPoseConvet(location loc);



};
