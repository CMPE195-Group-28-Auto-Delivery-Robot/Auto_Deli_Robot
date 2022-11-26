#include <string>
#include <stdio.h>
#include <sstream>
#include "ros/ros.h"
#include <vector>
#include "zed_interfaces/ObjectsStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "laser_line_extraction/LineSegmentList.h"

class Avoidance{
private:
    const int avoid_angle;
    const int move_distance;
    const int search_distance;

    zed_interfaces::ObjectsStamped m_objectDetectionMsg;
    nav_msgs::Odometry m_robotOdometryMsg;
    laser_line_extraction::LineSegmentList m_lineExtrationLst;

public:
    geometry_msgs::PoseStamped getTarget(double input_start_point[2], double input_end_point[2], std::vector<double[2]> input_lidar_info);
    std::vector<double> getPosition(double input_start_point[2], double input_end_point[2]);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void ObjectCallback(const zed_interfaces::ObjectsStamped::ConstPtr& objectMsg);
    void LineExtraCallback(const laser_line_extraction::LineSegmentList::ConstPtr& lineExtraLts);



};
