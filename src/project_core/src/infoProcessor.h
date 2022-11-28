#include <string>
#include <stdio.h>
#include <sstream>
#include "ros/ros.h"
#include <vector>
#include "zed_interfaces/ObjectsStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/LineSegment.h"
#include <unordered_set>
#include "robot_msgs/dest_list_msg.h"
#include "robot_msgs/destPoint_msg.h"
#include <cmath>


class infoProcessor{
private:
    struct location{
        float x;
        float y;
        bool operator==(const location& t) const
        {
            return ((this->x == t.x) && (this->y == t.y));
        }
    };
    class locationHash{
    public:
        // id is returned as hash function
        size_t operator()(const location& t) const
        {
            return t.x;
        }
    };
    struct obstacle{
        location start;
        location end;
    } ;
    int avoid_angle;                                                    //Robot will avoid the obstacle in the angle
    int move_distance;                                                  //Robot's move distance each time
    int search_distance;                                                //Robot will ignore obstacle further than this range
    location current_position;                                          //Robot's current position
    location dest_position;                                             //Robot's step destination position
    std::vector<location> progress;                                     //All the path the robot should pass to reach the goal position
    zed_interfaces::ObjectsStamped m_objectDetectionMsg;                //Callback object of object detection 
    nav_msgs::Odometry m_robotOdometryMsg;                              //Callback object of odometry
    laser_line_extraction::LineSegmentList m_lineExtrationLst;          //Callback object of line extraction
    robot_msgs::dest_list_msg m_goalPositions;                          //Callback object of path planning
    std::unordered_set<location, locationHash> obstacle_map;            //Map of obstacles
    std::vector<obstacle> input_lidar_info;                             //Vector to store obstacles
    int progressCounter;                                                //Step counter for path
    float min_reach_distance;                                           //If distance between robot and goal within this range, consider robot can go next step



public:
    infoProcessor(int alert_angle, int step_size, int alert_distance);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void ObjectCallback(const zed_interfaces::ObjectsStamped::ConstPtr& objectMsg);
    void LineExtraCallback(const laser_line_extraction::LineSegmentList::ConstPtr& lineExtraLts);
    void DestListCallback(const robot_msgs::dest_list_msg::ConstPtr& lineExtraLts);
    geometry_msgs::PoseStamped getNextStep();
    location getPosition(float angle);
    float getAngle();
    bool isClear(location nextStep);
    void addObstacle(obstacle line);
    geometry_msgs::PoseStamped locToPoseConvet(location loc);
    obstacle laserToMap(laser_line_extraction::LineSegment seg);
    location earthToMap(robot_msgs::destPoint_msg loc);
    float distance(location a, location b);


};
