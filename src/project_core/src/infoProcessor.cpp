#include "infoProcessor.h"

infoProcessor::infoProcessor(int alert_angle, int step_size, int alert_distance){
    avoid_angle = alert_angle;
    move_distance = step_size;
    search_distance = alert_distance;

}

//  Update odometry info, it is in map level
void infoProcessor::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    m_robotOdometryMsg = *(odomMsg.get());
    current_position.x = m_robotOdometryMsg.pose.pose.position.x;
    current_position.y = m_robotOdometryMsg.pose.pose.position.y;
}

//  Update object detection in map level
//  TODO: Transform function need to be implemented
void infoProcessor::ObjectCallback(const zed_interfaces::ObjectsStamped::ConstPtr &objectMsg)
{
    m_objectDetectionMsg = *(objectMsg.get());
}

//  Update obstacle position in map level
void infoProcessor::LineExtraCallback(const laser_line_extraction::LineSegmentList::ConstPtr &lineExtraLts)
{
    m_lineExtrationLst = *(lineExtraLts.get());
    for (auto temp : m_lineExtrationLst.line_segments)
    {
        input_lidar_info.push_back(laserToMap(temp));
    }
}

//  Reciveing the path from Google Maps API, Transfer the points into location struct.
//  Convert from destPoint type to location struct in code for easier implement and RESET counter.
void infoProcessor::DestListCallback(const robot_msgs::dest_list_msg::ConstPtr &lineExtraLts){
    m_goalPositions = *(lineExtraLts.get());
    std::vector<robot_msgs::destPoint_msg> destList = m_goalPositions.dest_list;
    for(auto point : destList){
        progress.push_back(earthToMap(point));
    }
    progressCounter = 0;
}  

//TODO: Early version of implementing tf transform between laser(line_extraction) to map. Map will only process location struct data.
infoProcessor::obstacle infoProcessor::laserToMap(laser_line_extraction::LineSegment seg){
    obstacle temp;
    temp.start.x = seg.start[0];
    temp.start.y = seg.start[1];
    temp.end.x = seg.end[0];
    temp.end.y = seg.end[1];
    return temp;
}

//TODO: Early version of implementing tf transform between earth(satellite GPS) to map. Map will only process location struct data.
infoProcessor::location infoProcessor::earthToMap(robot_msgs::destPoint_msg loc){
    location temp;
    temp.x = loc.lat;
    temp.y = loc.lng;
    return temp;
}

// Return distance between 2 location
float infoProcessor::distance(location a, location b){
    return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

//Major algorithum, based on current position, goal position, obstacles posistion and mvoe distance to calculate the next move's location
geometry_msgs::PoseStamped infoProcessor::getNextStep()
{ 
    if(distance(current_position, progress[progressCounter]) <= min_reach_distance && progressCounter <= progress.size()){
        progressCounter++;
    }
    //when robot reach step goal, switch to next goal. If robot already reach final goal, nothing will change.
    dest_position = progress[progressCounter]; //Set current goal position.
    for (auto temp : input_lidar_info)
    {
        addObstacle(temp);
    }
    bool isFront = true;
    bool isTurn = false;
    int left_pass = avoid_angle;
    int right_pass = avoid_angle;
    int left = 0;
    int right = 0;
    bool left_noPass = true;
    bool right_noPass = true;
    float left_target_angel = 0;
    float right_target_angel = 0;

    float current_angle = this->getAngle();
    int half_angle = avoid_angle / 2;
    while (left != -90 && right != 90)
    {
        if (isFront)
        {
            if (left_noPass && right_noPass)
            {
                if (this->isClear(getPosition(current_angle)))
                {
                    left_target_angel = current_angle;
                    right_target_angel = current_angle;
                    left_noPass = false;
                    right_noPass = false;
                }
                else
                {
                    isFront = false;
                }
            }
            else
            {

                if (this->isClear(getPosition(current_angle)))
                {
                    left_pass--;
                    right_pass--;
                }
                else
                {
                    left_pass = avoid_angle;
                    isFront = false;
                    left_noPass = true;
                    right_pass = avoid_angle;
                    isFront = false;
                    right_noPass = true;
                }

                if (left_pass == half_angle && right_pass == half_angle)
                {   
                    return locToPoseConvet(getPosition(current_angle));
                }
            }
            if (isTurn) {
                left--;
                isTurn = false;
            } else {
                right++;
                isTurn = true;
            }
        }else {
                if (left_noPass) {
                    if (this->isClear(this->getPosition(current_angle + left))) {
                        left_target_angel = current_angle + left;
                        left_noPass = false;
                    }
                } else {
                    if (this->isClear(this->getPosition(current_angle + left))) {
                        left_pass--;
                        if (left_pass == 0) {
                            return locToPoseConvet(getPosition(left_target_angel - half_angle));
                        }
                    } else {
                        left_pass = avoid_angle;
                        left_noPass = true;
                    }
                }
                left--;
                if (right_noPass) {
                    if (this->isClear(this->getPosition(current_angle + right))) {
                        right_target_angel = current_angle + right;
                        right_noPass = false;
                    }
                } else {
                    if (this->isClear(this->getPosition(current_angle + right))) {
                        right_pass--;
                        if (right_pass == 0) {
                            return locToPoseConvet(getPosition(right_target_angel + half_angle));
                        }
                    } else {
                        right_pass = avoid_angle;
                        right_noPass = true;
                    }
                }
                right++;
            }
        }
        return locToPoseConvet(current_position);
    }

    geometry_msgs::PoseStamped infoProcessor::locToPoseConvet(infoProcessor::location loc){
        geometry_msgs::PoseStamped stamp;
        stamp.pose.position.x = loc.x;
        stamp.pose.position.y = loc.x;
        return stamp;
    }

    infoProcessor::location infoProcessor::getPosition(float angle)
    {
        location nextStep;
        nextStep.x = current_position.x + (move_distance * sin(angle * M_PI / 90));
        nextStep.y = current_position.y + (move_distance * cos(angle * M_PI / 90));
        return nextStep;
    }

    float infoProcessor::getAngle()
    {
        double x_diff = (dest_position.x - current_position.x);
        double y_diff = (dest_position.y - current_position.y);
        return atan2(x_diff, y_diff) / M_PI * 90;
    }

    bool infoProcessor::isClear(location nextStep)
    {
        float x_diff = current_position.x - nextStep.x;
        float y_diff = current_position.y - nextStep.y;
        float x_rate = x_diff / (y_diff + x_diff);
        float y_rate = y_diff / (y_diff + x_diff);
        float x_forward = x_rate;
        float y_forward = y_rate;
        if (current_position.x > nextStep.x)
        {
            x_forward = -x_rate;
        }
        if (current_position.y > nextStep.y)
        {
            y_forward = -y_rate;
        }
        while (x_diff >= 0 && y_diff >= 0)
        {
            location temp_point;
            temp_point.x = (int)current_position.x;
            temp_point.y = (int)current_position.y;
            for (auto temp : obstacle_map)
            {
                if (temp_point.x == temp.x && temp_point.y == temp.y)
                {
                    return false;
                }
            }
            current_position.x += x_forward;
            current_position.y += y_forward;
            x_diff -= x_rate;
            y_diff -= y_rate;
        }

        return true;
    }

    void infoProcessor::addObstacle(obstacle segment)
    {
        float x_diff = segment.start.x - segment.end.x;
        float y_diff = segment.start.y - segment.end.y;
        float x_rate = x_diff / (y_diff + x_diff);
        float y_rate = y_diff / (y_diff + x_diff);
        float x_forward = x_rate;
        float y_forward = y_rate;
        if (segment.start.x > segment.end.x)
        {
            x_forward = -x_rate;
        }
        if (segment.start.y > segment.end.y)
        {
            y_forward = -y_rate;
        }
        location cur_loc;
        cur_loc.x = segment.start.x;
        cur_loc.y = segment.start.y;
        while (x_diff >= 0 && y_diff >= 0)
        {
            location temp_point;
            temp_point.x = (int)segment.start.x;
            temp_point.y = (int)segment.start.y;
            obstacle_map.emplace(temp_point);
            cur_loc.x += x_forward;
            cur_loc.y += y_forward;
            x_diff -= x_rate;
            y_diff -= y_rate;
        }
        obstacle_map.emplace(segment.end);
    }