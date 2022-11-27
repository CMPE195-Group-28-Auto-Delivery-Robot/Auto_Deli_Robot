#include "infoProcessor.h"

infoProcessor::infoProcessor(int alert_angle, int step_size, int alert_distance){
    avoid_angle = alert_angle;
    move_distance = step_size;
    search_distance = alert_distance;
}

void infoProcessor::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    m_robotOdometryMsg = *(odomMsg.get());
}

void infoProcessor::ObjectCallback(const zed_interfaces::ObjectsStamped::ConstPtr &objectMsg)
{
    m_objectDetectionMsg = *(objectMsg.get());
}

void infoProcessor::LineExtraCallback(const laser_line_extraction::LineSegmentList::ConstPtr &lineExtraLts)
{
    m_lineExtrationLst = *(lineExtraLts.get());
    for (auto temp : m_lineExtrationLst.line_segments)
    {
        obstacle obstacle_line;
        obstacle_line.start.x = (int)temp.start[0];
        obstacle_line.start.y = (int)temp.start[1];
        obstacle_line.end.x = (int)temp.end[0];
        obstacle_line.end.y = (int)temp.end[1];
        input_lidar_info.push_back(obstacle_line);
    }
}

void infoProcessor::GoalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &lineExtraLts)
{
    m_goalPosition = *(lineExtraLts.get());
    dest_position.x = m_goalPosition.pose.position.x;
    dest_position.y = m_goalPosition.pose.position.y;
}

geometry_msgs::PoseStamped infoProcessor::getTarget()
{
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
            obstacle_map.push_back(temp_point);
            cur_loc.x += x_forward;
            cur_loc.y += y_forward;
            x_diff -= x_rate;
            y_diff -= y_rate;
        }
        obstacle_map.push_back(segment.end);
    }