#include "Avoidance.h"

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        m_robotOdometryMsg = *(odomMsg.get());
    }

void ObjectCallback(const zed_interfaces::ObjectsStamped::ConstPtr& objectMsg)
    {
        m_objectDetectionMsg = *(objectMsg.get());
    }

void LineExtraCallback(const laser_line_extraction::LineSegmentList::ConstPtr& lineExtraLts)
    {
        m_lineExtrationLst = *(lineExtraLts.get());
    }