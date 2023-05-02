#pragma once
#include "SerialGPSObject.h"
#include "robot_msgs/RebootGPS.h"

class NmeaGPSObject : public SerialGPSObject{
public:
    void RebootGPS();
    bool RebootGPS( robot_msgs::RebootGPS::Request &req,
                    robot_msgs::RebootGPS::Response &res );
};