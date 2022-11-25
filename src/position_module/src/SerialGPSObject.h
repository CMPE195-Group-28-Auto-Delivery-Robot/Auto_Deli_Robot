#include <string>
#include <stdio.h>
#include <sstream>
#include "ros/ros.h"
#include "robot_msgs/RebootGPS.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#pragma once

class SerialGPSObject{
public:
    int serialFp;
    std::string portPath;
    int baudrate;
    int dataBit;
    bool parity;
    bool stopBitEven;
    bool flowCtrl;

    int chipType;

    int openSerial();
    void sendCommand(std::string content);
    void RebootGPS();
    bool RebootGPS( robot_msgs::RebootGPS::Request &req,
                    robot_msgs::RebootGPS::Response &res );

private:
    uint8_t stringCheckSum(std::string inStr);
};