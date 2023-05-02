#pragma once

#include <string>
#include <stdio.h>
#include <sstream>
#include "ros/ros.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

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
    void sendCommand(std::vector<uint8_t> content);
    static std::vector<uint8_t> GetU8VecFromStr(std::string content);

private:
    uint8_t stringCheckSum(std::string inStr);
};