#include <stdio.h>
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "minmea/minmea.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

typedef struct {
    std::string portPath;
    int baudrate;
    int dataBit;
    bool parity;
    bool stopBitEven;
    bool flowCtrl;
} portConfig;

int openSerial(portConfig serialCnf){
    int serialFp;
    struct termios ttyCnf;
    serialFp = open(serialCnf.portPath.c_str(), O_RDWR);
    if (serialFp < 0) {
        ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }
    if(tcgetattr(serialFp, &ttyCnf) != 0) {
        ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        close(serialFp);
        return -1;
    }
    if(serialCnf.parity){
        ttyCnf.c_cflag |= PARENB; 
    }else{
        ttyCnf.c_cflag &= ~PARENB;
    }
    if(serialCnf.stopBitEven){
        ttyCnf.c_cflag |= CSTOPB;; 
    }else{
        ttyCnf.c_cflag &= ~CSTOPB;
    }
    if(serialCnf.flowCtrl){
        ttyCnf.c_cflag |= CRTSCTS; 
    }else{
        ttyCnf.c_cflag &= ~CRTSCTS;
    }
    switch(serialCnf.dataBit){
        case 5:
            ttyCnf.c_cflag |= CS5; 
            break;
        case 6:
            ttyCnf.c_cflag |= CS6; 
            break;
        case 7:
            ttyCnf.c_cflag |= CS7; 
            break;
        case 8:
            ttyCnf.c_cflag |= CS8; 
            break;
        default:
            ROS_ERROR("Data Bit set invalid");
            close(serialFp);
            return -1;
    }
    ttyCnf.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    ttyCnf.c_lflag &= ~ICANON; // Disabling Canonical Mode
    ttyCnf.c_lflag &= ~ECHO; // Disable echo
    ttyCnf.c_lflag &= ~ECHOE; // Disable erasure
    ttyCnf.c_lflag &= ~ECHONL; // Disable new-line echo
    ttyCnf.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    ttyCnf.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    ttyCnf.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    ttyCnf.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    ttyCnf.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    ttyCnf.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    ttyCnf.c_cc[VMIN] = 0;
    switch(serialCnf.baudrate){
        case 1200:
            cfsetispeed(&ttyCnf, B1200);
            break;
        case 1800:
            cfsetispeed(&ttyCnf, B1800);
            break;
        case 2400:
            cfsetispeed(&ttyCnf, B2400);
            break;
        case 4800:
            cfsetispeed(&ttyCnf, B4800);
            break;
        case 9600:
            cfsetispeed(&ttyCnf, B9600);
            break;
        case 19200:
            cfsetispeed(&ttyCnf, B19200);
            break;
        case 38400:
            cfsetispeed(&ttyCnf, B38400);
            break;
        case 57600:
            cfsetispeed(&ttyCnf, B57600);
            break;
        case 115200:
            cfsetispeed(&ttyCnf, B115200);
            break;
        default:
            ROS_ERROR("Unsupported Baud Rate");
            close(serialFp);
            return -1;
    }
    // Save tty settings, also checking for error
    if (tcsetattr(serialFp, TCSANOW, &ttyCnf) != 0) {
        ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(serialFp);
        return -1;
    }
    return serialFp;
} // Code Above Partially From https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#configuration-setup

uint8_t stringCheckSum(std::string inStr){
    uint8_t checkSum = inStr[0];
    for(int i=1; i<inStr.size();i++){
        checkSum ^= inStr[i];
    }
    return checkSum;
}

void sendCommand(int ioFp,std::string content){
    char numStr[3];
    char readStr[32];
    int commandNum, result;
    sprintf(numStr, "%X", stringCheckSum(content));
    content = "$"+content+"*"+numStr+"\r\n";
    // ROS_INFO("Send: %s",content.c_str());
    write(ioFp, content.c_str(), content.size());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "NMEA_GPS_Node");
    ros::NodeHandle rosHandle;
    int serialFp;
    int chipType;
    portConfig serialCnf;
    std::string frameID;
    std_msgs::String serialMsg;
    std::stringstream serialStream;
    // Get Parameter Config
    if (!rosHandle.getParam(ros::this_node::getName()+"/PortPath", serialCnf.portPath))
    {
        ROS_ERROR("Failed to get param 'PortPath'");
        return -1;
    }
    rosHandle.param<std::string>(ros::this_node::getName()+"/frame_id", frameID, "map");
    rosHandle.param(ros::this_node::getName()+"/BaudRate", serialCnf.baudrate, 9600);
    rosHandle.param(ros::this_node::getName()+"/DataBit", serialCnf.dataBit, 8);
    if( serialCnf.dataBit>8 || serialCnf.dataBit<5 ){
        ROS_ERROR("Data Bit set invalid");
        return -1;
    }
    rosHandle.param(ros::this_node::getName()+"/Parity", serialCnf.parity, false);
    rosHandle.param(ros::this_node::getName()+"/StopBit_Even", serialCnf.stopBitEven, false);
    rosHandle.param(ros::this_node::getName()+"/FlowCtrl", serialCnf.flowCtrl, false);
    rosHandle.param(ros::this_node::getName()+"/ChipType", chipType, 0);

    serialFp = openSerial(serialCnf);
    if(serialFp < 0){
        return serialFp;
    }

    char readBuff[256];
    memset(&readBuff, '\0', sizeof(readBuff));
    ros::Publisher gps_pub = rosHandle.advertise<sensor_msgs::NavSatFix>(ros::this_node::getName()+"/fix", 1000);
    ros::Publisher serial_pub = rosHandle.advertise<std_msgs::String>(ros::this_node::getName()+"/Serial", 1000);

    switch (chipType)
    {
    case 1:
        ROS_INFO("Chip: MT3339");
        sendCommand(serialFp, "PMTK102");
        ros::Duration(3).sleep();
        ROS_INFO("Boot Complete");
        sendCommand(serialFp, "PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
        sendCommand(serialFp, "PMTK220,100");
        break;

    case 2:
        ROS_INFO("Chip: UBlox M8");
        //TODO: Ublox Command
        break;
    
    default:
        ROS_INFO("Chip: Unknown");
        break;
    }


    ROS_INFO("GPS %s Node Started", ros::this_node::getName().c_str());
    while(ros::ok()){
        ros::spinOnce();
        sensor_msgs::NavSatFix gpsMsg;
        gpsMsg.header.frame_id = frameID;
        int num_bytes = read(serialFp, &readBuff, sizeof(readBuff));
        if (num_bytes < 0) {
            ROS_ERROR("Error reading: %s", strerror(errno));
            close(serialFp);
            return -1;
        }
        std::stringstream msgStream(readBuff);
        std::string inLine;
        while(std::getline(msgStream, inLine, '$')){
            if(inLine.empty()){
                continue;
            }
            inLine = "$"+inLine;
            inLine.erase(remove(inLine.begin(), inLine.end(), '\n'), inLine.end());
            inLine.erase(remove(inLine.begin(), inLine.end(), '\r'), inLine.end());
            serialStream << inLine;
            serialMsg.data = serialStream.str();
            serial_pub.publish(serialMsg);
            serialStream.str("");
            switch(minmea_sentence_id(inLine.c_str(), false)) {
            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if(minmea_parse_gga(&frame, inLine.c_str())) {
                    gpsMsg.latitude = minmea_tocoord(&frame.latitude);
                    gpsMsg.longitude = minmea_tocoord(&frame.longitude);
                    gpsMsg.altitude = minmea_tocoord(&frame.altitude);
                    gpsMsg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                    gpsMsg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
                    switch(frame.fix_quality){
                        case 1:
                            gpsMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                            break;
                        case 2:
                            gpsMsg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
                            break;
                        case 4:
                        case 5:
                            gpsMsg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                            break;
                        default:
                            gpsMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                            gpsMsg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
                            break;
                    }
                    gpsMsg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                    gpsMsg.position_covariance[0] = minmea_tocoord(&frame.hdop) * minmea_tocoord(&frame.hdop);
                    gpsMsg.position_covariance[4] = minmea_tocoord(&frame.hdop) * minmea_tocoord(&frame.hdop);
                    gpsMsg.position_covariance[8] = (minmea_tocoord(&frame.hdop)*2) * (minmea_tocoord(&frame.hdop)*2);
                    serialStream << "GGA Parsed";
                    serialMsg.data = serialStream.str();
                    serial_pub.publish(serialMsg);
                    serialStream.str("");
                }
            } break;

            default:
                break;
            }
        }
        gpsMsg.header.stamp = ros::Time::now();
        gps_pub.publish(gpsMsg);
        memset(&readBuff, '\0', sizeof(readBuff));
    }
    ROS_INFO("GPS %s Node End", ros::this_node::getName().c_str());
    close(serialFp);
    return 0;
}
