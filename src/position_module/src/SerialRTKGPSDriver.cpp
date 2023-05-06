#include <regex>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "SerialGPSObject.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "RTK_GPS_Node");
    ros::NodeHandle rosHandle;
    int serialFp;
    SerialGPSObject gpsDevice;
    std::string frameID;
    std_msgs::String serialMsg;

    // Get Parameter Config
    if (!rosHandle.getParam(ros::this_node::getName()+"/PortPath", gpsDevice.portPath))
    {
        ROS_ERROR("Failed to get param 'PortPath'");
        return -1;
    }
    rosHandle.param<std::string>(ros::this_node::getName()+"/frame_id", frameID, "gps");
    rosHandle.param(ros::this_node::getName()+"/BaudRate", gpsDevice.baudrate, 9600);
    rosHandle.param(ros::this_node::getName()+"/DataBit", gpsDevice.dataBit, 8);
    if( gpsDevice.dataBit>8 || gpsDevice.dataBit<5 ){
        ROS_ERROR("Data Bit set invalid");
        return -1;
    }
    rosHandle.param(ros::this_node::getName()+"/Parity", gpsDevice.parity, false);
    rosHandle.param(ros::this_node::getName()+"/StopBit_Even", gpsDevice.stopBitEven, false);
    rosHandle.param(ros::this_node::getName()+"/FlowCtrl", gpsDevice.flowCtrl, false);
    rosHandle.param(ros::this_node::getName()+"/ChipType", gpsDevice.chipType, 0);

    serialFp = gpsDevice.openSerial();
    if(serialFp < 0){
        return serialFp;
    }

    char readBuff[512];
    memset(&readBuff, '\0', sizeof(readBuff));
    ros::Publisher gps_pub = rosHandle.advertise<sensor_msgs::NavSatFix>(ros::this_node::getName()+"/fix", 1000);
    ros::Publisher serial_pub = rosHandle.advertise<std_msgs::String>(ros::this_node::getName()+"/Serial", 1000);

    // gpsDevice.sendCommand(SerialGPSObject::GetU8VecFromStr("S\n")); // Send Random Char to start RTK

    ROS_INFO("GPS %s Node Started", ros::this_node::getName().c_str());

    std::regex lat_regex("Lat: ([\\d\\.-]+)");
    std::regex long_regex("Long: ([\\d\\.-]+)");
    std::regex height_regex("Height: ([\\d\\.]+)");
    std::regex fix_regex("Fix: (\\d+) \\((.+)\\)");
    std::regex carrier_regex("Carrier Solution: (\\d+) \\((.+)\\)");
    std::regex accuracy_regex("Horizontal Accuracy Estimate: (\\d+) \\((.+)\\)");

    while(ros::ok()){
        ros::spinOnce();
        sensor_msgs::NavSatFix gpsMsg;
        std::stringstream serialStream;
        gpsMsg.header.frame_id = frameID;
        
        int num_bytes = 0;
        std::string inLine = "";

        do{
            num_bytes = read(serialFp, &readBuff, sizeof(readBuff));
            if (num_bytes < 0) {
                ROS_ERROR("Error reading: %s", strerror(errno));
                close(serialFp);
                return -1;
            }
            serialStream << readBuff;
            memset(&readBuff, '\0', sizeof(readBuff));
        }while(num_bytes == 64);

        serialMsg.data = serialStream.str();
        serial_pub.publish(serialMsg);

        if(serialStream.str().empty())
            continue;

        std::string t_begin;
        serialStream >> t_begin;
        if(t_begin != "Lat:")
            continue;

        // Extract the values for each parameter using regex
        double latitude;
        double longitude;
        double height;
        int fix;
        int carrier_solution;
        int accuracy;

        try{
        std::smatch match;
        std::string sentence = serialStream.str();
        std::regex_search(sentence, match, lat_regex);
        latitude = std::stod(match[1]);
        std::regex_search(sentence, match, long_regex);
        longitude = std::stod(match[1]);
        std::regex_search(sentence, match, height_regex);
        height = std::stod(match[1]);
        std::regex_search(sentence, match, fix_regex);
        fix = std::stoi(match[1]);
        std::string fix_type = match[2];
        std::regex_search(sentence, match, carrier_regex);
        carrier_solution = std::stoi(match[1]);
        std::string carrier_type = match[2];
        std::regex_search(sentence, match, accuracy_regex);
        accuracy = std::stoi(match[1]);
        std::string accuracy_units = match[2];
        }catch(...){
            continue;
        }
        
        gpsMsg.header.stamp = ros::Time::now();
        gpsMsg.latitude = latitude;
        gpsMsg.longitude = longitude;
        gpsMsg.altitude = height;
        gpsMsg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        gpsMsg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        gpsMsg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        gps_pub.publish(gpsMsg);
    }
    ROS_INFO("GPS %s Node End", ros::this_node::getName().c_str());
    close(serialFp);
    return 0;
}
