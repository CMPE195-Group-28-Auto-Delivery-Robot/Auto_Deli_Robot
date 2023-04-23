#include <stdio.h>
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "minmea/minmea.h"
#include "SerialGPSObject.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "NMEA_GPS_Node");
    ros::NodeHandle rosHandle;
    int serialFp;
    SerialGPSObject gpsDevice;
    std::string frameID;
    std_msgs::String serialMsg;
    std::stringstream serialStream;
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

    char readBuff[256];
    memset(&readBuff, '\0', sizeof(readBuff));
    ros::Publisher gps_pub = rosHandle.advertise<sensor_msgs::NavSatFix>(ros::this_node::getName()+"/fix", 1000);
    ros::Publisher serial_pub = rosHandle.advertise<std_msgs::String>(ros::this_node::getName()+"/Serial", 1000);
    ros::ServiceServer service = rosHandle.advertiseService(ros::this_node::getName()+"/RebootGPS", &SerialGPSObject::RebootGPS, &gpsDevice);

    gpsDevice.RebootGPS();

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
                    gpsMsg.altitude = minmea_tofloat(&frame.altitude);
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
                    gpsMsg.position_covariance[0] = minmea_tofloat(&frame.hdop) ;//* minmea_tocoord(&frame.hdop);
                    gpsMsg.position_covariance[4] = minmea_tofloat(&frame.hdop) ;//* minmea_tocoord(&frame.hdop);
                    gpsMsg.position_covariance[8] = (minmea_tofloat(&frame.hdop)*4) ;//* (minmea_tocoord(&frame.hdop)*2);
                    serialStream << "GGA Parsed";
                    gpsMsg.header.stamp = ros::Time::now();
                    gps_pub.publish(gpsMsg);
                    serialMsg.data = serialStream.str();
                    serial_pub.publish(serialMsg);
                    serialStream.str("");
                }
            } break;

            default:
                break;
            }
        }
        memset(&readBuff, '\0', sizeof(readBuff));
    }
    ROS_INFO("GPS %s Node End", ros::this_node::getName().c_str());
    close(serialFp);
    return 0;
}
