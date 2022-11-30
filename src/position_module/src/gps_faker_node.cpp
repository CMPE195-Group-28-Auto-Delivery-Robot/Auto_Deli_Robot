// Originally located at:
// http://controls.ece.unm.edu/documents/gpsTest.cpp
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

sensor_msgs::NavSatFix RandomGPSMsgs(int seq){
    sensor_msgs::NavSatFix fakeMsg;

    fakeMsg.header.frame_id = "/gps";
    fakeMsg.header.seq = seq;
    fakeMsg.header.stamp = ros::Time::now();

    fakeMsg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    fakeMsg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    srandom(fakeMsg.header.stamp.nsec);

    fakeMsg.altitude = 10 + double(rand()%100000-50000)/100000.00;
    fakeMsg.latitude = 37.420221660199104 + double(rand()%100000-50000)/1000000000.00;
    fakeMsg.longitude = -121.87291934241432 + double(rand()%100000-50000)/1000000000.00;
    // [651.7809000000001, 0.0, 0.0, 0.0, 651.7809000000001, 0.0, 0.0, 0.0, 2607.1236000000004]
    fakeMsg.position_covariance.assign(0);
    fakeMsg.position_covariance.at(0) = 651.7809000000001;
    fakeMsg.position_covariance.at(4) = 651.7809000000001;
    fakeMsg.position_covariance.at(8) = 2607.1236000000004;
    fakeMsg.position_covariance_type = 1;
    
    ROS_INFO("Fake GPS: %f, %f", fakeMsg.latitude, fakeMsg.longitude);
    return fakeMsg;
}

int main(int argc, char** argv)
{
    // Variables to store the Latitude and Longitude from the GPS respectively
    int counter = 1;
    ros::init(argc, argv, "gps_faker");
    ros::NodeHandle n;

    ros::Publisher fakeGPSpub = n.advertise<sensor_msgs::NavSatFix>("fix", 1000);

    ros::Rate loop_rate(1);

    while(ros::ok()){
        fakeGPSpub.publish(RandomGPSMsgs(counter++));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}