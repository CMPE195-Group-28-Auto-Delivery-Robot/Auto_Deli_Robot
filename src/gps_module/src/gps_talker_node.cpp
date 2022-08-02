// Originally located at:
// http://controls.ece.unm.edu/documents/gpsTest.cpp
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

void locationCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_location){
  ROS_INFO("Current Location: [ Latitude: %.5f, longitude: %.5f ]\n", gps_location->latitude, gps_location->longitude);
  ROS_INFO("Current High: %.5f\n",gps_location->altitude);
}

int main(int argc, char** argv)
{
  // Variables to store the Latitude and Longitude from the GPS respectively
  double gpsLat = 0;
  double gpsLong = 0;

  // Initializing the node for the GPS
  ros::init(argc, argv, "gps_Subscriber");
  ros::NodeHandle gps_node_handle;

  ros::Subscriber gps_subber = gps_node_handle.subscribe("fix", 100, locationCallback);

  ros::spin();
  return 0;
}