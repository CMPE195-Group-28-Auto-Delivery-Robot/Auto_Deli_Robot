#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"


#define RAD2DEG(x) ((x)*180./M_PI)

class scanCovertCloud{
private:
    laser_geometry::LaserProjection Projecter;
    sensor_msgs::PointCloud2 CloudMap;

public:
    scanCovertCloud(){

    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        Projecter.projectLaser(*scan, CloudMap);
    }

    sensor_msgs::PointCloud2 getCloudMsgs(){
        return CloudMap;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LaserScan_to_PointCloud");
    ros::NodeHandle n;

    ros::Publisher CloudMap_pub = n.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1000);

    scanCovertCloud Converter;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/deli_robot/scan", 1000, &scanCovertCloud::scanCallback, &Converter);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        CloudMap_pub.publish(Converter.getCloudMsgs());
        loop_rate.sleep();
    }

    return 0;
} 
