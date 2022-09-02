#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"


#define RAD2DEG(x) ((x)*180./M_PI)

class scanCovertCloud{
private:
    laser_geometry::LaserProjection Projecter;
    sensor_msgs::PointCloud CloudMap;

public:
    scanCovertCloud(){

    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        int count = scan->scan_time / scan->time_increment;
        ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
        ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

        Projecter.projectLaser(*scan, CloudMap);
        for(auto point: CloudMap.points){
            ROS_INFO(": [%f, %f, %f]", point.x, point.y, point.z);
        }
    }

    sensor_msgs::PointCloud getCloudMsgs(){
        return CloudMap;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LaserScan_to_PointCloud");
    ros::NodeHandle n;

    ros::Publisher CloudMap_pub = n.advertise<sensor_msgs::PointCloud>("deli_robot/CloudMap", 1000);

    scanCovertCloud Converter;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/deli_robot/scan", 1000, &scanCovertCloud::scanCallback, &Converter);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        CloudMap_pub.publish(Converter.getCloudMsgs());
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
} 
