#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/deli_robot/Auto_Deli_Robot/devel/setup.bash
systemctl stop ros_field_sev.service
sleep 1
systemctl start roscore_sev.service
# systemctl start ros_dev_sev.service
sleep 1
roslaunch lidar_module launch_lidar.launch
