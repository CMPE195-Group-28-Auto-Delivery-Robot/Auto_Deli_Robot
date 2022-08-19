#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/deli_robot/Auto_Deli_Robot/devel/setup.bash

roslaunch gps_module gps_driver_node.launch
