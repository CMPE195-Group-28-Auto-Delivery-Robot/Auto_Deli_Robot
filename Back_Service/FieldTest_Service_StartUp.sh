#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/deli_robot/Auto_Deli_Robot/devel/setup.bash

ubxtool -p COLDBOOT -v 5
ubxtool -p HOTBOOT -v 2

roslaunch project_core Robot_StartUp.launch
