# Auto_Deli_Robot  

## Setup Workspace  

### First Install All Essential Package

1. Install Python 3
``` sudo apt install python3 ```
2. Install ROS Medodic follow this guide <https://wiki.ros.org/melodic/Installation/Ubuntu>
3. Install the required Component  

```bash
   cd <path to Auto_Deli_Robot>/Auto_Deli_Robot
   sudo ./Package_Install.sh
```

### Build ROS WorkSpace

1. Bulid Project
   ``` catkin_make ```
2. source the setup.bash so ROS can regonize paskage in the workspace
   ``` source devel/setup.bash ```
   If you don't want to run this line everytime you can add

```bash
   echo "source <path to the Auto_Deli_Robot>/Auto_Deli_Robot/devel/setup.bash" >> ~/.bashrc
```

---

## Project Wiki

- [Source Code](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Deli_Robot/tree/main/src)
- [Web GUI](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Delivery_Robot_Web_Control_GUI)
- [Help Script](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Deli_Robot/tree/main/Help_Script)
- [Data Record](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Deli_Robot/tree/main/data_record)
- [Back Service](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Deli_Robot/tree/main/Back_Service)
- [Project Wiki](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Deli_Robot/tree/main/Project_Wiki) ***Work In Progress***

---  

## Some Dependence of the Project

- [Lidar Driver and Node Example](https://github.com/Slamtec/rplidar_ros)
- ZED CAM ROS Library
  - [ZED ROS Wrapper](https://github.com/stereolabs/zed-ros-wrapper)
  - [ZED ROS Interfaces](https://github.com/stereolabs/zed-ros-interfaces)
  - [ZED ROS Examples](https://github.com/stereolabs/zed-ros-examples)
- [Line Extraction Module](https://github.com/kam3k/laser_line_extraction.git)  
- [NMEA Message Parser](https://github.com/kosma/minmea.git)
