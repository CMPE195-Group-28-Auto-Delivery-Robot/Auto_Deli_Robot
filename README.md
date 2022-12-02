# Auto_Deli_Robot  

## Project Demo

You can check out our Full [Project Demo Video](https://www.youtube.com/watch?v=tzTTkthyS_k) on Youtube.
You can also watch this [Quick Demo Video](https://www.youtube.com/watch?v=kx7PS1gOcAw) which is a replay of one of our test run

![Robot Image](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Deli_Robot/blob/main/RobotImage.jpg?raw=true)

---

## Setup Workspace  

### First Install All Essential Package

1. Install Python 3
``` sudo apt install python3 ```
2. Install ROS Medodic follow this guide <https://wiki.ros.org/melodic/Installation/Ubuntu>
3. Install the required Component
   1. If set up on local machine (Must be Ubuntu 18.04 Environment)

      ```bash
         cd <path to Auto_Deli_Robot>/Auto_Deli_Robot
         sudo ./Developer_Packages.sh
      ```

   2. If set up on Robot

      ```bash
         cd <path to Auto_Deli_Robot>/Auto_Deli_Robot
         sudo ./Robot_Packages.sh
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
- [Project Web GUI](https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/Auto_Delivery_Robot_Web_Control_GUI) ***External***

---  

## Some Dependence of the Project

- [Lidar Driver and Node Example](https://github.com/Slamtec/rplidar_ros)
- ZED CAM ROS Library
  - [ZED ROS Wrapper](https://github.com/stereolabs/zed-ros-wrapper)
  - [ZED ROS Interfaces](https://github.com/stereolabs/zed-ros-interfaces)
  - [ZED ROS Examples](https://github.com/stereolabs/zed-ros-examples)
- [Line Extraction Module](https://github.com/kam3k/laser_line_extraction.git)  
- [NMEA Message Parser](https://github.com/kosma/minmea.git)
