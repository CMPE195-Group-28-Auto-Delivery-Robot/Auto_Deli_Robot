# Auto_Deli_Robot  

## Setup Workspace  
### First Install All Essential Package
1. Install Python 3   
``` sudo apt install python3 ```
2. Install ROS Medodic follow this guide https://wiki.ros.org/melodic/Installation/Ubuntu
3. Install the required Component  
   ``` 
   cd <path to Auto_Deli_Robot>/Auto_Deli_Robot
   sudo ./Package_Install.sh
### Build ROS WorkSpace
1. Bulid Project   
   ``` catkin_make ```   
2. source the setup.bash so ROS can regonize paskage in the workspace   
   ``` source devel/setup.bash ```   
   If you don't want to run this line everytime you can add   
   ``` source <path to the Auto_Deli_Robot>/Auto_Deli_Robot/devel/setup.bash ```   
   to ```~/.bashrc``` 

---  
## Some Dependence of the Project
* Lidar Driver and Node Example   
https://github.com/Slamtec/rplidar_ros   
* ZED CAM ROS Library   
https://github.com/stereolabs/zed-ros-wrapper   
https://github.com/stereolabs/zed-ros-interfaces   
https://github.com/stereolabs/zed-ros-examples
* Line Extraction Module  
https://github.com/kam3k/laser_line_extraction.git   
* NMEA Message Parser   
https://github.com/kosma/minmea.git   

