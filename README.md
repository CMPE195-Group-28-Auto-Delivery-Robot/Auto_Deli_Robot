# Auto_Deli_Robot  

## Setup Workspace  
### First Install All Essential Package
1. Install Python 3   
``` sudo apt install python3 ```
2. Install CMake
2. Install ROS Medodic follow this guide https://wiki.ros.org/melodic/Installation/Ubuntu

### Build ROS WorkSpace
1. First time bulid   
   ``` catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 ```   
   After the first time it is fine to just run    
   ``` catkin_make ```   
2. source the setup.bash so ROS can regonize paskage in the workspace   
   ``` source devel/setup.bash ```   
   If you don't want to run this line everytime you can add   
   ``` source <path to the folder>/devel/setup.bash ```   
   to ```~/.bashrc``` 

---  
## Some Dependence of the Project
* Lidar Driver and Node Example
https://github.com/Slamtec/rplidar_ros  
