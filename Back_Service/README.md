# Robot Back Service   
## This part contain the run script to run the robot system in one command   
   
### ROSCore_StartUp ( roscore_sev.service )
Most Important Service, it will run a roscore. So you don't have to use a terminal to run it. 
### FieldTest_Service_StartUp.sh ( ros_field_sev.service )
***When running this service, you don't need to run roscore_sev.service seperatly***   
This script will run the entire Robot into complete running mode. It is automatically run on boot of the robot by its serives   
### Dev_Service_StartUp.sh ( ros_dev_sev.service ) 
***Required roscore_sev.service running***   
This script will run a ZED Cam node, fake GPS Node, and all web gui service node. It is good to be use at home when no real gps signal avaliable   
### GUI_Service_StartUp.sh ( ros_gui_sev.service )
***Required roscore_sev.service running***   
This script will run all web gui service node. 
### ZED_Wrapper_StartUp.sh ( ros_zed.service )
***Required roscore_sev.service running***   
This script will run a ZED Cam node.    

