cmake_minimum_required(VERSION 3.0.2)
project(lidar_module)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rplidar_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_module
  CATKIN_DEPENDS roscpp rospy rplidar_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lidar_module.cpp
# )

add_executable(lidar_avoid src/lidar_avoid.cpp)
target_link_libraries(lidar_avoid ${catkin_LIBRARIES})
