cmake_minimum_required(VERSION 3.0.2)
project(position_module)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
  robot_localization
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES position_module
#  CATKIN_DEPENDS rospy std_msgs
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
#   src/${PROJECT_NAME}/position_module.cpp
# )
add_library(minmea STATIC src/minmea/minmea.c)
add_library(SerialGPSObject STATIC src/SerialGPSObject.cpp)
add_library(pidRegulater STATIC src/pidAlgorithm/pidRegulater.cpp)
add_library(pidController STATIC src/pidController.cpp)
target_link_libraries(pidController pidRegulater)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(pid_processor src/pid_processor.cpp)
target_link_libraries(pid_processor ${catkin_LIBRARIES})
target_link_libraries(pid_processor pidController)
target_link_libraries(pid_processor jsoncpp)

add_executable(gps_faker_node src/gps_faker_node.cpp)
target_link_libraries(gps_faker_node ${catkin_LIBRARIES})
add_dependencies(gps_faker_node test_program_generate_messages_cpp)

add_executable(SerialGPSDriver src/SerialGPSDriver.cpp)
target_link_libraries(SerialGPSDriver ${catkin_LIBRARIES})
target_link_libraries(SerialGPSDriver minmea)
target_link_libraries(SerialGPSDriver SerialGPSObject)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/BNO055_IMU.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
