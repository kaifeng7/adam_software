cmake_minimum_required(VERSION 2.8.3)
project(adam_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    ControlCommand.msg
    ControlCommandStamped.msg
    lamp_cmd.msg
    VehicleCmd.msg
    Lane.msg
    LaneStamped.msg
    WayPoint.msg
    WayPointStamped.msg
    Road.msg
    #autorally msgs
    pathIntegralStatus.msg
    pathIntegralTiming.msg
    chassisCommand.msg
    chassisState.msg
    runstop.msg
    #Car msgs

    CarBasicInfo.msg
    CarLimitInfo.msg
    VCU.msg
    pid.msg
    VCUEPS.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#   
  nav_msgs#   
  sensor_msgs#   
  std_msgs
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
  DEPENDS
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
