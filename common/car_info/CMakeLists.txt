cmake_minimum_required(VERSION 2.8.3)
project(car_info)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    adam_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)





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
INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS adam_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(car_basic_info src/car_basic_info_core.cpp
                                src/car_basic_info_node.cpp)

target_link_libraries(car_basic_info ${catkin_LIBRARIES})

add_executable(car_limit_info src/car_limit_info_core.cpp
                                src/car_limit_info_node.cpp)

target_link_libraries(car_limit_info ${catkin_LIBRARIES})



