cmake_minimum_required(VERSION 2.8.3)
project(local_planning)
set(ROS_VERSION $ENV{ROS_DISTRO})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs  
  sensor_msgs
  adam_msgs
  tf
  utility
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS 
  roscpp 
  geometry_msgs 
  nav_msgs
  adam_msgs
  utility
#  DEPENDS system_lib
)

###########
## Build ##
###########


if ("${ROS_VERSION}" MATCHES "(kinetic)")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROS_KINETIC")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(local_planning_node src/local_planning_node.cpp 
                              src/local_planning_core.cpp 
                              src/local_planning_vis.cpp
                            )
target_link_libraries(local_planning_node ${catkin_LIBRARIES} )
