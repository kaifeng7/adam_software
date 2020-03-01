cmake_minimum_required(VERSION 2.8.3)
project(waypoint_following)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
        adam_msgs
        tf
        utility
        )

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp
        tf
        geometry_msgs
        nav_msgs
        std_msgs
        adam_msgs
        utility
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(pure_pursuit
        src/pure_pursuit_node.cpp
        src/pure_pursuit_core.cpp
        src/pure_pursuit_viz.cpp
        )
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})
add_dependencies(pure_pursuit      ${catkin_EXPORTED_TARGETS})


add_executable(after_currentpose_path src/after_currentpose_path.cpp)

target_link_libraries(after_currentpose_path  ${catkin_LIBRARIES})
add_dependencies(after_currentpose_path       ${catkin_EXPORTED_TARGETS})
