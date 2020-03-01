cmake_minimum_required(VERSION 2.8.3)
project(global_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  adam_msgs
  tf
  )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs adam_msgs tf
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_executable(global_planning_node src/global_planning_core.cpp
                                    src/global_planning_node.cpp)

target_link_libraries(global_planning_node ${catkin_LIBRARIES})
