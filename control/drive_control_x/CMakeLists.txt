cmake_minimum_required(VERSION 2.8.3)
project(drive_control_x)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  adam_msgs
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

generate_dynamic_reconfigure_options(cfg/PathIntegralParams.cfg)

catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs nodelet dynamic_reconfigure adam_msgs
  INCLUDE_DIRS include
  LIBRARIES adam_plant param-getter 
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${Eigen3_INCLUDE_DIRS})

add_subdirectory(src/path_integral)
