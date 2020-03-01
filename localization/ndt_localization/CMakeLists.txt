cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  rosbag
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  ndt_cpu
)

find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf ${PCL_OPENMP_PACKAGES}
 DEPENDS system_lib PCL
)

string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
link_directories(${LIBRARY_DIRS})  # 可以将libgpu_ndt.so加到这个路径里，然后就可以链接到了

# link_directories(lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ../../common_0725/user_protocol/include
  ../../common_0725/utils/include
)

add_library(ndt_localization
  src/ndt_localization.cpp
)


target_link_libraries(ndt_localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(ndt_localization PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

add_executable(ndt_localization_node
  src/ndt_localization_node.cpp
)

add_dependencies(ndt_localization_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ndt_localization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_localization
)

# add_executable(ndt_test
#   src/ndt_test.cpp
# )

# target_link_libraries(ndt_test
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_executable(tf_test
#   src/tf_test.cpp
# )

# add_dependencies(tf_test
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(tf_test
#   ${catkin_LIBRARIES}
# )
