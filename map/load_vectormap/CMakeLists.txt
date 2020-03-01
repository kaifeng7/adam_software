cmake_minimum_required(VERSION 2.8.3)
project(load_vectormap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  adam_msgs
  tf
)
find_package(Protobuf REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(PROTOBUF_FOLDER ${PROJECT_SOURCE_DIR}/lib/protobuf-3.6.1)

set(Protobuf_INCLUDE_DIRS ${PROTOBUF_FOLDER}/include)
set(Protobuf_LIBRARIES ${PROTOBUF_FOLDER}/lib/libprotobuf.so)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES map_protobuf
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs adam_msgs tf
  #DEPENDS system_lib
)


include_directories(
  include
  /usr/local/include/
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)
link_directories(/usr/local/lib/)

## Declare a C++ library
add_library(map_protobuf include/map.pb.cc)

#add_dependencies(map_protobuf ${catkin_EXPORTED_TARGETS})
#target_link_libraries(map_protobuf ${PROTOBUF_LIBRARIES})

## Declare a C++ executable
add_executable(load_vectormap_node src/load_vectormap_core.cpp
                                  src/load_vectormap_vis.cpp
                                  src/load_vectormap_node.cpp)

target_link_libraries(load_vectormap_node ${catkin_LIBRARIES} map_protobuf ${PROTOBUF_LIBRARIES})

#add_dependencies(ros_protobuf_node ${catkin_EXPORTED_TARGETS})
