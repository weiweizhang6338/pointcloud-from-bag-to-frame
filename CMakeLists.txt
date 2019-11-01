cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_from_bag_to_frame)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pointcloud_from_bag_to_frame_node src/pointcloud_from_bag_to_frame_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pointcloud_from_bag_to_frame_node ${catkin_LIBRARIES})