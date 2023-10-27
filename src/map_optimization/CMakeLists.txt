cmake_minimum_required(VERSION 3.0.2)
project(map_optimization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  pcl_msgs
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
  jsk_rviz_plugins
  tf2_sensor_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_optimization
#  CATKIN_DEPENDS other_catkin_pkg
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
  /usr/include/pcl-1.10
  /usr/include/eigen3
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/map_optimization.cpp
# )

add_executable(candinate_F.cpp src/candinate_F.cpp)
add_executable(create_Ad.cpp src/create_Ad.cpp)

target_link_libraries(candinate_F.cpp ${catkin_LIBRARIES})
target_link_libraries(create_Ad.cpp ${catkin_LIBRARIES})
# target_link_libraries(create_Ad.cpp
#   # 他のリンク対象
#   tf2_ros
# )