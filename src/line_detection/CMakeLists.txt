cmake_minimum_required(VERSION 3.0.2)
project(line_detection)

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

#rojects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES line_detection
#  CATKIN_DEPENDS cv_bridge geometry_msgs pcl_msgs roscpp rospy std_msgs tf
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


add_executable(rviz_while_lane_pub.cpp src/rviz_while_lane_pub.cpp)
target_link_libraries(rviz_while_lane_pub.cpp ${catkin_LIBRARIES})

add_executable(sample_rviz.cpp src/sample_rviz.cpp)
target_link_libraries(sample_rviz.cpp ${catkin_LIBRARIES})

add_executable(rviz_white_lane_expand.cpp src/rviz_white_lane_expand.cpp)
target_link_libraries(rviz_white_lane_expand.cpp ${catkin_LIBRARIES})


add_executable(rviz_white_lane_sb.cpp src/rviz_white_lane_sb.cpp)
target_link_libraries(rviz_white_lane_sb.cpp ${catkin_LIBRARIES})


add_executable(polygon_publisher.cpp src/polygon_publisher.cpp)
target_link_libraries(polygon_publisher.cpp ${catkin_LIBRARIES})

add_executable(pointcloud_hsv_filter src/rgbdxyz_check.cpp)
target_link_libraries(pointcloud_hsv_filter ${catkin_LIBRARIES})
