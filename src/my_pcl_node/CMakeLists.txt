cmake_minimum_required(VERSION 3.0.2)
project(my_pcl_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  tf2
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_node src/pcl_node.cpp)
target_link_libraries(pcl_node ${catkin_LIBRARIES})