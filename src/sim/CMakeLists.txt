cmake_minimum_required(VERSION 3.0.2)
project(sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_plugins
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /usr/include/pcl-1.10
  /usr/include/eigen3
)

add_executable(particle_filter_propose_for_gazebo.cpp src/particle_filter_propose_for_gazebo.cpp)
target_link_libraries(particle_filter_propose_for_gazebo.cpp ${catkin_LIBRARIES})
