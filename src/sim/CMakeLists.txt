cmake_minimum_required(VERSION 3.0.2)
project(sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_plugins
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
)


