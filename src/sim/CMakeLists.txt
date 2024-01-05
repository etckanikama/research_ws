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
  jsk_rviz_plugins
  geometry_msgs
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
add_executable(likelyfood_distribution_for_gazebo.cpp src/likelyfood_distribution_for_gazebo.cpp)
add_executable(get_yaw.cpp src/get_yaw.cpp)
add_executable(rviz_white_lane_sb_for_gazebo.cpp src/rviz_white_lane_sb_for_gazebo.cpp)
add_executable(sample_marker_publisher.cpp src/sample_marker_publisher.cpp)
add_executable(points_sub_distance.cpp src/points_sub_distance.cpp)
add_executable(beego_odom_publisher.cpp src/beego_odom_publisher.cpp)

target_link_libraries(beego_odom_publisher.cpp ${catkin_LIBRARIES})
target_link_libraries(particle_filter_propose_for_gazebo.cpp ${catkin_LIBRARIES})
target_link_libraries(likelyfood_distribution_for_gazebo.cpp ${catkin_LIBRARIES})
target_link_libraries(get_yaw.cpp ${catkin_LIBRARIES})
target_link_libraries(rviz_white_lane_sb_for_gazebo.cpp ${catkin_LIBRARIES})
target_link_libraries(sample_marker_publisher.cpp ${catkin_LIBRARIES})
target_link_libraries(points_sub_distance.cpp  ${catkin_LIBRARIES})