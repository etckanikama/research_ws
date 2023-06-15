cmake_minimum_required(VERSION 3.0.2)
project(particlefilter_simulation_basic)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES particlefilter_simulation_basic
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /usr/include/pcl-1.10
  /usr/include/eigen3
)

## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/particlefilter_simulation_basic.cpp)
add_executable(particle_filter_propose.cpp src/particle_filter_propose.cpp)
add_executable(likelihood_distribution.cpp src/likelihood_distribution.cpp)
add_executable(sub_odom_to_csv.cpp src/sub_odom_to_csv.cpp)
add_executable(sub_amcl_pose_to_csv.cpp src/sub_amcl_pose_to_csv.cpp)
add_executable(Inpedance_control_check.cpp src/Inpedance_control_check.cpp)
add_executable(resample_test.cpp src/resample_test.cpp)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(particle_filter_propose.cpp ${catkin_LIBRARIES})
target_link_libraries(likelihood_distribution.cpp ${catkin_LIBRARIES})
target_link_libraries(sub_odom_to_csv.cpp ${catkin_LIBRARIES})
target_link_libraries(sub_amcl_pose_to_csv.cpp ${catkin_LIBRARIES})
target_link_libraries(Inpedance_control_check.cpp ${catkin_LIBRARIES})
target_link_libraries(resample_test.cpp  ${catkin_LIBRARIES})


