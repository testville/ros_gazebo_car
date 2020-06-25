cmake_minimum_required(VERSION 2.8.3)
project(ackermann_vehicle_gazebo)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
  pluginlib
  ompl
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /usr/include
)
add_library(
  global_planner_lib scripts/RRT_optimization_global_planner.cpp scripts/global_planner.cpp
)

include_directories("/opt/ros/melodic/include")
target_link_libraries(global_planner_lib "/usr/local/lib/libnlopt.so" "/opt/ros/melodic/lib/libompl.so")
target_include_directories(global_planner_lib PUBLIC /opt/ros/melodic/include /usr/include "/usr/include" "/opt/ros/melodic/include")
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
