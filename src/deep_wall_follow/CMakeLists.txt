cmake_minimum_required(VERSION 3.0.2)
project(deep_wall_follow)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
  urdf
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs tf urdf geometry_msgs sensor_msgs 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

