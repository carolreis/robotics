cmake_minimum_required(VERSION 3.0.2)
project(collision_avoidance)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	rospy
	std_msgs
	message_generation
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

