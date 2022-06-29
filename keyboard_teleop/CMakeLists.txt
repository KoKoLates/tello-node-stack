cmake_minimum_required(VERSION 3.0.2)
project(keyboard_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roslib
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES keyboard_teleop
#  CATKIN_DEPENDS geometry_msgs roslib rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
