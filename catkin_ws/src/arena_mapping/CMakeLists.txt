cmake_minimum_required(VERSION 3.0.2)
project(arena_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

generate_messages( DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
)

#catkin_install_python(PROGRAMS scripts/video_converter.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

include_directories(${catkin_INCLUDE_DIRS})
