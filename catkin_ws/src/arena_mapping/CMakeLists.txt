cmake_minimum_required(VERSION 3.0.2)
project(arena_mapping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  message_generation
)

generate_messages( DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf2_ros tf2_geometry_msgs message_runtime
)

#catkin_install_python(PROGRAMS scripts/video_converter.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mobile_base_node_simul src/mobile_base_node_simul.cpp)
add_dependencies(mobile_base_node_simul ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mobile_base_node_simul ${catkin_LIBRARIES})