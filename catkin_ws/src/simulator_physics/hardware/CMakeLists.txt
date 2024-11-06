cmake_minimum_required(VERSION 3.0.2)
project(hardware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
  random_numbers
)

add_service_files(
  FILES
  LightReadings.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hardware
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(light_sensors_node src/light_sensors_node.cpp)
add_executable(laser_scan_node src/laser_scan_node.cpp)

add_dependencies(light_sensors_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(laser_scan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(light_sensors_node ${catkin_LIBRARIES})
target_link_libraries(laser_scan_node ${catkin_LIBRARIES})