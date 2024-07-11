cmake_minimum_required(VERSION 3.0.2)
project(arena_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arena_camera
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(OpenCV)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(arena_camera_node src/arena_camera_node.cpp)
add_dependencies(arena_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arena_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
