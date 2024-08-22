cmake_minimum_required(VERSION 3.0.2)
project(arena_control)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  control_lights.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
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
