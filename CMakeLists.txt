cmake_minimum_required(VERSION 3.1.0)
project(rs_publisher)

find_package(catkin REQUIRED
  roscpp
  roslaunch
  std_msgs
  tf
  image_transport
  sensor_msgs
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package()

# For including librealsense2 library
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  /usr/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  /usr/lib
  /usr/local/lib
)

add_executable(rs_publisher src/rs_publisher.cpp)
add_dependencies(rs_publisher rs_publisher_generate_messages_cpp)
set(
  LIBRARIES_TO_LINK
  realsense2
)
set_property(TARGET rs_publisher PROPERTY CXX_STANDARD 11)

add_executable(cam_info_publisher src/cam_info_publisher.cpp)
add_dependencies(cam_info_publisher rs_publisher_generate_messages_cpp)
set_property(TARGET cam_info_publisher PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME}
  realsense2
  ${catkin_LIBRARIES}
)
target_link_libraries(cam_info_publisher
  ${catkin_LIBRARIES}
)

set_target_properties(rs_publisher PROPERTIES FORDER "src")
set_target_properties(cam_info_publisher PROPERTIES FORDER "src")
