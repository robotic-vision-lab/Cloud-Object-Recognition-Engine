cmake_minimum_required(VERSION 2.8.3)
project(core_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  core_msgs
  std_msgs
  sensor_msgs
  message_generation
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CORE REQUIRED 
  core_filters-0.1
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  core_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${CORE_INCLUDE_DIRS}
)

add_executable(client_pointcloud_processor src/client_pointcloud_processor.cpp)

add_dependencies(client_pointcloud_processor core_client_generate_messages_cpp)

target_link_libraries(
  client_pointcloud_processor
  ${catkin_LIBRARIES}
  ${CORE_LIBRARIES}
)
