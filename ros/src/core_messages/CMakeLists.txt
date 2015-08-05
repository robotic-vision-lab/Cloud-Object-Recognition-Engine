cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  PointCloud.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  message_generation
)
