cmake_minimum_required(VERSION 3.0.2)
project(tMap_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  TrvMapDet_node.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
