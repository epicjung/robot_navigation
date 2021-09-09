cmake_minimum_required(VERSION 3.0.2)
project(traj_result)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  src
  utils
)

set(algorithm_source

)

add_executable(traj_get_result src/traj_get_result.cpp ${algorithm_source})
target_link_libraries(traj_get_result ${catkin_LIBRARIES})
add_dependencies(traj_get_result ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
